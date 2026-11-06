//! `SetupParallelEvalPolicy` (`PARALLEL_EVAL`, experimental upstream): batches of moves chosen
//! across many targets at once, each target's candidates estimated and scored, the batch
//! committed under one timing update and judged against a checkpoint.
//!
//! Ported stage group by stage group, in the reference's order. Each stage is its own function
//! named after the reference's, and `start` / `iterate` do nothing but call them, so a trace that
//! diverges from the reference's points at one stage. Modelled so far: `start` (S1) and the
//! target collection of a batch (S2: `collectTargetEndpoints`, `collectTargets`). Candidate
//! generation onward (S3: `generateCandidates`) is refused.

use std::collections::HashSet;

use vyges_sta::fuzzy;

use super::{PathView, Repair, Stop};
use crate::repair_timing::{Effort, Move};

/// `BatchPriorityConfig`: how stalled batches favour targets in shared violating cones.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatchPriorityConfig {
    pub bottleneck_scale: f32,
    pub bottleneck_exponent: f32,
    pub bottleneck_max_wns_stall_batches: i32,
    pub bottleneck_max_endpoints: i32,
    pub bottleneck_max_startpoints: i32,
    pub bottleneck_refresh_batches: i32,
}

impl Default for BatchPriorityConfig {
    fn default() -> Self {
        BatchPriorityConfig { bottleneck_scale: 2.0, bottleneck_exponent: 2.0, bottleneck_max_wns_stall_batches: 8, bottleneck_max_endpoints: 1024, bottleneck_max_startpoints: 1024, bottleneck_refresh_batches: 4 }
    }
}

/// `BatchSelectorConfig`: the score floor and the worst-endpoint reserve of batch admission.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatchSelectorConfig {
    pub score_floor_start: i32,
    pub min_score_ratio: f32,
    pub wns_reserve_pct: i32,
    /// Endpoints at or above this slack take no reserved slot: `start` sets it to the run's
    /// `-setup_margin` before the selector copies the config.
    pub slack_margin: f32,
}

impl Default for BatchSelectorConfig {
    fn default() -> Self {
        BatchSelectorConfig { score_floor_start: 16, min_score_ratio: 0.0, wns_reserve_pct: 0, slack_margin: 0.0 }
    }
}

/// `ParallelEvalPolicyConfig`: batch size, the evaluation funnel, selection, checkpoints, the stall
/// rules, the WNS tail and the work meter (reference seconds, so a run decides the same on every
/// machine). `*_period_pct` fields are percent of the run's reference period.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParallelEvalConfig {
    pub min_batch_cap: i32,
    pub max_batch_cap: i32,
    pub retry_batch_cap: i32,
    pub eval_endpoint_factor: i32,
    pub eval_min_endpoints: i32,
    pub rebuffer_max_targets: i32,
    pub priority: BatchPriorityConfig,
    pub selector: BatchSelectorConfig,
    pub max_checkpoint_misses: i32,
    pub culprit_delay_ratio: f32,
    pub checkpoint_wns_tolerance_period_pct: f32,
    pub miss_tabu_batches: i32,
    pub max_undos_per_instance: i32,
    pub stall_batches: i32,
    pub stall_max_batches: i32,
    pub progress_tns_pct: f32,
    pub progress_wns_period_pct: f32,
    pub stall_window: i32,
    pub stall_wns_period_pct: f32,
    pub stall_failing_endpoint_pct: f32,
    pub stall_window_min_work: f32,
    pub wns_tail_endpoints: i32,
    pub wns_tail_batch: i32,
    pub wns_tail_stall: i32,
    pub wns_tail_max_batches: i32,
    pub wns_tail_grow: i32,
    pub wns_tail_max_endpoints: i32,
    pub wns_tail_tns_ratio: f32,
    pub wns_tail_min_gain_period_pct: f32,
    pub wns_tail_work_pct: f32,
    pub wns_tail_min_work: f32,
    pub wns_tail_speculate: i32,
    pub work_sta_per_vertex: f32,
    pub work_per_pending_net: f32,
    pub work_per_estimate: f32,
    pub work_per_plan: f32,
    pub work_per_endpoint: f32,
}

impl Default for ParallelEvalConfig {
    fn default() -> Self {
        ParallelEvalConfig {
            min_batch_cap: 64,
            max_batch_cap: 512,
            retry_batch_cap: 8,
            eval_endpoint_factor: 2,
            eval_min_endpoints: 256,
            rebuffer_max_targets: 128,
            priority: BatchPriorityConfig::default(),
            // The policy's own admission: a 5% score floor and a 10% worst-path reserve.
            selector: BatchSelectorConfig { min_score_ratio: 0.05, wns_reserve_pct: 10, ..BatchSelectorConfig::default() },
            max_checkpoint_misses: 8,
            culprit_delay_ratio: 0.01,
            checkpoint_wns_tolerance_period_pct: 0.02,
            miss_tabu_batches: 8,
            max_undos_per_instance: 3,
            stall_batches: 6,
            stall_max_batches: 24,
            progress_tns_pct: 0.5,
            progress_wns_period_pct: 0.04,
            stall_window: 6,
            stall_wns_period_pct: 0.2,
            stall_failing_endpoint_pct: 1.0,
            stall_window_min_work: 30.0,
            wns_tail_endpoints: 8,
            wns_tail_batch: 8,
            wns_tail_stall: 6,
            wns_tail_max_batches: 300,
            wns_tail_grow: 2,
            wns_tail_max_endpoints: 64,
            wns_tail_tns_ratio: 10.0,
            wns_tail_min_gain_period_pct: 0.1,
            wns_tail_work_pct: 400.0,
            wns_tail_min_work: 60.0,
            wns_tail_speculate: 6,
            work_sta_per_vertex: 5.0e-7,
            work_per_pending_net: 1.93e-3,
            work_per_estimate: 1.35e-5,
            work_per_plan: 1.14e-3,
            work_per_endpoint: 5.65e-4,
        }
    }
}

/// The Mt moves' knobs (`OptimizationPolicyConfig` past the estimator's two) as
/// `forParallelEval(effort)` sets them, with the policy's own [`ParallelEvalConfig`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MtConfig {
    pub fanin_side_loss: bool,
    pub fanin_side_loss_weight: f32,
    pub fanin_side_offpath_weight: f32,
    pub side_floor_margin: f32,
    pub dual_rf_eval: bool,
    pub dual_rf_drive_guard: bool,
    pub size_up_mt_score_scale: f32,
    pub size_up_mt_max_drive_ratio: f32,
    pub rebuffer_mt_score_scale: f32,
    pub rebuffer_mt_min_score: f32,
    pub rebuffer_mt_max_score: f32,
    pub rebuffer_mt_max_candidates: i32,
    pub allow_rebuffer_mt_single_buffer: bool,
    pub rebuffer_mt_side_loads: bool,
    pub split_load_mt_score_scale: f32,
    pub size_down_fanout_mt_score_scale: f32,
    pub parallel_eval: ParallelEvalConfig,
}

impl Default for MtConfig {
    /// The shipped values, before `forParallelEval`.
    fn default() -> Self {
        MtConfig {
            fanin_side_loss: false,
            fanin_side_loss_weight: 3.0,
            fanin_side_offpath_weight: 0.0,
            side_floor_margin: 1.0,
            dual_rf_eval: false,
            dual_rf_drive_guard: true,
            size_up_mt_score_scale: 0.75,
            size_up_mt_max_drive_ratio: 0.0,
            rebuffer_mt_score_scale: 0.10,
            rebuffer_mt_min_score: 5.0e-12,
            rebuffer_mt_max_score: 45.0e-12,
            rebuffer_mt_max_candidates: 3,
            allow_rebuffer_mt_single_buffer: false,
            rebuffer_mt_side_loads: false,
            split_load_mt_score_scale: 0.50,
            size_down_fanout_mt_score_scale: 0.75,
            parallel_eval: ParallelEvalConfig::default(),
        }
    }
}

/// `OptimizationPolicyConfig::forParallelEval(effort)`: the side effects charged (fanin loads,
/// buffer-tree side loads, the opposite transition), the size-up jump bounded and its rise/fall
/// guard off, one-buffer trees in the pool with no upper score gate; then `-effort`: low has no
/// WNS tail, medium stalls sooner and widens the tail less, high is the default.
pub fn for_parallel_eval(effort: Effort) -> MtConfig {
    let mut c = MtConfig { fanin_side_loss: true, rebuffer_mt_side_loads: true, dual_rf_eval: true, size_up_mt_max_drive_ratio: 2.5, dual_rf_drive_guard: false, allow_rebuffer_mt_single_buffer: true, rebuffer_mt_max_score: 0.0, rebuffer_mt_score_scale: 0.40, ..MtConfig::default() };
    let pe = &mut c.parallel_eval;
    match effort {
        Effort::Low => pe.wns_tail_endpoints = 0,
        Effort::Medium => {
            pe.stall_max_batches = 12;
            pe.progress_tns_pct = 1.0;
            pe.progress_wns_period_pct = 0.08;
            pe.wns_tail_max_endpoints = 16;
            pe.wns_tail_work_pct = 100.0;
            pe.wns_tail_min_work = 15.0;
        }
        Effort::High => {}
    }
    c
}

/// `StallMonitor`: the batch stage's progress against its stall rules (the rules: S5). Here, what
/// `reset` and `beginBatch` keep.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StallMonitor {
    pub initial_tns: f32,
    pub reference_period: f32,
    pub stalled_batches: i32,
    pub stalled_moves: i32,
    pub last_batch_moves: i32,
    pub wns_stall_batches: i32,
    pub window_stalled: bool,
    pub last_cap: i32,
    pub failing_endpoints: usize,
}

impl StallMonitor {
    /// `reset(initial_tns, reference_period)`.
    pub fn reset(&mut self, initial_tns: f32, reference_period: f32) {
        *self = StallMonitor { initial_tns, reference_period, last_cap: self.last_cap, failing_endpoints: self.failing_endpoints, ..StallMonitor::default() };
    }

    /// `beginBatch(cap, failing_endpoints)`.
    pub fn begin_batch(&mut self, batch_cap: i32, failing_endpoints: usize) {
        self.last_cap = batch_cap;
        self.failing_endpoints = failing_endpoints;
    }
}

/// `BatchScope`: the violating endpoints a batch draws from and its move cap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatchScope {
    pub target_endpoints: usize,
    pub batch_cap: i32,
}

/// `Target` in its path-driver view (`makePathDriverTarget`): a driver on an endpoint's worst path.
#[derive(Debug, Clone, PartialEq)]
pub struct Target {
    pub driver_pin: String,
    /// The pin of the path the driver was found on: the endpoint, or a latch D for a latch fanin
    /// segment.
    pub endpoint_pin: String,
    /// The failing endpoint the target was collected for.
    pub violated_endpoint_pin: String,
    /// The path's transition at the driver.
    pub driver_rf: usize,
    /// The driver's index in its expanded path, root first.
    pub path_index: usize,
    /// The load the driver feeds on the path; none when the path ends at the driver.
    pub load_pin: Option<String>,
    /// The endpoint's slack as the collector holds it.
    pub slack: f32,
    /// `targetFanout`: the driver vertex's wire edges out.
    pub fanout: usize,
    /// The path's scene.
    pub scene: usize,
    /// The driver's instance and its cell.
    pub inst: String,
    pub cell: String,
}

/// `BatchPool`: the targets of one batch (a deque upstream, so they never move).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BatchPool {
    pub targets: Vec<Target>,
}

/// `adaptiveBatchSize`: the failing endpoint count clamped to [min_cap, max_cap].
pub fn adaptive_batch_size(failing_endpoint_count: usize, min_cap: i32, max_cap: i32) -> i32 {
    failing_endpoint_count.max(min_cap as usize).min(max_cap as usize) as i32
}

/// `evaluationEndpointLimit`: endpoints per chunk, the cap times the factor, at least the floor.
pub fn evaluation_endpoint_limit(batch_cap: i32, eval_endpoint_factor: i32, min_endpoints: i32) -> usize {
    (batch_cap as usize * eval_endpoint_factor as usize).max(min_endpoints as usize)
}

/// The VYGP printer's `{:.9e}` as fmt (C++) prints it: the exponent signed, at least two digits.
fn e9(v: f32) -> String {
    let s = format!("{:.9e}", v);
    let (m, e) = s.split_once('e').expect("Rust's {:e} always has an exponent");
    let e: i32 = e.parse().expect("Rust's {:e} exponent is an integer");
    format!("{m}e{}{:02}", if e < 0 { '-' } else { '+' }, e.abs())
}

/// The policy's run state (`SetupParallelEvalPolicy`'s members), as `start` leaves it.
#[derive(Debug, Clone, PartialEq)]
pub struct ParallelEval {
    pub config: MtConfig,
    /// `setRecordCritPathLoadPins(fanin_side_loss)`: the collector keeps the critical path load pins.
    pub record_crit_path_load_pins: bool,
    /// `move_sequence_`: the moves of the run, each with a PARALLEL_EVAL form.
    pub move_sequence: Vec<Move>,
    pub unbuffer_in_sequence: bool,
    /// `move_generators_`: the moves `makeParallelEvalGenerator` makes a generator for.
    pub generators: Vec<Move>,
    /// The selector's config, copied after the margin went in.
    pub selector: BatchSelectorConfig,
    pub committed_moves: i32,
    pub batch_index: i32,
    pub tentative_moves: i32,
    pub checkpoint_misses: i32,
    pub small_batch_retry_started: bool,
    pub max_utilization_warned: bool,
    pub batch_journal_open: bool,
    pub rebuffer_targets_in_batch: i32,
    pub max_end_repairs: usize,
    pub checkpoint_wns: f32,
    pub checkpoint_tns: f32,
    pub best_checkpoint_wns: f32,
    pub reference_period: f32,
    pub graph_vertices: usize,
    pub converged: bool,
    pub stall: StallMonitor,
    /// `critPathLoadPins`: every load pin on a collected worst path.
    pub crit_path_load_pins: HashSet<String>,
    /// The phase's progress-row marker (`phaseMarkerForIndex`).
    pub marker: char,
}

/// `selectMoveSequence`'s exclusion: BufferToInverters has no gain estimate to rank it; Clone,
/// SizeUpMatch and Reroute have no Mt form.
fn has_mt_form(m: Move) -> bool {
    !matches!(m, Move::BufferToInverters | Move::Clone | Move::SizeUpMatch | Move::Reroute)
}

/// `makeParallelEvalGenerator`: a generator for every move with an Mt form (none for the rest).
fn has_generator(m: Move) -> bool {
    has_mt_form(m)
}

/// `keepShortest`: the shortest positive value seen.
fn keep_shortest(shortest: &mut f32, value: f32) {
    if value > 0.0 && (*shortest == 0.0 || value < *shortest) {
        *shortest = value;
    }
}

/// `referencePeriod`: the time scale of the run's `*_period_pct` knobs. The shortest period of a
/// clock with a source pin in any mode; else the shortest virtual clock (IO budgets alone are
/// rough); else, with no clock, the shortest `set_max_delay`; else |initial WNS|.
pub fn reference_period(sdc: &vyges_sta::sdc::Sdc, initial_wns: f32) -> f32 {
    let (mut real, mut virt) = (0.0f32, 0.0f32);
    for c in &sdc.clocks {
        keep_shortest(if c.source.is_empty() { &mut virt } else { &mut real }, c.period);
    }
    if real > 0.0 {
        return real;
    }
    if virt > 0.0 {
        return virt;
    }
    let mut max_delay = 0.0f32;
    for p in &sdc.path_delays {
        // `isPathDelay() && minMax()->matches(max)`: a set_max_delay (false paths, multicycles and
        // group paths are other exception kinds).
        if !p.false_path && p.multicycle.is_none() && p.group.is_none() && p.min_max == vyges_sta::liberty::MAX {
            keep_shortest(&mut max_delay, p.delay);
        }
    }
    if max_delay > 0.0 {
        return max_delay;
    }
    initial_wns.abs()
}

impl Repair<'_, '_> {
    /// `Optimizer::run` for the policy: `start`, then `iterate` until converged, then `finish`.
    pub(super) fn parallel_eval_policy(&mut self, marker: char) -> Result<(), Stop> {
        let mut pe = self.pe_start(marker)?;
        while !pe.converged {
            self.pe_iterate(&mut pe)?;
        }
        Ok(())
    }

    /// `SetupParallelEvalPolicy::start`.
    fn pe_start(&mut self, marker: char) -> Result<ParallelEval, Stop> {
        let config = for_parallel_eval(self.args.effort.unwrap_or(Effort::High));
        self.pe_optimization_policy_start();
        let mut pe = ParallelEval {
            config,
            record_crit_path_load_pins: config.fanin_side_loss,
            move_sequence: Vec::new(),
            unbuffer_in_sequence: false,
            generators: Vec::new(),
            selector: config.parallel_eval.selector,
            committed_moves: 0,
            batch_index: 0,
            tentative_moves: 0,
            checkpoint_misses: 0,
            small_batch_retry_started: false,
            max_utilization_warned: false,
            batch_journal_open: false,
            rebuffer_targets_in_batch: 0,
            max_end_repairs: usize::MAX,
            checkpoint_wns: 0.0,
            checkpoint_tns: 0.0,
            best_checkpoint_wns: 0.0,
            reference_period: 0.0,
            graph_vertices: 0,
            converged: false,
            stall: StallMonitor::default(),
            crit_path_load_pins: HashSet::new(),
            marker,
        };
        self.pe_select_move_sequence(&mut pe);
        pe_build_move_generators(&mut pe);
        // The reserve follows the margin the targets are collected with: in before the copy.
        pe.selector = BatchSelectorConfig { slack_margin: self.ctx.margin, ..pe.config.parallel_eval.selector };
        pe_reset_run_state(&mut pe);
        self.pe_collector_init();
        self.pe_max_end_repairs(&mut pe);
        self.pe_save_initial_checkpoint(&mut pe)?;
        if self.tracker_level() >= 1 {
            self.capture_initial_slack_distribution(false)?;
        }
        self.print_progress_header(false);
        self.pe_trace_start(&pe);
        Ok(pe)
    }

    /// `OptimizationPolicy::start` for an experimental policy: RSZ-2024.
    fn pe_optimization_policy_start(&mut self) {
        self.report("[WARNING RSZ-2024] Experimental repair setup policy 'SetupParallelEvalPolicy' selected. Do not use this for production.".into());
    }

    /// `selectMoveSequence`: `-sequence`, else the default (Unbuffer, VtSwap, SizeUp, Buffer,
    /// SplitLoad), each move kept when `moveEnabled`; a move with no Mt form warns (RSZ-2028) and
    /// is skipped.
    fn pe_select_move_sequence(&mut self, pe: &mut ParallelEval) {
        let requested = if self.args.sequence.is_empty() { vec![Move::Unbuffer, Move::VtSwap, Move::SizeUp, Move::Buffer, Move::SplitLoad] } else { self.args.sequence.clone() };
        pe.move_sequence.clear();
        for m in requested {
            if !self.pe_move_enabled(m) {
                continue;
            }
            if !has_mt_form(m) {
                self.report(format!("[WARNING RSZ-2028] PARALLEL_EVAL has no form of move '{}'; it is skipped.", m.name()));
                continue;
            }
            pe.move_sequence.push(m);
        }
        pe.unbuffer_in_sequence = pe.move_sequence.contains(&Move::Unbuffer);
    }

    /// `moveEnabled`: the `-skip_*` option of each move; VtSwap also needs more than one VT
    /// category.
    fn pe_move_enabled(&self, m: Move) -> bool {
        let a = self.args;
        match m {
            Move::VtSwap => !a.skip_vt_swap && self.ctx.vt_category_count > 1,
            Move::SizeUp | Move::SizeUpMatch | Move::Reroute => true,
            Move::Buffer | Move::SplitLoad => !a.skip_buffering,
            Move::SwapPins => !a.skip_pin_swap,
            Move::Clone => !a.skip_gate_cloning,
            Move::Unbuffer => !a.skip_buffer_removal,
            Move::SizeDownFanout => !a.skip_size_down_fanout,
            Move::BufferToInverters => !a.skip_buffer_to_inverters,
        }
    }

    /// `target_collector_->init(setup_slack_margin)`: the violating endpoints, worst first, before
    /// the first QoR sample.
    fn pe_collector_init(&mut self) {
        let ends = crate::repair_timing::collect_violating(&self.timing.ends, self.ctx.margin);
        self.collector_violating = ends.len();
        self.collector_ends = ends.iter().map(|p| (p.pin.clone(), p.slack)).collect();
        self.current_endpoint = ends.first().map(|p| p.pin.clone());
    }

    /// `max_end_repairs_`: with `-repair_tns` under 100%, as many worst endpoints as LEGACY would
    /// repair now (`int(count * percent)`, at least one); else no limit.
    fn pe_max_end_repairs(&self, pe: &mut ParallelEval) {
        pe.max_end_repairs = if self.args.repair_tns_end_percent < 1.0 { ((self.collector_violating as f64 * self.args.repair_tns_end_percent) as i32).max(1) as usize } else { usize::MAX };
    }

    /// `saveInitialCheckpoint`: the collector's WNS and TNS (`measureQor`), the best WNS, the
    /// reference period (debug `repair_setup` 1), the graph size the work meter reads.
    fn pe_save_initial_checkpoint(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        pe.checkpoint_wns = self.timing.worst().0;
        pe.checkpoint_tns = self.timing.tns();
        pe.best_checkpoint_wns = pe.checkpoint_wns;
        pe.reference_period = reference_period(self.ctx.ssdc, pe.checkpoint_wns);
        pe.stall.reset(pe.checkpoint_tns, pe.reference_period);
        pe.graph_vertices = self.pe_graph_vertices()?;
        self.debug("repair_setup", 1, format!("PARALLEL_EVAL reference clock period {:.3e}", pe.reference_period));
        Ok(())
    }

    /// `graph_->vertexCount()`.
    fn pe_graph_vertices(&mut self) -> Result<usize, Stop> {
        let edits = self.design.take_timer_edits();
        let ctx = self.ctx;
        super::timed_all(ctx, self.design.as_design(), &mut self.timer, edits, |gs, _, _| Ok(gs[0].vertices.len()))
    }

    /// `iterate`: one batch. Collect, weigh, evaluate; then commit, judge, stop (S3 onward: not
    /// modelled).
    fn pe_iterate(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        if pe.converged {
            return Ok(());
        }
        if self.pe_finish_if_no_setup_violations(pe)? || pe_finish_at_max_area() {
            return Ok(());
        }
        let scope = self.pe_collect_target_endpoints(pe)?;
        // `analyzeBottlenecks`: the weights apply only while saved batches stall WNS, which no
        // batch before the first commit does.
        if pe.stall.wns_stall_batches > 0 {
            return Err(Stop::refused("RSZ-ABSENT", "repair_timing -phases PARALLEL_EVAL: analyzeBottlenecks is not modelled".into()));
        }
        let _pool = self.pe_evaluate_targets(pe, scope)?;
        Err(Stop::refused("RSZ-ABSENT", "repair_timing -phases PARALLEL_EVAL: the batch commit (commitBatch onward) is not modelled".into()))
    }

    /// `finishIfNoSetupViolations`: WNS at or above the margin ends the run (its `finish`: not
    /// modelled).
    fn pe_finish_if_no_setup_violations(&mut self, _pe: &mut ParallelEval) -> Result<bool, Stop> {
        if fuzzy::less(self.timing.worst().0, self.ctx.margin) {
            return Ok(false);
        }
        Err(Stop::refused("RSZ-ABSENT", "repair_timing -phases PARALLEL_EVAL with no setup violation (finish): not modelled".into()))
    }

    /// `collectTargetEndpoints`: the collector re-initialized, the progress row (every batch
    /// with -verbose, else the first), the endpoints the batch draws from (`-repair_tns` bounds
    /// them) and its cap.
    fn pe_collect_target_endpoints(&mut self, pe: &mut ParallelEval) -> Result<BatchScope, Stop> {
        pe.rebuffer_targets_in_batch = 0;
        self.trk_capture_pre_phase_slack()?;
        self.pe_collector_init();
        if self.args.verbose || pe.batch_index == 0 {
            self.pe_print_row(&format!("{}{}", pe.batch_index, pe.marker));
        }
        let failing_endpoint_count = self.collector_violating;
        let target_endpoints = failing_endpoint_count.min(pe.max_end_repairs);
        let batch_cap = pe_effective_batch_cap(pe, failing_endpoint_count);
        pe.stall.begin_batch(batch_cap, failing_endpoint_count);
        let mut t = format!("VYGP|scope|{}|fep={failing_endpoint_count}|target_endpoints={target_endpoints}|cap={batch_cap}\n", pe.batch_index);
        for (i, (pin, slack)) in self.collector_ends.iter().enumerate() {
            t.push_str(&format!("VYGP|vend|{}|{i}|{pin}|{}\n", pe.batch_index, e9(*slack)));
        }
        pe_trace(&t);
        Ok(BatchScope { target_endpoints, batch_cap })
    }

    /// `ProgressReporter::printRow` (no startpoint metrics): the collector's WNS, TNS, Viol and
    /// worst endpoint.
    fn pe_print_row(&mut self, label: &str) {
        let (wns, en_tns, worst) = self.timing.row(self.ctx.margin);
        let row = super::progress_row(
            &super::Row {
                iter: label,
                removed: self.committer.total(Move::Unbuffer),
                resized: self.committer.total(Move::SizeUp) + self.committer.total(Move::SizeDownFanout) + self.committer.total(Move::SizeUpMatch) + self.committer.total(Move::VtSwap),
                inserted: self.committer.total(Move::Buffer) + self.committer.total(Move::SplitLoad),
                cloned: self.committer.total(Move::Clone),
                swaps: self.committer.total(Move::SwapPins),
                area_growth_percent: self.area_growth_percent(),
                wns,
                st_tns: None,
                en_tns,
                viol: self.collector_violating,
                worst: worst.as_deref().unwrap_or(""),
            },
            self.ctx.time_scale,
        );
        // A refused run reports no log: the trace keeps the row.
        pe_trace(&format!("PE|line|{row}\n"));
        self.report(row);
    }

    /// `evaluateTargets`: the endpoints evaluated worst first, a chunk at a time, the next chunk
    /// only while the batch is not full. Here up to the first chunk's targets: generation (S3)
    /// is refused.
    fn pe_evaluate_targets(&mut self, pe: &mut ParallelEval, scope: BatchScope) -> Result<BatchPool, Stop> {
        let cfg = pe.config.parallel_eval;
        let chunk_endpoints = evaluation_endpoint_limit(scope.batch_cap, cfg.eval_endpoint_factor, cfg.eval_min_endpoints);
        let mut pool = BatchPool::default();
        let mut visited_pins: HashSet<String> = HashSet::new();
        let mut next_endpoint = 0;
        while next_endpoint < scope.target_endpoints {
            let chunk_end = scope.target_endpoints.min(next_endpoint + chunk_endpoints);
            let chunk = self.pe_collect_targets(pe, next_endpoint, chunk_end, &mut visited_pins)?;
            pe_trace(&format!("VYGP|chunk|{}|{next_endpoint}|{chunk_end}|{}\n", pe.batch_index, chunk.len()));
            next_endpoint = chunk_end;
            if !chunk.is_empty() {
                pool.targets.extend(chunk);
                return Err(Stop::refused("RSZ-ABSENT", "repair_timing -phases PARALLEL_EVAL: candidate generation (prepareTargets, generateCandidates) is not modelled".into()));
            }
        }
        Ok(pool)
    }

    /// `collectTargets`: the chunk's path-driver targets, each with its fanout.
    fn pe_collect_targets(&mut self, pe: &mut ParallelEval, begin_endpoint: usize, end_endpoint: usize, visited_pins: &mut HashSet<String>) -> Result<Vec<Target>, Stop> {
        let mut targets = self.pe_collect_crit_path_driver_pin_targets(pe, begin_endpoint, end_endpoint, visited_pins)?;
        let mut t = String::new();
        for target in &mut targets {
            // `targetFanout` was read with the target (the driver stage's wire edges out).
            t.push_str(&format!(
                "VYGP|target|{}|{}|ep={}|vep={}|rf={}|slack={}|fanout={}|load={}|path_index={}\n",
                pe.batch_index,
                target.driver_pin,
                target.endpoint_pin,
                target.violated_endpoint_pin,
                if target.driver_rf == 0 { '^' } else { 'v' },
                e9(target.slack),
                target.fanout,
                target.load_pin.as_deref().unwrap_or("-"),
                target.path_index
            ));
        }
        pe_trace(&t);
        Ok(targets)
    }

    /// `RepairTargetCollector::collectCritPathDriverPinTargets`: the violating endpoints, stably
    /// re-sorted by fuzzy slack, `[begin, end)` of them; each endpoint's worst path's drivers
    /// (`collectPathDriverTargets`), each taken once and only on an editable logic cell.
    fn pe_collect_crit_path_driver_pin_targets(&mut self, pe: &mut ParallelEval, begin_endpoint: usize, end_endpoint: usize, visited_pins: &mut HashSet<String>) -> Result<Vec<Target>, Stop> {
        let mut violated: Vec<(String, f32)> = self.collector_ends.clone();
        // `stable_sort(pathEndpointLess)`: the collector's order kept on fuzzily equal slacks.
        violated.sort_by(|a, b| if fuzzy::less(a.1, b.1) { std::cmp::Ordering::Less } else if fuzzy::less(b.1, a.1) { std::cmp::Ordering::Greater } else { std::cmp::Ordering::Equal });
        let end = end_endpoint.min(violated.len());
        let want: Vec<String> = violated.get(begin_endpoint..end).unwrap_or_default().iter().map(|(p, _)| p.clone()).collect();
        let edits = self.design.take_timer_edits();
        let snap = super::snapshot(self.ctx, self.design.as_design(), &want, &mut self.timer, edits)?;
        let dont_touch = self.design.net_info().dont_touch_insts.clone();
        let mut targets = Vec::new();
        for (pin, slack) in violated.get(begin_endpoint..end).unwrap_or_default() {
            // `findWorstSlackPath`: none, no targets.
            let Some(view) = snap.paths.get(pin) else { continue };
            for target in pe_collect_path_driver_targets(pe, view, *slack) {
                let editable = !dont_touch.contains(&target.inst) && self.pe_is_editable_logic_std_cell(&target.cell);
                if visited_pins.contains(&target.driver_pin) || !editable {
                    continue;
                }
                visited_pins.insert(target.driver_pin.clone());
                targets.push(target);
            }
        }
        Ok(targets)
    }

    /// `Resizer::isEditableLogicStdCell` (dont_touch tested by the caller): a logic standard cell
    /// with a liberty cell.
    fn pe_is_editable_logic_std_cell(&self, cell: &str) -> bool {
        self.ctx.sizing.masters.get(cell).is_some_and(|m| m.logic_std) && self.ctx.libs.link_cell(cell).is_some()
    }

    /// The start state as the reference's profiler records it (`metadata.csv`, the
    /// `checkpoint_initial` sample) and the lines so far, to `VYGES_RSZ_PE_TRACE`.
    fn pe_trace_start(&self, pe: &ParallelEval) {
        let Ok(path) = std::env::var("VYGES_RSZ_PE_TRACE") else { return };
        let seq: String = pe.move_sequence.iter().map(|m| format!("{} ", m.name())).collect();
        let mut s = format!(
            "PE|start|sequence|{seq}\nPE|start|generators|{}\nPE|start|graph_vertices|{}\nPE|start|reference_period|{:.6e}\nPE|start|checkpoint_wns|{:.9e}\nPE|start|checkpoint_tns|{:.9e}\nPE|start|max_end_repairs|{}\n",
            pe.generators.len(),
            pe.graph_vertices,
            pe.reference_period,
            pe.checkpoint_wns,
            pe.checkpoint_tns,
            pe.max_end_repairs
        );
        for l in &self.out.lines {
            s.push_str(&format!("PE|line|{l}\n"));
        }
        for l in self.out.trace.iter().filter(|l| l.starts_with("[DEBUG")) {
            s.push_str(&format!("PE|debug|{l}\n"));
        }
        let _ = std::fs::write(path, s);
    }
}

/// `effectiveBatchCap` before the tail and the small-batch retry (S5): `adaptiveBatchSize`.
fn pe_effective_batch_cap(pe: &ParallelEval, failing_endpoint_count: usize) -> i32 {
    let cfg = pe.config.parallel_eval;
    adaptive_batch_size(failing_endpoint_count, cfg.min_batch_cap, cfg.max_batch_cap)
}

/// `finishAtMaxArea`: `-max_utilization` is refused before the policy runs, so the design is
/// always within it.
fn pe_finish_at_max_area() -> bool {
    false
}

/// `collectPathDriverTargets`: the path's drivers from its start (`collectExpandedPathDriver-
/// Targets`), then each latch D segment's (`visitLatchFaninSegments`), all for the endpoint. A
/// load pin on the path is a critical path load pin; a top-level port driver is no target.
fn pe_collect_path_driver_targets(pe: &mut ParallelEval, view: &PathView, path_slack: f32) -> Vec<Target> {
    let violated = view.stages.last().map(|s| s.pin.clone()).unwrap_or_default();
    let mut targets = Vec::new();
    for seg in std::iter::once(view).chain(view.latch_segments.iter()) {
        let endpoint_pin = seg.stages.last().map(|s| s.pin.clone()).unwrap_or_default();
        for (index, stage) in seg.stages.iter().enumerate().skip(seg.start) {
            if !stage.is_driver {
                pe.crit_path_load_pins.insert(stage.pin.clone());
                continue;
            }
            if stage.top_port {
                continue;
            }
            targets.push(Target {
                driver_pin: stage.pin.clone(),
                endpoint_pin: endpoint_pin.clone(),
                violated_endpoint_pin: violated.clone(),
                driver_rf: stage.rf,
                path_index: index,
                load_pin: seg.stages.get(index + 1).map(|s| s.pin.clone()),
                slack: path_slack,
                fanout: stage.fanout,
                scene: seg.scene,
                inst: stage.inst.clone().unwrap_or_default(),
                cell: stage.cell.clone().unwrap_or_default(),
            });
        }
    }
    targets
}

/// The VYGP lines, appended to `VYGES_RSZ_PE_TRACE` (after the start state).
fn pe_trace(lines: &str) {
    let Some(path) = std::env::var_os("VYGES_RSZ_PE_TRACE") else { return };
    use std::io::Write;
    if let Ok(mut f) = std::fs::OpenOptions::new().append(true).create(true).open(path) {
        let _ = f.write_all(lines.as_bytes());
    }
}

/// `buildMoveGenerators(move_sequence_)`.
fn pe_build_move_generators(pe: &mut ParallelEval) {
    pe.generators = pe.move_sequence.iter().copied().filter(|&m| has_generator(m)).collect();
}

/// `resetRunState`.
fn pe_reset_run_state(pe: &mut ParallelEval) {
    pe.committed_moves = 0;
    pe.batch_index = 0;
    pe.tentative_moves = 0;
    pe.checkpoint_misses = 0;
    pe.small_batch_retry_started = false;
    pe.max_utilization_warned = false;
    pe.batch_journal_open = false;
    pe.rebuffer_targets_in_batch = 0;
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_sta::sdc::{Clock, Sdc};

    fn clock(name: &str, period: f32, source: &str) -> Clock {
        Clock::new(name, period, source, false)
    }

    #[test]
    fn a_batch_takes_the_failing_count_clamped_and_a_chunk_twice_the_cap_at_least_256() {
        // Upstream adaptiveBatchSize: std::clamp(failing endpoints, min_batch_cap 64, max_batch_cap
        // 512). evaluationEndpointLimit: max(cap * eval_endpoint_factor 2, eval_min_endpoints 256).
        let c = ParallelEvalConfig::default();
        assert_eq!(adaptive_batch_size(0, c.min_batch_cap, c.max_batch_cap), 64);
        assert_eq!(adaptive_batch_size(28, c.min_batch_cap, c.max_batch_cap), 64);
        assert_eq!(adaptive_batch_size(300, c.min_batch_cap, c.max_batch_cap), 300);
        assert_eq!(adaptive_batch_size(5000, c.min_batch_cap, c.max_batch_cap), 512);
        assert_eq!(evaluation_endpoint_limit(64, c.eval_endpoint_factor, c.eval_min_endpoints), 256);
        assert_eq!(evaluation_endpoint_limit(300, c.eval_endpoint_factor, c.eval_min_endpoints), 600);
    }

    #[test]
    fn the_trace_prints_slacks_as_fmt_does_with_a_signed_two_digit_exponent() {
        // fmt's {:.9e}: nine fraction digits, the exponent signed and at least two digits.
        assert_eq!(e9(-(2.0f32).powi(-33)), "-1.164153218e-10");
        assert_eq!(e9(-0.5), "-5.000000000e-01");
        assert_eq!(e9(1.5), "1.500000000e+00");
        assert_eq!(e9(0.0), "0.000000000e+00");
    }

    #[test]
    fn effort_sets_the_parallel_eval_knobs_as_for_parallel_eval_does() {
        // Upstream forParallelEval: side losses, opposite transition and buffer-tree side loads on;
        // size-up jump bounded at 2.5; one-buffer trees in, no upper gate, scale 0.40. Low: no WNS
        // tail. Medium: stall at 12, progress 1% TNS / 0.08% period, tail 16 endpoints, 100% work,
        // 15 s floor. High: the defaults.
        let high = for_parallel_eval(Effort::High);
        assert!(high.fanin_side_loss && high.rebuffer_mt_side_loads && high.dual_rf_eval && !high.dual_rf_drive_guard && high.allow_rebuffer_mt_single_buffer);
        assert_eq!((high.size_up_mt_max_drive_ratio, high.rebuffer_mt_max_score, high.rebuffer_mt_score_scale), (2.5, 0.0, 0.40));
        assert_eq!(high.parallel_eval, ParallelEvalConfig::default());
        assert_eq!((high.parallel_eval.selector.min_score_ratio, high.parallel_eval.selector.wns_reserve_pct), (0.05, 10));
        assert_eq!(for_parallel_eval(Effort::Low).parallel_eval.wns_tail_endpoints, 0);
        let m = for_parallel_eval(Effort::Medium).parallel_eval;
        assert_eq!((m.stall_max_batches, m.progress_tns_pct, m.progress_wns_period_pct, m.wns_tail_max_endpoints, m.wns_tail_work_pct, m.wns_tail_min_work), (12, 1.0, 0.08, 16, 100.0, 15.0));
        assert_eq!(m.wns_tail_endpoints, 8);
    }

    #[test]
    fn the_reference_period_is_the_shortest_real_clock_then_virtual_then_max_delay_then_wns() {
        // Upstream referencePeriod: a clock with a source pin sets the scale; a virtual clock only
        // when there is no such clock; a set_max_delay only with no clock at all; else |WNS|.
        let mut sdc = Sdc { clocks: Vec::new(), input_delays: Vec::new(), output_delays: Vec::new(), path_delays: Vec::new() };
        sdc.clocks = vec![clock("core", 2.6e-10, "clk"), clock("vclk", 1.0e-10, ""), clock("c2", 3.0e-10, "clk2")];
        assert_eq!(reference_period(&sdc, -1.0), 2.6e-10);
        sdc.clocks = vec![clock("vclk", 1.0e-10, ""), clock("v2", 0.9e-10, "")];
        assert_eq!(reference_period(&sdc, -1.0), 0.9e-10);
        sdc.clocks.clear();
        assert_eq!(reference_period(&sdc, -3.5e-11), 3.5e-11);
        let pd = |delay: f32, mm: usize| vyges_sta::sdc::PathDelay { from_pins: vec![], from_clock: false, to_pins: vec!["o".into()], to_clock: false, min_max: mm, ignore_clk_latency: false, break_path: true, delay, false_path: false, multicycle: None, thrus: vec![], group: None };
        sdc.path_delays = vec![pd(4.0e-10, vyges_sta::liberty::MAX), pd(1.0e-10, vyges_sta::liberty::MIN), pd(3.0e-10, vyges_sta::liberty::MAX)];
        assert_eq!(reference_period(&sdc, -1.0), 3.0e-10, "set_min_delay sets no scale");
        // A zero period is no scale.
        sdc.clocks = vec![clock("z", 0.0, "clk")];
        assert_eq!(reference_period(&sdc, -1.0), 3.0e-10);
    }

    #[test]
    fn a_move_without_an_mt_form_has_no_generator() {
        // Upstream makeParallelEvalGenerator / selectMoveSequence: no Mt form of Clone,
        // SizeUpMatch, Reroute, BufferToInverters; every other move has one.
        for m in [Move::Clone, Move::SizeUpMatch, Move::Reroute, Move::BufferToInverters] {
            assert!(!has_mt_form(m) && !has_generator(m), "{m:?}");
        }
        for m in [Move::SizeUp, Move::Buffer, Move::VtSwap, Move::SwapPins, Move::SplitLoad, Move::SizeDownFanout, Move::Unbuffer] {
            assert!(has_mt_form(m) && has_generator(m), "{m:?}");
        }
    }
}
