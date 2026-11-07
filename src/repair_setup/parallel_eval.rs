//! `SetupParallelEvalPolicy` (`PARALLEL_EVAL`, experimental upstream): batches of moves chosen
//! across many targets at once, each target's candidates estimated and scored, the batch
//! committed under one timing update and judged against a checkpoint.
//!
//! Ported stage group by stage group, in the reference's order. Each stage is its own function
//! named after the reference's, and `start` / `iterate` do nothing but call them, so a trace that
//! diverges from the reference's points at one stage. Modelled so far: `start` (S1), the
//! target collection of a batch (S2: `collectTargetEndpoints`, `collectTargets`) and the targets'
//! preparation (S3a: `prewarmTargets`, `prepareTargets`). Candidate generation onward
//! (`generateCandidates`) is refused.

use std::collections::{HashMap, HashSet};

use vyges_sta::liberty::MAX;

use vyges_sta::fuzzy;

use super::{PathView, Repair, Stop};
use crate::delay_estimator::{ContextOptions, EstimatorContext, FaninSideLoadOptions, PathPin, Reader};
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
    /// The path the driver was found on (its segment's pins and tags), with its start index
    /// (`endpoint_path`, `PathExpanded::startIndex`).
    pub(super) path: Vec<super::ChainKey>,
    pub(super) path_start: usize,
    /// That segment's latch D path expanded on its own (`latchDataPath`): the next segment.
    pub(super) latch_data_path: Option<(Vec<super::ChainKey>, usize)>,
    /// `estimator_context` (`prepareEstimatorContext`).
    pub estimator_context: Option<EstimatorContext>,
    /// `sequential_timing_estimator` is set (`prepareSequentialEstimator`); its contents are not
    /// modelled yet.
    pub sequential_estimator: bool,
    /// Why the context build left the target without one (the trace's `why=`).
    pub prepare_why: String,
    /// `unbuffer_gain` (`UnbufferMtGenerator::prepareInputs`): the removal's estimated gain, when
    /// the buffer is eligible.
    pub unbuffer_gain: Option<f32>,
    /// `swap_pins_input` (`SwapPinsMtGenerator::prepareInputs`).
    pub swap_pins_input: Option<SwapPinsInput>,
    /// `input_cap_budgets` (`prepareInputCapBudgets`, SizeUp's and VtSwap's prepareInputs).
    pub input_cap_budgets: Option<Vec<InputCapBudget>>,
    /// `split_load_input` (`SplitLoadMtGenerator::prepareInputs`).
    pub split_load_input: Option<SplitLoadInput>,
}

/// `SplitLoadMtInput`: the loads a new buffer takes, the cell it is sized to, and the split
/// model — the moved loads' capacitance and slack budgets, and the wire capacitance each net
/// gains or loses.
#[derive(Debug, Clone, PartialEq)]
pub struct SplitLoadInput {
    pub drvr_net: String,
    pub drvr_loc: (i32, i32),
    pub load_pins: Vec<String>,
    pub buffer_cell: String,
    pub resized_cell: String,
    pub moved_pin_cap: f32,
    pub target_wire_cap_saved: f32,
    pub moved_wire_cap: f32,
    pub target_load_delta: f32,
    pub moved_loads: Vec<crate::delay_estimator::SideLoad>,
}

/// `Resizer::InputCapBudget`: an input port of the current cell, its capacitance, and the least
/// `maxCapIncrease` of its net's drivers (infinite without a limit).
#[derive(Debug, Clone, PartialEq)]
pub struct InputCapBudget {
    pub port: String,
    pub current_cap: f32,
    pub max_cap_increase: f32,
}

/// `SwapPinsMtInput`: the input the path enters through, its delay, and the equivalent inputs
/// that can take the critical signal.
#[derive(Debug, Clone, PartialEq)]
pub struct SwapPinsInput {
    pub input_port: String,
    pub input_delay: f32,
    pub equivalents: Vec<SwapPinsEquivalent>,
}

/// `SwapPinsMtInput::Equivalent`: the port and its delay; with the side model, the pin and the
/// slack of the signal it displaces (none and 0 otherwise).
#[derive(Debug, Clone, PartialEq)]
pub struct SwapPinsEquivalent {
    pub port: String,
    pub delay: f32,
    pub pin: Option<String>,
    pub slack: f32,
}

/// `PrepareCacheMask`: what `prepareTarget` builds (bit 1 is unused upstream).
pub const ESTIMATOR_CONTEXT_CACHE: u8 = 1 << 0;
pub const SEQUENTIAL_ESTIMATOR_CACHE: u8 = 1 << 2;

/// `MoveGenerator::prepareRequirements` of each Mt generator: SizeUp and VtSwap read the context
/// and the flip-flop checks, Unbuffer nothing, the others the context.
fn prepare_requirements(m: Move) -> u8 {
    match m {
        Move::SizeUp | Move::VtSwap => ESTIMATOR_CONTEXT_CACHE | SEQUENTIAL_ESTIMATOR_CACHE,
        Move::Unbuffer => 0,
        _ => ESTIMATOR_CONTEXT_CACHE,
    }
}

/// `accumulatePrepareRequirements`: the union over the run's generators.
fn accumulate_prepare_requirements(generators: &[Move]) -> u8 {
    generators.iter().fold(0, |mask, &m| mask | prepare_requirements(m))
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

impl ParallelEval {
    /// The policy's members as constructed, before `start` fills them.
    pub fn new(config: MtConfig, marker: char) -> ParallelEval {
        ParallelEval {
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
        }
    }
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
        let mut pe = ParallelEval::new(config, marker);
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
        self.pe_scan_crit_path_load_pins(pe)?;
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

    /// `collectViolatingEndpoints`' crit-path scan: the set is cleared at every scan; with
    /// `record_crit_path_load_pins_` (fanin side loss on) every violating endpoint's worst path adds
    /// its load pins from the start index — side loads off this chunk's paths but on another
    /// violating endpoint's worst path take the full weight. Target collection adds the paths it
    /// expands on top, latch segments included.
    fn pe_scan_crit_path_load_pins(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        pe.crit_path_load_pins.clear();
        if !pe.record_crit_path_load_pins {
            return Ok(());
        }
        let want: Vec<String> = self.collector_ends.iter().map(|(p, _)| p.clone()).collect();
        let edits = self.design.take_timer_edits();
        let snap = super::snapshot(self.ctx, self.design.as_design(), &want, &mut self.timer, edits)?;
        for pin in &want {
            let Some(view) = snap.paths.get(pin) else { continue };
            pe.crit_path_load_pins.extend(pe_path_load_pins(view));
        }
        pe_trace(&format!("VYGP|ev|scan_crit_path_load_pins|metric=|move=|count={}\n", pe.crit_path_load_pins.len()));
        Ok(())
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
                let mut chunk = chunk;
                pe_prewarm_targets(&chunk);
                self.pe_prepare_targets(pe, &mut chunk)?;
                self.pe_generate_candidates(pe, &mut chunk)?;
                pool.targets.extend(chunk);
                return Err(Stop::refused("RSZ-ABSENT", "repair_timing -phases PARALLEL_EVAL: candidate generation (generateCandidates) is not modelled".into()));
            }
        }
        Ok(pool)
    }

    /// `OptimizationPolicy::prepareTargets`: the mask of what the run's generators read, then
    /// `prepareTarget` on each target in chunk order — all on the timer as the collection left
    /// it (nothing is edited in between).
    fn pe_prepare_targets(&mut self, pe: &mut ParallelEval, targets: &mut [Target]) -> Result<(), Stop> {
        let mask = accumulate_prepare_requirements(&pe.generators);
        let edits = self.design.take_timer_edits();
        let ctx = self.ctx;
        let (equiv, dont_use) = (ctx.sizing.equiv, ctx.sizing.dont_use);
        let pin_id = self.design.net_info().pin_id.clone();
        let cfg = pe.config;
        let levels = ctx.policy.delay_estimation_levels;
        let slew_bias = ctx.policy.sta_slew_bias;
        let crit = &pe.crit_path_load_pins;
        let design = self.design.as_design();
        let mut trace = String::new();
        super::timed_all(ctx, design, &mut self.timer, edits, |gs, ss, cs| {
            // `worstSlack(maxAnalysisMode)`, read live for each context.
            let ends = super::setup_ends(ctx, gs, ss, cs)?;
            let (wns, _) = crate::repair_timing::worst_slack(&ends);
            for target in targets.iter_mut() {
                pe_prepare_target(target, mask, gs, ss, cs, design, ctx, &PrepareInputs { equiv, dont_use, pin_id: &pin_id, crit, cfg: &cfg, levels, slew_bias, wns });
                trace.push_str(&pe_prepared_lines(target, mask, ctx.libs));
            }
            Ok(())
        })?;
        pe_trace(&trace);
        Ok(())
    }

    /// `generateCandidates`, its main-thread half: `planGeneration` on each target in chunk order
    /// (the skip rules, then each generator's `generatorAllowed`, `isApplicable` and
    /// `prepareInputs`, all on the timer as preparation left it). Generation itself is refused
    /// after; so is a run with a generator whose planning is not modelled yet.
    fn pe_generate_candidates(&mut self, pe: &mut ParallelEval, targets: &mut [Target]) -> Result<(), Stop> {
        let edits = self.design.take_timer_edits();
        let ctx = self.ctx;
        let facts: Vec<PlanFacts> = targets.iter().map(|t| self.pe_plan_facts(t)).collect();
        let design = self.design.as_design();
        let mut trace = String::new();
        super::timed_all(ctx, design, &mut self.timer, edits, |gs, ss, _cs| {
            for (target, facts) in targets.iter_mut().zip(&facts) {
                pe_plan_generation(pe, target, facts, gs, ss, design, ctx, &mut trace).map_err(|e| Stop::refused("RSZ-ABSENT", e))?;
            }
            Ok(())
        })?;
        pe_trace(&trace);
        if let Some(m) = pe.generators.iter().find(|&&m| !plan_modelled(m)) {
            return Err(Stop::refused("RSZ-ABSENT", format!("repair_timing -phases PARALLEL_EVAL: planGeneration for {} (isApplicable, prepareInputs) is not modelled", m.name())));
        }
        Ok(())
    }

    /// What planning reads off the database and the committer rather than the timer — nothing
    /// edits either while a chunk is planned, so it is read up front.
    fn pe_plan_facts(&self, target: &Target) -> PlanFacts {
        let inst = self.design.inst_id(&target.inst);
        let buffer_ports = self.ctx.libs.link_cell(&target.cell).filter(|c| c.is_buffer()).and_then(|c| c.buffer_ports()).map(|(i, o)| (i.name.clone(), o.name.clone()));
        PlanFacts {
            driver_is_buffer: buffer_ports.is_some(),
            driver_is_inverter: self.ctx.libs.link_cell(&target.cell).is_some_and(|c| c.is_inverter()),
            dont_touch: self.design.net_info().dont_touch_insts.contains(&target.inst),
            pins_swapped: self.committer.has_moves(Move::SwapPins, inst),
            fixed: self.design.net_info().fixed_insts.contains(&target.inst),
            ok_to_buffer_net: self.ok_to_buffer_net(&target.driver_pin),
            logic_std: self.ctx.sizing.masters.get(&target.cell).is_some_and(|m| m.logic_std),
            vt_swappable: self.ctx.vt_category_count >= 2 && self.ctx.sizing.masters.contains_key(&target.cell) && self.ctx.sizing.vt_equiv_cells(&target.cell, self.ctx.vt_category_count).len() > 1,
            driver_liberty: target.driver_pin.rsplit_once('/').is_some_and(|(_, port)| self.ctx.libs.link_cell(&target.cell).is_some_and(|c| c.port(port).is_some())),
            // UnbufferMtGenerator::isApplicable: the moves that block a serial removal (a resize
            // does not).
            unbuffer_blocked: [Move::SwapPins, Move::Clone, Move::SplitLoad, Move::Buffer].into_iter().any(|m| self.committer.has_moves(m, inst)),
            // `canRemoveBuffer(buffer, true)`.
            can_remove_buffer: buffer_ports.is_some_and(|(i, o)| self.ctx.sizing.masters.get(&target.cell).is_some_and(|m| m.logic_std) && self.design.can_remove_buffer(&target.inst, &i, &o)),
        }
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

/// The load pins of an expanded path from its start index: the pins its vertices are not drivers
/// of.
fn pe_path_load_pins(view: &PathView) -> impl Iterator<Item = String> + '_ {
    view.stages.iter().skip(view.start).filter(|s| !s.is_driver).map(|s| s.pin.clone())
}

/// `collectPathDriverTargets`: the path's drivers from its start (`collectExpandedPathDriver-
/// Targets`), then each latch D segment's (`visitLatchFaninSegments`), all for the endpoint. A
/// load pin on the path is a critical path load pin; a top-level port driver is no target.
fn pe_collect_path_driver_targets(pe: &mut ParallelEval, view: &PathView, path_slack: f32) -> Vec<Target> {
    let violated = view.stages.last().map(|s| s.pin.clone()).unwrap_or_default();
    let mut targets = Vec::new();
    let segments: Vec<&PathView> = std::iter::once(view).chain(view.latch_segments.iter()).collect();
    for (k, seg) in segments.iter().enumerate() {
        let endpoint_pin = seg.stages.last().map(|s| s.pin.clone()).unwrap_or_default();
        // The segment's latch D path is the segment walked after it.
        let latch_data_path = segments.get(k + 1).map(|d| (d.chain.clone(), d.start));
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
                path: seg.chain.clone(),
                path_start: seg.start,
                latch_data_path: latch_data_path.clone(),
                estimator_context: None,
                sequential_estimator: false,
                prepare_why: String::new(),
                unbuffer_gain: None,
                swap_pins_input: None,
                input_cap_budgets: None,
                split_load_input: None,
            });
        }
    }
    targets
}

/// See [`Repair::pe_plan_facts`].
struct PlanFacts {
    /// The driver's liberty cell is a buffer, an inverter.
    driver_is_buffer: bool,
    driver_is_inverter: bool,
    /// `Resizer::dontTouch(driver instance)`.
    dont_touch: bool,
    /// `committer_.hasMoves(kSwapPins, driver instance)`.
    pins_swapped: bool,
    /// `network->libertyPort(driver_pin)` exists (a liberty cell with that port).
    driver_liberty: bool,
    /// `Resizer::isFixed(inst)`.
    fixed: bool,
    /// `Resizer::okToBufferNet(driver_pin)`.
    ok_to_buffer_net: bool,
    /// `isLogicStdCell(inst)`.
    logic_std: bool,
    /// `staToDb(cell)` is a master and `getVTEquivCells(cell)` has more than the cell, with at
    /// least two VT categories.
    vt_swappable: bool,
    unbuffer_blocked: bool,
    can_remove_buffer: bool,
}

/// The generators whose `isApplicable` and `prepareInputs` are modelled: the others are left out
/// of the plan, and the run is refused after the chunk is planned.
fn plan_modelled(m: Move) -> bool {
    matches!(m, Move::Unbuffer | Move::SwapPins | Move::SizeUp | Move::VtSwap | Move::SplitLoad)
}

/// `planGeneration`: per generator in `move_sequence_` order, `generatorAllowed`, `isApplicable`
/// (else `skip_not_applicable`), then `prepareInputs`; the plan's generators in that order. The
/// tabu checks come first: both are empty until a batch is undone or missed, and only the first
/// chunk of batch 0 is planned here.
#[allow(clippy::too_many_arguments)]
fn pe_plan_generation(pe: &mut ParallelEval, target: &mut Target, facts: &PlanFacts, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], design: &dyn super::Design, ctx: &super::Ctx<'_>, trace: &mut String) -> Result<(), String> {
    let pin = target.driver_pin.clone();
    let b = pe.batch_index;
    let mut planned = Vec::new();
    for m in pe.generators.clone() {
        if !plan_modelled(m) {
            continue;
        }
        if !pe_generator_allowed(pe, m) {
            trace.push_str(&format!("VYGP|deny|{b}|{pin}|{}|rebuffer_in_batch={}\n", m.name(), pe.rebuffer_targets_in_batch));
            continue;
        }
        if !pe_is_applicable(m, target, facts) {
            trace.push_str(&format!("VYGP|na|{b}|{pin}|{}\n", m.name()));
            continue;
        }
        if m == Move::Buffer {
            pe.rebuffer_targets_in_batch += 1;
        }
        pe_prepare_inputs(m, target, facts, gs, ss, design, ctx, &pe.config, &pe.crit_path_load_pins)?;
        trace.push_str(&format!("VYGP|inputs|{b}|{pin}|{}|{}\n", m.name(), pe_inputs_line(m, target)));
        planned.push(m.name());
    }
    if pe.generators.iter().all(|&m| plan_modelled(m)) {
        trace.push_str(&format!("VYGP|plan|{b}|{pin}|gens={}\n", planned.join(",")));
    }
    Ok(())
}

/// `generatorAllowed`: Rebuffer within `rebuffer_max_targets` per batch; Unbuffer only when the
/// sequence asks for it or the tail allows its own removal generator (single-move batches after
/// the greedy stall — never in batch 0, the tail is reset with the run); the rest always.
fn pe_generator_allowed(pe: &ParallelEval, m: Move) -> bool {
    match m {
        Move::Buffer => pe.rebuffer_targets_in_batch < pe.config.parallel_eval.rebuffer_max_targets,
        Move::Unbuffer => pe.unbuffer_in_sequence || pe_tail_unbuffer_allowed(pe),
        _ => true,
    }
}

/// `ParallelEvalState::unbufferAllowed`: `active && batch_cap == 1 && greedy_stalled`. The tail
/// is not modelled: it is inactive from `resetRunState` until a later batch stalls.
fn pe_tail_unbuffer_allowed(_pe: &ParallelEval) -> bool {
    false
}

/// `MoveGenerator::isApplicable` and its overrides. Every target here is in the path-driver
/// view, which the base check asks for.
fn pe_is_applicable(m: Move, target: &Target, facts: &PlanFacts) -> bool {
    match m {
        // UnbufferMtGenerator: the buffer and the stage that drives it on the path; a buffer's
        // liberty port; no move committed on it that blocks a removal.
        Move::Unbuffer => target.path_index >= 2 && facts.driver_is_buffer && !target.inst.is_empty() && !facts.unbuffer_blocked,
        // SwapPinsMtGenerator: the path enters the gate through an input, the gate has inputs to
        // swap (not a buffer or inverter), and it may change and has not been swapped.
        // SizeUpMtGenerator: a changeable, unfixed instance with an estimator context.
        Move::SizeUp => !target.inst.is_empty() && !facts.dont_touch && !facts.fixed && target.estimator_context.is_some(),
        // VtSwapMtGenerator (path-driver or instance view): a context, a changeable logic
        // standard cell with a liberty cell, two VT categories, a master, and a VT equivalent.
        Move::VtSwap => target.estimator_context.is_some() && !target.inst.is_empty() && !target.cell.is_empty() && !facts.dont_touch && facts.logic_std && facts.vt_swappable,
        // SplitLoadMtGenerator: a context, more than kSplitLoadMinFanout (the legacy split's 8)
        // loads, and a net that accepts a buffer.
        Move::SplitLoad => target.estimator_context.is_some() && target.fanout > crate::clone::CLONE_MIN_FANOUT && facts.ok_to_buffer_net,
        Move::SwapPins => target.path_index > 0 && facts.driver_liberty && !facts.driver_is_buffer && !facts.driver_is_inverter && !target.inst.is_empty() && !facts.dont_touch && !facts.pins_swapped,
        _ => true,
    }
}

/// `MoveGenerator::prepareInputs`: the generator's live timer reads, stored on the target.
#[allow(clippy::too_many_arguments)]
fn pe_prepare_inputs(m: Move, target: &mut Target, facts: &PlanFacts, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], design: &dyn super::Design, ctx: &super::Ctx<'_>, cfg: &MtConfig, crit: &HashSet<String>) -> Result<(), String> {
    match m {
        Move::Unbuffer => unbuffer_mt_prepare_inputs(target, facts, gs, ss, design, ctx),
        Move::SwapPins => swap_pins_mt_prepare_inputs(target, gs, ss, design, ctx, cfg)?,
        // SizeUpMt also builds its replacement options here (a cache generate reads; it decides
        // nothing yet).
        Move::SizeUp | Move::VtSwap => prepare_input_cap_budgets(target, gs, design, ctx),
        Move::SplitLoad => split_load_mt_prepare_inputs(target, gs, ss, design, ctx, crit, cfg)?,
        _ => {}
    }
    Ok(())
}

/// `SplitLoadMtGenerator::prepareInputs`: the ranked fanouts (`collectRankedFanoutSlacks`), the
/// less critical half without top-level ports in that order (`chooseSplitLoads`), the lowest-drive
/// buffer and the driver's net; then the split model at the target arc's scene
/// (`buildLoadSplitModel`), the buffer sized for the moved load (`findTargetCell`), and the
/// target's load change.
fn split_load_mt_prepare_inputs(target: &mut Target, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], design: &dyn super::Design, ctx: &super::Ctx<'_>, crit: &HashSet<String>, cfg: &MtConfig) -> Result<(), String> {
    let scene = target.scene;
    let g = &gs[scene];
    let Some(drvr) = g.vertices.iter().position(|x| x.name == target.driver_pin && x.is_driver) else { return Ok(()) };
    let fanout_slacks = super::fanout_slacks(gs, scene, drvr, target.driver_rf, true, ss);
    if fanout_slacks.is_empty() {
        return Ok(());
    }
    let load_pins: Vec<String> = fanout_slacks[..fanout_slacks.len() / 2].iter().filter(|f| !f.top_port).map(|f| f.pin.clone()).collect();
    if load_pins.is_empty() {
        return Ok(());
    }
    let buffer_cell = ctx.lowest_buffer.to_string();
    let Some(net) = g.vertex_net[drvr] else { return Ok(()) };
    if buffer_cell.is_empty() {
        return Ok(());
    }
    let Some(context) = target.estimator_context.as_ref() else { return Ok(()) };
    let arc_scene = context.target().arc.scene;
    let mut input = SplitLoadInput {
        drvr_net: g.netlist.nets[net].name.clone(),
        drvr_loc: design.pin_location(&target.driver_pin),
        load_pins,
        buffer_cell,
        resized_cell: String::new(),
        moved_pin_cap: 0.0,
        target_wire_cap_saved: 0.0,
        moved_wire_cap: 0.0,
        target_load_delta: 0.0,
        moved_loads: Vec::new(),
    };
    let weights = crate::delay_estimator::FaninSideLoadOptions { critical_load_pins: crit, off_path_weight: cfg.fanin_side_offpath_weight };
    build_load_split_model(&mut input, g, gs, ss, design, ctx, arc_scene, net, &weights);
    // apply() resizes the new buffer to its load's target slew: the estimate scores that cell.
    input.resized_cell = ctx.sizing.find_target_cell(&input.buffer_cell, input.moved_pin_cap + input.moved_wire_cap, false).map_err(|e| format!("{e:?}"))?;
    let Some(scene_buffer) = ctx.libs.scene_cell(arc_scene, &input.resized_cell).or_else(|| ctx.libs.link_cell(&input.resized_cell)) else { return Ok(()) };
    let Some((buffer_input, _)) = scene_buffer.buffer_ports() else { return Ok(()) };
    input.target_load_delta = super::port_capacitance(scene_buffer, &buffer_input.name).unwrap_or(0.0) - input.moved_pin_cap - input.target_wire_cap_saved;
    target.split_load_input = Some(input);
    Ok(())
}

/// `buildLoadSplitModel`: over the driver net's load pins in pin-id order (a PinSet), the
/// half-perimeters of all, kept and moved pins with the driver; each moved pin's capacitance at
/// the arc's scene and, when constrained, its slack and weight; the wire capacitance the target
/// net saves and the new net gets.
#[allow(clippy::too_many_arguments)]
fn build_load_split_model(input: &mut SplitLoadInput, g: &vyges_sta::graph::Graph<'_>, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], design: &dyn super::Design, ctx: &super::Ctx<'_>, arc_scene: usize, net: usize, weights: &crate::delay_estimator::FaninSideLoadOptions<'_>) {
    let cap_per_meter = (ctx.wire_rc.h_cap + ctx.wire_rc.v_cap) / 2.0;
    let moved: HashSet<&str> = input.load_pins.iter().map(String::as_str).collect();
    let (mut all_box, mut kept_box, mut moved_box) = (BoundingBox::default(), BoundingBox::default(), BoundingBox::default());
    for b in [&mut all_box, &mut kept_box, &mut moved_box] {
        b.add(input.drvr_loc);
    }
    let nl = &g.netlist;
    let pin_id = &design.net_info().pin_id;
    let mut pins: Vec<String> = nl.nets[net].pins.iter().map(|c| nl.pin_name(c)).collect();
    pins.sort_by_key(|p| pin_id.get(p).copied().unwrap_or(u64::MAX));
    for pin in &pins {
        let Some(v) = g.vertices.iter().position(|x| x.name == *pin && !x.is_driver) else { continue };
        if !pe_is_load(g, v) {
            continue;
        }
        let location = design.pin_location(pin);
        all_box.add(location);
        if !moved.contains(pin.as_str()) {
            kept_box.add(location);
            continue;
        }
        moved_box.add(location);
        let vx = &g.vertices[v];
        if let (Some(cell), Some(port)) = (vx.cell.as_deref(), vx.port.as_deref()) {
            if let Some(c) = ctx.libs.scene_cell(arc_scene, cell).or_else(|| ctx.libs.link_cell(cell)) {
                input.moved_pin_cap += super::port_capacitance(c, port).unwrap_or(0.0);
            }
        }
        // The moved load keeps its slack as the budget for the new driver's extra delay.
        let slack = gs[0].vertices.iter().position(|x| x.name == *pin && !x.is_driver).map_or(super::INF, |v0| super::slack_over_scenes(gs, ss, v0, MAX, None));
        if crate::delay_estimator::delay_inf(slack) {
            continue;
        }
        input.moved_loads.push(crate::delay_estimator::SideLoad { pin: pin.clone(), slack, weight: crate::delay_estimator::side_load_weight(pin, weights) });
    }
    let meters_per_dbu = 1.0 / (f64::from(ctx.dbu) * 1e6);
    input.target_wire_cap_saved = (cap_per_meter * meters_per_dbu * f64::from((all_box.half_perimeter() - kept_box.half_perimeter()).max(0))) as f32;
    input.moved_wire_cap = (cap_per_meter * meters_per_dbu * f64::from(moved_box.half_perimeter())) as f32;
}

/// `Network::isLoad`: an instance input (or bidirectional) pin, or a top-level output port — the
/// graph's load vertices (a flat netlist has no hierarchical pins).
fn pe_is_load(g: &vyges_sta::graph::Graph<'_>, v: usize) -> bool {
    let vx = &g.vertices[v];
    !vx.is_driver
}

/// The split model's `BoundingBox`: half-perimeter of the points added, 0 when empty.
#[derive(Debug, Clone, Copy, Default)]
struct BoundingBox {
    rect: Option<(i32, i32, i32, i32)>,
}

impl BoundingBox {
    fn add(&mut self, (x, y): (i32, i32)) {
        self.rect = Some(match self.rect {
            None => (x, y, x, y),
            Some((x0, y0, x1, y1)) => (x0.min(x), y0.min(y), x1.max(x), y1.max(y)),
        });
    }

    fn half_perimeter(&self) -> i32 {
        self.rect.map_or(0, |(x0, y0, x1, y1)| (x1 - x0) + (y1 - y0))
    }
}

/// `prepareInputCapBudgets`: `Resizer::inputCapBudgets(inst)` once per target (the second
/// cell-swap generator reuses it): per pin of the instance in pin order, an input with a liberty
/// port on a net — its capacitance (`inputPinCapacitance`) and the least `maxCapIncrease` over the
/// net's drivers (the capacitance check over every scene; a violating driver allows nothing, one
/// without a limit allows anything).
fn prepare_input_cap_budgets(target: &mut Target, gs: &[vyges_sta::graph::Graph<'_>], design: &dyn super::Design, ctx: &super::Ctx<'_>) {
    if target.input_cap_budgets.is_some() {
        return;
    }
    let mut budgets = Vec::new();
    let g = &gs[0];
    if let (Some(cell), Some(pins)) = (ctx.libs.link_cell(&target.cell), ctx.master_pins.get(&target.cell)) {
        let sc = crate::timing::Scenes::new(gs);
        let parasitics: Vec<&HashMap<String, vyges_sta::graph::NetParasitics>> = (0..gs.len()).map(|k| design.parasitics(k)).collect();
        let clocks: std::collections::BTreeSet<usize> = g.ideal_clock.iter().copied().collect();
        for port in pins {
            let Some(p) = cell.port(port).filter(|p| matches!(p.direction, vyges_sta::liberty::Direction::Input | vyges_sta::liberty::Direction::Bidirect)) else { continue };
            let pin = format!("{}/{port}", target.inst);
            let Some(u) = g.vertices.iter().position(|x| x.name == pin && !x.is_driver) else { continue };
            if g.vertex_net[u].is_none() {
                continue;
            }
            let mut budget = InputCapBudget { port: p.name.clone(), current_cap: super::input_pin_capacitance(cell, port), max_cap_increase: f32::INFINITY };
            for d in crate::timing::net_drivers(g, u) {
                let (cap, max_cap, slack, limited, _) = crate::timing::check_capacitance(&sc, d, &parasitics, &clocks, ctx.limits);
                budget.max_cap_increase = budget.max_cap_increase.min(super::max_cap_increase(cap, max_cap, slack, limited));
            }
            budgets.push(budget);
        }
    }
    target.input_cap_budgets = Some(budgets);
}

/// `pathInputPort`: the from-port of the arc the target's path takes into the driver, when it is
/// not an output.
fn pe_path_input_port(target: &Target, g: &vyges_sta::graph::Graph<'_>, search: &vyges_sta::search::Search<'_, '_>, ctx: &super::Ctx<'_>) -> Option<String> {
    let find = |pin: &str, is_driver: bool| g.vertices.iter().position(|x| x.name == pin && x.is_driver == is_driver);
    let chain = super::refind_chain(&target.path, find, &search.paths).ok()?;
    let prev = chain.get(target.path_index)?.1.prev?;
    let port = super::edge_arc_set(g, prev.edge)?.from.clone();
    let cell = ctx.libs.link_cell(&target.cell)?;
    cell.port(&port).filter(|p| !crate::swap_pins::is_output(p)).map(|_| port)
}

/// `SwapPinsMtGenerator::prepareInputs`: the path's input, its equivalent inputs (`equivCellPins`,
/// in port order), the arc delays at the gate's input slews and load (`swapPinDelays`; nothing
/// when the input has no arc), then each equivalent input that has a delay, is an input and is
/// neither the path's input nor the driver — with the side model on and a context built, the pin
/// it is on and that pin's slack when constrained.
fn swap_pins_mt_prepare_inputs(target: &mut Target, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], design: &dyn super::Design, ctx: &super::Ctx<'_>, cfg: &MtConfig) -> Result<(), String> {
    let scene = target.scene;
    let g = &gs[scene];
    let Some(input_port) = pe_path_input_port(target, g, &ss[scene], ctx) else { return Ok(()) };
    let Some(cell) = ctx.libs.link_cell(&target.cell) else { return Ok(()) };
    let Some((_, drvr_port)) = target.driver_pin.rsplit_once('/') else { return Ok(()) };
    let equiv_ports = crate::swap_pins::equiv_cell_pins(cell, &input_port)?;
    if equiv_ports.is_empty() {
        return Ok(());
    }
    let Some(drvr) = g.vertices.iter().position(|x| x.name == target.driver_pin && x.is_driver) else { return Ok(()) };
    let load_cap = g.load_cap(drvr, design.parasitics(scene));
    // annotateInputSlews(drvr, scene): the instance's input pins' timed slews.
    let prefix = format!("{}/", target.inst);
    let mut annotated: HashMap<String, [f32; 2]> = HashMap::new();
    for (u, ux) in g.vertices.iter().enumerate() {
        let Some(port) = ux.name.strip_prefix(&prefix) else { continue };
        if !ux.is_driver && cell.port(port).is_some_and(crate::swap_pins::is_input) {
            annotated.insert(port.to_string(), [g.slew[u][0][MAX], g.slew[u][1][MAX]]);
        }
    }
    let tgt = ctx.sizing.tgt_slews;
    let in_slew = |port: &str, rf: usize| annotated.get(port).map_or(tgt[rf], |s| s[rf]);
    let scene_cell = ctx.libs.scene_cell(scene, &target.cell).unwrap_or(cell);
    let (input_delay, port_delays) = crate::swap_pins::swap_pin_delays(cell, scene_cell, &input_port, drvr_port, load_cap, &in_slew);
    if input_delay == -super::INF {
        return Ok(());
    }
    let side_model = cfg.fanin_side_loss && target.estimator_context.is_some();
    let mut equivalents = Vec::new();
    for port in &equiv_ports {
        let Some(&delay) = port_delays.get(port) else { continue };
        if !cell.port(port).is_some_and(crate::swap_pins::is_input) || *port == input_port || port == drvr_port {
            continue;
        }
        let mut equivalent = SwapPinsEquivalent { port: port.clone(), delay, pin: None, slack: 0.0 };
        if side_model {
            let pin = format!("{prefix}{port}");
            // `Sta::slack(pinLoadVertex(pin), max)`: over every scene.
            if let Some(v0) = gs[0].vertices.iter().position(|x| x.name == pin && !x.is_driver) {
                let slack = super::slack_over_scenes(gs, ss, v0, MAX, None);
                if !crate::delay_estimator::delay_inf(slack) {
                    equivalent.pin = Some(pin);
                    equivalent.slack = slack;
                }
            }
        }
        equivalents.push(equivalent);
    }
    target.swap_pins_input = Some(SwapPinsInput { input_port, input_delay, equivalents });
    Ok(())
}

/// `UnbufferMtGenerator::prepareInputs`: `isEligible` (driver and path context, the fanout and
/// capacitance guards, the slack guard at the capacitance check's scene or else the path's,
/// `canRemoveBuffer`), then `estimateRemovalGain`.
fn unbuffer_mt_prepare_inputs(target: &mut Target, facts: &PlanFacts, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], design: &dyn super::Design, ctx: &super::Ctx<'_>) {
    let scene = target.scene;
    let g = &gs[scene];
    let find = |pin: &str, is_driver: bool| g.vertices.iter().position(|x| x.name == pin && x.is_driver == is_driver);
    let Ok(chain) = super::refind_chain(&target.path, find, &ss[scene].paths) else { return };
    let owned: Vec<(Option<String>, Option<String>, Option<String>)> = chain
        .iter()
        .map(|(v, p)| {
            let vx = &g.vertices[*v];
            let inst = match &vx.conn {
                vyges_sta::netlist::Conn::Inst(k, _) => Some(g.netlist.insts[*k].0.clone()),
                vyges_sta::netlist::Conn::Port(_) => None,
            };
            let in_port = p.prev.and_then(|prev| super::edge_arc_set(g, prev.edge)).map(|set| set.from.clone());
            (inst, vx.cell.clone(), in_port)
        })
        .collect();
    let stages: Vec<crate::unbuffer::PathStage<'_>> = chain
        .iter()
        .zip(&owned)
        .map(|((v, p), (inst, cell, in_port))| crate::unbuffer::PathStage { vertex: *v, rf: p.tag.rf, inst: inst.as_deref(), cell: cell.as_deref(), in_port: in_port.as_deref() })
        .collect();
    let index = target.path_index;
    let ideal: std::collections::BTreeSet<usize> = g.ideal_clock.iter().copied().collect();
    let locate = |pin: &str| design.pin_location(pin);
    target.unbuffer_gain = super::with_unbuffer_timer(ctx, gs, ss, design, scene, &ideal, |uctx, t| {
        let Some(crate::unbuffer::Verdict::Resolved(r)) = crate::unbuffer::probe(uctx, t, &stages, index, target.fanout, Some(scene)) else { return None };
        if r.fanout_reject.is_some() || r.cap_reject.is_some() || !r.slack_ok || !facts.can_remove_buffer {
            return None;
        }
        Some(crate::unbuffer::estimate_removal_gain(uctx, t, &stages, index, &r, &locate))
    });
}

/// The VYGP printer's `prepareInputs` summary for one generator.
fn pe_inputs_line(m: Move, target: &Target) -> String {
    match m {
        Move::Unbuffer => format!("gain={}", target.unbuffer_gain.map_or("-".into(), e9i)),
        Move::SizeUp | Move::VtSwap => match &target.input_cap_budgets {
            None => "budgets=-".into(),
            Some(b) => {
                let mut s = format!("budgets={}", b.len());
                for x in b {
                    s.push_str(&format!("|{}:{}:{}", x.port, e9i(x.current_cap), e9i(x.max_cap_increase)));
                }
                s
            }
        },
        Move::SplitLoad => match &target.split_load_input {
            None => "split=-".into(),
            Some(i) => {
                let mut s = format!(
                    "split={}|loc={},{}|loads={}|buf={}|resized={}|moved_pin_cap={}|wire_saved={}|moved_wire={}|delta={}|moved={}",
                    i.drvr_net, i.drvr_loc.0, i.drvr_loc.1, i.load_pins.len(), i.buffer_cell, i.resized_cell, e9i(i.moved_pin_cap), e9i(i.target_wire_cap_saved), e9i(i.moved_wire_cap), e9i(i.target_load_delta), i.moved_loads.len()
                );
                for l in &i.moved_loads {
                    s.push_str(&format!("|{}:{}:{}", l.pin, e9i(l.slack), e9i(l.weight)));
                }
                s
            }
        },
        Move::SwapPins => match &target.swap_pins_input {
            None => "swap=-".into(),
            Some(sp) => {
                let mut s = format!("swap={}|in_delay={}|equiv={}", sp.input_port, e9i(sp.input_delay), sp.equivalents.len());
                for e in &sp.equivalents {
                    s.push_str(&format!("|{}:{}:{}:{}", e.port, e9i(e.delay), e.pin.as_deref().unwrap_or("-"), e9i(e.slack)));
                }
                s
            }
        },
        _ => String::new(),
    }
}

/// `prewarmTargets`: the swappable and VT-equivalent cell caches; it decides nothing.
fn pe_prewarm_targets(_targets: &[Target]) {}

/// What `prepareTarget` reads besides the timer.
struct PrepareInputs<'a> {
    equiv: &'a crate::sizing::EquivCells,
    dont_use: &'a std::collections::BTreeSet<String>,
    pin_id: &'a std::collections::HashMap<String, u64>,
    crit: &'a HashSet<String>,
    cfg: &'a MtConfig,
    levels: i32,
    slew_bias: bool,
    /// `worstSlack(max)` now.
    wns: f32,
}

/// `prepareTarget(target, mask)`: the context, then the sequential estimator, as the mask asks.
#[allow(clippy::too_many_arguments)]
fn pe_prepare_target(target: &mut Target, mask: u8, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], cs: &[std::collections::BTreeSet<usize>], design: &dyn super::Design, ctx: &super::Ctx<'_>, inputs: &PrepareInputs<'_>) {
    if mask & ESTIMATOR_CONTEXT_CACHE != 0 {
        pe_prepare_estimator_context(target, gs, ss, cs, design, ctx, inputs);
    }
    if mask & SEQUENTIAL_ESTIMATOR_CACHE != 0 {
        pe_prepare_sequential_estimator(target, ctx);
    }
}

/// `prepareEstimatorContext` → `buildPolicyContext`: `DelayEstimator::buildContext` with the
/// latch fanin, the fanin side loads (critical load pins weigh 1) and the opposite transition as
/// the config has them; then, with any side model on, the side-path floor at the worst slack.
#[allow(clippy::too_many_arguments)]
fn pe_prepare_estimator_context(target: &mut Target, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], cs: &[std::collections::BTreeSet<usize>], design: &dyn super::Design, ctx: &super::Ctx<'_>, inputs: &PrepareInputs<'_>) {
    if target.estimator_context.is_some() {
        return;
    }
    target.prepare_why = String::new();
    let scene = target.scene;
    let (g, search) = (&gs[scene], &ss[scene]);
    let find = |pin: &str, is_driver: bool| g.vertices.iter().position(|x| x.name == pin && x.is_driver == is_driver);
    let pins = |keys: &[super::ChainKey]| -> Option<Vec<PathPin>> {
        let chain = super::refind_chain(keys, find, &search.paths).ok()?;
        Some(chain.iter().map(|(v, p)| PathPin { vertex: *v, in_edge: p.prev.map(|x| (x.edge, x.arc)) }).collect())
    };
    let Some(path) = pins(&target.path) else {
        target.prepare_why = "fail:path_gone".into();
        return;
    };
    let latch = target.latch_data_path.as_ref().and_then(|(keys, start)| pins(keys).map(|p| (p, *start)));
    let slack = |v: usize, rf: Option<usize>| {
        let vx = &g.vertices[v];
        let v0 = if scene == 0 { Some(v) } else { gs[0].vertices.iter().position(|x| x.name == vx.name && x.is_driver == vx.is_driver) };
        v0.map_or(super::INF, |v0| super::slack_over_scenes(gs, ss, v0, vyges_sta::liberty::MAX, rf))
    };
    let reader = Reader { libs: ctx.libs, g, parasitics: design.parasitics(scene), scene, equiv: inputs.equiv, dont_use: inputs.dont_use, slack: &slack, clock_pins: &cs[scene], pin_id: inputs.pin_id, master_pins: ctx.master_pins };
    let index = target.path_index;
    let cfg = inputs.cfg;
    let options = ContextOptions {
        delay_levels: inputs.levels,
        use_sta_slew_bias: inputs.slew_bias,
        latch_data_path: latch.as_ref().map(|(p, s)| (p.as_slice(), *s)),
        fanin_side_loads: cfg.fanin_side_loss.then_some(FaninSideLoadOptions { critical_load_pins: inputs.crit, off_path_weight: cfg.fanin_side_offpath_weight }),
        dual_rf_eval: cfg.dual_rf_eval,
    };
    match reader.build_context(&path, target.path_start, index, &options) {
        Ok(mut context) => {
            if cfg.fanin_side_loss || cfg.dual_rf_eval || cfg.rebuffer_mt_side_loads {
                // Side paths may not be pushed below the current worst slack.
                context.side_slack_floor = inputs.wns;
                context.side_floor_margin = cfg.side_floor_margin;
                context.fanin_side_loss_weight = cfg.fanin_side_loss_weight;
            }
            target.estimator_context = Some(context);
            // `"fail:" + failReasonName(kNone)`: the printer's line after a built context.
            target.prepare_why = "fail:".into();
        }
        Err(reason) => target.prepare_why = format!("fail:{}", reason.name()),
    }
}

/// `prepareSequentialEstimator`: with a context, on a flip-flop target
/// (`SequentialTimingEstimator::prepare`: a cell whose sequentials are all registers). What it
/// prepares — the setup/hold checks per candidate cell — is not modelled yet.
fn pe_prepare_sequential_estimator(target: &mut Target, ctx: &super::Ctx<'_>) {
    if target.sequential_estimator || target.estimator_context.is_none() {
        return;
    }
    target.sequential_estimator = ctx.libs.link_cell(&target.cell).is_some_and(|c| !c.sequentials.is_empty() && c.sequentials.iter().all(|(_, is_register, _)| *is_register));
}

/// The VYGP printer's `{:.9e}` of a value that may be infinite (fmt prints `inf`).
fn e9i(v: f32) -> String {
    if v.is_infinite() {
        return if v > 0.0 { "inf".into() } else { "-inf".into() };
    }
    e9(v)
}

/// `RiseFall::shortName`.
fn rf_short(rf: usize) -> char {
    if rf == 0 { '^' } else { 'v' }
}

/// The VYGP `prep`, `pctx`, `pstage` and `pfsn` lines of a prepared target.
fn pe_prepared_lines(target: &Target, mask: u8, libs: &crate::preamble::Libs) -> String {
    let drv = &target.driver_pin;
    let why = if mask & ESTIMATOR_CONTEXT_CACHE == 0 { "nomask" } else { target.prepare_why.as_str() };
    let mut s = format!("VYGP|prep|{drv}|mask={mask}|ctx={}|why={why}|seq={}\n", target.estimator_context.is_some() as i32, target.sequential_estimator as i32);
    let Some(c) = &target.estimator_context else { return s };
    s.push_str(&format!(
        "VYGP|pctx|{drv}|stages={}|tsi={}|levels={}|floor={}|margin={}|fslw={}|opp={}|total={}|fsn={}\n",
        c.path_stages.len(),
        c.target_stage_index,
        c.delay_estimation_levels,
        e9i(c.side_slack_floor),
        e9i(c.side_floor_margin),
        e9i(c.fanin_side_loss_weight),
        e9i(c.opposite_rf_slack),
        e9i(c.current_total_delay),
        c.fanin_side_nets.len()
    ));
    for (k, st) in c.path_stages.iter().enumerate() {
        let cell = libs.scene_cell(st.arc.scene, &st.arc.cell);
        // An arc of the stage's cell: `port>rf` of its from port and output transition.
        let arc_name = |set: usize, arc: usize| cell.map_or("->-".to_string(), |c| format!("{}>{}", c.arc_sets[set].from, rf_short(c.arc_sets[set].arcs[arc].to_rf)));
        let sel = cell.map(|c| &c.arc_sets[st.arc.set]);
        let merge: Vec<String> = st.output_slew_merge_arcs.iter().map(|m| format!("{}:{}:{}", arc_name(m.set, m.arc), e9i(m.input_slew), e9i(m.current_model_slew))).collect();
        let opp: Vec<String> = st.opposite_rf_arcs.iter().map(|o| format!("{}:{}:{}", arc_name(o.set, o.arc), e9i(o.input_slew), e9i(o.current_model_delay))).collect();
        let bias = match (st.sta_slew_bias.valid, st.sta_slew_bias.table_worst_arc, cell) {
            (true, Some((set, _)), Some(c)) => format!("1:{}:{}{}", c.arc_sets[set].from, e9i(st.sta_slew_bias.input_slew), st.sta_slew_bias.samples.iter().map(|b| format!(":{}/{}/{}", e9i(b.load_cap), e9i(b.table_slew), e9i(b.sta_slew))).collect::<String>()),
            (true, ..) => "1:?".into(),
            _ => "0".into(),
        };
        s.push_str(&format!(
            "VYGP|pstage|{drv}|{k}|{}|pi={}|cell={}|in={}|out={}|irf={}|orf={}|slew={}|cap={}|cmd={}|cd={}|cwd={}|cms={}|cs={}|merge={}|bias={bias}|opparcs={}\n",
            st.driver_pin,
            st.path_index,
            st.arc.cell,
            sel.map_or("-", |a| a.from.as_str()),
            sel.map_or("-", |a| a.to.as_str()),
            sel.map_or('-', |a| rf_short(a.arcs[st.arc.arc].from_rf)),
            sel.map_or('-', |a| rf_short(a.arcs[st.arc.arc].to_rf)),
            e9i(st.input_slew),
            e9i(st.load_cap),
            e9i(st.current_model_delay),
            e9i(st.current_delay),
            e9i(st.current_wire_delay),
            e9i(st.current_model_slew),
            e9i(st.current_slew),
            merge.join(","),
            opp.join(",")
        ));
    }
    for (k, n) in c.fanin_side_nets.iter().enumerate() {
        let loads: Vec<String> = n.loads.iter().map(|l| format!("{}:{}:{}", l.pin, e9i(l.slack), e9i(l.weight))).collect();
        s.push_str(&format!("VYGP|pfsn|{drv}|{k}|port={}|cap={}|dpc={}|sel={}|loads={}\n", n.input_port, e9i(n.input_cap), e9i(n.delay_per_cap), n.selected_arc as i32, loads.join(",")));
    }
    s
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

    #[test]
    fn the_prepare_mask_is_the_union_of_every_generators_requirements() {
        // Upstream accumulatePrepareRequirements ORs each generator's prepareRequirements: SizeUpMt
        // and VtSwapMt ask for the estimator context and the sequential estimator, UnbufferMt for
        // nothing, every other Mt generator for the context alone. The mask is run-wide, so an
        // Unbuffer-only run prepares nothing and one SizeUp makes every target build both.
        assert_eq!(accumulate_prepare_requirements(&[Move::Unbuffer]), 0);
        assert_eq!(accumulate_prepare_requirements(&[]), 0);
        assert_eq!(accumulate_prepare_requirements(&[Move::SwapPins, Move::Unbuffer]), ESTIMATOR_CONTEXT_CACHE);
        for m in [Move::SwapPins, Move::SplitLoad, Move::SizeDownFanout, Move::Buffer] {
            assert_eq!(prepare_requirements(m), ESTIMATOR_CONTEXT_CACHE, "{m:?}");
        }
        assert_eq!(accumulate_prepare_requirements(&[Move::Unbuffer, Move::VtSwap]), ESTIMATOR_CONTEXT_CACHE | SEQUENTIAL_ESTIMATOR_CACHE);
        // kEstimatorContextCache is bit 0, kSequentialEstimatorCache bit 2 (the trace's mask=5).
        assert_eq!(prepare_requirements(Move::SizeUp), 5);
    }

    #[test]
    fn the_trace_prints_an_infinite_value_as_fmt_does() {
        // fmt {:.9e} of +-infinity is `inf` / `-inf`: an unset side floor is -inf, an unset opposite
        // slack +inf.
        assert_eq!(e9i(f32::NEG_INFINITY), "-inf");
        assert_eq!(e9i(f32::INFINITY), "inf");
        assert_eq!(e9i(1.0), "1.000000000e+00");
    }

    fn path_stage(pin: &str, is_driver: bool) -> super::super::Stage {
        super::super::Stage {
            pin: pin.into(),
            is_driver,
            top_port: false,
            inst: None,
            cell: None,
            port: None,
            load_delay: None,
            arc_delay: None,
            in_wire_delay: None,
            in_port: None,
            fanout: 1,
            load_cap: 0.0,
            fanin_caps: Vec::new(),
            rf: 0,
            unbuffer: None,
            in_slews: Vec::new(),
            rebuffer: None,
            fanout_slacks: None,
            split_slacks: None,
        }
    }

    #[test]
    fn the_crit_path_scan_takes_a_paths_load_pins_from_its_start_index() {
        // Rule (collectViolatingEndpoints under record_crit_path_load_pins_): over a violating
        // endpoint's expanded worst path from startIndex, every pin whose vertex is not a driver.
        // The clock pins before the start index are not on it.
        let stages = [("clk", true), ("r0/CK", false), ("r0/Q", true), ("u1/A", false), ("u1/Y", true), ("r1/D", false)].iter().map(|&(p, d)| path_stage(p, d)).collect();
        let view = PathView { stages, start: 2, scene: 0, latch_segments: Vec::new(), chain: Vec::new() };
        assert_eq!(pe_path_load_pins(&view).collect::<Vec<_>>(), vec!["u1/A", "r1/D"]);
    }

    fn target_at(path_index: usize) -> Target {
        Target {
            driver_pin: "b1/Z".into(),
            endpoint_pin: "r1/D".into(),
            violated_endpoint_pin: "r1/D".into(),
            driver_rf: 0,
            path_index,
            load_pin: None,
            slack: -1e-10,
            fanout: 1,
            scene: 0,
            inst: "b1".into(),
            cell: "BUF_X1".into(),
            path: Vec::new(),
            path_start: 0,
            latch_data_path: None,
            estimator_context: None,
            sequential_estimator: false,
            prepare_why: String::new(),
            unbuffer_gain: None,
            swap_pins_input: None,
            input_cap_budgets: None,
            split_load_input: None,
        }
    }

    #[test]
    fn a_rebuffer_target_budget_and_the_unbuffer_sequence_rule_gate_the_generators() {
        // Rule (generatorAllowed): BufferMove runs on at most rebuffer_max_targets targets per
        // batch (the count is taken before it is raised); UnbufferMove runs when the sequence
        // names it, or when the tail allows its own removal generator (never in batch 0: the tail
        // is reset with the run); every other move always runs.
        let mut pe = ParallelEval::new(for_parallel_eval(Effort::High), 'P');
        let max = pe.config.parallel_eval.rebuffer_max_targets;
        pe.rebuffer_targets_in_batch = max - 1;
        assert!(pe_generator_allowed(&pe, Move::Buffer));
        pe.rebuffer_targets_in_batch = max;
        assert!(!pe_generator_allowed(&pe, Move::Buffer));
        pe.unbuffer_in_sequence = false;
        assert!(!pe_generator_allowed(&pe, Move::Unbuffer));
        pe.unbuffer_in_sequence = true;
        assert!(pe_generator_allowed(&pe, Move::Unbuffer));
        for m in [Move::SizeUp, Move::VtSwap, Move::SwapPins, Move::SplitLoad, Move::SizeDownFanout] {
            assert!(pe_generator_allowed(&pe, m), "{m:?}");
        }
    }

    fn facts() -> PlanFacts {
        PlanFacts { driver_is_buffer: false, driver_is_inverter: false, dont_touch: false, pins_swapped: false, driver_liberty: true, fixed: false, ok_to_buffer_net: true, logic_std: true, vt_swappable: true, unbuffer_blocked: false, can_remove_buffer: true }
    }

    fn with_context(mut t: Target) -> Target {
        t.estimator_context = Some(crate::delay_estimator::EstimatorContext::new(2));
        t
    }

    #[test]
    fn a_size_up_target_is_a_changeable_unfixed_instance_with_a_context() {
        // Rule (SizeUpMtGenerator::isApplicable): an instance that is neither dont_touch nor fixed,
        // and whose estimator context was built.
        let t = with_context(target_at(2));
        assert!(pe_is_applicable(Move::SizeUp, &t, &facts()));
        assert!(!pe_is_applicable(Move::SizeUp, &target_at(2), &facts()));
        assert!(!pe_is_applicable(Move::SizeUp, &t, &PlanFacts { dont_touch: true, ..facts() }));
        assert!(!pe_is_applicable(Move::SizeUp, &t, &PlanFacts { fixed: true, ..facts() }));
    }

    #[test]
    fn a_vt_swap_target_is_a_changeable_logic_cell_with_a_vt_equivalent_and_a_context() {
        // Rule (VtSwapMtGenerator::isApplicable): a context; not dont_touch; a logic standard cell;
        // two VT categories, a master, and more than one VT-equivalent cell. A fixed instance may
        // still change VT (same footprint).
        let t = with_context(target_at(2));
        assert!(pe_is_applicable(Move::VtSwap, &t, &facts()));
        assert!(pe_is_applicable(Move::VtSwap, &t, &PlanFacts { fixed: true, ..facts() }));
        assert!(!pe_is_applicable(Move::VtSwap, &target_at(2), &facts()));
        assert!(!pe_is_applicable(Move::VtSwap, &t, &PlanFacts { dont_touch: true, ..facts() }));
        assert!(!pe_is_applicable(Move::VtSwap, &t, &PlanFacts { logic_std: false, ..facts() }));
        assert!(!pe_is_applicable(Move::VtSwap, &t, &PlanFacts { vt_swappable: false, ..facts() }));
    }

    #[test]
    fn the_trace_prints_input_cap_budgets_as_the_printer_does() {
        // The printer: `budgets=-` before the budgets are taken; else the count and per input
        // port:current_cap:max_cap_increase, an unlimited increase as `inf`.
        let mut t = target_at(2);
        assert_eq!(pe_inputs_line(Move::SizeUp, &t), "budgets=-");
        t.input_cap_budgets = Some(vec![InputCapBudget { port: "A".into(), current_cap: 0.5, max_cap_increase: f32::INFINITY }]);
        assert_eq!(pe_inputs_line(Move::VtSwap, &t), "budgets=1|A:5.000000000e-01:inf");
    }

    #[test]
    fn a_split_load_target_has_a_context_more_than_eight_loads_and_a_bufferable_net() {
        // Rule (SplitLoadMtGenerator::isApplicable): a context, a fanout above the legacy split's
        // kSplitLoadMinFanout (8), and okToBufferNet on the driver.
        let mut t = with_context(target_at(2));
        t.fanout = 9;
        assert!(pe_is_applicable(Move::SplitLoad, &t, &facts()));
        t.fanout = 8;
        assert!(!pe_is_applicable(Move::SplitLoad, &t, &facts()));
        t.fanout = 9;
        assert!(!pe_is_applicable(Move::SplitLoad, &t, &PlanFacts { ok_to_buffer_net: false, ..facts() }));
        let mut u = target_at(2);
        u.fanout = 9;
        assert!(!pe_is_applicable(Move::SplitLoad, &u, &facts()));
    }

    #[test]
    fn the_split_box_half_perimeter_grows_point_by_point_and_is_zero_empty() {
        // The split model's BoundingBox: dx + dy of the points added; 0 before any.
        let mut b = BoundingBox::default();
        assert_eq!(b.half_perimeter(), 0);
        b.add((10, 5));
        assert_eq!(b.half_perimeter(), 0);
        b.add((4, 9));
        b.add((7, 1));
        assert_eq!(b.half_perimeter(), 6 + 8);
    }

    #[test]
    fn a_swap_pins_target_is_a_changeable_gate_with_inputs_to_swap_entered_through_an_input() {
        // Rule (SwapPinsMtGenerator::isApplicable): a path index past the root, a liberty driver
        // port, a cell that is neither buffer nor inverter, not dont_touch, not swapped before.
        let ok = facts();
        assert!(pe_is_applicable(Move::SwapPins, &target_at(1), &ok));
        assert!(!pe_is_applicable(Move::SwapPins, &target_at(0), &ok));
        assert!(!pe_is_applicable(Move::SwapPins, &target_at(1), &PlanFacts { driver_liberty: false, ..facts() }));
        assert!(!pe_is_applicable(Move::SwapPins, &target_at(1), &PlanFacts { driver_is_buffer: true, ..facts() }));
        assert!(!pe_is_applicable(Move::SwapPins, &target_at(1), &PlanFacts { driver_is_inverter: true, ..facts() }));
        assert!(!pe_is_applicable(Move::SwapPins, &target_at(1), &PlanFacts { dont_touch: true, ..facts() }));
        assert!(!pe_is_applicable(Move::SwapPins, &target_at(1), &PlanFacts { pins_swapped: true, ..facts() }));
    }

    #[test]
    fn the_trace_prints_swap_pins_inputs_as_the_printer_does() {
        // The printer: `swap=-` without inputs; else the input, its delay, the count, and per
        // equivalent port:delay:pin:slack (pin `-` and slack 0 without the side model).
        let mut t = target_at(1);
        assert_eq!(pe_inputs_line(Move::SwapPins, &t), "swap=-");
        t.swap_pins_input = Some(SwapPinsInput { input_port: "A".into(), input_delay: 0.5, equivalents: vec![SwapPinsEquivalent { port: "B".into(), delay: 0.25, pin: None, slack: 0.0 }] });
        assert_eq!(pe_inputs_line(Move::SwapPins, &t), "swap=A|in_delay=5.000000000e-01|equiv=1|B:2.500000000e-01:-:0.000000000e+00");
    }

    #[test]
    fn an_unbuffer_target_is_a_buffer_two_stages_in_that_no_blocking_move_touched() {
        // Rule (UnbufferMtGenerator::isApplicable): the path holds the buffer and the stage that
        // drives it (path index 2 or more), the driver's cell is a buffer, and no SwapPins, Clone,
        // SplitLoad or Buffer move is committed on the instance (a resize does not block).
        let facts = |buffer: bool, blocked: bool| PlanFacts { driver_is_buffer: buffer, unbuffer_blocked: blocked, ..facts() };
        assert!(pe_is_applicable(Move::Unbuffer, &target_at(2), &facts(true, false)));
        assert!(!pe_is_applicable(Move::Unbuffer, &target_at(1), &facts(true, false)));
        assert!(!pe_is_applicable(Move::Unbuffer, &target_at(2), &facts(false, false)));
        assert!(!pe_is_applicable(Move::Unbuffer, &target_at(2), &facts(true, true)));
    }

    #[test]
    fn the_trace_prints_an_unset_unbuffer_gain_as_a_dash() {
        // The printer's vygOpt: an unset optional prints `-`, a set one {:.9e}.
        let mut t = target_at(2);
        assert_eq!(pe_inputs_line(Move::Unbuffer, &t), "gain=-");
        t.unbuffer_gain = Some(0.25);
        assert_eq!(pe_inputs_line(Move::Unbuffer, &t), "gain=2.500000000e-01");
    }
}
