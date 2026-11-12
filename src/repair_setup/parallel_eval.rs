//! `SetupParallelEvalPolicy` (`PARALLEL_EVAL`, experimental upstream): batches of moves chosen
//! across many targets at once, each target's candidates estimated and scored, the batch
//! committed under one timing update and judged against a checkpoint.
//!
//! Ported stage group by stage group, in the reference's order. Each stage is its own function
//! named after the reference's, and `start` / `iterate` do nothing but call them, so a trace that
//! diverges from the reference's points at one stage. Modelled so far: `start` (S1), the
//! target collection of a batch (S2: `collectTargetEndpoints`, `collectTargets`), the targets'
//! preparation (S3a: `prewarmTargets`, `prepareTargets`), candidate generation and estimation
//! (S3b/S3c: `generateCandidates`, `estimateTargets`), the batch (`selectBatch`:
//! `makeBatchCandidates`, `BatchSelector`), and (S4a) its commit (`commitBatch`, each candidate's
//! `apply`), the timing update, the verdict when the checkpoint accepts the batch
//! (`saveBatchCheckpoint`), the stop rules and the run's ends, with the work meter and the stall
//! monitor. A missed batch (salvage, undo, tabu), the small-batch retry, the WNS tail and the
//! bottleneck weights are refused.

use std::collections::{HashMap, HashSet};

use vyges_sta::liberty::MAX;

use vyges_sta::fuzzy;

use super::batch_selector::{BatchCandidate, BatchConflictReason, BatchSelection, BatchSelector};
use super::{PathView, Repair, Stop};
use crate::sequential_timing::SequentialResult;
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

/// `StallMonitor`: the batch stage's progress against its stall rules. The config is the run's
/// (`ParallelEvalPolicyConfig`), passed to the rules that read it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StallMonitor {
    pub initial_tns: f32,
    pub reference_period: f32,
    pub stalled_batches: i32,
    pub stalled_moves: i32,
    pub last_batch_moves: i32,
    pub wns_stall_batches: i32,
    pub window: std::collections::VecDeque<SavedCheckpoint>,
    pub window_stalled: bool,
    pub last_cap: i32,
    pub failing_endpoints: usize,
}

/// `StallMonitor::SavedCheckpoint`: one saved batch in the stall window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SavedCheckpoint {
    pub wns: f32,
    pub failing_endpoints: usize,
    pub work: f64,
}

/// `StallMonitor::Stall`, its printed value the enumerator's index.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stall {
    None = 0,
    Moves = 1,
    Batches = 2,
}

impl StallMonitor {
    /// `reset(initial_tns, reference_period)`: the counters and the window cleared; the last cap
    /// and failing endpoint count (`beginBatch`'s) kept.
    pub fn reset(&mut self, initial_tns: f32, reference_period: f32) {
        *self = StallMonitor { initial_tns, reference_period, last_cap: self.last_cap, failing_endpoints: self.failing_endpoints, ..StallMonitor::default() };
    }

    /// `beginBatch(cap, failing_endpoints)`.
    pub fn begin_batch(&mut self, batch_cap: i32, failing_endpoints: usize) {
        self.last_cap = batch_cap;
        self.failing_endpoints = failing_endpoints;
    }

    /// `endBatch(committed_moves)`.
    pub fn end_batch(&mut self, committed_moves: i32) {
        self.last_batch_moves = committed_moves;
    }

    /// `onMissed`: the batch's moves count against the stall budget.
    pub fn on_missed(&mut self) {
        self.stalled_batches += 1;
        self.stalled_moves += self.last_batch_moves;
        self.last_batch_moves = 0;
    }

    /// `onSaved(prev_tns, best_wns, wns, tns, work)`: progress is a WNS gain over the best
    /// checkpoint past `progress_wns_period_pct` of the reference period, or a TNS gain over the
    /// previous checkpoint past `progress_tns_pct` of |initial TNS|. The window takes the save;
    /// WNS stalls count while WNS does not progress; without progress the batch and its moves
    /// count as stalled (a salvaged batch counts twice). Returns the stalled moves after the
    /// save (the profiler event `stall_moves_after_save`).
    pub fn on_saved(&mut self, cfg: &ParallelEvalConfig, prev_tns: f32, best_wns: f32, wns: f32, tns: f32, work: f64) -> i32 {
        let tns_step = self.initial_tns.abs() * cfg.progress_tns_pct / 100.0;
        let wns_step = self.reference_period * cfg.progress_wns_period_pct / 100.0;
        let wns_progressed = wns - best_wns > wns_step;
        let tns_progressed = tns - prev_tns > tns_step;
        let progressed = wns_progressed || tns_progressed;
        self.update_window(cfg, wns, tns_progressed, work);
        self.wns_stall_batches = if wns_progressed { 0 } else { self.wns_stall_batches + 1 };
        self.stalled_batches = if progressed { 0 } else { self.stalled_batches + 1 };
        self.stalled_moves = if progressed { 0 } else { self.stalled_moves + self.last_batch_moves };
        self.last_batch_moves = 0;
        self.stalled_moves
    }

    /// `weightsPriority`: WNS stalled, for at most `bottleneck_max_wns_stall_batches` saves.
    pub fn weights_priority(&self, cfg: &ParallelEvalConfig) -> bool {
        self.wns_stall_batches > 0 && self.wns_stall_batches <= cfg.priority.bottleneck_max_wns_stall_batches
    }

    /// `stall`: the stalled moves reach the move budget, else the stalled batches reach
    /// `stall_max_batches`.
    pub fn stall(&self, cfg: &ParallelEvalConfig) -> Stall {
        if self.stalled_moves >= self.move_budget(cfg) {
            return Stall::Moves;
        }
        if self.stalled_batches >= cfg.stall_max_batches {
            return Stall::Batches;
        }
        Stall::None
    }

    /// `moveBudget`: `stall_batches` batches at the larger of the configured and the last cap.
    pub fn move_budget(&self, cfg: &ParallelEvalConfig) -> i32 {
        cfg.stall_batches * cfg.min_batch_cap.max(self.last_cap)
    }

    /// `updateWindow`: the save goes on the window; once it holds more than `stall_window` the
    /// oldest drops, and — unless TNS progressed, or the window spent less than
    /// `stall_window_min_work` — the window stalls when neither the average WNS gain per interval
    /// reaches `stall_wns_period_pct` of the reference period nor the failing endpoints fall by
    /// `stall_failing_endpoint_pct` of the last count per interval.
    fn update_window(&mut self, cfg: &ParallelEvalConfig, wns: f32, tns_progressed: bool, work: f64) {
        self.window.push_back(SavedCheckpoint { wns, failing_endpoints: self.failing_endpoints, work });
        if self.window.len() as i32 <= cfg.stall_window {
            return;
        }
        self.window.pop_front();
        if tns_progressed {
            return;
        }
        let (Some(&first), Some(&last)) = (self.window.front(), self.window.back()) else { return };
        if last.work - first.work < f64::from(cfg.stall_window_min_work) {
            return;
        }
        let intervals = (self.window.len() - 1) as f32;
        let wns_gain = (last.wns - first.wns) / intervals;
        let failing_endpoint_gain = first.failing_endpoints as f32 - last.failing_endpoints as f32;
        let wns_moving = wns_gain >= self.reference_period * cfg.stall_wns_period_pct / 100.0;
        let failing_endpoints_moving = failing_endpoint_gain / intervals >= cfg.stall_failing_endpoint_pct / 100.0 * last.failing_endpoints as f32;
        if !wns_moving && !failing_endpoints_moving {
            self.window_stalled = true;
            pe_event("stall_window_triggered", "", "", 1);
        }
    }
}

/// `WorkMeter`: the batch stage's work in seconds from counts — timing updates times the graph's
/// vertices, the nets each update re-estimates, the estimates, the rebuffer plans, the endpoints
/// evaluated — times the config's weights. No wall clock, so the stall and tail budgets are
/// portable.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct WorkMeter {
    pub vertices: f64,
    pub timing_updates: u64,
    pub pending_nets: u64,
    pub estimates: u64,
    pub plan_requests: u64,
    pub endpoints: u64,
}

impl WorkMeter {
    /// `reset(graph_vertices)`.
    pub fn reset(&mut self, graph_vertices: usize) {
        *self = WorkMeter { vertices: graph_vertices as f64, ..WorkMeter::default() };
    }

    /// `addTimingUpdate(pending_nets)`.
    pub fn add_timing_update(&mut self, pending_nets: usize) {
        self.timing_updates += 1;
        self.pending_nets += pending_nets as u64;
        pe_event("work_timing_updates", "", "", 1);
        pe_event("work_pending_nets", "", "", pending_nets);
    }

    /// `addEstimates(estimates)`.
    pub fn add_estimates(&mut self, estimates: usize) {
        self.estimates += estimates as u64;
    }

    /// `addPlanRequest`.
    pub fn add_plan_request(&mut self) {
        self.plan_requests += 1;
    }

    /// `addEndpoints(endpoints)`.
    pub fn add_endpoints(&mut self, endpoints: usize) {
        self.endpoints += endpoints as u64;
    }

    /// `seconds(config)`, in double as the reference computes it.
    pub fn seconds(&self, cfg: &ParallelEvalConfig) -> f64 {
        f64::from(cfg.work_sta_per_vertex) * self.vertices * self.timing_updates as f64
            + f64::from(cfg.work_per_pending_net) * self.pending_nets as f64
            + f64::from(cfg.work_per_estimate) * self.estimates as f64
            + f64::from(cfg.work_per_plan) * self.plan_requests as f64
            + f64::from(cfg.work_per_endpoint) * self.endpoints as f64
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
    /// `sequential_timing_estimator` (`prepareSequentialEstimator`): a flip-flop target's setup
    /// and hold checks, for the cell swaps' side costs.
    pub sequential_estimator: Option<crate::sequential_timing::SequentialTimingEstimator>,
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
    /// `size_down_fanout_loads` (`SizeDownFanoutMtGenerator::prepareInputs`): the loads that
    /// can step down, in ranked order; empty until prepared.
    pub size_down_fanout_loads: Vec<SizeDownFanoutLoad>,
    /// `rebuffer_net` (`RebufferMtGenerator::prepareInputs`): the driver's net as a buffered net
    /// and its root; none when the net cannot be rebuffered or was not prepared.
    pub rebuffer_net: Option<(crate::buffered_net::BufferedNet, usize)>,
    /// `rebuffer_load_timing` (`collectLoadTiming`): per load with a finite slack, its slack, the
    /// wire delay from the driver and its side-load weight. Upstream's map is lookup-only; this
    /// keeps the driver's wire-edge order.
    pub rebuffer_load_timing: Vec<RebufferLoadTiming>,
    /// `GenerationTask::generators`: the generators `planGeneration` kept, in sequence order.
    pub plan: Vec<Move>,
}

/// `MoveCandidate` as the batch reads it: its move, its `estimate()`, the printer's description
/// and its `impact()`.
#[derive(Debug, Clone, PartialEq)]
pub struct Candidate {
    pub kind: Move,
    pub estimate: Estimate,
    pub desc: String,
    pub impact: MoveImpact,
    /// A RebufferMt candidate's tree plan (`plan_`): the planner's arena as planning left it and
    /// the plan's root, which its `apply` exports. None for every other move.
    pub tree: Option<(std::sync::Arc<Vec<crate::rebuffer::Node>>, usize)>,
    /// A RebufferMt candidate's side contexts' load pins (`guardedSidePins`); the other moves'
    /// come from their target ([`pe_guarded_side_pins`]).
    pub rebuffer_side_pins: Vec<String>,
}

/// `Estimate`: legality, the score the batch ranks by, and the parts the printer shows.
#[derive(Debug, Clone, PartialEq)]
pub struct Estimate {
    pub legal: bool,
    pub score: f32,
    pub raw_gain: Option<f32>,
    pub reason: &'static str,
    pub tail_admissible: bool,
    pub side_loss: Option<f32>,
    pub sequential_penalty: Option<f32>,
    pub sequential_side_loss: Option<f32>,
}

/// `MoveImpact`: complete or not (false when an estimator could not enumerate every read and
/// write), the instances and nets the move writes, the pins and instances it reconnects, the
/// instances and nets whose timing it can change, and those whose timing its estimate read.
/// `BatchSelector` compares them by identity; here by name, the top instance as [`TOP_INSTANCE`].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MoveImpact {
    pub complete: bool,
    pub write_instances: Vec<String>,
    pub write_nets: Vec<String>,
    pub reconnect_pins: Vec<String>,
    pub reconnect_instances: Vec<String>,
    pub timing_effect_instances: Vec<String>,
    pub timing_effect_nets: Vec<String>,
    pub timing_dependency_instances: Vec<String>,
    pub timing_dependency_nets: Vec<String>,
}

/// `Network::instance` of a top-level port: the top instance, which every port shares.
pub const TOP_INSTANCE: &str = "<top>";

/// `appendUnique`: a value that is set and not in the list yet goes on its end.
fn append_unique(values: &mut Vec<String>, value: Option<String>) {
    if let Some(value) = value {
        if !values.contains(&value) {
            values.push(value);
        }
    }
}

/// What a move's `impact()` reads off the network, by name over the timer's netlist: a pin's
/// instance (`Network::instance`: the top instance for a top-level port), its net
/// (`Network::net`: none when unconnected), and an instance's pins (`pinIterator`: its master's
/// terminals in order, supply terminals skipped).
pub struct ImpactNames<'a> {
    pin_net: HashMap<String, &'a str>,
    ports: HashSet<&'a str>,
    inst_cell: HashMap<&'a str, &'a str>,
    master_pins: &'a HashMap<String, Vec<String>>,
}

impl<'a> ImpactNames<'a> {
    pub fn new(nl: &'a vyges_sta::netlist::Netlist, master_pins: &'a HashMap<String, Vec<String>>) -> ImpactNames<'a> {
        let mut pin_net = HashMap::new();
        for net in &nl.nets {
            for c in &net.pins {
                pin_net.insert(nl.pin_name(c), net.name.as_str());
            }
        }
        ImpactNames { pin_net, ports: nl.ports.iter().map(|(p, _)| p.as_str()).collect(), inst_cell: nl.insts.iter().map(|i| (i.0.as_str(), i.1.as_str())).collect(), master_pins }
    }

    /// `network->instance(pin)`.
    fn instance(&self, pin: &str) -> String {
        if self.ports.contains(pin) {
            return TOP_INSTANCE.to_string();
        }
        pin.rsplit_once('/').map_or(pin, |(inst, _)| inst).to_string()
    }

    /// `network->net(pin)`.
    fn net(&self, pin: &str) -> Option<String> {
        self.pin_net.get(pin).map(|n| (*n).to_string())
    }

    /// `network->pinIterator(inst)`.
    fn pins(&self, inst: &str) -> Vec<String> {
        let ports = self.inst_cell.get(inst).and_then(|cell| self.master_pins.get(*cell));
        ports.map_or(Vec::new(), |ports| ports.iter().map(|p| format!("{inst}/{p}")).collect())
    }
}

/// `MoveCandidate::impact`: none without an estimator context; else the instance and net of each
/// path stage's driver are timing dependencies, each once.
fn move_candidate_impact(target: &Target, names: &ImpactNames<'_>) -> MoveImpact {
    let mut impact = MoveImpact::default();
    let Some(context) = &target.estimator_context else { return impact };
    for stage in &context.path_stages {
        if stage.driver_pin.is_empty() {
            continue;
        }
        append_unique(&mut impact.timing_dependency_instances, Some(names.instance(&stage.driver_pin)));
        append_unique(&mut impact.timing_dependency_nets, names.net(&stage.driver_pin));
    }
    impact
}

/// `MoveCandidate::cellSwapImpact`: the base impact, complete; the instance written and in the
/// effect cone, and every net on its pins (a new cell changes its input caps and output timing).
fn cell_swap_impact(target: &Target, names: &ImpactNames<'_>, inst: &str) -> MoveImpact {
    let mut impact = move_candidate_impact(target, names);
    impact.complete = true;
    append_unique(&mut impact.write_instances, Some(inst.to_string()));
    append_unique(&mut impact.timing_effect_instances, Some(inst.to_string()));
    for pin in names.pins(inst) {
        append_unique(&mut impact.timing_effect_nets, names.net(&pin));
    }
    impact
}

/// `RebufferLoadTiming`: one load of a rebuffer target's net as `collectLoadTiming` reads it.
#[derive(Debug, Clone, PartialEq)]
pub struct RebufferLoadTiming {
    pub pin: String,
    pub slack: f32,
    pub wire_delay: f32,
    pub weight: f32,
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

/// `SizeDownFanoutMtLoad`: one fanout load of the target driver that may change cell, the slack
/// it may spend, and the one-step-smaller cell it is measured against.
#[derive(Debug, Clone, PartialEq)]
pub struct SizeDownFanoutLoad {
    pub inst: String,
    pub pin: String,
    pub port: String,
    pub cell: String,
    pub load_slack: f32,
    /// A register's worst output slack; else the worse of the pin's and the instance's worst
    /// input slack.
    pub delay_budget: f32,
    /// The load port's capacitance at the target's scene.
    pub input_cap: f32,
    pub step_down: SizeDownFanoutCellOption,
}

/// `SizeDownFanoutMtCellOption`: the smaller cell, its input capacitance, whether it breaks an
/// output's cap or slew limit, the worst output delay change (0 when it does), and the input
/// capacitance change at the target arc's scene (none when either cell lacks the port there).
#[derive(Debug, Clone, PartialEq)]
pub struct SizeDownFanoutCellOption {
    pub cell: String,
    pub input_cap: f32,
    pub violates_output_limits: bool,
    pub worst_delay_change: f32,
    pub target_load_delta: Option<f32>,
}

/// `SizeDownOutput`: one output of a fanout load as it is timed now.
#[derive(Debug, Clone, PartialEq)]
struct SizeDownOutput {
    pin: String,
    port: String,
    cap: f32,
    slew_factor: f32,
    delay: f32,
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

/// `BatchPool`: the targets of one batch (a deque upstream, so they never move), their
/// evaluations index-aligned with them, the batch candidates made from those and the selection.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BatchPool {
    pub targets: Vec<Target>,
    pub evaluations: Vec<TargetEvaluation>,
    pub candidates: Vec<BatchCandidate>,
    pub selection: BatchSelection,
}

impl BatchPool {
    /// `BatchPool::adopt`: the chunk's targets go on the end of the pool.
    fn adopt(&mut self, chunk: Vec<Target>) {
        self.targets.extend(chunk);
    }

    /// `BatchPool::append`: the chunk's evaluations go on the end of the pool's.
    fn append(&mut self, chunk_evaluations: Vec<TargetEvaluation>) {
        self.evaluations.extend(chunk_evaluations);
    }
}

/// `TargetEvaluation`: one target's candidates in generation order and the index of its best
/// (`best_candidate`; its `best_estimate` is that candidate's estimate after `estimateTarget`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TargetEvaluation {
    pub candidates: Vec<Candidate>,
    pub best: Option<usize>,
}

impl TargetEvaluation {
    /// `bestCandidate()`, when `hasBestCandidate()`.
    fn best_candidate(&self) -> Option<&Candidate> {
        self.best.map(|k| &self.candidates[k])
    }
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
    e9d(f64::from(v))
}

/// [`e9`] of a double (`fmt::format("{:.9e}", double)`).
fn e9d(v: f64) -> String {
    let s = format!("{:.9e}", v);
    let (m, e) = s.split_once('e').expect("Rust's {:e} always has an exponent");
    let e: i32 = e.parse().expect("Rust's {:e} exponent is an integer");
    format!("{m}e{}{:02}", if e < 0 { '-' } else { '+' }, e.abs())
}

/// `fmt::format("{:.3e}", v)`: the exponent signed, at least two digits.
fn e3(v: f32) -> String {
    let s = format!("{:.3e}", v);
    let (m, e) = s.split_once('e').expect("Rust's {:e} always has an exponent");
    let e: i32 = e.parse().expect("Rust's {:e} exponent is an integer");
    format!("{m}e{}{:02}", if e < 0 { '-' } else { '+' }, e.abs())
}

/// The bottleneck analysis the policy keeps between batches: its age (`INT_MAX` when none
/// applies), whether it was made in the tail, and each analyzed cone vertex's severity keyed by
/// the vertex's pin name and driver flag (the graph is rebuilt between batches; a new vertex is
/// outside the cone, as a new vertex id is upstream).
#[derive(Debug, Clone, PartialEq)]
pub struct Bottlenecks {
    pub age: i32,
    pub in_tail: bool,
    pub severity: HashMap<(String, bool), f32>,
    pub max_severity: f32,
}

impl Default for Bottlenecks {
    fn default() -> Self {
        Bottlenecks { age: i32::MAX, in_tail: false, severity: HashMap::new(), max_severity: 0.0 }
    }
}

impl Bottlenecks {
    /// `bottleneckPriorityScore(local_score, vertex, priority)`: `local_score * (1 + scale *
    /// ratio^exponent)`, the ratio the vertex's severity over the largest (0 outside the cone).
    pub fn priority_score(&self, local_score: f32, vertex: Option<&(String, bool)>, priority: &BatchPriorityConfig) -> f32 {
        let severity = vertex.and_then(|k| self.severity.get(k));
        let ratio = match severity {
            Some(&sv) if self.max_severity > 0.0 => sv / self.max_severity,
            _ => 0.0,
        };
        local_score * (1.0 + priority.bottleneck_scale * ratio.powf(priority.bottleneck_exponent))
    }
}

/// What an analysis leaves: each cone vertex's severity by key, the largest, the cone size.
type ConeSeverity = (HashMap<(String, bool), f32>, f32, usize);

/// One violating-cone vertex (`BottleneckAnalysis::Node`).
#[derive(Debug, Clone, Default)]
struct ConeNode {
    vertex: usize,
    slack_deficit: f32,
    has_cone_fanin: bool,
    startpoint_deficit_sum: f32,
    endpoint_deficit_sum: f32,
}

/// `forEachDataFanin`: the fanin edges carrying data arrivals, in the reference's in-edge
/// iterator order (newest first): no timing checks, clock-to-Q or latch D-to-Q, no set/clear arcs
/// (`preset_clr_arcs_enabled` is false), no disabled bidirect instance paths. Disabled loop
/// edges cannot occur: a timing loop is refused when the graph is built.
fn data_fanins(g: &vyges_sta::graph::Graph<'_>, v: usize) -> Vec<usize> {
    use vyges_sta::liberty::Role;
    let mut out = Vec::new();
    for &e in g.in_edges[v].iter().rev() {
        if let vyges_sta::graph::EdgeKind::Gate { set } = g.edges[e].kind {
            let role = &g.arc_set(e, set).role;
            if role.is_timing_check() || matches!(role, Role::RegClkToQ | Role::LatchDtoQ | Role::RegSetClr) || g.bidirect_inst_paths.contains(&e) {
                continue;
            }
        }
        out.push(g.edges[e].from);
    }
    out
}

/// `BottleneckAnalysis::analyze(violating_endpoints, slack_margin, max_endpoints,
/// max_startpoints)` on scene 0's graph (slacks over every scene): the cone of the first
/// `max_endpoints` endpoints (a backward search, LIFO, through the vertices that still violate),
/// each vertex's distinct downstream endpoint deficits and distinct upstream startpoint deficits
/// (the startpoints: cone vertices without a cone fanin, worst deficit first, at most
/// `max_startpoints`), and its severity, their geometric mean. Returns the severities by vertex
/// key, the largest, and the cone size.
fn bottleneck_analyze(gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], endpoints: &[(String, f32)], slack_margin: f32, max_startpoints: usize) -> Result<ConeSeverity, String> {
    let g = &gs[0];
    let mut nodes: Vec<ConeNode> = Vec::new();
    let mut node_of: Vec<i32> = vec![-1; g.vertices.len()];
    let admit = |nodes: &mut Vec<ConeNode>, node_of: &mut Vec<i32>, v: usize, deficit: f32| -> usize {
        nodes.push(ConeNode { vertex: v, slack_deficit: deficit, ..ConeNode::default() });
        node_of[v] = (nodes.len() - 1) as i32;
        nodes.len() - 1
    };
    // collectCone.
    let mut endpoint_nodes = Vec::new();
    let mut queue = Vec::new();
    for (pin, slack) in endpoints {
        let v = g.vertices.iter().position(|x| x.name == *pin && !x.is_driver).or_else(|| g.vertices.iter().position(|x| x.name == *pin));
        let deficit = slack_margin - slack;
        let Some(v) = v else { continue };
        if deficit <= 0.0 || node_of[v] >= 0 {
            continue;
        }
        let k = admit(&mut nodes, &mut node_of, v, deficit);
        endpoint_nodes.push(k);
        queue.push(k);
    }
    while let Some(k) = queue.pop() {
        for fanin in data_fanins(g, nodes[k].vertex) {
            if node_of[fanin] >= 0 {
                nodes[k].has_cone_fanin = true;
                continue;
            }
            let deficit = slack_margin - super::slack_over_scenes(gs, ss, fanin, MAX, None);
            if deficit <= 0.0 {
                continue;
            }
            let f = admit(&mut nodes, &mut node_of, fanin, deficit);
            queue.push(f);
            nodes[k].has_cone_fanin = true;
        }
    }
    if nodes.is_empty() {
        return Ok((HashMap::new(), 0.0, 0));
    }
    // Level order (a stable sort: ties stay in admission order); a fanin has the lower level.
    let levels = g.levels()?;
    let mut order: Vec<usize> = (0..nodes.len()).collect();
    order.sort_by_key(|&k| levels[nodes[k].vertex]);
    // sumEndpointDeficits: reach sets handed to the fanins in descending level.
    let words = endpoint_nodes.len().div_ceil(64);
    let mut reach = vec![0u64; nodes.len() * words];
    for (bit, &k) in endpoint_nodes.iter().enumerate() {
        reach[k * words + bit / 64] |= 1u64 << (bit % 64);
    }
    for &k in order.iter().rev() {
        for fanin in data_fanins(g, nodes[k].vertex) {
            let f = node_of[fanin];
            if f < 0 {
                continue;
            }
            for w in 0..words {
                let bits = reach[k * words + w];
                reach[f as usize * words + w] |= bits;
            }
        }
    }
    for k in 0..nodes.len() {
        for w in 0..words {
            let mut bits = reach[k * words + w];
            while bits != 0 {
                let bit = w * 64 + bits.trailing_zeros() as usize;
                nodes[k].endpoint_deficit_sum += nodes[endpoint_nodes[bit]].slack_deficit;
                bits &= bits - 1;
            }
        }
    }
    // sumStartpointDeficits: the worst startpoints, reach sets collected in ascending level.
    let mut starts: Vec<usize> = (0..nodes.len()).filter(|&k| !nodes[k].has_cone_fanin).collect();
    starts.sort_by(|&a, &b| nodes[b].slack_deficit.partial_cmp(&nodes[a].slack_deficit).unwrap_or(std::cmp::Ordering::Equal));
    starts.truncate(max_startpoints);
    let words = starts.len().div_ceil(64);
    let mut reach = vec![0u64; nodes.len() * words];
    for (bit, &k) in starts.iter().enumerate() {
        reach[k * words + bit / 64] |= 1u64 << (bit % 64);
    }
    for &k in &order {
        for fanin in data_fanins(g, nodes[k].vertex) {
            let f = node_of[fanin];
            if f < 0 {
                continue;
            }
            for w in 0..words {
                let bits = reach[f as usize * words + w];
                reach[k * words + w] |= bits;
            }
        }
    }
    for k in 0..nodes.len() {
        for w in 0..words {
            let mut bits = reach[k * words + w];
            while bits != 0 {
                let bit = w * 64 + bits.trailing_zeros() as usize;
                nodes[k].startpoint_deficit_sum += nodes[starts[bit]].slack_deficit;
                bits &= bits - 1;
            }
        }
    }
    // bottleneckSeverity: the geometric mean keeps seconds and balances both sides.
    let mut severity = HashMap::with_capacity(nodes.len());
    let mut max_severity = 0.0f32;
    for n in &nodes {
        let sv = (n.startpoint_deficit_sum * n.endpoint_deficit_sum).sqrt();
        max_severity = max_severity.max(sv);
        let x = &g.vertices[n.vertex];
        severity.insert((x.name.clone(), x.is_driver), sv);
    }
    Ok((severity, max_severity, nodes.len()))
}

/// `WnsTail::Step`: the tail's decision after a batch (printed as its index).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TailStep {
    Continue = 0,
    Speculate = 1,
    Widen = 2,
    Stop = 3,
}

/// `WnsTail`: after the batches stall, the run works on the worst `wns_tail_endpoints` endpoints
/// with small batches (one move after a miss, doubling after a save up to the stage's cap); a
/// stalled stage first speculates (keeps missed batches in the open journal), then widens.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct WnsTail {
    pub active: bool,
    pub started: bool,
    /// Batches so far, and batches without a better WNS.
    pub batches: i32,
    pub batches_without_gain: i32,
    pub endpoints: i32,
    pub max_batch: i32,
    pub batch_cap: i32,
    /// Work when the tail started and the work it may spend.
    pub work_start: f64,
    pub budget_work: f64,
    /// The smallest best-WNS gain that counts as tail progress.
    pub min_gain: f32,
    /// Missed tail batches kept in the open journal, and the instances they committed.
    pub speculative_batches: i32,
    pub speculative_instances: Vec<InstanceRecord>,
    pub greedy_stalled: bool,
}

impl WnsTail {
    /// `reset`.
    pub fn reset(&mut self) {
        *self = WnsTail::default();
    }

    /// `start(work, reference_period)`: once, and only with tail endpoints configured; the
    /// budget is `wns_tail_work_pct` of the work so far, at least `wns_tail_min_work`.
    pub fn start(&mut self, cfg: &ParallelEvalConfig, work: f64, reference_period: f32) -> bool {
        if cfg.wns_tail_endpoints <= 0 || self.started {
            return false;
        }
        self.reset();
        self.active = true;
        self.started = true;
        self.endpoints = cfg.wns_tail_endpoints;
        self.max_batch = cfg.wns_tail_batch;
        self.batch_cap = 1;
        self.work_start = work;
        self.budget_work = (self.work_start * f64::from(cfg.wns_tail_work_pct) / 100.0).max(f64::from(cfg.wns_tail_min_work));
        self.min_gain = reference_period * cfg.wns_tail_min_gain_period_pct / 100.0;
        true
    }

    /// `unbufferAllowed`: buffer removal only in the single-move batches of a stalled greedy stage.
    pub fn unbuffer_allowed(&self) -> bool {
        self.active && self.batch_cap == 1 && self.greedy_stalled
    }

    /// `afterBatch(best_wns_before, best_wns_after, batch_saved)`: only a best-WNS gain over
    /// `min_gain` is progress (a speculative run is judged when it ends); a save doubles the cap
    /// up to the stage's, a miss drops it to one move.
    pub fn after_batch(&mut self, best_wns_before: f32, best_wns_after: f32, batch_saved: bool) {
        self.batches += 1;
        let progressed = best_wns_after - best_wns_before > self.min_gain;
        if self.speculative_batches == 0 {
            self.batches_without_gain = if progressed { 0 } else { self.batches_without_gain + 1 };
        }
        self.batch_cap = if batch_saved { self.max_batch.min(self.batch_cap * 2) } else { 1 };
    }

    /// `nextStep(committed_moves, work, reason)`: the first rule that fires is the reason; the
    /// work budget and the batch limit stop the tail; a stall speculates once per greedy stage;
    /// otherwise the tail widens while it can.
    pub fn next_step(&self, cfg: &ParallelEvalConfig, committed_moves: i32, work: f64) -> (TailStep, Option<&'static str>) {
        let reason = if committed_moves == 0 {
            "wns_tail_no_committed_moves"
        } else if self.batches_without_gain >= cfg.wns_tail_stall {
            "wns_tail_stalled"
        } else if self.batches >= cfg.wns_tail_max_batches {
            "wns_tail_max_batches"
        } else if work - self.work_start > self.budget_work {
            "wns_tail_work_budget"
        } else {
            return (TailStep::Continue, None);
        };
        if reason == "wns_tail_work_budget" || self.batches >= cfg.wns_tail_max_batches {
            return (TailStep::Stop, Some(reason));
        }
        if !self.greedy_stalled && reason == "wns_tail_stalled" {
            return (TailStep::Speculate, Some(reason));
        }
        if self.endpoints < cfg.wns_tail_max_endpoints {
            return (TailStep::Widen, Some(reason));
        }
        (TailStep::Stop, Some(reason))
    }

    /// `beginSpeculativeStage`.
    pub fn begin_speculative_stage(&mut self) {
        self.batches_without_gain = 0;
        self.greedy_stalled = true;
    }

    /// `widen`: an open speculative run dropped (the caller's restore undid it); endpoints and
    /// the stage cap grown by `wns_tail_grow`.
    pub fn widen(&mut self, cfg: &ParallelEvalConfig) {
        self.speculative_batches = 0;
        self.speculative_instances.clear();
        self.endpoints = cfg.wns_tail_max_endpoints.min(self.endpoints * cfg.wns_tail_grow);
        self.max_batch *= cfg.wns_tail_grow;
        self.batches_without_gain = 0;
        self.greedy_stalled = true;
    }

    /// `maySpeculate`: only once a greedy stage stalled, up to `wns_tail_speculate` batches.
    pub fn may_speculate(&self, cfg: &ParallelEvalConfig) -> bool {
        self.speculative_batches < cfg.wns_tail_speculate && self.greedy_stalled
    }

    /// `keepSpeculative(instances)`.
    pub fn keep_speculative(&mut self, instances: &[InstanceRecord]) {
        self.speculative_batches += 1;
        self.speculative_instances.extend_from_slice(instances);
    }

    /// `speculating`.
    pub fn speculating(&self) -> bool {
        self.speculative_batches > 0
    }

    /// `endSpeculation`: the run ended; its instances returned.
    pub fn end_speculation(&mut self) -> Vec<InstanceRecord> {
        self.speculative_batches = 0;
        std::mem::take(&mut self.speculative_instances)
    }
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
    /// The selector's config, copied after the margin went in (`batch_selector_`).
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
    /// `markRunComplete`'s result (`result_`).
    pub result: bool,
    pub stall: StallMonitor,
    pub work: WorkMeter,
    /// `batch_instances_`: the committed batch's targets' instances (`InstanceRecord`: the name
    /// read before each move, and the odb slot the reference's `Instance*` addresses).
    pub batch_instances: Vec<InstanceRecord>,
    /// `batch_candidates_`: the committed candidates, by evaluation index, in commit order.
    pub batch_candidates: Vec<usize>,
    /// `batch_swaps_`: the batch's single-instance size and VT swaps a salvage can revert.
    pub batch_swaps: Vec<SwapRecord>,
    /// `tabu_`.
    pub tabu: MissTabu,
    /// `tail_`: the WNS tail, after the batches stall.
    pub tail: WnsTail,
    /// `bottleneck_`, `bottleneck_age_`, `bottleneck_in_tail_`.
    pub bottleneck: Bottlenecks,
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
            result: false,
            stall: StallMonitor::default(),
            work: WorkMeter::default(),
            batch_instances: Vec::new(),
            batch_candidates: Vec::new(),
            batch_swaps: Vec::new(),
            tabu: MissTabu::default(),
            tail: WnsTail::default(),
            bottleneck: Bottlenecks::default(),
            crit_path_load_pins: HashSet::new(),
            marker,
        }
    }
}

/// `SwapRecord`: a committed size or VT swap — its evaluation, its instance and the cell it
/// replaced (the location is the instance's own: a swap does not move it), the move's result
/// once kept, and whether a salvage reverted it.
#[derive(Debug, Clone, PartialEq)]
pub struct SwapRecord {
    pub evaluation_index: usize,
    pub inst: String,
    pub before_cell: String,
    pub result: ApplyResult,
    pub reverted: bool,
}

/// `InstanceRecord`: an instance's name read before a move, and its odb slot (the reference's
/// `Instance*`). `keptInstances` keeps a record whose name still finds the same slot.
#[derive(Debug, Clone, PartialEq)]
pub struct InstanceRecord {
    pub name: String,
    pub id: u32,
}

/// `MissTabu`: instances skipped for `miss_tabu_batches` batches after a miss, and those excluded
/// for the run after `max_undos_per_instance` undone batches. Keyed by the odb slot (the
/// reference's address) with the name recorded: an entry counts only while the slot still holds
/// an instance of that name (a deleted instance's slot can go to another instance). Lookups and
/// erasure only, so the maps' order decides nothing.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MissTabu {
    pub until: HashMap<u32, (String, i32)>,
    pub undo_counts: HashMap<u32, (String, i32)>,
    pub excluded: HashMap<u32, String>,
}

impl MissTabu {
    /// `reset`.
    pub fn reset(&mut self) {
        *self = MissTabu::default();
    }

    /// `addTabu(inst, now)`: skipped until batch `now + miss_tabu_batches`.
    pub fn add_tabu(&mut self, cfg: &ParallelEvalConfig, id: u32, name: &str, now: i32) {
        self.until.insert(id, (name.to_string(), now + cfg.miss_tabu_batches));
    }

    /// `recordUndo(inst, now)`: `addTabu` plus the undo count (an entry left by another
    /// instance in the slot starts over); true once the count reaches `max_undos_per_instance`
    /// and the instance is excluded.
    pub fn record_undo(&mut self, cfg: &ParallelEvalConfig, id: u32, name: &str, now: i32) -> bool {
        self.add_tabu(cfg, id, name, now);
        if !self.undo_counts.get(&id).is_some_and(|(n, _)| n == name) {
            self.undo_counts.insert(id, (name.to_string(), 0));
        }
        let count = self.undo_counts.get_mut(&id).expect("the entry was inserted above");
        count.1 += 1;
        if count.1 >= cfg.max_undos_per_instance {
            self.excluded.insert(id, name.to_string());
            return true;
        }
        false
    }

    /// `expire(now)`: the tabu entries ending at or before `now` dropped.
    pub fn expire(&mut self, now: i32) {
        self.until.retain(|_, (_, until)| *until > now);
    }

    /// `clearTemporary`: the tabu dropped, the exclusions kept.
    pub fn clear_temporary(&mut self) {
        self.until.clear();
    }

    /// `isExcluded`.
    pub fn is_excluded(&self, id: u32, name: &str) -> bool {
        self.excluded.get(&id).is_some_and(|n| n == name)
    }

    /// `isTabu`.
    pub fn is_tabu(&self, id: u32, name: &str) -> bool {
        self.until.get(&id).is_some_and(|(n, _)| n == name)
    }
}

/// `MoveResult` of a candidate's `apply`: accepted, the move count and the touched instances.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ApplyResult {
    pub accepted: bool,
    pub move_count: i64,
    pub touched: Vec<String>,
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
        // The reserve follows the margin the targets are collected with: written into the
        // policy config's selector (which the retry rebuilds from), then the selector made.
        pe.config.parallel_eval.selector.slack_margin = self.ctx.margin;
        pe.selector = pe.config.parallel_eval.selector;
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
        pe.work.reset(pe.graph_vertices);
        self.debug("repair_setup", 1, format!("PARALLEL_EVAL reference clock period {}", e3(pe.reference_period)));
        Ok(())
    }

    /// `graph_->vertexCount()`.
    fn pe_graph_vertices(&mut self) -> Result<usize, Stop> {
        let edits = self.design.take_timer_edits();
        let ctx = self.ctx;
        super::timed_all(ctx, self.design.as_design(), &mut self.timer, edits, |gs, _, _| Ok(gs[0].vertices.len()))
    }

    /// `iterate`: one batch. Collect, weigh, evaluate; then commit, judge, stop.
    fn pe_iterate(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        pe_trace(&format!("VYGP|iter|{}|converged={}\n", pe.batch_index, u8::from(pe.converged)));
        if pe.converged {
            return Ok(());
        }
        pe.tabu.expire(pe.batch_index);
        if self.pe_finish_if_no_setup_violations(pe)? || self.pe_finish_at_max_area(pe)? {
            return Ok(());
        }
        let scope = self.pe_collect_target_endpoints(pe)?;
        self.pe_analyze_bottlenecks(pe, &scope)?;
        // Design wide even under -path_group: the side pins of a missed batch may lie on any
        // group's paths.
        let pre_batch_wns = self.timing.design_worst();
        pe_trace(&format!("VYGP|prewns|{}|{}\n", pe.batch_index, e9(pre_batch_wns)));
        let pool = self.pe_evaluate_targets(pe, scope)?;
        if pool.targets.is_empty() {
            self.pe_finish_without_targets(pe)?;
            return Ok(());
        }
        self.pe_open_batch_journal(pe)?;
        let mut batch_moves = self.pe_commit_batch(pe, &pool.targets, &pool.evaluations, &pool.selection, None)?;
        pe_trace(&format!("VYGP|cmoves|{}|{batch_moves}|committed={}|tentative={}\n", pe.batch_index, pe.committed_moves, pe.tentative_moves));
        if batch_moves > 0 {
            self.pe_update_timing(pe)?;
        }
        pe.batch_index += 1;
        let best_wns_before = pe.best_checkpoint_wns;
        let batch_saved = self.pe_judge_batch(pe, &pool, pre_batch_wns, &mut batch_moves)?;
        if pe.tail.active {
            pe.tail.after_batch(best_wns_before, pe.best_checkpoint_wns, batch_saved);
        }
        self.pe_apply_stop_rules(pe, batch_moves)?;
        self.pe_print_tracker_batch_summary(pe)?;
        pe.batch_candidates.clear();
        pe.batch_swaps.clear();
        Ok(())
    }

    /// `judgeBatch`: the QoR after the batch, the stall monitor's batch end, and the verdict — a
    /// batch the checkpoint accepts is saved; a missed one is noted, then salvaged when it can be,
    /// else settled (undone: the WNS tail's speculation is not modelled).
    fn pe_judge_batch(&mut self, pe: &mut ParallelEval, pool: &BatchPool, pre_batch_wns: f32, batch_moves: &mut i32) -> Result<bool, Stop> {
        let (wns, tns) = self.pe_measure_qor();
        pe.stall.end_batch(*batch_moves);
        let utilization_legal = self.pe_within_max_utilization();
        pe_trace(&format!(
            "VYGP|judge|{}|wns={}|tns={}|ck_wns={}|ck_tns={}|best={}|tol={}|util={}\n",
            pe.batch_index,
            e9(wns),
            e9(tns),
            e9(pe.checkpoint_wns),
            e9(pe.checkpoint_tns),
            e9(pe.best_checkpoint_wns),
            e9(pe_checkpoint_wns_tolerance(pe)),
            u8::from(utilization_legal)
        ));
        if utilization_legal && pe_accepts_batch(pe, wns, tns) {
            if !fuzzy::less(pe.checkpoint_wns, wns) {
                pe_event("checkpoints_saved_wns_tolerance", "", "", 1);
            }
            pe_trace(&format!("VYGP|verdict|{}|save\n", pe.batch_index));
            self.pe_save_batch_checkpoint(pe, wns, tns)?;
            return Ok(true);
        }
        self.pe_note_missed_batch(pe, utilization_legal);
        if self.pe_salvage_missed_batch(pe, pool, pre_batch_wns, batch_moves)? {
            pe_trace(&format!("VYGP|verdict|{}|salvaged\n", pe.batch_index));
            return Ok(true);
        }
        pe_trace(&format!("VYGP|verdict|{}|missed\n", pe.batch_index));
        self.pe_settle_missed_batch(pe, *batch_moves)?;
        Ok(false)
    }

    /// `analyzeBottlenecks(scope)`: only while saved batches stall WNS (`weightsPriority`; a
    /// pause makes the next analysis start fresh); one analysis serves
    /// `bottleneck_refresh_batches` batches, and entering the tail makes a fresh one. The cone is
    /// cut at `bottleneck_max_endpoints` of the batch's target endpoints.
    fn pe_analyze_bottlenecks(&mut self, pe: &mut ParallelEval, scope: &BatchScope) -> Result<(), Stop> {
        let priority = pe.config.parallel_eval.priority;
        if !pe.stall.weights_priority(&pe.config.parallel_eval) {
            pe.bottleneck.age = i32::MAX;
            return Ok(());
        }
        let tail_changed = pe.tail.active != pe.bottleneck.in_tail;
        pe.bottleneck.age = pe.bottleneck.age.saturating_add(1);
        if !tail_changed && pe.bottleneck.age < priority.bottleneck_refresh_batches {
            return Ok(());
        }
        pe.bottleneck.age = 0;
        pe.bottleneck.in_tail = pe.tail.active;
        let limit = (priority.bottleneck_max_endpoints.max(0) as usize).min(scope.target_endpoints);
        let ends: Vec<(String, f32)> = self.collector_ends.iter().take(limit).cloned().collect();
        let margin = self.ctx.margin;
        let max_startpoints = priority.bottleneck_max_startpoints.max(0) as usize;
        let edits = self.design.take_timer_edits();
        let ctx = self.ctx;
        let (severity, max_severity, cone) = super::timed_all(ctx, self.design.as_design(), &mut self.timer, edits, |gs, ss, _| {
            bottleneck_analyze(gs, ss, &ends, margin, max_startpoints).map_err(|e| Stop::refused("RSZ-ABSENT", format!("repair_timing -phases PARALLEL_EVAL: the bottleneck analysis: {e}")))
        })?;
        pe.bottleneck.severity = severity;
        pe.bottleneck.max_severity = max_severity;
        pe_event("bottleneck_analyses", "", "", 1);
        pe_event("bottleneck_cone_vertices", "", "", cone);
        Ok(())
    }

    /// `settleMissedBatch(batch_moves)`: while the tail runs, a stalled greedy stage keeps a
    /// missed batch that committed moves (within the area limit and the speculation budget) in
    /// the open journal; otherwise the batch is undone (`undoMissedBatch`).
    fn pe_settle_missed_batch(&mut self, pe: &mut ParallelEval, batch_moves: i32) -> Result<(), Stop> {
        let cfg = pe.config.parallel_eval;
        if pe.tail.active && batch_moves > 0 && self.pe_within_max_utilization() && pe.tail.may_speculate(&cfg) {
            pe_trace(&format!("VYGP|settle|{}|speculate\n", pe.batch_index));
            pe.tail.keep_speculative(&pe.batch_instances);
            pe_event("wns_tail_speculate", "wns_tail_speculative_batches", "", 1);
            return Ok(());
        }
        pe_trace(&format!("VYGP|settle|{}|undo\n", pe.batch_index));
        self.pe_undo_missed_batch(pe)
    }

    /// `undoMissedBatch`: a failed speculative run ends, and every batch of it is undone with
    /// the last.
    fn pe_undo_missed_batch(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        let mut speculative = Vec::new();
        if pe.tail.speculating() {
            pe_event("wns_tail_speculation_failed", "", "", 1);
            speculative = pe.tail.end_speculation();
        }
        self.pe_undo_batch(pe, &speculative)
    }

    /// `undoBatch(speculative)`: the open journal rolled back after the work meter's update; the
    /// speculative run's kept instances go on the tabu list, and the batch's kept instances on it
    /// with their undo count (only the last batch counts toward the undo limit).
    fn pe_undo_batch(&mut self, pe: &mut ParallelEval, speculative: &[InstanceRecord]) -> Result<(), Stop> {
        if !pe.batch_journal_open {
            return Ok(());
        }
        pe_trace(&format!("VYGP|undo|{}|tentative={}|speculative={}|instances={}\n", pe.batch_index, pe.tentative_moves, speculative.len(), pe.batch_instances.len()));
        pe_event("batches_undone", "", "", 1);
        pe_event("moves_undone", "", "", pe.tentative_moves.max(0) as usize);
        self.pe_note_timing_update(pe);
        self.pe_roll_back_journal(pe)?;
        let cfg = pe.config.parallel_eval;
        for record in self.pe_kept_instances(speculative) {
            pe_trace(&format!("VYGP|undo|{}|tabu|spec|{}\n", pe.batch_index, record.name));
            pe.tabu.add_tabu(&cfg, record.id, &record.name, pe.batch_index);
        }
        for record in self.pe_kept_instances(&pe.batch_instances) {
            let blocked = pe.tabu.record_undo(&cfg, record.id, &record.name, pe.batch_index);
            pe_trace(&format!("VYGP|undo|{}|tabu|undo|{}|blocked={}\n", pe.batch_index, record.name, u8::from(blocked)));
            if blocked {
                pe_event("undo_blocked_instances", "", "", 1);
            }
        }
        pe.batch_instances.clear();
        Ok(())
    }

    /// `keptInstances`: the records whose name still finds the instance in the same slot.
    fn pe_kept_instances(&self, records: &[InstanceRecord]) -> Vec<InstanceRecord> {
        records.iter().filter(|r| self.design.inst_id(&r.name) == r.id && r.id != u32::MAX).cloned().collect()
    }

    /// `salvageMissedBatch`: with the journal open and moves committed, no buffer removed (a
    /// removed buffer comes back as a new instance on rollback), and some but not all moves
    /// culprits: the culprits go on the tabu list and are reverted — cell swaps in place, else
    /// the batch rolled back and re-committed without them — then the batch is timed and judged
    /// again; spared victims get a second pass.
    fn pe_salvage_missed_batch(&mut self, pe: &mut ParallelEval, pool: &BatchPool, pre_batch_wns: f32, batch_moves: &mut i32) -> Result<bool, Stop> {
        let b = pe.batch_index;
        if !pe.batch_journal_open || pe.batch_candidates.is_empty() {
            pe_trace(&format!("VYGP|salv|{b}|skip=journal_or_empty\n"));
            return Ok(false);
        }
        if pe.batch_candidates.iter().any(|&k| pool.evaluations[k].best_candidate().is_some_and(|c| c.kind == Move::Unbuffer)) {
            pe_event("batch_salvage_skipped_removal", "", "", 1);
            pe_trace(&format!("VYGP|salv|{b}|skip=removal\n"));
            return Ok(false);
        }
        let batch_size = pe.batch_candidates.len();
        let (culprits, mut victims) = self.pe_find_batch_culprits(pe, pool, pre_batch_wns)?;
        if culprits.is_empty() || culprits.len() >= batch_size {
            pe_trace(&format!("VYGP|salv|{b}|skip=culprits|n={}|batch={batch_size}\n", culprits.len()));
            return Ok(false);
        }
        pe_event("batch_salvage_attempts", "", "", 1);
        pe_event("batch_culprits", "", "", culprits.len());
        let cfg = pe.config.parallel_eval;
        let mut skip: HashSet<usize> = HashSet::new();
        let tabu_moves = |pe: &mut ParallelEval, skip: &mut HashSet<usize>, moves: &[usize], id: &dyn Fn(&str) -> u32| {
            for &k in moves {
                skip.insert(k);
                let inst = &pool.targets[k].inst;
                pe.tabu.add_tabu(&cfg, id(inst), inst, pe.batch_index);
            }
        };
        let design = &self.design;
        tabu_moves(pe, &mut skip, &culprits, &|i| design.inst_id(i));
        let mut in_place = true;
        let mut blocked = culprits;
        for pass in 0..2 {
            pe_trace(&format!("VYGP|salv|{b}|pass={pass}|in_place={}|blocked={}\n", u8::from(in_place), blocked.len()));
            if in_place && self.pe_revert_in_place(pe, pool, &blocked, batch_moves)? {
                pe_event("batch_salvage_in_place", "", "", 1);
            } else {
                // The roll-back would also drop the speculative batches in the same journal.
                if pe.tail.speculating() {
                    pe_event("batch_salvage_skipped_speculating", "", "", 1);
                    pe_trace(&format!("VYGP|salv|{b}|skip=speculating\n"));
                    return Ok(false);
                }
                in_place = false;
                self.pe_note_timing_update(pe);
                self.pe_roll_back_journal(pe)?;
                self.pe_open_batch_journal(pe)?;
                *batch_moves = self.pe_commit_batch(pe, &pool.targets, &pool.evaluations, &pool.selection, Some(&skip))?;
                pe_trace(&format!("VYGP|salv|{b}|recommit={batch_moves}\n"));
                if *batch_moves == 0 {
                    return Ok(false);
                }
            }
            self.pe_update_timing(pe)?;
            let (wns, tns) = self.pe_measure_qor();
            pe_trace(&format!("VYGP|salv|{b}|after|wns={}|tns={}|util={}\n", e9(wns), e9(tns), u8::from(self.pe_within_max_utilization())));
            if self.pe_within_max_utilization() && pe_accepts_batch(pe, wns, tns) {
                pe_event("batches_salvaged", "", "", 1);
                self.pe_save_batch_checkpoint(pe, wns, tns)?;
                return Ok(true);
            }
            if victims.is_empty() || skip.len() + victims.len() >= batch_size {
                break;
            }
            pe_event("batch_salvage_second_pass", "", "", 1);
            let design = &self.design;
            tabu_moves(pe, &mut skip, &victims, &|i| design.inst_id(i));
            blocked = std::mem::take(&mut victims);
        }
        pe_event("batch_salvage_failed", "", "", 1);
        pe_trace(&format!("VYGP|salv|{b}|failed\n"));
        Ok(false)
    }

    /// `findBatchCulprits(pre_batch_wns, spared)`: per committed move in commit order — a move
    /// whose own endpoint ends below the slack it was collected with is a culprit, unless it is
    /// a cell swap whose own stages did not slow down (`selfStageDelayDelta` at or below 0: a
    /// victim); a move not yet a culprit is one when a guarded side pin ends below the
    /// pre-batch WNS. Victims with no culprit are the culprits (the endpoint fallback); else
    /// they are spared. All slacks and delays are read on the timer as the batch left it.
    fn pe_find_batch_culprits(&mut self, pe: &ParallelEval, pool: &BatchPool, pre_batch_wns: f32) -> Result<(Vec<usize>, Vec<usize>), Stop> {
        let ctx = self.ctx;
        let edits = self.design.take_timer_edits();
        let design = self.design.as_design();
        let batch: Vec<usize> = pe.batch_candidates.clone();
        let ratio = pe.config.parallel_eval.culprit_delay_ratio;
        let rows = super::timed_all(ctx, design, &mut self.timer, edits, |gs, ss, _| {
            // A target carries the slack its endpoint was collected with, the group's under
            // -path_group: one group query (up to the batch's highest target slack) covers the
            // batch; an endpoint missing from it is above every target slack (INF).
            let group_slacks: Option<HashMap<String, f32>> = ctx.path_group.map(|name| {
                let slack_max = batch.iter().map(|&k| pool.targets[k].slack).fold(-super::INF, f32::max);
                ss[0].group_path_ends(crate::path_group::sta_group_name(name), MAX, usize::MAX, -super::INF, slack_max).iter().map(|e| (gs[0].vertices[e.vertex].name.clone(), e.slack)).collect()
            });
            let mut rows = Vec::with_capacity(batch.len());
            for &k in &batch {
                let target = &pool.targets[k];
                let candidate = pool.evaluations[k].best_candidate().expect("a committed evaluation has its best candidate");
                let eslack = match &group_slacks {
                    None => pe_load_slack(gs, ss, &target.endpoint_pin),
                    Some(g) => pe_load_slack(gs, ss, &target.endpoint_pin).map(|_| g.get(&target.endpoint_pin).copied().unwrap_or(super::INF)),
                };
                let mut culprit = false;
                let mut victim = false;
                let mut self_delta = None;
                if let Some(es) = eslack {
                    if fuzzy::less(es, target.slack) {
                        culprit = true;
                        self_delta = pe_self_stage_delay_delta(ctx, gs, candidate, target, ratio);
                        if self_delta.is_some_and(|d| d <= 0.0) {
                            victim = true;
                            culprit = false;
                        }
                    }
                }
                let endpoint_culprit = culprit;
                if !culprit {
                    culprit = pe_guarded_side_pins(candidate, target).iter().any(|pin| pe_load_slack(gs, ss, pin).is_some_and(|sl| fuzzy::less(sl, pre_batch_wns)));
                }
                rows.push((k, candidate.kind.name(), target.driver_pin.clone(), eslack, target.slack, self_delta, endpoint_culprit, culprit, victim));
            }
            Ok(rows)
        })?;
        let (mut culprits, mut victims) = (Vec::new(), Vec::new());
        let mut t = String::new();
        for (k, name, pin, eslack, tslack, self_delta, ep, culprit, victim) in rows {
            t.push_str(&format!(
                "VYGP|cul|{}|{name}|{pin}|eslack={}|tslack={}|self={}|ep={}|culprit={}|victim={}\n",
                pe.batch_index,
                eslack.map_or("-".to_string(), e9),
                e9(tslack),
                self_delta.map_or("-".to_string(), e9),
                u8::from(ep),
                u8::from(culprit),
                u8::from(victim)
            ));
            if culprit {
                culprits.push(k);
            } else if victim {
                victims.push(k);
            }
        }
        if !victims.is_empty() {
            pe_event("batch_victims_spared", "", "", victims.len());
            if culprits.is_empty() {
                pe_event("batch_culprits_endpoint_fallback", "", "", 1);
                t.push_str(&format!("VYGP|culres|{}|culprits={}|victims=0|fallback=1\n", pe.batch_index, victims.len()));
                pe_trace(&t);
                return Ok((victims, Vec::new()));
            }
        }
        t.push_str(&format!("VYGP|culres|{}|culprits={}|victims={}|fallback=0\n", pe.batch_index, culprits.len(), victims.len()));
        pe_trace(&t);
        Ok((culprits, victims))
    }

    /// `revertInPlace(moves, batch_moves)`: every move must be a recorded cell swap (else
    /// nothing is reverted); each not yet reverted goes back to its old cell (`replaceCell`, in
    /// the open journal) and its pending accounting is dropped (`rejectPendingCellMove`).
    fn pe_revert_in_place(&mut self, pe: &mut ParallelEval, pool: &BatchPool, moves: &[usize], batch_moves: &mut i32) -> Result<bool, Stop> {
        let mut swaps = Vec::with_capacity(moves.len());
        for &k in moves {
            let Some(at) = pe.batch_swaps.iter().position(|sw| sw.evaluation_index == k) else {
                pe_trace(&format!("VYGP|revert|{}|no_swap\n", pe.batch_index));
                return Ok(false);
            };
            if !pe.batch_swaps[at].reverted {
                swaps.push(at);
            }
        }
        for &at in &swaps {
            let sw = pe.batch_swaps[at].clone();
            self.design.swap_master(&sw.inst, &sw.before_cell).map_err(|e| Stop::error("RSZ-2037", format!("PARALLEL_EVAL salvage could not revert a cell swap: {e}")))?;
            // `setLocation(x, y)` back to where it was: a cell swap does not move the instance.
            let kind = pool.evaluations[sw.evaluation_index].best_candidate().expect("a recorded swap is a committed evaluation's best candidate").kind;
            self.pe_reject_pending_cell_move(&sw.result, pe_result_type(kind), &pool.targets[sw.evaluation_index].driver_pin)?;
            pe.batch_swaps[at].reverted = true;
        }
        let reverted = swaps.len() as i32;
        pe_trace(&format!("VYGP|revert|{}|n={reverted}\n", pe.batch_index));
        *batch_moves -= reverted;
        pe.committed_moves -= reverted;
        pe.tentative_moves -= reverted;
        pe_event("batch_salvage_in_place_reverts", "", "", reverted.max(0) as usize);
        Ok(true)
    }

    /// `MoveCommitter::rejectPendingCellMove(result, driver_pin)`: the latest matching accepted
    /// size or VT swap in the open level unrecorded, the tracker's pending move of the driver
    /// rejected, and the result dropped from the level. The legacy rejected-move count feeds only
    /// the `opt_moves` TOTAL debug line, which is not printed: not kept. A false return is
    /// RSZ-2038 at the caller.
    fn pe_reject_pending_cell_move(&mut self, result: &ApplyResult, kind: Move, driver_pin: &str) -> Result<(), Stop> {
        if !result.accepted || result.move_count != 1 || result.touched.len() != 1 || !matches!(kind, Move::SizeUp | Move::VtSwap) {
            return Err(Stop::error("RSZ-2038", "PARALLEL_EVAL salvage lost its pending cell move".into()));
        }
        let ids: Vec<u32> = result.touched.iter().map(|i| self.design.inst_id(i)).collect();
        let Some(level) = self.committer.levels.last_mut() else { return Err(Stop::error("RSZ-2038", "PARALLEL_EVAL salvage lost its pending cell move".into())) };
        let Some(at) = level.iter().rposition(|(r, rid)| r.kind == kind && r.count == 1 && *rid == ids) else {
            return Err(Stop::error("RSZ-2038", "PARALLEL_EVAL salvage lost its pending cell move".into()));
        };
        let (r, rid) = level[at].clone();
        self.committer.unrecord(&r, &rid);
        self.trk_reject_pending(driver_pin, kind)?;
        if let Some(level) = self.committer.levels.last_mut() {
            level.remove(at);
        }
        Ok(())
    }

    /// `measureQor`: the collector's worst slack and TNS.
    fn pe_measure_qor(&self) -> (f32, f32) {
        (self.timing.worst().0, self.timing.tns())
    }

    /// `finishIfNoSetupViolations`: WNS at or above the margin ends the run, an open journal
    /// saved first; the run complete with result true.
    fn pe_finish_if_no_setup_violations(&mut self, pe: &mut ParallelEval) -> Result<bool, Stop> {
        let (wns, tns) = self.pe_measure_qor();
        if fuzzy::less(wns, self.ctx.margin) {
            return Ok(false);
        }
        if pe.batch_journal_open {
            self.pe_save_batch_checkpoint(pe, wns, tns)?;
        }
        pe_trace(&format!("VYGP|end|{}|no_setup_violations|journal={}\n", pe.batch_index, u8::from(pe.batch_journal_open)));
        pe_stop("no_setup_violations");
        pe_mark_run_complete(pe, true);
        Ok(true)
    }

    /// `finishAtMaxArea`: as LEGACY, nothing runs at the `-max_utilization` area. The verdict
    /// keeps batches below it, so normally only a design that starts there stops here.
    fn pe_finish_at_max_area(&mut self, pe: &mut ParallelEval) -> Result<bool, Stop> {
        if self.pe_within_max_utilization() {
            return Ok(false);
        }
        self.pe_stop_at_checkpoint(pe, "max_area")?;
        Ok(true)
    }

    /// `withinMaxUtilization`: `!overMaxArea()`; without `-max_utilization` any area is legal.
    fn pe_within_max_utilization(&self) -> bool {
        !self.over_max_area()
    }

    /// `noteMissedBatch(utilization_legal)`: the miss counted against the stall budget; one over
    /// the area limit is also a utilization reject, and the first warns (RSZ-2033): the undone
    /// batches leave the design below the limit, so nothing else would show it. The area it names
    /// is the progress table's (`computeDesignArea`), not the tracked one.
    fn pe_note_missed_batch(&mut self, pe: &mut ParallelEval, utilization_legal: bool) {
        pe.checkpoint_misses += 1;
        pe_trace(&format!("VYGP|miss|{}|misses={}|util={}\n", pe.batch_index, pe.checkpoint_misses, u8::from(utilization_legal)));
        pe.stall.on_missed();
        pe_event("checkpoint_misses", "", "", 1);
        if !utilization_legal {
            pe_event("checkpoint_reject_utilization", "", "", 1);
            if !pe.max_utilization_warned {
                pe.max_utilization_warned = true;
                let core_area = self.design.core_area();
                let line = format!(
                    "[WARNING RSZ-2033] PARALLEL_EVAL reached the -max_utilization limit of {:.2}%: batch {} would grow the design area to {:.1} um^2, over the {:.1} um^2 limit. Batches over the limit are undone.",
                    100.0 * self.max_area / core_area,
                    pe.batch_index,
                    self.design.design_area() * 1e12,
                    self.max_area * 1e12
                );
                self.report(line);
            }
        }
    }

    /// `stopAtCheckpoint(reason)`: the open batch (or speculative run) undone, a running tail
    /// stopped, the run complete with result false.
    fn pe_stop_at_checkpoint(&mut self, pe: &mut ParallelEval, reason: &str) -> Result<(), Stop> {
        pe_trace(&format!("VYGP|end|{}|{reason}|at_checkpoint\n", pe.batch_index));
        pe_stop(reason);
        self.pe_restore_batch_checkpoint(pe)?;
        if pe.tail.active {
            pe_event("wns_tail_batches", "", "", pe.tail.batches.max(0) as usize);
            pe.tail.active = false;
        }
        pe_mark_run_complete(pe, false);
        Ok(())
    }

    /// `finishWithoutTargets`.
    fn pe_finish_without_targets(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        pe_trace(&format!("VYGP|end|{}|no_targets\n", pe.batch_index));
        pe_stop("no_targets");
        self.pe_restore_batch_checkpoint(pe)?;
        pe_mark_run_complete(pe, false);
        Ok(())
    }

    /// `openBatchJournal`: the committer's journal begun once per open batch run.
    fn pe_open_batch_journal(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        if !pe.batch_journal_open {
            self.begin_journal()?;
            pe.batch_journal_open = true;
        }
        Ok(())
    }

    /// `commitBatch(evaluations, selection)`: each selected evaluation's best candidate in
    /// selection order — its instance's name read before the move, the swap remembered, the
    /// candidate committed (`MoveCommitter::commit`: `apply`, then the result recorded in the
    /// open level); an accepted move keeps its record, its swap when the move touched only its
    /// instance once, and is tracked as an attempt after the apply.
    fn pe_commit_batch(&mut self, pe: &mut ParallelEval, targets: &[Target], evaluations: &[TargetEvaluation], selection: &BatchSelection, skip: Option<&HashSet<usize>>) -> Result<i32, Stop> {
        let mut batch_moves = 0;
        pe.batch_instances.clear();
        pe.batch_candidates.clear();
        pe.batch_swaps.clear();
        for &evaluation_index in &selection.evaluation_indices {
            let evaluation = &evaluations[evaluation_index];
            let Some(candidate) = evaluation.best_candidate() else { continue };
            if skip.is_some_and(|sk| sk.contains(&evaluation_index)) {
                pe_trace(&format!("VYGP|cskip|{}|{evaluation_index}\n", pe.batch_index));
                continue;
            }
            let target = &targets[evaluation_index];
            let record = InstanceRecord { name: target.inst.clone(), id: self.design.inst_id(&target.inst) };
            let swap = pe_record_swap(candidate, target, evaluation_index);
            let result = self.pe_commit(candidate, target)?;
            let front = result.touched.first().is_some_and(|i| *i == target.inst);
            let keep = swap.is_some() && result.move_count == 1 && result.touched.len() == 1 && front;
            pe_trace(&format!(
                "VYGP|cmt|{}|{evaluation_index}|{}|{}|accepted={}|moves={}|touched={}|front={}|swap={}|kept={}\n",
                pe.batch_index,
                candidate.kind.name(),
                record.name,
                u8::from(result.accepted),
                result.move_count,
                result.touched.len(),
                u8::from(front),
                u8::from(swap.is_some()),
                u8::from(result.accepted && keep)
            ));
            if !result.accepted {
                continue;
            }
            pe.batch_instances.push(record);
            pe.batch_candidates.push(evaluation_index);
            if let (true, Some(mut swap)) = (keep, swap) {
                swap.result = result.clone();
                pe.batch_swaps.push(swap);
            }
            if self.tracker_level() >= 2 {
                self.trk_set_current_endpoint(&target.endpoint_pin)?;
                self.trk_attempt(&target.driver_pin, candidate.kind)?;
            }
            batch_moves += 1;
            pe.committed_moves += 1;
            pe.tentative_moves += 1;
        }
        Ok(batch_moves)
    }

    /// `MoveCommitter::commit(candidate)`: `apply`, then an accepted result recorded and kept in
    /// the open journal level.
    fn pe_commit(&mut self, candidate: &Candidate, target: &Target) -> Result<ApplyResult, Stop> {
        let result = self.pe_apply(candidate, target)?;
        if result.accepted {
            self.commit(super::MoveResult { kind: pe_result_type(candidate.kind), count: result.move_count, insts: result.touched.clone() });
        }
        Ok(result)
    }

    /// The candidates' `apply`, per move (`*MtCandidate::apply`).
    fn pe_apply(&mut self, c: &Candidate, target: &Target) -> Result<ApplyResult, Stop> {
        let accepted = |touched: String, move_count: i64| ApplyResult { accepted: true, move_count, touched: vec![touched] };
        let rejected = ApplyResult::default();
        match c.kind {
            // SizeUpMtCandidate: the max-capacitance check again on the state the batch has made
            // (a REJECT), then `replaceCell`.
            Move::SizeUp => {
                if !self.pe_replacement_preserves_max_cap(target, &c.desc)? {
                    self.debug("opt_moves", 1, format!("REJECT size_up_mt {}: {} -> {} max-cap re-check failed", target.driver_pin, target.cell, c.desc));
                    return Ok(rejected);
                }
                self.design.swap_master(&target.inst, &c.desc).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
                self.debug("opt_moves", 1, format!("ACCEPT size_up_mt {}: {} -> {}", target.driver_pin, target.cell, c.desc));
                Ok(accepted(target.inst.clone(), 1))
            }
            // VtSwapMtCandidate: `replaceCell`.
            Move::VtSwap => {
                self.design.swap_master(&target.inst, &c.desc).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
                self.debug("opt_moves", 1, format!("ACCEPT vt_swap_mt {}: {} -> {}", target.driver_pin, target.cell, c.desc));
                Ok(accepted(target.inst.clone(), 1))
            }
            // SwapPinsMtCandidate: the ACCEPT line, then `swapPins(driver, input, swap port)`.
            Move::SwapPins => {
                let Some(input) = &target.swap_pins_input else { return Err(Stop::error("RSZ-SWAP", format!("{}: a SwapPins candidate without its input", target.driver_pin))) };
                self.debug("swap_pins_move", 1, format!("ACCEPT SwapPinsMove {}: Cell {}, pins {} <-> {}", target.driver_pin, target.cell, input.input_port, c.desc));
                match self.design.swap_pins(&target.inst, &input.input_port, &c.desc).map_err(|e| Stop::error("RSZ-SWAP", e))? {
                    super::PinSwap::DontTouch(_) => Ok(rejected),
                    super::PinSwap::Swapped | super::PinSwap::NoNet => Ok(accepted(target.inst.clone(), 1)),
                }
            }
            // UnbufferMtCandidate: `removeBuffer(buffer)`.
            Move::Unbuffer => {
                self.pe_remove_buffer(&target.inst, &target.cell)?;
                Ok(accepted(target.inst.clone(), 1))
            }
            // SizeDownFanoutMtCandidate: the load's `replaceCell` to the step down.
            Move::SizeDownFanout => {
                let Some(load) = target.size_down_fanout_loads.iter().find(|l| format!("{}/{}", l.pin, if l.step_down.cell.is_empty() { "-" } else { &l.step_down.cell }) == c.desc) else {
                    return Err(Stop::error("RSZ-REPLACE", format!("{}: SizeDownFanout candidate {} without its load", target.driver_pin, c.desc)));
                };
                self.design.swap_master(&load.inst, &load.step_down.cell).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
                self.debug("size_down_fanout_move", 3, format!("ACCEPT SizeDownFanoutMove {} -> {}: ({} -> {}) slack={}", target.driver_pin, load.pin, load.cell, load.step_down.cell, crate::repair_timing::delay_as_string(load.load_slack, 3, self.ctx.time_scale)));
                Ok(accepted(load.inst.clone(), 1))
            }
            // SplitLoadMtCandidate: a buffer before a copy of the loads, then its resize to the
            // target slew (the insertion invalidates both nets).
            Move::SplitLoad => {
                let Some(input) = &target.split_load_input else { return Err(Stop::error("RSZ-INSERT", format!("{}: a SplitLoad candidate without its input", target.driver_pin))) };
                let rep = match self.design.insert_buffer_before_loads(Some(&input.drvr_net), &input.load_pins, &input.buffer_cell, input.drvr_loc, "split", false, "IF_NEEDED") {
                    Ok(r) => r,
                    Err(_) => {
                        self.debug("split_load_move", 2, format!("REJECT SplitLoadMove {}: Couldn't insert buffer", target.driver_pin));
                        return Ok(rejected);
                    }
                };
                self.debug("split_load_move", 1, format!("ACCEPT SplitLoadMove {}: Inserted buffer {}", target.driver_pin, rep.inst));
                let cell = input.buffer_cell.clone();
                self.resize_to_target_slew(&rep, &cell)?;
                Ok(accepted(rep.inst, 1))
            }
            // RebufferMtCandidate: `applyRebufferTreePlan(plan, "rebuffer_mt")` (exportBufferTree
            // on the driver's flat net); accepted when it inserted any buffer, touching the driver.
            Move::Buffer => {
                let Some((nodes, root)) = c.tree.clone() else { return Err(Stop::error("RSZ-REBUFFER", format!("{}: a RebufferMt candidate without its tree", target.driver_pin))) };
                pe_event("buffer_tree_apply_requests", "", "", 1);
                let d = self.design.as_design();
                let Some(net) = d.netlist().nets.iter().find(|n| n.pins.iter().any(|p| d.netlist().pin_name(p) == target.driver_pin)).map(|n| n.name.clone()) else { return Ok(rejected) };
                let (mut net, mut count, mut inputs, mut loads) = (net, 0i64, HashMap::new(), Vec::new());
                self.export_buffer_tree(&nodes, root, &mut net, &mut loads, &mut count, &mut inputs, "rebuffer_mt")?;
                if count <= 0 {
                    self.debug("rebuffer_mt", 1, format!("REJECT rebuffer_mt {}: exportBufferTree failed", target.driver_pin));
                    return Ok(rejected);
                }
                Ok(accepted(target.inst.clone(), count))
            }
            other => Err(Stop::refused("RSZ-ABSENT", format!("repair_timing -phases PARALLEL_EVAL: the apply of {} is not modelled", other.name()))),
        }
    }

    /// `Resizer::replacementPreservesMaxCap(inst, replacement, mode)` at the apply, on the state
    /// the batch has made: for each input pin of the instance with drivers — a register clock
    /// pin of an ideal clock and a scan pin skipped — a capacitance growth must fit every
    /// driver's `maxCapIncrease` (`checkCapacitance` over the scenes now). While no earlier move
    /// of the batch edited one of the instance's fanin nets, nothing it reads changed since
    /// `prepareInputCapBudgets`, whose stricter budgets the candidate already passed: true. Else
    /// the timer is asked on the edits so far (the reference's never rejects in the corpus: the
    /// reject branch is unwitnessed).
    fn pe_replacement_preserves_max_cap(&mut self, target: &Target, replacement: &str) -> Result<bool, Stop> {
        let edited = self.design.pending_nets();
        let nl = self.design.as_design().netlist();
        let fanin_edited = nl.nets.iter().any(|n| edited.contains(&n.name) && n.pins.iter().any(|p| matches!(p, vyges_sta::netlist::Conn::Inst(k, _) if nl.insts[*k].0 == target.inst)));
        if !fanin_edited {
            return Ok(true);
        }
        let ctx = self.ctx;
        let (Some(current), Some(rep)) = (ctx.libs.link_cell(&target.cell), ctx.libs.link_cell(replacement)) else { return Ok(true) };
        let inst = target.inst.clone();
        let edits = self.design.take_timer_edits();
        let design = self.design.as_design();
        super::timed_all(ctx, design, &mut self.timer, edits, |gs, _, _| {
            let g = &gs[0];
            let sc = crate::timing::Scenes::new(gs);
            let parasitics: Vec<&HashMap<String, vyges_sta::graph::NetParasitics>> = (0..gs.len()).map(|k| design.parasitics(k)).collect();
            let ideal: std::collections::BTreeSet<usize> = g.ideal_clock.iter().copied().collect();
            for (u, ux) in g.vertices.iter().enumerate() {
                if ux.is_driver || !matches!(&ux.conn, vyges_sta::netlist::Conn::Inst(k, _) if g.netlist.insts[*k].0 == inst) {
                    continue;
                }
                let Some(port) = ux.port.as_deref() else { continue };
                if !current.port(port).is_some_and(|p| matches!(p.direction, vyges_sta::liberty::Direction::Input | vyges_sta::liberty::Direction::Bidirect)) {
                    continue;
                }
                let reg_clk = current.arc_sets.iter().any(|a| a.from == port && matches!(a.role, vyges_sta::liberty::Role::RegClkToQ | vyges_sta::liberty::Role::LatchEnToQ));
                if super::max_cap_skips_pin(reg_clk, Some(ideal.contains(&u))) || super::is_scan_pin(current, port) {
                    continue;
                }
                let cap_delta = super::input_pin_capacitance(rep, port) - super::input_pin_capacitance(current, port);
                if cap_delta <= 0.0 {
                    continue;
                }
                for d in crate::timing::net_drivers(g, u) {
                    let (cap, max_cap, slack, limited, _) = crate::timing::check_capacitance(&sc, d, &parasitics, &ideal, ctx.limits);
                    if cap_delta > super::max_cap_increase(cap, max_cap, slack, limited) {
                        return Ok(false);
                    }
                }
            }
            Ok(true)
        })
    }

    /// `Resizer::removeBuffer(buffer)` as the serial Unbuffer applies it: its debug lines, the
    /// nets named as they stand before the removal.
    fn pe_remove_buffer(&mut self, inst: &str, cell: &str) -> Result<(), Stop> {
        let Some((i, o)) = self.ctx.libs.link_cell(cell).and_then(|c| c.buffer_ports()).map(|(i, o)| (i.name.clone(), o.name.clone())) else {
            return Err(Stop::error("RSZ-0168", format!("{inst}: {cell} has no buffer ports")));
        };
        self.debug("repair_setup", 3, format!("remove_buffer {inst} ({cell})"));
        let (in_name, out_name) = (self.net_display_name(inst, &i), self.net_display_name(inst, &o));
        let removed = self.design.remove_buffer(inst, &i, &o).map_err(|e| Stop::error("RSZ-0168", e))?;
        let h = self.ctx.hierarchy;
        let in_name = in_name.unwrap_or_else(|| crate::repair_timing::network_name(&removed.in_net, h).to_string());
        let out_name = out_name.unwrap_or_else(|| crate::repair_timing::network_name(&removed.out_net, h).to_string());
        self.debug("remove_buffer", 1, format!("remove_buffer {in_name} (input net) - {inst} ({cell}) - {out_name} (output net)"));
        Ok(())
    }

    /// `noteTimingUpdate`: the work meter's timing update with the estimator's pending nets.
    fn pe_note_timing_update(&mut self, pe: &mut ParallelEval) {
        let pending = self.design.pending_nets().len();
        pe.work.add_timing_update(pending);
    }

    /// `updateTiming`: `noteTimingUpdate`, then `updateParasiticsAndTiming`.
    fn pe_update_timing(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        self.pe_note_timing_update(pe);
        self.retime(&[])
    }

    /// `saveBatchCheckpoint(wns, tns)`: the open journal committed (timing is current), the
    /// stall monitor's save with the work so far, the checkpoint and the best WNS moved, the
    /// tentative moves and the misses cleared.
    fn pe_save_batch_checkpoint(&mut self, pe: &mut ParallelEval, wns: f32, tns: f32) -> Result<(), Stop> {
        pe_trace(&format!("VYGP|ckpt|{}|wns={}|tns={}|journal={}\n", pe.batch_index, e9(wns), e9(tns), u8::from(pe.batch_journal_open)));
        pe_event("checkpoints_saved", "", "", 1);
        if pe.tail.speculating() {
            pe_event("wns_tail_speculation_paid", "", "", 1);
            pe_event("wns_tail_speculation_paid_batches", "", "", pe.tail.speculative_batches.max(0) as usize);
            pe.tail.end_speculation();
        }
        if pe.batch_journal_open {
            self.commit_journal()?;
            pe.batch_journal_open = false;
        }
        let cfg = pe.config.parallel_eval;
        let work = pe.work.seconds(&cfg);
        let stalled = pe.stall.on_saved(&cfg, pe.checkpoint_tns, pe.best_checkpoint_wns, wns, tns, work);
        pe_event("stall_moves_after_save", "", "", stalled.max(0) as usize);
        pe.checkpoint_wns = wns;
        pe.checkpoint_tns = tns;
        pe.best_checkpoint_wns = pe.best_checkpoint_wns.max(wns);
        pe.tentative_moves = 0;
        pe.checkpoint_misses = 0;
        Ok(())
    }

    /// `restoreBatchCheckpoint`: an open journal rolled back, after the work meter's update.
    fn pe_restore_batch_checkpoint(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        if !pe.batch_journal_open {
            return Ok(());
        }
        pe_trace(&format!("VYGP|restore|{}|tentative={}\n", pe.batch_index, pe.tentative_moves));
        pe_event("checkpoints_restored", "", "", 1);
        self.pe_note_timing_update(pe);
        self.pe_roll_back_journal(pe)
    }

    /// `rollBackJournal`: the committer's restore, the tracked area counted again under a limit
    /// (`recountDesignAreaAfterUndo`: `init` counted it before the undo), the tentative moves
    /// uncounted.
    fn pe_roll_back_journal(&mut self, pe: &mut ParallelEval) -> Result<(), Stop> {
        self.restore_journal(&[])?;
        self.recount_design_area_after_undo();
        pe.batch_journal_open = false;
        pe.committed_moves -= pe.tentative_moves;
        pe.tentative_moves = 0;
        Ok(())
    }

    /// `applyStopRules(batch_moves)`: no violation left finishes; `-max_iterations` stops at the
    /// checkpoint; then (no tail runs) no committed move, a stalled window, a stall, or the
    /// misses at their limit end the batches (`startTailOrFinish`).
    fn pe_apply_stop_rules(&mut self, pe: &mut ParallelEval, batch_moves: i32) -> Result<(), Stop> {
        let cfg = pe.config.parallel_eval;
        let stall = pe.stall.stall(&cfg);
        pe_trace(&format!("VYGP|rule|{}|moves={batch_moves}|misses={}|stall={}|window={}|tail={}|committed={}\n", pe.batch_index, pe.checkpoint_misses, stall as i32, u8::from(pe.stall.window_stalled), u8::from(pe.tail.active), pe.committed_moves));
        if self.pe_finish_if_no_setup_violations(pe)? {
            return Ok(());
        }
        if self.args.max_iterations > 0 && i64::from(pe.batch_index) >= self.args.max_iterations {
            return self.pe_stop_at_checkpoint(pe, "max_iterations");
        }
        if pe.tail.active {
            return self.pe_apply_tail_stop_rules(pe, batch_moves);
        }
        if batch_moves == 0 {
            return self.pe_start_tail_or_finish(pe, "no_committed_moves");
        }
        if pe.stall.window_stalled {
            return self.pe_start_tail_or_finish(pe, "stalled_window");
        }
        match stall {
            Stall::Moves => return self.pe_start_tail_or_finish(pe, "stalled_moves"),
            Stall::Batches => return self.pe_start_tail_or_finish(pe, "stalled_batches"),
            Stall::None => {}
        }
        if pe.checkpoint_misses >= cfg.max_checkpoint_misses {
            return self.pe_start_tail_or_finish(pe, "max_checkpoint_misses");
        }
        Ok(())
    }

    /// `startTailOrFinish(reason)`: a run that kept no move retries with small batches first
    /// (not modelled); else the WNS tail (not modelled) when its config allows one; else the run
    /// ends, complete with result false.
    fn pe_start_tail_or_finish(&mut self, pe: &mut ParallelEval, reason: &str) -> Result<(), Stop> {
        pe_trace(&format!("VYGP|end|{}|{reason}|tail_or_finish|retry_started={}|committed={}\n", pe.batch_index, u8::from(pe.small_batch_retry_started), pe.committed_moves));
        if !pe.small_batch_retry_started && pe.committed_moves == 0 {
            pe_trace(&format!("VYGP|retry|{}\n", pe.batch_index));
            pe_start_small_batch_retry(pe);
            return Ok(());
        }
        if self.pe_start_wns_tail(pe, reason) {
            pe_trace(&format!("VYGP|tail|{}|start\n", pe.batch_index));
            return Ok(());
        }
        pe_trace(&format!("VYGP|end|{}|{reason}|finish\n", pe.batch_index));
        pe_stop(reason);
        pe_mark_run_complete(pe, false);
        Ok(())
    }

    /// `startWnsTail(reason)`: the tail started (once, with tail endpoints configured) at the
    /// work so far; misses cleared, the temporary tabu dropped (the tail works on exactly the
    /// instances the stalled loop left there), the tail's generators added.
    fn pe_start_wns_tail(&mut self, pe: &mut ParallelEval, reason: &str) -> bool {
        let cfg = pe.config.parallel_eval;
        let work = pe.work.seconds(&cfg);
        if !pe.tail.start(&cfg, work, pe.reference_period) {
            return false;
        }
        pe.checkpoint_misses = 0;
        pe.tabu.clear_temporary();
        self.pe_add_tail_generators(pe);
        pe_event("wns_tail_begin", "wns_tail_starts", "", 1);
        self.debug("repair_setup", 1, format!("PARALLEL_EVAL WNS tail after {reason} at batch {} (WNS {})", pe.batch_index, e3(pe.checkpoint_wns)));
        true
    }

    /// `addTailGenerators`: pin swaps, buffer removal and size down join the tail when the
    /// default sequence left them out (`tailMayAddMove`: no explicit -sequence, the move's
    /// -skip_* option not given, no generator for it yet).
    fn pe_add_tail_generators(&mut self, pe: &mut ParallelEval) {
        for m in [Move::SwapPins, Move::Unbuffer, Move::SizeDownFanout] {
            if self.args.sequence.is_empty() && self.pe_move_enabled(m) && !pe.generators.contains(&m) {
                pe.generators.push(m);
                pe_event("wns_tail_generator", "", m.name(), 1);
            }
        }
    }

    /// `applyTailStopRules(batch_moves)`: the tail's next step — continue; speculate on the
    /// stalled stage; widen it (the checkpoint restored first: that also drops an open
    /// speculative run); or stop at the checkpoint.
    fn pe_apply_tail_stop_rules(&mut self, pe: &mut ParallelEval, batch_moves: i32) -> Result<(), Stop> {
        let cfg = pe.config.parallel_eval;
        let (step, reason) = pe.tail.next_step(&cfg, batch_moves, pe.work.seconds(&cfg));
        pe_trace(&format!("VYGP|tail|{}|step={}|reason={}\n", pe.batch_index, step as i32, reason.unwrap_or("-")));
        match step {
            TailStep::Continue => return Ok(()),
            TailStep::Speculate => {
                pe.tail.begin_speculative_stage();
                pe.tabu.clear_temporary();
                pe_event("wns_tail_speculative_stage", "wns_tail_speculative_stages", "", 1);
                return Ok(());
            }
            TailStep::Widen => {
                self.pe_restore_batch_checkpoint(pe)?;
                pe.tail.widen(&cfg);
                pe.tabu.clear_temporary();
                pe_event("wns_tail_grow", "", "", 1);
                return Ok(());
            }
            TailStep::Stop => {}
        }
        let reason = reason.expect("a stop step always names its rule");
        pe_event("wns_tail_batches", "", "", pe.tail.batches.max(0) as usize);
        pe_stop(reason);
        self.pe_restore_batch_checkpoint(pe)?;
        pe.tail.active = false;
        pe_mark_run_complete(pe, false);
        Ok(())
    }

    /// `printTrackerBatchSummary`: the tracker's batch summary and endpoint profile once the
    /// journal is closed (the verdict saved or undid the batch).
    fn pe_print_tracker_batch_summary(&mut self, pe: &ParallelEval) -> Result<(), Stop> {
        if !pe.batch_journal_open {
            self.trk_phase_summary("PARALLEL_EVAL Batch Summary", Some("PARALLEL_EVAL Batch Endpoint Profiler"))?;
        }
        Ok(())
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
        // -repair_tns and the WNS tail visit only the worst endpoints.
        let mut target_endpoints = failing_endpoint_count.min(pe.max_end_repairs);
        if pe.tail.active {
            target_endpoints = target_endpoints.min(pe.tail.endpoints.max(0) as usize);
        }
        let batch_cap = pe_effective_batch_cap(pe, target_endpoints);
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

    /// `evaluateTargets`: the endpoints evaluated worst first, a chunk at a time — prewarm,
    /// prepare, adopt, generate, estimate, append — and the batch selected after every chunk; the
    /// next chunk only while the batch is not full.
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
                let generated = self.pe_generate_candidates(pe, &mut chunk)?;
                let evaluations = pe_estimate_targets(pe, &chunk, generated);
                if let Some(m) = pe.generators.iter().find(|&&m| !generate_modelled(m)) {
                    return Err(Stop::refused("RSZ-ABSENT", format!("repair_timing -phases PARALLEL_EVAL: candidate generation for {} (generate, estimate) is not modelled", m.name())));
                }
                pool.adopt(chunk);
                pool.append(evaluations);
            }
            // The selection so far decides whether another chunk is needed; a batch cut short by
            // the score floor still takes another chunk.
            pe_select_batch(pe, &mut pool, scope.batch_cap)?;
            if next_endpoint >= scope.target_endpoints || pool.selection.evaluation_indices.len() as i32 >= scope.batch_cap {
                break;
            }
            pe_event("batch_eval_chunks_extended", "", "", 1);
        }
        pe.work.add_endpoints(next_endpoint);
        pe_event("batch_endpoints_evaluated", "", "", next_endpoint);
        if !pool.targets.is_empty() {
            pe_record_selection(&pool);
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
                let seq_lines = pe_prepare_target(target, mask, gs, ss, cs, design, ctx, &PrepareInputs { equiv, dont_use, pin_id: &pin_id, crit, cfg: &cfg, levels, slew_bias, wns })?;
                // The estimator's own lines print inside `prepare`, before the target's.
                trace.push_str(&seq_lines);
                trace.push_str(&pe_prepared_lines(target, mask, ctx.libs));
            }
            Ok(())
        })?;
        pe_trace(&trace);
        Ok(())
    }

    /// `generateCandidates`: `planGeneration` on each target in chunk order (the skip rules, then
    /// each generator's `generatorAllowed`, `isApplicable` and `prepareInputs`, all on the timer as
    /// preparation left it), then each task's generators in plan order, collected in target then
    /// generator order (`collectGenerated`). A run with a generator whose planning is not modelled
    /// yet is refused.
    fn pe_generate_candidates(&mut self, pe: &mut ParallelEval, targets: &mut [Target]) -> Result<Vec<Vec<Candidate>>, Stop> {
        let edits = self.design.take_timer_edits();
        let ctx = self.ctx;
        let facts: Vec<PlanFacts> = targets.iter().map(|t| self.pe_plan_facts(t)).collect();
        let design = self.design.as_design();
        let mut trace = String::new();
        let committer = &self.committer;
        let setup_design = &self.design;
        let inst_id = |inst: &str| setup_design.inst_id(inst);
        let cfg = pe.config;
        let generated = super::timed_all(ctx, design, &mut self.timer, edits, |gs, ss, _cs| {
            let names = ImpactNames::new(&gs[0].netlist, ctx.master_pins);
            let plan = PlanTimer { gs, ss, design, ctx, committer, inst_id: &inst_id, names: &names };
            for (target, facts) in targets.iter_mut().zip(&facts) {
                pe_plan_generation(pe, target, facts, &plan, &mut trace).map_err(|e| Stop::refused("RSZ-ABSENT", e))?;
            }
            let mut rb = String::new();
            let generated = targets.iter().map(|target| pe_run_generators(target, &plan, &cfg, &mut rb)).collect::<Result<Vec<_>, Stop>>();
            trace.push_str(&rb);
            Ok(generated)
        })?;
        // A generator that reaches an unported part refuses the run after the plan lines print.
        let generated = match generated {
            Ok(g) => g,
            Err(e) => {
                pe_trace(&trace);
                return Err(e);
            }
        };
        pe_collect_generated(pe, targets, &generated, &mut trace);
        pe_trace(&trace);
        if let Some(m) = pe.generators.iter().find(|&&m| !plan_modelled(m)) {
            return Err(Stop::refused("RSZ-ABSENT", format!("repair_timing -phases PARALLEL_EVAL: planGeneration for {} (isApplicable, prepareInputs) is not modelled", m.name())));
        }
        Ok(generated.into_iter().map(|per_generator| per_generator.into_iter().flatten().collect()).collect())
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
fn pe_effective_batch_cap(pe: &ParallelEval, target_endpoints: usize) -> i32 {
    let cfg = pe.config.parallel_eval;
    if pe.tail.active {
        return pe.tail.batch_cap;
    }
    if pe.small_batch_retry_started {
        return cfg.retry_batch_cap;
    }
    adaptive_batch_size(target_endpoints, cfg.min_batch_cap, cfg.max_batch_cap)
}



/// `checkpointWnsTolerance`: `checkpoint_wns_tolerance_period_pct` of the reference period; 0
/// while the WNS tail runs (it works on the worst path and allows no drift).
fn pe_checkpoint_wns_tolerance(pe: &ParallelEval) -> f32 {
    if pe.tail.active {
        return 0.0;
    }
    pe.reference_period * pe.config.parallel_eval.checkpoint_wns_tolerance_period_pct / 100.0
}

/// `batchImprovesCheckpoint`: WNS strictly better than the checkpoint's; else, WNS within the
/// band (fuzzily equal to the checkpoint's, or at least the best less the tolerance), TNS
/// strictly better.
pub fn batch_improves_checkpoint(checkpoint_wns: f32, checkpoint_tns: f32, current_wns: f32, current_tns: f32, best_wns: f32, wns_tolerance: f32) -> bool {
    if fuzzy::less(checkpoint_wns, current_wns) {
        return true;
    }
    let wns_within_band = fuzzy::equal(checkpoint_wns, current_wns) || fuzzy::less_equal(best_wns - wns_tolerance, current_wns);
    wns_within_band && fuzzy::less(checkpoint_tns, current_tns)
}

/// `acceptsBatch`: the batch improves the checkpoint; while the tail runs (it also takes
/// estimator-rejected moves) a WNS gain may cost TNS only up to `wns_tail_tns_ratio` times itself.
fn pe_accepts_batch(pe: &ParallelEval, wns_after: f32, tns_after: f32) -> bool {
    if !batch_improves_checkpoint(pe.checkpoint_wns, pe.checkpoint_tns, wns_after, tns_after, pe.best_checkpoint_wns, pe_checkpoint_wns_tolerance(pe)) {
        return false;
    }
    if pe.tail.active {
        let wns_gain = (wns_after - pe.checkpoint_wns).max(0.0);
        if fuzzy::less(tns_after, pe.checkpoint_tns - wns_gain * pe.config.parallel_eval.wns_tail_tns_ratio) {
            pe_event("wns_tail_reject_tns", "", "", 1);
            return false;
        }
    }
    true
}

/// `recordSwap`: a size or VT swap remembers the cell it replaces. (`replaceCell` finds masters
/// by the liberty cell's name, and every master here is named as its cell, so the check that the
/// name maps to the instance's master always holds.)
fn pe_record_swap(candidate: &Candidate, target: &Target, evaluation_index: usize) -> Option<SwapRecord> {
    matches!(candidate.kind, Move::SizeUp | Move::VtSwap).then(|| SwapRecord { evaluation_index, inst: target.inst.clone(), before_cell: target.cell.clone(), result: ApplyResult::default(), reverted: false })
}

/// The `MoveResult` type an Mt candidate reports (its own move type: RebufferMt is kBuffer).
fn pe_result_type(m: Move) -> Move {
    m
}


/// `startSmallBatchRetry`: once, after a run that kept no move — misses cleared, the stall
/// monitor reset, the temporary tabu dropped, and a new selector from the policy config's
/// (which carries the setup slack margin) without its score floor and worst-path reserve.
fn pe_start_small_batch_retry(pe: &mut ParallelEval) {
    pe_event("retry", "retries", "", 1);
    pe.small_batch_retry_started = true;
    pe.checkpoint_misses = 0;
    pe.stall.reset(pe.checkpoint_tns, pe.reference_period);
    pe.tabu.clear_temporary();
    pe.selector = BatchSelectorConfig { min_score_ratio: 0.0, wns_reserve_pct: 0, ..pe.config.parallel_eval.selector };
}

/// `MoveCandidate::guardedSidePins`: a cell swap's fanin side loads (`faninSidePins`), a pin
/// swap's displaced pin, a split's moved loads, a rebuffer's side contexts' loads; none for the
/// rest.
fn pe_guarded_side_pins(c: &Candidate, target: &Target) -> Vec<String> {
    match c.kind {
        Move::SizeUp | Move::VtSwap => target.estimator_context.as_ref().map(|ctx| ctx.fanin_side_nets.iter().flat_map(|n| n.loads.iter().map(|l| l.pin.clone())).collect()).unwrap_or_default(),
        Move::SwapPins => target.swap_pins_input.as_ref().and_then(|i| i.equivalents.iter().find(|e| e.port == c.desc)).and_then(|e| e.pin.clone()).into_iter().collect(),
        Move::SplitLoad => target.split_load_input.as_ref().map(|i| i.moved_loads.iter().map(|l| l.pin.clone()).collect()).unwrap_or_default(),
        Move::Buffer => c.rebuffer_side_pins.clone(),
        _ => Vec::new(),
    }
}

/// `sta_->slack(graph->pinLoadVertex(pin), max)`: the least max slack of the pin's load vertex
/// over the scenes; none without one.
fn pe_load_slack(gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], pin: &str) -> Option<f32> {
    let mut slack: Option<f32> = None;
    for (k, g) in gs.iter().enumerate() {
        let Some(u) = g.vertices.iter().position(|x| x.name == pin && !x.is_driver) else { continue };
        let v = ss[k].slack_of(u, MAX, None);
        slack = Some(match slack {
            Some(s) if !fuzzy::less(v, s) => s,
            _ => v,
        });
    }
    slack
}

/// `selfStageDelayDelta(candidate)`: a cell swap's own arc and its fanin arc (the window's
/// target stage and the one before it), each stage's graph delay now (`graphStageDelay`) less
/// its prepared delay, less `culprit_delay_ratio` of the stages' prepared total. None for other
/// moves, without a window, or when a stage has no matching arc on the graph.
fn pe_self_stage_delay_delta(ctx: &super::Ctx<'_>, gs: &[vyges_sta::graph::Graph<'_>], c: &Candidate, target: &Target, culprit_delay_ratio: f32) -> Option<f32> {
    if !matches!(c.kind, Move::SizeUp | Move::VtSwap) {
        return None;
    }
    let state = target.estimator_context.as_ref()?;
    if state.path_stages.is_empty() {
        return None;
    }
    let (mut delta, mut baseline) = (0.0f32, 0.0f32);
    let first = state.target_stage_index.saturating_sub(1);
    for stage in &state.path_stages[first..=state.target_stage_index] {
        let delay = pe_graph_stage_delay(ctx, gs, stage)?;
        delta += delay - stage.current_delay;
        baseline += stage.current_delay;
    }
    Some(delta - culprit_delay_ratio * baseline.abs())
}

/// `graphStageDelay(stage)`: the driver's first non-wire in-edge (the reference's in-edge
/// iterator order, the reverse of ours) whose arc set holds an arc matching the stage's
/// reference arc exactly (`findMatchingTimingArc`: the swapped cell has a new arc set), its max
/// delay at the stage's scene.
fn pe_graph_stage_delay(ctx: &super::Ctx<'_>, gs: &[vyges_sta::graph::Graph<'_>], stage: &crate::delay_estimator::DelayStageState) -> Option<f32> {
    let arc = &stage.arc;
    let ref_cell = ctx.libs.scene_cell(arc.scene, &arc.cell)?;
    let ref_set = ref_cell.arc_sets.get(arc.set)?;
    let g = gs.get(arc.scene)?;
    let driver = g.vertices.iter().position(|x| x.name == stage.driver_pin && x.is_driver)?;
    for &e in g.in_edges[driver].iter().rev() {
        let vyges_sta::graph::EdgeKind::Gate { set } = g.edges[e].kind else { continue };
        let cand_set = g.arc_set(e, set);
        if let Some(k) = crate::delay_estimator::find_matching_arc(ref_set, arc.arc, cand_set) {
            return Some(g.delay[e][k][MAX]);
        }
    }
    None
}

/// `RepairSetupProfiler::stop(reason)`, as the printer writes it.
fn pe_stop(reason: &str) {
    pe_event("stop", &format!("stop_{reason}"), "", 1);
}

/// `markRunComplete(result)`.
fn pe_mark_run_complete(pe: &mut ParallelEval, result: bool) {
    pe.result = result;
    pe.converged = true;
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
                sequential_estimator: None,
                prepare_why: String::new(),
                unbuffer_gain: None,
                swap_pins_input: None,
                input_cap_budgets: None,
                split_load_input: None,
            size_down_fanout_loads: Vec::new(),
            rebuffer_net: None,
            rebuffer_load_timing: Vec::new(),
            plan: Vec::new(),
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
    matches!(m, Move::Unbuffer | Move::SwapPins | Move::SizeUp | Move::VtSwap | Move::SplitLoad | Move::SizeDownFanout | Move::Buffer)
}

/// `planGeneration`: per generator in `move_sequence_` order, `generatorAllowed`, `isApplicable`
/// (else `skip_not_applicable`), then `prepareInputs`; the plan's generators in that order. The
/// tabu checks come first: both are empty until a batch is undone or missed, and only the first
/// chunk of batch 0 is planned here.
#[allow(clippy::too_many_arguments)]
fn pe_plan_generation(pe: &mut ParallelEval, target: &mut Target, facts: &PlanFacts, plan: &PlanTimer<'_, '_, '_>, trace: &mut String) -> Result<(), String> {
    let pin = target.driver_pin.clone();
    let b = pe.batch_index;
    // The miss tabu first: an excluded or tabu instance plans no generator.
    let id = (plan.inst_id)(&target.inst);
    if pe.tabu.is_excluded(id, &target.inst) {
        trace.push_str(&format!("VYGP|plan|{b}|{pin}|skip=undo_blocked\n"));
        pe_event("skip_undo_blocked", "", "", 1);
        return Ok(());
    }
    if pe.tabu.is_tabu(id, &target.inst) {
        trace.push_str(&format!("VYGP|plan|{b}|{pin}|skip=miss_tabu\n"));
        pe_event("skip_miss_tabu", "", "", 1);
        return Ok(());
    }
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
            pe.work.add_plan_request();
        }
        pe_prepare_inputs(m, target, facts, plan, &pe.config, &pe.crit_path_load_pins)?;
        trace.push_str(&format!("VYGP|inputs|{b}|{pin}|{}|{}\n", m.name(), pe_inputs_line(m, target)));
        planned.push(m.name());
        target.plan.push(m);
    }
    if pe.generators.iter().all(|&m| plan_modelled(m)) {
        trace.push_str(&format!("VYGP|plan|{b}|{pin}|gens={}\n", planned.join(",")));
    }
    Ok(())
}

/// The generators whose `generate` and candidate `estimate` are modelled.
fn generate_modelled(m: Move) -> bool {
    matches!(m, Move::Unbuffer | Move::SwapPins | Move::SizeUp | Move::VtSwap | Move::SizeDownFanout | Move::SplitLoad | Move::Buffer)
}

/// A task's generators in plan order (`MoveGenerator::generate`): per generator its candidates,
/// none for one not modelled yet.
/// `rb` collects the printer's RebufferMt lines (the planner's, the plan gates, the estimates).
fn pe_run_generators(target: &Target, plan: &PlanTimer<'_, '_, '_>, cfg: &MtConfig, rb: &mut String) -> Result<Vec<Vec<Candidate>>, Stop> {
    target.plan.iter().map(|&m| if generate_modelled(m) { pe_generate(m, target, plan, cfg, rb) } else { Ok(Vec::new()) }).collect()
}

/// `collectGenerated`: per target, per generator in plan order, the trace's candidate count.
fn pe_collect_generated(pe: &ParallelEval, targets: &[Target], generated: &[Vec<Vec<Candidate>>], trace: &mut String) {
    for (target, per_generator) in targets.iter().zip(generated) {
        for (&m, candidates) in target.plan.iter().zip(per_generator) {
            if generate_modelled(m) {
                trace.push_str(&format!("VYGP|gen|{}|{}|{}|n={}\n", pe.batch_index, target.driver_pin, m.name(), candidates.len()));
            }
        }
    }
}

/// `MoveGenerator::generate` with each candidate's `estimate()`, `vygDescribe()` and `impact()`.
fn pe_generate(m: Move, target: &Target, plan: &PlanTimer<'_, '_, '_>, cfg: &MtConfig, rb: &mut String) -> Result<Vec<Candidate>, Stop> {
    match m {
        // UnbufferMtGenerator: one removal when the prepared gain is positive.
        Move::Unbuffer => Ok(unbuffer_mt_generate(target, || unbuffer_mt_impact(target, plan))),
        Move::SwapPins => Ok(swap_pins_mt_generate(target, plan.names)),
        Move::SizeUp => size_up_mt_generate(target, plan.ctx, cfg, plan.names),
        Move::VtSwap => vt_swap_mt_generate(target, plan.ctx, cfg, plan.names),
        Move::SizeDownFanout => Ok(size_down_fanout_mt_generate(target, plan.ctx.libs, cfg, plan.names)),
        Move::SplitLoad => Ok(split_load_mt_generate(target, plan.ctx.libs, cfg, plan.names)),
        Move::Buffer => rebuffer_mt_generate(target, plan, cfg, rb),
        _ => Ok(Vec::new()),
    }
}

/// `RebufferMtGenerator::generate`: nothing for a driver whose instance already has a buffer move
/// or a target without a load pin; else the prepared net planned (`makeTreePlans`), the plans
/// gated, one candidate per plan in plan order.
fn rebuffer_mt_generate(target: &Target, plan: &PlanTimer<'_, '_, '_>, cfg: &MtConfig, rb: &mut String) -> Result<Vec<Candidate>, Stop> {
    if plan.committer.has_moves(Move::Buffer, (plan.inst_id)(&target.inst)) {
        return Ok(Vec::new());
    }
    let Some(load_pin) = target.load_pin.as_deref() else { return Ok(Vec::new()) };
    let (plans, arena) = rebuffer_mt_make_tree_plans(target, plan, cfg, load_pin, rb)?;
    Ok(rebuffer_mt_add_tree_plan_candidates(target, plan, cfg, &plans, &arena, rb))
}

/// `RebufferMtGenerator::makeTreePlans`: `makeRebufferTreePlans` on the prepared net (none without
/// one), then the plan gates: no tree without buffers or with a dont-touch load behind a buffer;
/// a one-buffer tree only with `allow_rebuffer_mt_single_buffer`; no tree past the score cap —
/// the anchor tree exempt from the last two.
fn rebuffer_mt_make_tree_plans(target: &Target, plan: &PlanTimer<'_, '_, '_>, cfg: &MtConfig, load_pin: &str, rb: &mut String) -> Result<(Vec<crate::rebuffer::TreePlan>, std::sync::Arc<Vec<crate::rebuffer::Node>>), Stop> {
    let none = || (Vec::new(), std::sync::Arc::new(Vec::new()));
    let Some((bn, root)) = target.rebuffer_net.as_ref() else { return Ok(none()) };
    let rctx = plan.ctx.rebuffer.ok_or_else(|| Stop::refused("RSZ-ABSENT", "RebufferMt without characterized buffers".into()))?;
    let g = &plan.gs[0];
    let Some(drvr) = g.vertices.iter().position(|x| x.name == target.driver_pin && x.is_driver) else { return Ok(none()) };
    let Some(probe) = super::rebuffer_probe_of_net(plan.ctx, plan.gs, plan.ss, plan.design, drvr, bn, *root) else { return Ok(none()) };
    if let Some(w) = probe.warnings.first() {
        return Err(Stop::refused("RSZ-ABSENT", format!("RebufferMt planning warnings are not modelled: {w}")));
    }
    let mut rbuf = crate::rebuffer::Rebuf::new(rctx, &probe);
    let mut lines = Vec::new();
    let mut plans = rbuf.make_rebuffer_tree_plans(Some(load_pin), cfg.rebuffer_mt_max_candidates.max(1) as usize, &mut lines);
    for l in lines {
        rb.push_str(&l);
        rb.push('\n');
    }
    if let Some(e) = rbuf.failed.as_ref().or(rbuf.refused.as_ref()) {
        return Err(Stop::refused("RSZ-ABSENT", format!("RebufferMt planning of {}: {e}", target.driver_pin)));
    }
    if let Some((_, w)) = rbuf.trace.iter().find(|(level, _)| level.is_none()) {
        return Err(Stop::refused("RSZ-ABSENT", format!("RebufferMt planning warnings are not modelled: {w}")));
    }
    for (k, p) in plans.iter().enumerate() {
        rb.push_str(&format!("VYGP|rbgate|{}|{k}|bufs={}|anchor={}|dsg={}|dt={}\n", target.driver_pin, p.buffer_count, u8::from(p.anchor_tree), e9(p.driver_slack_gain), u8::from(rebuffer_has_dont_touch_load_behind_buffer(plan, p))));
    }
    plans.retain(|p| {
        if p.buffer_count == 0 || rebuffer_has_dont_touch_load_behind_buffer(plan, p) {
            return false;
        }
        if p.buffer_count == 1 && !p.anchor_tree && !cfg.allow_rebuffer_mt_single_buffer {
            return false;
        }
        !(!p.anchor_tree && cfg.rebuffer_mt_max_score > 0.0 && p.driver_slack_gain.abs() > cfg.rebuffer_mt_max_score)
    });
    Ok((plans, std::sync::Arc::new(rbuf.nodes)))
}

/// `hasDontTouchLoadBehindBuffer`: a load behind a buffer that is dont-touch (`Resizer::dontTouch`:
/// its instance or its net).
fn rebuffer_has_dont_touch_load_behind_buffer(plan: &PlanTimer<'_, '_, '_>, p: &crate::rebuffer::TreePlan) -> bool {
    let info = plan.design.net_info();
    p.load_paths.iter().any(|lp| {
        let behind = lp.stages.iter().any(|s| s.kind == crate::rebuffer::StageKind::Buffer);
        let inst_dt = lp.load_pin.rsplit_once('/').is_some_and(|(inst, _)| info.dont_touch_insts.contains(inst));
        let net_dt = plan.design.pin_net(&lp.load_pin).is_some_and(|(net, _)| info.dont_touch.contains(&net));
        behind && (inst_dt || net_dt)
    })
}

/// `addTreePlanCandidates` with `makeSideContexts`: one candidate per plan; its side contexts the
/// plan's other loads that have load timing, in load-path order.
fn rebuffer_mt_add_tree_plan_candidates(target: &Target, plan: &PlanTimer<'_, '_, '_>, cfg: &MtConfig, plans: &[crate::rebuffer::TreePlan], arena: &std::sync::Arc<Vec<crate::rebuffer::Node>>, rb: &mut String) -> Vec<Candidate> {
    let mut out = Vec::with_capacity(plans.len());
    for p in plans {
        let sides = rebuffer_make_side_contexts(p, &target.rebuffer_load_timing);
        let Some(context) = target.estimator_context.as_ref() else { continue };
        out.push(Candidate {
            kind: Move::Buffer,
            estimate: rebuffer_mt_estimate(plan.ctx.libs, context, target, p, &sides, cfg, rb),
            desc: format!("bufs={}/loads={}/anchor={}/dsg={:.6}/cap={:.6}", p.buffer_count, p.load_count, u8::from(p.anchor_tree), p.driver_slack_gain, p.tree_input_cap),
            impact: rebuffer_mt_impact(target, plan, p),
            tree: Some((arena.clone(), p.tree)),
            rebuffer_side_pins: sides.iter().map(|side| side.load_pin.clone()).collect(),
        });
    }
    out
}

/// `makeSideContexts`: the plan's loads other than the target load that have load timing (none
/// without the side-load model), in load-path order.
fn rebuffer_make_side_contexts(p: &crate::rebuffer::TreePlan, load_timing: &[RebufferLoadTiming]) -> Vec<crate::delay_estimator::buffer_tree::SidePathContext> {
    if load_timing.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::new();
    for lp in &p.load_paths {
        if p.target_load_pin.as_deref() == Some(lp.load_pin.as_str()) {
            continue;
        }
        let Some(t) = load_timing.iter().find(|t| t.pin == lp.load_pin) else { continue };
        out.push(crate::delay_estimator::buffer_tree::SidePathContext { load_pin: lp.load_pin.clone(), current_driver_to_load_delay: t.wire_delay, current_slack: t.slack, weight: t.weight });
    }
    out
}

/// `RebufferMtCandidate::estimate`: the buffer-tree estimate; the score is the gain less the side
/// loss, scaled; an estimator reject keeps its gain only when non-improving or below the side
/// floor; a legal estimate is still rejected at a score at or below zero
/// (`side_loss_non_improving`, tail-admissible for a positive own gain) or under the minimum
/// (`min_score`).
fn rebuffer_mt_estimate(libs: &crate::preamble::Libs, context: &EstimatorContext, target: &Target, p: &crate::rebuffer::TreePlan, sides: &[crate::delay_estimator::buffer_tree::SidePathContext], cfg: &MtConfig, rb: &mut String) -> Estimate {
    use crate::delay_estimator::buffer_tree as bt;
    let mut trace = Vec::new();
    let mut side_evals = Vec::new();
    let d = bt::estimate_buffer_tree_with_side_paths(libs, context, p, sides, Some(&mut trace), Some(&mut side_evals));
    rb.push_str(&rebuffer_est_line(&target.driver_pin, &d, &trace, &side_evals));
    rebuffer_mt_estimate_of(&d, cfg.rebuffer_mt_score_scale, cfg.rebuffer_mt_min_score)
}

/// The score rules of [`rebuffer_mt_estimate`] on a delay estimate.
fn rebuffer_mt_estimate_of(d: &crate::delay_estimator::DelayEstimate, score_scale: f32, min_score: f32) -> Estimate {
    use crate::delay_estimator::FailReason;
    let raw_score = d.arrival_impr - d.side_loss;
    let mut e = Estimate { legal: false, score: raw_score * score_scale, raw_gain: None, reason: d.reason.name(), tail_admissible: false, side_loss: (d.side_paths_evaluated > 0).then_some(d.side_loss), sequential_penalty: None, sequential_side_loss: None };
    if !d.legal {
        if matches!(d.reason, FailReason::EstimateNonImproving | FailReason::SideSlackFloorViolation) {
            e.raw_gain = Some(d.arrival_impr);
        }
        return e;
    }
    e.raw_gain = Some(d.arrival_impr);
    if raw_score <= 0.0 || raw_score < min_score {
        e.reason = if raw_score <= 0.0 { "side_loss_non_improving" } else { "min_score" };
        e.tail_admissible = d.arrival_impr > 0.0;
        return e;
    }
    e.legal = true;
    e
}

/// The printer's `rbest` line: the estimate, its stage trace and its side paths.
fn rebuffer_est_line(drvr: &str, d: &crate::delay_estimator::DelayEstimate, trace: &[crate::delay_estimator::buffer_tree::StageEvaluation], sides: &[crate::delay_estimator::buffer_tree::SidePathEvaluation]) -> String {
    let stages = |t: &[crate::delay_estimator::buffer_tree::StageEvaluation]| {
        t.iter()
            .map(|e| format!("{}:{}:{}:{}:{}:{}:{}", e.path_index, e.cell.as_deref().unwrap_or("-"), e9(e.input_slew), e9(e.load_cap), e9(e.stage_delay), e9(e.output_slew), e9(e.wire_delay)))
            .collect::<Vec<_>>()
            .join(";")
    };
    let sides: Vec<String> = sides.iter().map(|s| format!("{}:{}:{}:{}:{{{}}}", s.load_pin, u8::from(s.legal), e9(s.arrival_impr), e9(s.slack_after), stages(&s.trace))).collect();
    format!("VYGP|rbest|{drvr}|legal={}|reason={}|impr={}|side={}|nside={}|stages={}|sides={}\n", u8::from(d.legal), d.reason.name(), e9(d.arrival_impr), e9(d.side_loss), d.side_paths_evaluated, stages(trace), sides.join(","))
}

/// `RebufferMtCandidate::impact`: complete; the driver's net and instance written; every plan
/// load reconnects, in load-path order.
fn rebuffer_mt_impact(target: &Target, plan: &PlanTimer<'_, '_, '_>, p: &crate::rebuffer::TreePlan) -> MoveImpact {
    let net = plan.design.pin_net(&target.driver_pin).map(|(n, _)| n);
    rebuffer_impact_on(move_candidate_impact(target, plan.names), plan.names, net, &target.driver_pin, p)
}

/// [`rebuffer_mt_impact`] given the base impact and the plan's flat net: complete; the net written
/// and in the effect cone; the driver's instance written and in the effect cone; each load path's
/// load pin reconnected, with its instance reconnected and in the effect cone (`appendReconnectedLoad`),
/// each once.
fn rebuffer_impact_on(mut impact: MoveImpact, names: &ImpactNames<'_>, net: Option<String>, driver_pin: &str, p: &crate::rebuffer::TreePlan) -> MoveImpact {
    impact.complete = true;
    append_unique(&mut impact.write_nets, net.clone());
    append_unique(&mut impact.timing_effect_nets, net);
    let driver_inst = names.instance(driver_pin);
    append_unique(&mut impact.write_instances, Some(driver_inst.clone()));
    append_unique(&mut impact.timing_effect_instances, Some(driver_inst));
    for lp in &p.load_paths {
        append_unique(&mut impact.reconnect_pins, Some(lp.load_pin.clone()));
        append_unique(&mut impact.reconnect_instances, Some(names.instance(&lp.load_pin)));
        append_unique(&mut impact.timing_effect_instances, Some(names.instance(&lp.load_pin)));
    }
    impact
}

/// `UnbufferMtGenerator::generate`: one removal when `prepareInputs` left a gain above zero (none
/// for a gain of zero, a negative one, or none at all).
fn unbuffer_mt_generate(target: &Target, impact: impl FnOnce() -> MoveImpact) -> Vec<Candidate> {
    match target.unbuffer_gain {
        Some(gain) if gain > 0.0 => vec![Candidate { kind: Move::Unbuffer, estimate: unbuffer_mt_estimate(gain), desc: "remove".into(), impact: impact(), tree: None, rebuffer_side_pins: Vec::new() }],
        _ => Vec::new(),
    }
}

/// `UnbufferMtCandidate::estimate`: always legal, scored by the prepared gain.
fn unbuffer_mt_estimate(gain: f32) -> Estimate {
    Estimate { legal: true, score: gain, raw_gain: Some(gain), reason: "estimate_legal", tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None }
}

/// `UnbufferMtCandidate::impact`: the base impact, complete; the buffer written and in the effect
/// cone; its input then output net written and in the effect cone (when connected); every other
/// instance on them in the effect cone; the other pins of the net `removeBuffer` drops, and their
/// instances, reconnect — the input net's when the output net is kept (it has a top-level port and
/// the input net has none), else the output net's — in the net's connected-pin order.
fn unbuffer_mt_impact(target: &Target, plan: &PlanTimer<'_, '_, '_>) -> MoveImpact {
    let g = &plan.gs[0];
    let ports = plan.ctx.libs.link_cell(&target.cell).and_then(|c| c.buffer_ports()).map(|(i, o)| (i.name.clone(), o.name.clone()));
    let net_of = |port: &str| {
        let pin = format!("{}/{port}", target.inst);
        g.vertices.iter().position(|x| x.name == pin).and_then(|v| g.vertex_net[v])
    };
    match ports {
        Some((input, output)) => unbuffer_impact_on(move_candidate_impact(target, plan.names), &g.netlist, &target.inst, net_of(&input), net_of(&output)),
        None => unbuffer_impact_on(move_candidate_impact(target, plan.names), &g.netlist, &target.inst, None, None),
    }
}

/// [`unbuffer_mt_impact`] on the netlist, given the base impact and the buffer's input and output
/// nets. The lists are pushed, not kept unique, as the reference pushes them.
fn unbuffer_impact_on(mut impact: MoveImpact, nl: &vyges_sta::netlist::Netlist, buffer: &str, in_net: Option<usize>, out_net: Option<usize>) -> MoveImpact {
    use vyges_sta::netlist::Conn;
    impact.complete = true;
    impact.write_instances.push(buffer.to_string());
    impact.timing_effect_instances.push(buffer.to_string());
    let has_port = |net: Option<usize>| net.is_some_and(|n| nl.nets[n].pins.iter().any(|c| matches!(c, Conn::Port(_))));
    // removeBufferKeepsOutputNet: the output net survives when it reaches a top-level port and
    // the input net does not; then the input net's pins move.
    let moved = if has_port(out_net) && !has_port(in_net) { in_net } else { out_net };
    for net in [in_net, out_net].into_iter().flatten() {
        impact.write_nets.push(nl.nets[net].name.clone());
        impact.timing_effect_nets.push(nl.nets[net].name.clone());
        for c in &nl.nets[net].pins {
            if matches!(c, Conn::Inst(k, _) if nl.insts[*k].0 == buffer) {
                continue;
            }
            // `network->instance` of a port pin is the top instance.
            let inst = match c {
                Conn::Inst(k, _) => nl.insts[*k].0.clone(),
                Conn::Port(_) => TOP_INSTANCE.to_string(),
            };
            if Some(net) == moved {
                impact.reconnect_pins.push(nl.pin_name(c));
                impact.reconnect_instances.push(inst.clone());
            }
            impact.timing_effect_instances.push(inst);
        }
    }
    impact
}

/// `SwapPinsMtGenerator::generate`: the critical signal moves to the equivalent input with the
/// fastest arc when it beats its own — the first of equal delays wins — else no candidate.
fn swap_pins_mt_generate(target: &Target, names: &ImpactNames<'_>) -> Vec<Candidate> {
    let Some(input) = &target.swap_pins_input else { return Vec::new() };
    let mut swap: Option<&SwapPinsEquivalent> = None;
    let mut best_delay = input.input_delay;
    for equivalent in &input.equivalents {
        if equivalent.delay < best_delay {
            swap = Some(equivalent);
            best_delay = equivalent.delay;
        }
    }
    let Some(swap) = swap else { return Vec::new() };
    let estimate = swap_pins_mt_estimate(input, swap, target.estimator_context.as_ref());
    vec![Candidate { kind: Move::SwapPins, estimate, desc: swap.port.clone(), impact: swap_pins_mt_impact(target, names, input, swap), tree: None, rebuffer_side_pins: Vec::new() }]
}

/// `SwapPinsMtCandidate::estimate`: the gain is the input's delay less the swap's. Without a side
/// pin, legal iff the gain is above zero. With one, the displaced signal loses the gain: its loss
/// below zero slack, weighted, is charged (`side_loss`) and the score is the gain less it; not
/// legal for a gain at or below zero (`estimate_non_improving`), then for a displaced slack below
/// `sideSlackFloor(gain)` (`side_slack_floor_violation`), then, tail-admissible, for a score at or
/// below zero (`side_loss_non_improving`).
fn swap_pins_mt_estimate(input: &SwapPinsInput, swap: &SwapPinsEquivalent, context: Option<&crate::delay_estimator::EstimatorContext>) -> Estimate {
    let gain = input.input_delay - swap.delay;
    let plain = |legal: bool| Estimate { legal, score: gain, raw_gain: Some(gain), reason: if legal { "estimate_legal" } else { "estimate_non_improving" }, tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None };
    let (Some(_), Some(context)) = (&swap.pin, context) else { return plain(gain > 0.0) };
    let victim_after = swap.slack - gain;
    let side_loss = context.fanin_side_loss_weight * (swap.slack.min(0.0) - victim_after.min(0.0));
    let adjusted_gain = gain - side_loss;
    let mut e = Estimate { legal: adjusted_gain > 0.0, score: adjusted_gain, raw_gain: Some(gain), reason: "estimate_legal", tail_admissible: false, side_loss: Some(side_loss), sequential_penalty: None, sequential_side_loss: None };
    if gain <= 0.0 {
        e.legal = false;
        e.reason = "estimate_non_improving";
    } else if victim_after < context.side_slack_floor_for(gain) {
        e.legal = false;
        e.reason = "side_slack_floor_violation";
    } else if !e.legal {
        e.reason = "side_loss_non_improving";
        e.tail_admissible = true;
    }
    e
}

/// `SwapPinsMtCandidate::impact`: the base impact, complete; the driver written and in the effect
/// cone; its path input pin, then the swap's input pin, reconnect (no net is written); the nets of
/// those two pins and of the driver pin are in the effect cone, each once.
fn swap_pins_mt_impact(target: &Target, names: &ImpactNames<'_>, input: &SwapPinsInput, swap: &SwapPinsEquivalent) -> MoveImpact {
    let mut impact = move_candidate_impact(target, names);
    impact.complete = true;
    impact.write_instances.push(target.inst.clone());
    impact.timing_effect_instances.push(target.inst.clone());
    let driver_port = target.driver_pin.rsplit_once('/').map_or("", |(_, port)| port);
    for port in [input.input_port.as_str(), swap.port.as_str(), driver_port] {
        let pin = format!("{}/{port}", target.inst);
        if port != driver_port {
            impact.reconnect_pins.push(pin.clone());
        }
        append_unique(&mut impact.timing_effect_nets, names.net(&pin));
    }
    impact
}

/// `SizeUpMtGenerator::generate`: the size-up options of the target arc's output port, scene
/// and min/max, each replacement kept when it fits the instance's input cap budgets.
fn size_up_mt_generate(target: &Target, ctx: &super::Ctx<'_>, cfg: &MtConfig, names: &ImpactNames<'_>) -> Result<Vec<Candidate>, Stop> {
    let Some(context) = &target.estimator_context else { return Ok(Vec::new()) };
    let replacements = build_size_up_options(ctx, cfg, &context.target().arc)?;
    let budgets = target.input_cap_budgets.as_deref().unwrap_or(&[]);
    let mut candidates = Vec::new();
    for replacement in replacements {
        if !replacement_fits_input_cap_budgets(ctx, budgets, &replacement) {
            continue;
        }
        let estimate = size_up_mt_estimate(ctx.libs, target, &replacement, cfg.size_up_mt_score_scale)?;
        candidates.push(Candidate { kind: Move::SizeUp, estimate, impact: cell_swap_impact(target, names, &target.inst), desc: replacement, tree: None, rebuffer_side_pins: Vec::new() });
    }
    Ok(candidates)
}

/// `SizeUpMtGenerator::buildSizeUpOptions`: the swappable cells of the arc's cell, each other than
/// it, with a master, of the same VT class (`cellVTType`, so VT changes stay VtSwap's), with the
/// output port at the scene, passing the dual-RF drive guard (when on), and whose drive
/// resistance is at most the current one and, when the ratio limit is on (a positive
/// `size_up_mt_max_drive_ratio` and drive), at least the current over the ratio.
fn build_size_up_options(ctx: &super::Ctx<'_>, cfg: &MtConfig, arc: &crate::delay_estimator::SelectedArc) -> Result<Vec<String>, Stop> {
    let mut replacements = Vec::new();
    let Some((_, drvr_port)) = crate::delay_estimator::arc_ports(ctx.libs, arc) else { return Ok(replacements) };
    let cell = &arc.cell;
    let swappable = ctx.sizing.swappable_cells(cell)?;
    if swappable.is_empty() {
        return Ok(replacements);
    }
    let Some(scene_cell) = ctx.libs.scene_cell(arc.scene, cell).filter(|c| c.port(&drvr_port).is_some()) else { return Ok(replacements) };
    let drive_r = scene_cell.drive_resistance(&drvr_port);
    let max_ratio = cfg.size_up_mt_max_drive_ratio;
    let limit_ratio = max_ratio > 0.0 && drive_r > 0.0;
    let min_drive_r = if limit_ratio { drive_r / max_ratio } else { 0.0 };
    let current_master = ctx.sizing.masters.get(cell.as_str());
    for name in &swappable {
        if name == cell {
            continue;
        }
        let (Some(current), Some(master)) = (current_master, ctx.sizing.masters.get(name.as_str())) else { continue };
        if crate::sizing::implant_set(current) != crate::sizing::implant_set(master) {
            continue;
        }
        let Some(corner) = ctx.libs.scene_cell(arc.scene, name).filter(|c| c.port(&drvr_port).is_some()) else { continue };
        if cfg.dual_rf_drive_guard && !drive_res_no_higher_on_both_rf(scene_cell, corner, &drvr_port) {
            continue;
        }
        let swappable_drive_r = corner.drive_resistance(&drvr_port);
        if swappable_drive_r <= drive_r && (!limit_ratio || swappable_drive_r >= min_drive_r) {
            replacements.push(name.clone());
        }
    }
    Ok(replacements)
}

/// `driveResNoHigherOnBothRf`: the candidate's drive resistance is no higher at rise and at fall.
fn drive_res_no_higher_on_both_rf(current: &vyges_sta::liberty::Cell, candidate: &vyges_sta::liberty::Cell, port: &str) -> bool {
    use vyges_sta::liberty::{FALL, RISE};
    super::drive_resistance_rf(candidate, port, RISE) <= super::drive_resistance_rf(current, port, RISE) && super::drive_resistance_rf(candidate, port, FALL) <= super::drive_resistance_rf(current, port, FALL)
}

/// `Resizer::replacementFitsInputCapBudgets`: per budget, the replacement's port of that name
/// (its largest max capacitance over rise and fall, 0 without the port) may not grow the cap past
/// the budget's increase.
fn replacement_fits_input_cap_budgets(ctx: &super::Ctx<'_>, budgets: &[InputCapBudget], replacement: &str) -> bool {
    ctx.libs.link_cell(replacement).is_some_and(|cell| cell_fits_input_cap_budgets(cell, budgets))
}

/// [`replacement_fits_input_cap_budgets`] on the replacement's link cell.
fn cell_fits_input_cap_budgets(cell: &vyges_sta::liberty::Cell, budgets: &[InputCapBudget]) -> bool {
    budgets.iter().all(|b| {
        let new_cap = cell.port(&b.port).map_or(0.0, |p| p.capacitance.iter().fold(0.0f32, |m, rf| m.max(rf[vyges_sta::liberty::MAX])));
        let cap_delta = new_cap - b.current_cap;
        !(cap_delta > 0.0 && cap_delta > b.max_cap_increase)
    })
}

/// `MoveCandidate::cellSwapImpact`: complete, the instance written (its pins' nets only enter
/// the timing cone, which the trace does not print).
/// `SizeUpMtCandidate::estimate`: the window estimate of the replacement, scored by the gain
/// times the score scale. Not legal: the score kept, the raw gain only for a non-improving
/// estimate, tail-admissible for the opposite-transition reject with a positive score. Legal: the
/// cell swap's side costs.
fn size_up_mt_estimate(libs: &crate::preamble::Libs, target: &Target, replacement: &str, score_scale: f32) -> Result<Estimate, Stop> {
    let Some(context) = &target.estimator_context else { return Err(Stop::refused("RSZ-ABSENT", "SizeUpMt estimate without an estimator context".into())) };
    let d = crate::delay_estimator::estimate(libs, context, replacement);
    // `chargeCellSwapSideCosts` asks the sequential estimator only for a legal estimate.
    let seq = target.sequential_estimator.as_ref().filter(|_| d.legal).map(|s| s.estimate(libs, replacement));
    size_up_mt_estimate_of(target, &d, seq, score_scale)
}

/// [`size_up_mt_estimate`] from the window estimate.
fn size_up_mt_estimate_of(target: &Target, d: &crate::delay_estimator::DelayEstimate, seq: Option<SequentialResult>, score_scale: f32) -> Result<Estimate, Stop> {
    use crate::delay_estimator::FailReason;
    let score = d.arrival_impr * score_scale;
    if !d.legal {
        return Ok(Estimate { legal: false, score, raw_gain: (d.reason == FailReason::EstimateNonImproving).then_some(d.arrival_impr), reason: d.reason.name(), tail_admissible: d.reason == FailReason::DualRfNonImproving && score > 0.0, side_loss: None, sequential_penalty: None, sequential_side_loss: None });
    }
    let mut e = Estimate { legal: d.legal, score, raw_gain: Some(d.arrival_impr), reason: d.reason.name(), tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None };
    charge_cell_swap_side_costs(target, d, seq, score_scale, &mut e)?;
    Ok(e)
}

/// `VtSwapMtCandidate::estimate`: the window estimate of the VT cell. Not legal: a non-improving
/// estimate keeps its gain as score and raw gain, any other reject scores 0 with none. Legal:
/// scored by the gain, then the cell swap's side costs at score scale 1.
fn vt_swap_mt_estimate(libs: &crate::preamble::Libs, target: &Target, cell: &str) -> Result<Estimate, Stop> {
    let Some(context) = &target.estimator_context else { return Err(Stop::refused("RSZ-ABSENT", "VtSwapMt estimate without an estimator context".into())) };
    let d = crate::delay_estimator::estimate(libs, context, cell);
    let seq = target.sequential_estimator.as_ref().filter(|_| d.legal).map(|s| s.estimate(libs, cell));
    vt_swap_mt_estimate_of(target, &d, seq)
}

/// [`vt_swap_mt_estimate`] from the window estimate.
fn vt_swap_mt_estimate_of(target: &Target, d: &crate::delay_estimator::DelayEstimate, seq: Option<SequentialResult>) -> Result<Estimate, Stop> {
    use crate::delay_estimator::FailReason;
    if !d.legal {
        let raw = (d.reason == FailReason::EstimateNonImproving).then_some(d.arrival_impr);
        return Ok(Estimate { legal: false, score: raw.unwrap_or(0.0), raw_gain: raw, reason: d.reason.name(), tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None });
    }
    let mut e = Estimate { legal: true, score: d.arrival_impr, raw_gain: Some(d.arrival_impr), reason: "estimate_legal", tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None };
    charge_cell_swap_side_costs(target, d, seq, 1.0, &mut e)?;
    Ok(e)
}

/// `MoveCandidate::chargeCellSwapSideCosts`: with a sequential timing estimator on the target
/// (`seq`, its estimate of the replacement) its weighted loss comes off first — the penalty and
/// the max loss recorded, the score the gain left times the scale; an illegal side, or a gain left
/// at or below zero, rejects (the side's reason, else `sequential_non_improving`), tail-admissible
/// for a legal side with a positive own gain. Then, when the estimate charged side paths, their
/// loss likewise (`side_loss_non_improving`).
fn charge_cell_swap_side_costs(target: &Target, d: &crate::delay_estimator::DelayEstimate, seq: Option<SequentialResult>, score_scale: f32, e: &mut Estimate) -> Result<(), Stop> {
    debug_assert_eq!(seq.is_some(), target.sequential_estimator.is_some(), "the caller estimates the side exactly when the target has an estimator");
    let mut adjusted_gain = d.arrival_impr;
    if let Some(side) = seq {
        e.sequential_penalty = Some(side.weighted_side_slack_loss);
        e.sequential_side_loss = Some(side.max_side_slack_loss);
        adjusted_gain -= side.weighted_side_slack_loss;
        e.score = adjusted_gain * score_scale;
        if !side.legal || adjusted_gain <= 0.0 {
            e.legal = false;
            e.reason = if side.legal { "sequential_non_improving" } else { side.reason };
            e.tail_admissible = side.legal && d.arrival_impr > 0.0;
            return Ok(());
        }
    }
    if d.side_paths_evaluated > 0 {
        e.side_loss = Some(d.side_loss);
        adjusted_gain -= d.side_loss;
        e.score = adjusted_gain * score_scale;
        if adjusted_gain <= 0.0 {
            e.legal = false;
            e.reason = "side_loss_non_improving";
            e.tail_admissible = d.arrival_impr > 0.0;
        }
    }
    Ok(())
}

/// `VtSwapMtGenerator::generate`: the VT equivalents of the instance's cell other than it
/// (`selectCandidateCells`), each kept when it passes the dual-RF drive guard (when on and the
/// candidate has the arc's output port at its scene) and fits the input cap budgets.
fn vt_swap_mt_generate(target: &Target, ctx: &super::Ctx<'_>, cfg: &MtConfig, names: &ImpactNames<'_>) -> Result<Vec<Candidate>, Stop> {
    let Some(context) = &target.estimator_context else { return Ok(Vec::new()) };
    let arc = &context.target().arc;
    let budgets = target.input_cap_budgets.as_deref().unwrap_or(&[]);
    let output = crate::delay_estimator::arc_ports(ctx.libs, arc).map(|(_, o)| o);
    let current_scene = ctx.libs.scene_cell(arc.scene, &arc.cell);
    let mut candidates = Vec::new();
    for cell in select_candidate_cells(ctx, &target.cell) {
        if cfg.dual_rf_drive_guard {
            if let (Some(out), Some(current)) = (&output, current_scene) {
                if let Some(candidate) = ctx.libs.scene_cell(arc.scene, &cell).filter(|c| c.port(out).is_some()) {
                    if !drive_res_no_higher_on_both_rf(current, candidate, out) {
                        continue;
                    }
                }
            }
        }
        if !replacement_fits_input_cap_budgets(ctx, budgets, &cell) {
            continue;
        }
        let estimate = vt_swap_mt_estimate(ctx.libs, target, &cell)?;
        candidates.push(Candidate { kind: Move::VtSwap, estimate, impact: cell_swap_impact(target, names, &target.inst), desc: cell, tree: None, rebuffer_side_pins: Vec::new() });
    }
    Ok(candidates)
}

/// `VtSwapMtGenerator::selectCandidateCells`: the VT-equivalent cells but the current one.
fn select_candidate_cells(ctx: &super::Ctx<'_>, current: &str) -> Vec<String> {
    ctx.sizing.vt_equiv_cells(current, ctx.vt_category_count).into_iter().filter(|c| c != current).collect()
}

/// `SizeDownFanoutMtGenerator::generate`: per prepared load, its one drive step down when it keeps
/// the output limits, fits the load's delay budget and has a target load change.
fn size_down_fanout_mt_generate(target: &Target, libs: &crate::preamble::Libs, cfg: &MtConfig, names: &ImpactNames<'_>) -> Vec<Candidate> {
    let Some(context) = &target.estimator_context else { return Vec::new() };
    let mut candidates = Vec::new();
    for load in &target.size_down_fanout_loads {
        let step_down = &load.step_down;
        let Some(delta) = size_down_fanout_mt_step_down(load) else { continue };
        let d = crate::delay_estimator::estimate_load_change(libs, context, delta);
        candidates.push(Candidate {
            kind: Move::SizeDownFanout,
            estimate: size_down_fanout_mt_estimate_of(&d, cfg.size_down_fanout_mt_score_scale),
            desc: format!("{}/{}", load.pin, if step_down.cell.is_empty() { "-" } else { &step_down.cell }),
            impact: size_down_fanout_mt_impact(target, names, &load.inst),
            tree: None, rebuffer_side_pins: Vec::new(),
        });
    }
    candidates
}

/// `SizeDownFanoutMtCandidate::impact`: the load's cell swap (`cellSwapImpact`), and the target
/// driver's instance in the effect cone too (it sees the smaller pin).
fn size_down_fanout_mt_impact(target: &Target, names: &ImpactNames<'_>, load_inst: &str) -> MoveImpact {
    let mut impact = cell_swap_impact(target, names, load_inst);
    append_unique(&mut impact.timing_effect_instances, Some(names.instance(&target.driver_pin)));
    impact
}

/// The step down's target load change when `generate` keeps it: it keeps the output limits,
/// fits the delay budget, and has a load change.
fn size_down_fanout_mt_step_down(load: &SizeDownFanoutLoad) -> Option<f32> {
    let step_down = &load.step_down;
    if step_down.violates_output_limits || !fits_delay_budget(load.delay_budget, step_down.worst_delay_change) {
        return None;
    }
    step_down.target_load_delta
}

/// `fitsDelayBudget` (Mt): a positive budget bounds the delay change; zero or a negative one
/// allows no growth.
fn fits_delay_budget(delay_budget: f32, worst_delay_change: f32) -> bool {
    if delay_budget > 0.0 { worst_delay_change <= delay_budget } else { worst_delay_change <= 0.0 }
}

/// `SizeDownFanoutMtCandidate::estimate`: the window estimate of the load change on the target
/// net, legal as it says; the raw gain when legal or non-improving; scored by the gain times the
/// score scale. No side costs.
fn size_down_fanout_mt_estimate_of(d: &crate::delay_estimator::DelayEstimate, score_scale: f32) -> Estimate {
    let raw = (d.legal || d.reason == crate::delay_estimator::FailReason::EstimateNonImproving).then_some(d.arrival_impr);
    Estimate { legal: d.legal, score: d.arrival_impr * score_scale, raw_gain: raw, reason: d.reason.name(), tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None }
}

/// `SplitLoadMtGenerator::generate`: one split when `prepareInputs` left an input.
fn split_load_mt_generate(target: &Target, libs: &crate::preamble::Libs, cfg: &MtConfig, names: &ImpactNames<'_>) -> Vec<Candidate> {
    let (Some(input), Some(context)) = (&target.split_load_input, &target.estimator_context) else { return Vec::new() };
    let dash = |c: &str| if c.is_empty() { "-".to_string() } else { c.to_string() };
    vec![Candidate {
        kind: Move::SplitLoad,
        estimate: split_load_mt_estimate(libs, context, input, cfg.split_load_mt_score_scale),
        desc: format!("{}/{}", dash(&input.buffer_cell), dash(&input.resized_cell)),
        impact: split_load_mt_impact(target, names, input),
        tree: None, rebuffer_side_pins: Vec::new(),
    }]
}

/// `SplitLoadMtCandidate::estimate`: the driver sheds the moved loads (`estimateLoadChange`); a
/// reject keeps its scaled score and, non-improving, its raw gain. Else the moved loads arrive
/// through the faster driver (its arc delay at the reduced load) and the new buffer (the resized
/// cell at the current slew on the moved pin and wire cap; either missing:
/// `missing_buffer_timing_arc`), and pay that delay as side loads (`chargeMovedLoads`); the gain
/// less the side loss, scaled, is the score; a gain left at or below zero is
/// `side_loss_non_improving`, tail-admissible for a positive own gain.
fn split_load_mt_estimate(libs: &crate::preamble::Libs, context: &crate::delay_estimator::EstimatorContext, input: &SplitLoadInput, score_scale: f32) -> Estimate {
    use crate::delay_estimator::{self as de, FailReason};
    let stage = context.target();
    let mut d = de::estimate_load_change(libs, context, input.target_load_delta);
    let none = Estimate { legal: false, score: 0.0, raw_gain: None, reason: "", tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None };
    if !d.legal {
        return Estimate { score: d.arrival_impr * score_scale, raw_gain: (d.reason == FailReason::EstimateNonImproving).then_some(d.arrival_impr), reason: d.reason.name(), ..none };
    }
    let driver_delay = de::find_arc_delay(libs, &stage.arc, stage.input_slew, de::load_after_change(stage.load_cap, input.target_load_delta), &stage.arc.cell);
    let buffer_delay = de::find_buffer_delay(libs, context, &input.resized_cell, stage.current_slew, input.moved_pin_cap + input.moved_wire_cap);
    let (Some(driver_delay), Some(buffer_delay)) = (driver_delay, buffer_delay) else { return Estimate { reason: FailReason::MissingBufferTimingArc.name(), ..none } };
    let moved_delta = driver_delay - stage.current_model_delay + buffer_delay;
    de::charge_moved_loads(context, &input.moved_loads, moved_delta, &mut d);
    split_load_mt_score(&d, score_scale)
}

/// The end of [`split_load_mt_estimate`]: legal when the estimate is and the gain less the side
/// loss is above zero; scored by that times the scale.
fn split_load_mt_score(d: &crate::delay_estimator::DelayEstimate, score_scale: f32) -> Estimate {
    let gain = d.arrival_impr - d.side_loss;
    let mut e = Estimate { legal: d.legal && gain > 0.0, score: gain * score_scale, raw_gain: Some(d.arrival_impr), reason: d.reason.name(), tail_admissible: false, side_loss: Some(d.side_loss), sequential_penalty: None, sequential_side_loss: None };
    if d.legal && gain <= 0.0 {
        e.reason = "side_loss_non_improving";
        e.tail_admissible = d.arrival_impr > 0.0;
    }
    e
}

/// `SplitLoadMtCandidate::impact`: the base impact, complete; the driver's instance and its net
/// written and in the effect cone; the moved load pins reconnect, in order, their instances
/// reconnected and in the effect cone, each once.
fn split_load_mt_impact(target: &Target, names: &ImpactNames<'_>, input: &SplitLoadInput) -> MoveImpact {
    let mut impact = move_candidate_impact(target, names);
    impact.complete = true;
    let driver = names.instance(&target.driver_pin);
    append_unique(&mut impact.write_instances, Some(driver.clone()));
    append_unique(&mut impact.timing_effect_instances, Some(driver));
    append_unique(&mut impact.write_nets, Some(input.drvr_net.clone()));
    append_unique(&mut impact.timing_effect_nets, Some(input.drvr_net.clone()));
    for pin in &input.load_pins {
        impact.reconnect_pins.push(pin.clone());
        append_unique(&mut impact.reconnect_instances, Some(names.instance(pin)));
        append_unique(&mut impact.timing_effect_instances, Some(names.instance(pin)));
    }
    impact
}

/// `estimateTargets`: each target's candidates estimated in order (done at generation here; the
/// estimates read nothing the other targets change), then `estimateTarget` on each. The trace prints every candidate, then
/// the best, per target; the best only once every generator is modelled.
fn pe_estimate_targets(pe: &mut ParallelEval, targets: &[Target], generated: Vec<Vec<Candidate>>) -> Vec<TargetEvaluation> {
    let all = pe.generators.iter().all(|&m| generate_modelled(m));
    let permissive = pe_wns_tail_active(pe);
    let mut trace = String::new();
    let mut evaluations = Vec::with_capacity(generated.len());
    for (target, mut candidates) in targets.iter().zip(generated) {
        let best = estimate_target(&mut candidates, permissive);
        for (index, c) in candidates.iter().enumerate() {
            trace.push_str(&pe_cand_line(pe.batch_index, &target.driver_pin, index, c));
        }
        if let (true, Some(k)) = (all, best) {
            trace.push_str(&format!("VYGP|best|{}|{}|{}|{}\n", pe.batch_index, target.driver_pin, candidates[k].kind.name(), e9i(candidates[k].estimate.score)));
        }
        evaluations.push(TargetEvaluation { candidates, best });
    }
    pe_trace(&trace);
    // The candidate counters fold in after the estimates.
    for evaluation in &evaluations {
        pe.work.add_estimates(evaluation.candidates.len());
    }
    evaluations
}

/// `selectBatch`: the pool's batch candidates (`makeBatchCandidates`), then the selection
/// (`BatchSelector::select`), with the worst-endpoint reserve while WNS is stalled.
fn pe_select_batch(pe: &ParallelEval, pool: &mut BatchPool, batch_cap: i32) -> Result<(), Stop> {
    pool.candidates = pe_make_batch_candidates(pe, &pool.targets, &pool.evaluations)?;
    let mut t = pe_batch_candidate_lines(pe, pool);
    pool.selection = BatchSelector::new(pe.selector).select(&pool.candidates, batch_cap, pe_stall_wns_stalled(pe), &mut t);
    t.push_str(&pe_bc_lines(pe, pool));
    let indices: Vec<String> = pool.selection.evaluation_indices.iter().map(usize::to_string).collect();
    t.push_str(&format!("VYGP|sel|{}|cap={batch_cap}|{}\n", pe.batch_index, indices.join(",")));
    pe_trace(&t);
    Ok(())
}

/// `StallMonitor::wnsStalled`: the last saved batch did not move WNS past the best checkpoint.
fn pe_stall_wns_stalled(pe: &ParallelEval) -> bool {
    pe.stall.wns_stall_batches > 0
}

/// `recordSelection` (the profiler on): each selected best candidate (`moveSelected`), then the
/// selection's stats and its deferrals per conflict reason.
fn pe_record_selection(pool: &BatchPool) {
    for &index in &pool.selection.evaluation_indices {
        let kind = pool.evaluations[index].best_candidate().map_or("", |c| c.kind.name());
        pe_event("candidate_selected", "selected", kind, 1);
    }
    let stats = &pool.selection.stats;
    pe_event("batch_considered", "", "", stats.considered as usize);
    pe_event("batch_limited", "", "", stats.batch_limited as usize);
    pe_event("score_limited", "", "", stats.score_limited as usize);
    for reason in BatchConflictReason::ALL {
        pe_event(&format!("conflict_{}", reason.name()), "", "", stats.deferred[reason as usize] as usize);
    }
}

/// A profiler event as the printer writes it (`VYGP|ev|name|metric=|move=|count=`).
fn pe_event(name: &str, metric: &str, kind: &str, count: usize) {
    pe_trace(&format!("VYGP|ev|{name}|metric={metric}|move={kind}|count={count}\n"));
}

/// `makeBatchCandidates`: one batch candidate per evaluation with a best candidate, in pool order:
/// its index, the best estimate's score as the local score, the priority score (the local score;
/// while saved batches stall WNS, weighted by the target's share of the violating cone,
/// `bottleneckPriorityScore`), the target's endpoint slack, and the best candidate's impact.
fn pe_make_batch_candidates(pe: &ParallelEval, targets: &[Target], evaluations: &[TargetEvaluation]) -> Result<Vec<BatchCandidate>, Stop> {
    // A stalled batch ranks the targets by their share of the violating cone.
    let weighted = pe_stall_weights_priority(pe);
    let priority = pe.config.parallel_eval.priority;
    let mut candidates = Vec::with_capacity(evaluations.len());
    for (index, evaluation) in evaluations.iter().enumerate() {
        let Some(best) = evaluation.best_candidate() else { continue };
        let local_score = best.estimate.score;
        let priority_score = if weighted {
            // `target().vertex()`: the target pin's driver vertex.
            let pin = &targets[index].driver_pin;
            let key = [(pin.clone(), true), (pin.clone(), false)].into_iter().find(|k| pe.bottleneck.severity.contains_key(k));
            pe.bottleneck.priority_score(local_score, key.as_ref(), &priority)
        } else {
            local_score
        };
        candidates.push(BatchCandidate { evaluation_index: index, local_score, priority_score, endpoint_slack: targets[index].slack, impact: best.impact.clone() });
    }
    Ok(candidates)
}

/// `StallMonitor::weightsPriority`: a saved batch left WNS unchanged, for at most
/// `bottleneck_max_wns_stall_batches` such batches.
fn pe_stall_weights_priority(pe: &ParallelEval) -> bool {
    pe.stall.weights_priority(&pe.config.parallel_eval)
}

/// The trace's batch-candidate lines as the printer writes them after `makeBatchCandidates`:
/// `bmk` (weighted, count) and one `bimp` per candidate (its move and whole impact).
fn pe_batch_candidate_lines(pe: &ParallelEval, pool: &BatchPool) -> String {
    let list = |xs: &[String]| xs.iter().map(|x| format!(",{x}")).collect::<String>();
    let mut t = format!("VYGP|bmk|{}|weighted={}|n={}\n", pe.batch_index, u8::from(pe_stall_weights_priority(pe)), pool.candidates.len());
    for bc in &pool.candidates {
        let i = &bc.impact;
        let kind = pool.evaluations[bc.evaluation_index].best_candidate().map_or("-", |c| c.kind.name());
        t.push_str(&format!(
            "VYGP|bimp|{}|{}|{kind}|{}:wi{}:wn{}:rp{}:ri{}:tei{}:ten{}:tdi{}:tdn{}\n",
            pe.batch_index,
            bc.evaluation_index,
            if i.complete { "c" } else { "i" },
            list(&i.write_instances),
            list(&i.write_nets),
            list(&i.reconnect_pins),
            list(&i.reconnect_instances),
            list(&i.timing_effect_instances),
            list(&i.timing_effect_nets),
            list(&i.timing_dependency_instances),
            list(&i.timing_dependency_nets)
        ));
    }
    t
}

/// The trace's `bc` lines, printed after the selection: each batch candidate's scores and
/// endpoint slack.
fn pe_bc_lines(pe: &ParallelEval, pool: &BatchPool) -> String {
    let mut t = String::new();
    for bc in &pool.candidates {
        t.push_str(&format!("VYGP|bc|{}|{}|local={}|priority={}|eslack={}\n", pe.batch_index, bc.evaluation_index, e9i(bc.local_score), e9i(bc.priority_score), e9i(bc.endpoint_slack)));
    }
    t
}

/// `WnsTail::active()`: the tail runs (it starts once the batches stall).
fn pe_wns_tail_active(pe: &ParallelEval) -> bool {
    pe.tail.active
}

/// `estimateTarget`: while the WNS tail is active (`permissive`), a candidate not legal but
/// tail-admissible becomes legal (`wns_tail_permissive`); the best is the first legal candidate
/// whose score is strictly greater than the best so far.
fn estimate_target(candidates: &mut [Candidate], permissive: bool) -> Option<usize> {
    let mut best: Option<(usize, f32)> = None;
    for (k, c) in candidates.iter_mut().enumerate() {
        if permissive && !c.estimate.legal && c.estimate.tail_admissible {
            c.estimate.legal = true;
            c.estimate.reason = "wns_tail_permissive";
        }
        if c.estimate.legal && best.is_none_or(|(_, s)| c.estimate.score > s) {
            best = Some((k, c.estimate.score));
        }
    }
    best.map(|(k, _)| k)
}

/// The trace's candidate line: estimate, description and impact as the printer writes them.
fn pe_cand_line(batch: i32, driver_pin: &str, index: usize, c: &Candidate) -> String {
    let opt = |v: Option<f32>| v.map_or_else(|| "-".to_string(), e9i);
    let e = &c.estimate;
    let i = &c.impact;
    let list = |xs: &[String]| xs.iter().map(|x| format!(",{x}")).collect::<String>();
    format!(
        "VYGP|cand|{batch}|{driver_pin}|{index}|{}|legal={}|score={}|raw={}|reason={}|tail={}|side={}|seq={}|seqside={}|desc={}|impact={}:wi{}:wn{}:rp{}\n",
        c.kind.name(), u8::from(e.legal), e9i(e.score), opt(e.raw_gain), e.reason, u8::from(e.tail_admissible), opt(e.side_loss), opt(e.sequential_penalty), opt(e.sequential_side_loss), c.desc,
        if i.complete { "c" } else { "i" }, list(&i.write_instances), list(&i.write_nets), list(&i.reconnect_pins)
    )
}

/// What `prepareInputs` reads while a chunk is planned: the timer, the database and the committer.
struct PlanTimer<'p, 'g, 's> {
    gs: &'p [vyges_sta::graph::Graph<'g>],
    ss: &'p [vyges_sta::search::Search<'g, 's>],
    design: &'p dyn super::Design,
    ctx: &'p super::Ctx<'p>,
    committer: &'p super::Committer,
    /// The instance's odb id, as the committer keys its moves.
    inst_id: &'p dyn Fn(&str) -> u32,
    /// What the candidates' `impact()` reads off the network.
    names: &'p ImpactNames<'p>,
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

/// `WnsTail::unbufferAllowed`: `active && batch_cap == 1 && greedy_stalled`.
fn pe_tail_unbuffer_allowed(pe: &ParallelEval) -> bool {
    pe.tail.unbuffer_allowed()
}

/// `kRebufferMaxFanout`: RebufferMt plans trees for fewer loads than this.
const REBUFFER_MAX_FANOUT: usize = 20;

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
        // RebufferMtGenerator: one to kRebufferMaxFanout - 1 (19) loads, a net that accepts a
        // buffer, and an estimator context.
        Move::Buffer => target.fanout >= 1 && target.fanout < REBUFFER_MAX_FANOUT && facts.ok_to_buffer_net && target.estimator_context.is_some(),
        // SizeDownFanoutMtGenerator: an estimator context.
        Move::SizeDownFanout => target.estimator_context.is_some(),
        Move::SwapPins => target.path_index > 0 && facts.driver_liberty && !facts.driver_is_buffer && !facts.driver_is_inverter && !target.inst.is_empty() && !facts.dont_touch && !facts.pins_swapped,
        _ => true,
    }
}

/// `MoveGenerator::prepareInputs`: the generator's live timer reads, stored on the target.
#[allow(clippy::too_many_arguments)]
fn pe_prepare_inputs(m: Move, target: &mut Target, facts: &PlanFacts, plan: &PlanTimer<'_, '_, '_>, cfg: &MtConfig, crit: &HashSet<String>) -> Result<(), String> {
    let (gs, ss, design, ctx) = (plan.gs, plan.ss, plan.design, plan.ctx);
    match m {
        Move::Unbuffer => unbuffer_mt_prepare_inputs(target, facts, gs, ss, design, ctx),
        Move::SwapPins => swap_pins_mt_prepare_inputs(target, gs, ss, design, ctx, cfg)?,
        // SizeUpMt also builds its replacement options here (a cache generate reads; it decides
        // nothing yet).
        Move::SizeUp | Move::VtSwap => prepare_input_cap_budgets(target, gs, design, ctx),
        Move::SplitLoad => split_load_mt_prepare_inputs(target, gs, ss, design, ctx, crit, cfg)?,
        Move::SizeDownFanout => size_down_fanout_mt_prepare_inputs(target, plan)?,
        Move::Buffer => rebuffer_mt_prepare_inputs(target, plan, cfg, crit)?,
        _ => {}
    }
    Ok(())
}

/// `RebufferMtGenerator::prepareInputs`: `Rebuffer::makeBufferedNet(driver)` — none for a top-level
/// port driver, a pin off a net or without a liberty port, or a net with a top-level output port;
/// else `Resizer::makeBufferedNet` at the rebuffer's scene (`cmdScene`, scene 0) — then, with the
/// side-load model on, the loads' timing (`collectLoadTiming`), else none.
/// ⚠️ Upstream warns for a top-level port driver (RSZ-2020) and a failed buffered net here; the
/// run is refused before either would be compared.
fn rebuffer_mt_prepare_inputs(target: &mut Target, plan: &PlanTimer<'_, '_, '_>, cfg: &MtConfig, crit: &HashSet<String>) -> Result<(), String> {
    target.rebuffer_net = rebuffer_make_buffered_net(plan, &target.driver_pin)?;
    target.rebuffer_load_timing.clear();
    if cfg.rebuffer_mt_side_loads {
        target.rebuffer_load_timing = rebuffer_collect_load_timing(target, plan, cfg, crit);
    }
    Ok(())
}

/// `Rebuffer::makeBufferedNet` (see [`rebuffer_mt_prepare_inputs`]).
fn rebuffer_make_buffered_net(plan: &PlanTimer<'_, '_, '_>, drvr_pin: &str) -> Result<Option<(crate::buffered_net::BufferedNet, usize)>, String> {
    let (g, design, ctx) = (&plan.gs[0], plan.design, plan.ctx);
    let Some(drvr) = g.vertices.iter().position(|x| x.name == drvr_pin && x.is_driver) else { return Ok(None) };
    let vx = &g.vertices[drvr];
    let Some(lib) = vx.lib else { return Ok(None) };
    let Some(net) = g.vertex_net[drvr] else { return Ok(None) };
    let (Some(cell), Some(port)) = (vx.cell.as_deref(), vx.port.as_deref()) else { return Ok(None) };
    if g.libs[lib].cells.get(cell).is_none_or(|c| c.port(port).is_none()) {
        return Ok(None);
    }
    let top_out = g.netlist.nets[net].pins.iter().any(|c| matches!(c, vyges_sta::netlist::Conn::Port(k) if g.netlist.ports[*k].1 == vyges_sta::netlist::PortDir::Output));
    if top_out {
        return Ok(None);
    }
    let Some(shape) = design.net_shape(&g.netlist.nets[net].name, &vx.name) else { return Ok(None) };
    let bctx = crate::buffered_net::Ctx { graph: g, libs: ctx.libs, sdc: ctx.env, limits: ctx.limits, dbu: ctx.dbu, rc: ctx.wire_rc };
    crate::buffered_net::make_buffered_net(&bctx, &shape)
}

/// `collectLoadTiming`: per wire edge out of the driver, a load whose slack (over every scene, at
/// the target arc's min/max) is finite: that slack, the edge's wire delay for the arc's output
/// transition at the arc's scene, and the load's side weight (`sideLoadWeight`).
fn rebuffer_collect_load_timing(target: &Target, plan: &PlanTimer<'_, '_, '_>, cfg: &MtConfig, crit: &HashSet<String>) -> Vec<RebufferLoadTiming> {
    let mut out = Vec::new();
    let Some(context) = target.estimator_context.as_ref() else { return out };
    let arc = &context.target().arc;
    let Some(out_rf) = plan.ctx.libs.scene_cell(arc.scene, &arc.cell).map(|c| c.arc_sets[arc.set].arcs[arc.arc].to_rf) else { return out };
    let g = &plan.gs[arc.scene];
    let Some(drvr) = g.vertices.iter().position(|x| x.name == target.driver_pin && x.is_driver) else { return out };
    let weights = crate::delay_estimator::FaninSideLoadOptions { critical_load_pins: crit, off_path_weight: cfg.fanin_side_offpath_weight };
    for &e in &g.out_edges[drvr] {
        if !matches!(g.edges[e].kind, vyges_sta::graph::EdgeKind::Wire) {
            continue;
        }
        let pin = &g.vertices[g.edges[e].to].name;
        let slack = pe_slack_over_scenes(plan.gs, plan.ss, pin, false);
        if crate::delay_estimator::delay_inf(slack) {
            continue;
        }
        out.push(RebufferLoadTiming { pin: pin.clone(), slack, wire_delay: g.delay[e][out_rf][MAX], weight: crate::delay_estimator::side_load_weight(pin, &weights) });
    }
    out
}

/// `SizeDownFanoutMtGenerator::prepareInputs`: the driver's liberty port, then `measureLoads` —
/// the wire fanouts not sized down already, by slack (greater first, then name), each measured
/// at the target's scene (`measureLoad`): it may change (`resolveLoad`), its delay budget
/// (`computeDelayBudget`), its swappable cells ranked (`rankSwappableCells`), the one step down
/// (the largest smaller input pin at no more area; the last of equal ones), its outputs as timed
/// now (`measureOutputs`) and the step's measurements (`measureCellOption`). A load with no step
/// down is left out.
fn size_down_fanout_mt_prepare_inputs(target: &mut Target, plan: &PlanTimer<'_, '_, '_>) -> Result<(), String> {
    let (gs, ss, ctx) = (plan.gs, plan.ss, plan.ctx);
    let scene = target.scene;
    let g = &gs[scene];
    // resolveDriverContext.
    let Some(drvr) = g.vertices.iter().position(|x| x.name == target.driver_pin && x.is_driver) else { return Ok(()) };
    let Some(drvr_cell) = ctx.libs.link_cell(&target.cell) else { return Ok(()) };
    let Some(drvr_port) = target.driver_pin.rsplit_once('/').map(|(_, p)| p).filter(|p| drvr_cell.port(p).is_some()) else { return Ok(()) };
    let drvr_res = drvr_cell.drive_resistance(drvr_port);
    let Some(arc_scene) = target.estimator_context.as_ref().map(|c| c.target().arc.scene) else { return Ok(()) };
    // sortedFanoutSlacks.
    let mut fanouts: Vec<(usize, f32)> = Vec::new();
    for &e in &g.out_edges[drvr] {
        if !matches!(g.edges[e].kind, vyges_sta::graph::EdgeKind::Wire) {
            continue;
        }
        let v = g.edges[e].to;
        if let vyges_sta::netlist::Conn::Inst(k, _) = &g.vertices[v].conn {
            if plan.committer.has_moves(Move::SizeDownFanout, (plan.inst_id)(&g.netlist.insts[*k].0)) {
                continue;
            }
        }
        fanouts.push((v, pe_slack_over_scenes(gs, ss, &g.vertices[v].name, false)));
    }
    crate::order::libcxx_sort_by(&mut fanouts, |a, b| a.1 > b.1 || (a.1 == b.1 && g.vertices[a.0].name < g.vertices[b.0].name))
        .map_err(|e| format!("the size-down fanout sort over {} loads falls back to heap sort: not modelled", e.len))?;
    let mut loads = Vec::new();
    for (v, slack) in fanouts {
        if let Some(load) = size_down_measure_load(plan, scene, arc_scene, drvr_res, v, slack)? {
            loads.push(load);
        }
    }
    target.size_down_fanout_loads = loads;
    Ok(())
}

/// `Sta::slack(vertex, max)`: the pin's worst slack over every scene (its load vertex, or its
/// driver vertex when `driver`).
fn pe_slack_over_scenes(gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], pin: &str, driver: bool) -> f32 {
    let g0 = &gs[0];
    let v = g0.vertices.iter().position(|x| x.name == pin && x.is_driver == driver).or_else(|| g0.vertices.iter().position(|x| x.name == pin));
    v.map_or(super::INF, |v| super::slack_over_scenes(gs, ss, v, MAX, None))
}

/// `measureLoad` (see [`size_down_fanout_mt_prepare_inputs`]).
fn size_down_measure_load(plan: &PlanTimer<'_, '_, '_>, scene: usize, arc_scene: usize, drvr_res: f32, v: usize, load_slack: f32) -> Result<Option<SizeDownFanoutLoad>, String> {
    let (gs, ss, design, ctx) = (plan.gs, plan.ss, plan.design, plan.ctx);
    let g = &gs[scene];
    let x = &g.vertices[v];
    // resolveLoad: a liberty port on an instance that is neither dont_touch nor fixed, of a
    // logic standard cell, with a port at the scene.
    let (Some(cell_name), Some(port)) = (x.cell.clone(), x.port.clone()) else { return Ok(None) };
    let Some(cell) = ctx.libs.link_cell(&cell_name) else { return Ok(None) };
    let vyges_sta::netlist::Conn::Inst(k, _) = &x.conn else { return Ok(None) };
    let (inst, master) = (&g.netlist.insts[*k].0, &g.netlist.insts[*k].1);
    let ni = design.net_info();
    if ni.dont_touch_insts.contains(inst) || ni.fixed_insts.contains(inst) || !ctx.sizing.masters.get(&cell_name).is_some_and(|m| m.logic_std) {
        return Ok(None);
    }
    let at_scene = |k: usize, n: &str| ctx.libs.scene_cell(k, n).or_else(|| ctx.libs.link_cell(n));
    let Some(scene_cell) = at_scene(scene, &cell_name) else { return Ok(None) };
    let Some(input_cap) = super::port_capacitance(scene_cell, &port) else { return Ok(None) };
    let pins = ctx.master_pins.get(master).map_or(&[][..], |p| p.as_slice());
    let pin_of = |p: &str| g.netlist.pin_name(&vyges_sta::netlist::Conn::Inst(*k, p.to_string()));
    let direction = |p: &str| cell.port(p).map(|q| q.direction);
    // computeDelayBudget: a register spends its worst output slack; a gate the worse of the
    // pin's slack and its worst input slack.
    let sequential = !cell.sequentials.is_empty() || cell.has_seq_bank;
    let delay_budget = if sequential {
        pins.iter().filter(|p| direction(p) == Some(vyges_sta::liberty::Direction::Output)).fold(super::INF, |w, p| crate::rebuffer::std_min(w, pe_slack_over_scenes(gs, ss, &pin_of(p), true)))
    } else {
        let worst_input = pins.iter().filter(|p| direction(p) == Some(vyges_sta::liberty::Direction::Input)).fold(super::INF, |w, p| crate::rebuffer::std_min(w, pe_slack_over_scenes(gs, ss, &pin_of(p), false)));
        crate::rebuffer::std_min(load_slack, worst_input)
    };
    // rankSwappableCells: by input capacitance at the scene, then the higher worst intrinsic
    // delay first; a cell without the port last.
    let mut swaps: Vec<String> = if cell.is_buffer() {
        if ctx.fast_buffers.contains(&cell_name) {
            let mut v: Vec<String> = ctx.fast_buffers.to_vec();
            v.sort_by(|a, b| {
                let (ca, cb) = (ctx.libs.link_cell(a).map_or(0.0, |c| c.area), ctx.libs.link_cell(b).map_or(0.0, |c| c.area));
                ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal).then_with(|| a.cmp(b))
            });
            v
        } else {
            Vec::new()
        }
    } else {
        ctx.sizing.swappable_cells(&cell_name).map_err(|e| format!("{e:?}"))?
    };
    if swaps.len() > 1 {
        let key = |n: &str| -> Option<(f32, f32)> {
            let c = at_scene(scene, n)?;
            Some((super::port_capacitance(c, &port)?, super::worst_intrinsic_delay(c, &port)))
        };
        crate::order::libcxx_sort_by(&mut swaps, |a, b| match (key(a), key(b)) {
            (Some((c1, i1)), Some((c2, i2))) => (c1, i2) < (c2, i1),
            (x, _) => x.is_some(),
        })
        .map_err(|e| format!("the swappable cell sort over {} cells falls back to heap sort: not modelled", e.len))?;
    }
    // candidateInputCap: INF without the port at the scene.
    let cap_of = |n: &str| at_scene(scene, n).and_then(|c| super::port_capacitance(c, &port)).unwrap_or(super::INF);
    let mut step: Option<(String, f32)> = None;
    for s in &swaps {
        let c = cap_of(s);
        if *s == cell_name || c >= input_cap || ctx.libs.link_cell(s).map_or(super::INF, |c| c.area) > cell.area {
            continue;
        }
        if step.as_ref().is_none_or(|(_, sc)| c >= *sc) {
            step = Some((s.clone(), c));
        }
    }
    let Some((step, step_cap)) = step else { return Ok(None) };
    // measureOutputs: per output pin in pin order, its load at the scene, its slew over every
    // scene and transition, the Elmore slew factor and the gate delay now.
    let mut outputs = Vec::new();
    for p in pins.iter().filter(|p| direction(p) == Some(vyges_sta::liberty::Direction::Output)) {
        let name = pin_of(p);
        let Some(vo) = g.vertices.iter().position(|y| y.name == name && !y.is_driver).or_else(|| g.vertices.iter().position(|y| y.name == name)) else { continue };
        let cap = g.load_cap(vo, design.parasitics(scene));
        let mut slew = -super::INF;
        for gk in gs {
            if let Some(w) = gk.vertices.iter().position(|y| y.name == name && !y.is_driver).or_else(|| gk.vertices.iter().position(|y| y.name == name)) {
                slew = slew.max(gk.slew[w][0][MAX]).max(gk.slew[w][1][MAX]);
            }
        }
        let res = cell.drive_resistance(p);
        let slew_factor = if res <= 0.0 || cap <= 0.0 { 0.0 } else { slew / (res * cap) };
        let delay = super::gate_delay(ctx.sizing, scene_cell, p, cap);
        outputs.push(SizeDownOutput { pin: name, port: p.clone(), cap, slew_factor, delay });
    }
    // measureCellOption.
    let mut option = SizeDownFanoutCellOption { cell: step.clone(), input_cap: step_cap, violates_output_limits: size_down_violates_output_limits(ctx, scene, &outputs, &step), worst_delay_change: 0.0, target_load_delta: None };
    if !option.violates_output_limits {
        // computeWorstDelayChange: the driver gains from the smaller pin, but for a register.
        let drvr_delta = -drvr_res * (input_cap - step_cap);
        let step_scene = at_scene(scene, &step);
        let mut worst = -super::INF;
        for o in &outputs {
            let new_delay = step_scene.map_or(-super::INF, |c| super::gate_delay(ctx.sizing, c, &o.port, o.cap));
            let change = if sequential { new_delay - o.delay } else { new_delay + drvr_delta - o.delay };
            worst = crate::rebuffer::std_max(worst, change);
        }
        option.worst_delay_change = worst;
    }
    let cur = at_scene(arc_scene, &cell_name).and_then(|c| super::port_capacitance(c, &port));
    let new = at_scene(arc_scene, &step).and_then(|c| super::port_capacitance(c, &port));
    if let (Some(cur), Some(new)) = (cur, new) {
        option.target_load_delta = Some(new - cur);
    }
    Ok(Some(SizeDownFanoutLoad { inst: inst.clone(), pin: x.name.clone(), port, cell: cell_name, load_slack, delay_budget, input_cap, step_down: option }))
}

/// `violatesOutputLimits`: the cell lacks an output's port, or the output's load is over the
/// port's max capacitance, or its Elmore-scaled slew over the port's slew limit at the scene.
fn size_down_violates_output_limits(ctx: &super::Ctx<'_>, scene: usize, outputs: &[SizeDownOutput], cell: &str) -> bool {
    let Some(c) = ctx.libs.link_cell(cell) else { return true };
    for o in outputs {
        let Some(p) = c.port(&o.port) else { return true };
        if p.max_capacitance.is_some_and(|m| m > 0.0 && o.cap > m) {
            return true;
        }
        let new_slew = o.slew_factor * c.drive_resistance(&o.port) * o.cap;
        if let Some(lib) = ctx.libs.scene_library(scene, cell).or_else(|| ctx.libs.default_library()) {
            let (max_slew, exists) = crate::timing::find_slew_limit(lib, p.direction, p.max_transition, ctx.limits);
            if exists && new_slew > max_slew {
                return true;
            }
        }
    }
    false
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
        Move::Buffer => {
            // The printer sorts the map's rows as text.
            let mut rows: Vec<String> = target.rebuffer_load_timing.iter().map(|l| format!("{}:{}:{}:{}", l.pin, e9i(l.slack), e9i(l.wire_delay), e9i(l.weight))).collect();
            rows.sort();
            let mut s = format!("net={}|timing={}", target.rebuffer_net.is_some() as i32, rows.len());
            for r in rows {
                s.push('|');
                s.push_str(&r);
            }
            s
        }
        Move::SizeDownFanout => {
            let mut s = format!("loads={}", target.size_down_fanout_loads.len());
            for l in &target.size_down_fanout_loads {
                let d = &l.step_down;
                s.push_str(&format!(
                    "|{}:{}:slack={}:budget={}:cap={}:down={}:{}:{}:{}:{}",
                    l.pin,
                    l.cell,
                    e9i(l.load_slack),
                    e9i(l.delay_budget),
                    e9i(l.input_cap),
                    d.cell,
                    e9i(d.input_cap),
                    d.violates_output_limits as i32,
                    e9i(d.worst_delay_change),
                    d.target_load_delta.map_or("-".into(), e9i)
                ));
            }
            s
        }
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
/// Returns the sequential estimator's trace lines (empty unless one was prepared).
#[allow(clippy::too_many_arguments)]
fn pe_prepare_target(target: &mut Target, mask: u8, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], cs: &[std::collections::BTreeSet<usize>], design: &dyn super::Design, ctx: &super::Ctx<'_>, inputs: &PrepareInputs<'_>) -> Result<String, Stop> {
    if mask & ESTIMATOR_CONTEXT_CACHE != 0 {
        pe_prepare_estimator_context(target, gs, ss, cs, design, ctx, inputs);
    }
    if mask & SEQUENTIAL_ESTIMATOR_CACHE != 0 {
        return pe_prepare_sequential_estimator(target, gs, ss, cs, design, ctx, inputs);
    }
    Ok(String::new())
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
    let reader = Reader { libs: ctx.libs, g, parasitics: design.parasitics(scene), scene, equiv: inputs.equiv, dont_use: inputs.dont_use, slack: &slack, clock_pins: &cs[scene], pin_id: inputs.pin_id, master_pins: ctx.master_pins, mm: vyges_sta::liberty::MAX };
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

/// `prepareSequentialEstimator`: once, with a context — `SequentialTimingEstimator::prepare`
/// (none unless the target is a flip-flop), its candidates the target's swappable and
/// VT-equivalent cells. Returns the estimator's trace lines.
#[allow(clippy::too_many_arguments)]
fn pe_prepare_sequential_estimator(target: &mut Target, gs: &[vyges_sta::graph::Graph<'_>], ss: &[vyges_sta::search::Search<'_, '_>], cs: &[std::collections::BTreeSet<usize>], design: &dyn super::Design, ctx: &super::Ctx<'_>, inputs: &PrepareInputs<'_>) -> Result<String, Stop> {
    if target.sequential_estimator.is_some() || target.estimator_context.is_none() {
        return Ok(String::new());
    }
    if !crate::sequential_timing::is_flip_flop(ctx.libs.link_cell(&target.cell)) {
        return Ok(String::new());
    }
    let swappable = ctx.sizing.swappable_cells(&target.cell)?;
    let vt_equiv = ctx.sizing.vt_equiv_cells(&target.cell, ctx.vt_category_count);
    let parasitics = |scene: usize| design.parasitics(scene);
    let timer = crate::sequential_timing::Timer { libs: ctx.libs, gs, ss, parasitics: &parasitics, equiv: inputs.equiv, dont_use: inputs.dont_use, clock_pins: cs, pin_id: inputs.pin_id, master_pins: ctx.master_pins };
    let output = target.driver_pin.rsplit_once('/').map_or("", |(_, port)| port);
    target.sequential_estimator = crate::sequential_timing::SequentialTimingEstimator::prepare(&timer, &target.inst, &target.cell, output, &swappable, &vt_equiv);
    Ok(target.sequential_estimator.as_ref().map(|e| pe_sequential_lines(&target.driver_pin, e, ctx.libs)).unwrap_or_default())
}

/// The VYGP `seqprep`, `seqchk` and `seqest` lines of a prepared sequential estimator: its
/// failure, data ports, candidates (by name) and check count; each check's end, slack, margin,
/// derate, related load, candidate models and inputs; each candidate's estimate.
fn pe_sequential_lines(drv: &str, e: &crate::sequential_timing::SequentialTimingEstimator, libs: &crate::preamble::Libs) -> String {
    use crate::sequential_timing::PreparedInputTiming;
    let mm_name = |mm: usize| if mm == MAX { "max" } else { "min" };
    let input = |i: &PreparedInputTiming| {
        let caps: Vec<String> = i.candidate_connected_input_caps.iter().map(|c| e9(*c)).collect();
        let drv = i.driver.as_ref().map_or("-".to_string(), |d| {
            let merge: Vec<String> = d.output_slew_merge_arcs.iter().map(|m| format!("{}/{}", e9(m.input_slew), e9(m.current_model_slew))).collect();
            format!("{}/{}/{}/{}/{}/{}/n{}/{}", d.arc.cell, e9(d.load_cap), e9(d.input_slew), e9(d.current_model_delay), e9(d.current_model_slew), e9(i.delay_derate), i.driver_gate_models.len(), merge.join(";"))
        });
        format!("{}:ideal={}:slew={}:cur={}:caps={}:drv={}", mm_name(i.mm), u8::from(i.ideal_clock), e9(i.slew), e9(i.current_connected_input_cap), caps.join(";"), drv)
    };
    let mut s = format!("VYGP|seqprep|{drv}|fail={}|ports={}|cands={}|checks={}\n", e.prepare_failure_reason, e.data_ports.join(","), e.candidates.join(","), e.checks.len());
    for (i, c) in e.checks.iter().enumerate() {
        let models: String = c.candidate_check_models.iter().map(|m| if m.is_some() { '1' } else { '0' }).collect();
        s += &format!(
            "VYGP|seqchk|{drv}|{i}|setup={}|data={}{}|clk={}{}|scene={}|mm={}|rel=-|slack={}|margin={}|cderate={}|rcap={}|models={models}|D={}|C={}|S={}\n",
            u8::from(c.setup), c.data_pin, rf_short(c.data_rf), c.clock_pin, rf_short(c.clock_rf), libs.scenes.get(c.scene).map_or("default", String::as_str), mm_name(c.mm),
            e9d(c.baseline_slack), e9(c.baseline_check_margin), e9(c.check_derate), e9(c.related_out_cap), input(&c.data), input(&c.clock), c.clock_check_slew_input.as_ref().map_or("-".to_string(), input),
        );
    }
    for cell in &e.candidates {
        let r = e.estimate(libs, cell);
        s += &format!("VYGP|seqest|{drv}|{cell}|legal={}|w={}|m={}|reason={}\n", u8::from(r.legal), e9(r.weighted_side_slack_loss), e9(r.max_side_slack_loss), r.reason);
    }
    s
}

/// The VYGP printer's `{:.9e}` of a value that may be infinite (fmt prints `inf`).
pub(super) fn e9i(v: f32) -> String {
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
    let mut s = format!("VYGP|prep|{drv}|mask={mask}|ctx={}|why={why}|seq={}\n", target.estimator_context.is_some() as i32, target.sequential_estimator.is_some() as i32);
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
    pe.tail.reset();
    pe.tabu.reset();
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
            sequential_estimator: None,
            prepare_why: String::new(),
            unbuffer_gain: None,
            swap_pins_input: None,
            input_cap_budgets: None,
            split_load_input: None,
            size_down_fanout_loads: Vec::new(),
            rebuffer_net: None,
            rebuffer_load_timing: Vec::new(),
            plan: Vec::new(),
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
    fn a_rebuffer_target_has_one_to_nineteen_loads_a_bufferable_net_and_a_context() {
        // Rule (RebufferMtGenerator::isApplicable): fanout >= 1 and below kRebufferMaxFanout (20),
        // okToBufferNet on the driver, and an estimator context.
        let mut t = with_context(target_at(2));
        for (fanout, ok) in [(0, false), (1, true), (19, true), (20, false)] {
            t.fanout = fanout;
            assert_eq!(pe_is_applicable(Move::Buffer, &t, &facts()), ok, "fanout {fanout}");
        }
        t.fanout = 3;
        assert!(!pe_is_applicable(Move::Buffer, &t, &PlanFacts { ok_to_buffer_net: false, ..facts() }));
        let mut u = target_at(2);
        u.fanout = 3;
        assert!(!pe_is_applicable(Move::Buffer, &u, &facts()));
    }

    #[test]
    fn the_trace_prints_rebuffer_inputs_with_the_rows_sorted_as_text() {
        // The printer: net=1 when a buffered net was made, the row count, then the rows
        // pin:slack:wire delay:weight sorted as strings (the map has no order of its own).
        let mut t = target_at(2);
        assert_eq!(pe_inputs_line(Move::Buffer, &t), "net=0|timing=0");
        t.rebuffer_load_timing = vec![
            RebufferLoadTiming { pin: "u2/A".into(), slack: 0.5, wire_delay: 0.0, weight: 1.0 },
            RebufferLoadTiming { pin: "u10/A".into(), slack: -0.5, wire_delay: 0.25, weight: 0.0 },
        ];
        assert_eq!(pe_inputs_line(Move::Buffer, &t), "net=0|timing=2|u10/A:-5.000000000e-01:2.500000000e-01:0.000000000e+00|u2/A:5.000000000e-01:0.000000000e+00:1.000000000e+00");
    }

    #[test]
    fn a_size_down_fanout_target_needs_only_a_context() {
        // Rule (SizeDownFanoutMtGenerator::isApplicable): the base check and an estimator
        // context; the driver's own cell, dont_touch or fixity do not matter (its loads change).
        let t = with_context(target_at(0));
        assert!(pe_is_applicable(Move::SizeDownFanout, &t, &PlanFacts { dont_touch: true, fixed: true, logic_std: false, ..facts() }));
        assert!(!pe_is_applicable(Move::SizeDownFanout, &target_at(0), &facts()));
    }

    #[test]
    fn the_trace_prints_size_down_loads_as_the_printer_does() {
        // The printer: `loads=N`, then per load pin:cell:slack:budget:cap and the step down's
        // cell:cap:violates:worst change:target delta (`-` when unset).
        let mut t = target_at(0);
        assert_eq!(pe_inputs_line(Move::SizeDownFanout, &t), "loads=0");
        let step_down = SizeDownFanoutCellOption { cell: "INV_X1".into(), input_cap: 0.25, violates_output_limits: true, worst_delay_change: 0.0, target_load_delta: None };
        t.size_down_fanout_loads = vec![SizeDownFanoutLoad { inst: "u1".into(), pin: "u1/A".into(), port: "A".into(), cell: "INV_X2".into(), load_slack: 0.5, delay_budget: -0.5, input_cap: 1.0, step_down }];
        assert_eq!(
            pe_inputs_line(Move::SizeDownFanout, &t),
            "loads=1|u1/A:INV_X2:slack=5.000000000e-01:budget=-5.000000000e-01:cap=1.000000000e+00:down=INV_X1:2.500000000e-01:1:0.000000000e+00:-"
        );
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

    // UnbufferMtGenerator::generate: one candidate only for a prepared gain above zero; its
    // estimate is always legal, scored and raw-gained by that gain, reason estimate_legal, with no
    // tail, side or sequential terms; vygDescribe is "remove".
    #[test]
    fn unbuffer_mt_generates_one_legal_removal_for_a_positive_gain() {
        let mut t = target_at(0);
        for gain in [None, Some(0.0), Some(-1e-12)] {
            t.unbuffer_gain = gain;
            assert!(unbuffer_mt_generate(&t, MoveImpact::default).is_empty(), "{gain:?}");
        }
        t.unbuffer_gain = Some(2.5e-11);
        let c = unbuffer_mt_generate(&t, MoveImpact::default);
        assert_eq!(c.len(), 1);
        assert_eq!((c[0].kind, c[0].desc.as_str()), (Move::Unbuffer, "remove"));
        assert_eq!(c[0].estimate, Estimate { legal: true, score: 2.5e-11, raw_gain: Some(2.5e-11), reason: "estimate_legal", tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None });
    }

    // UnbufferMtCandidate::impact: complete; the buffer written; its input then output net
    // written; the moved net's pins other than the buffer's reconnect, in the net's pin order. The
    // moved net is the output one, unless removeBufferKeepsOutputNet (the output net reaches a top
    // port and the input net does not), then the input one; port pins count as pins.
    #[test]
    fn unbuffer_mt_impact_reconnects_the_dropped_nets_other_pins() {
        use vyges_sta::netlist::{inst, Conn, Net, Netlist, PortDir};
        let net = |name: &str, pins: Vec<Conn>| std::sync::Arc::new(Net { name: name.into(), pins });
        let pin = |k: usize, p: &str| Conn::Inst(k, p.into());
        let nl = Netlist {
            insts: vec![inst("d", "INV"), inst("b", "BUF"), inst("l1", "INV"), inst("l2", "INV")],
            ports: vec![("out".into(), PortDir::Output), ("in".into(), PortDir::Input)],
            nets: vec![net("n_in", vec![pin(0, "ZN"), pin(1, "A")]), net("n_out", vec![pin(2, "A"), pin(1, "Z"), pin(3, "A")])],
        };
        let base = || MoveImpact { timing_dependency_nets: vec!["n_in".into()], ..MoveImpact::default() };
        let i = unbuffer_impact_on(base(), &nl, "b", Some(0), Some(1));
        assert!(i.complete);
        assert_eq!(i.write_instances, ["b"]);
        assert_eq!(i.write_nets, ["n_in", "n_out"]);
        assert_eq!(i.reconnect_pins, ["l1/A", "l2/A"]);
        // The moved net's instances reconnect; both nets' other instances (and the buffer, first)
        // are in the effect cone, pushed as met; the base impact's dependencies are kept.
        assert_eq!(i.reconnect_instances, ["l1", "l2"]);
        assert_eq!(i.timing_effect_instances, ["b", "d", "l1", "l2"]);
        assert_eq!(i.timing_effect_nets, ["n_in", "n_out"]);
        assert_eq!(i.timing_dependency_nets, ["n_in"]);
        // The output net reaches a port: the input net's driver moves instead.
        let mut kept = nl.clone();
        kept.nets[1] = net("n_out", vec![pin(2, "A"), pin(1, "Z"), Conn::Port(0)]);
        let k = unbuffer_impact_on(base(), &kept, "b", Some(0), Some(1));
        assert_eq!((k.reconnect_pins.as_slice(), k.reconnect_instances.as_slice()), (&["d/ZN".to_string()][..], &["d".to_string()][..]));
        // A port pin's instance is the top instance (Network::instance), not none: it is in the
        // effect cone.
        assert_eq!(k.timing_effect_instances, ["b", "d", "l1", TOP_INSTANCE]);
        // Both nets reach a port: the output net's pins move, its port among them.
        kept.nets[0] = net("n_in", vec![pin(0, "ZN"), pin(1, "A"), Conn::Port(1)]);
        let k = unbuffer_impact_on(base(), &kept, "b", Some(0), Some(1));
        assert_eq!((k.reconnect_pins.as_slice(), k.reconnect_instances.as_slice()), (&["l1/A".to_string(), "out".to_string()][..], &["l1".to_string(), TOP_INSTANCE.to_string()][..]));
        // An unconnected input: only the output net is written.
        assert_eq!(unbuffer_impact_on(base(), &nl, "b", None, Some(1)).write_nets, ["n_out"]);
    }

    // estimateTarget: while the WNS tail is active (permissive), a candidate not legal but
    // tail-admissible becomes legal (wns_tail_permissive), else it stays rejected; the best is the
    // first legal candidate with a strictly greater score.
    #[test]
    fn estimate_target_keeps_the_first_of_equal_scores_and_admits_the_tail() {
        let c = |kind: Move, legal: bool, tail: bool, score: f32| Candidate { kind, estimate: Estimate { legal, score, raw_gain: None, reason: "x", tail_admissible: tail, side_loss: None, sequential_penalty: None, sequential_side_loss: None }, desc: String::new(), impact: MoveImpact::default(), tree: None, rebuffer_side_pins: Vec::new() };
        let mut cs = vec![c(Move::Unbuffer, false, false, 9.0), c(Move::SizeUp, true, false, 2.0), c(Move::SwapPins, true, false, 2.0), c(Move::VtSwap, false, true, 3.0)];
        let mut strict = cs.clone();
        assert_eq!(estimate_target(&mut strict, false), Some(1));
        assert_eq!((strict[3].estimate.legal, strict[3].estimate.reason), (false, "x"));
        assert_eq!(estimate_target(&mut cs, true), Some(3));
        assert_eq!((cs[3].estimate.legal, cs[3].estimate.reason), (true, "wns_tail_permissive"));
        assert_eq!((cs[0].estimate.legal, cs[0].estimate.reason), (false, "x"));
        cs.truncate(3);
        assert_eq!(estimate_target(&mut cs, true), Some(1));
        assert_eq!(estimate_target(&mut cs[..1], true), None);
    }


    // SwapPinsMtGenerator::generate: the equivalent with the smallest delay below the input's
    // own, the first of equal delays; none when no equivalent is faster.
    #[test]
    fn swap_pins_mt_generates_the_first_fastest_equivalent() {
        let eq = |port: &str, delay: f32| SwapPinsEquivalent { port: port.into(), delay, pin: None, slack: 0.0 };
        let mut t = target_at(0);
        t.inst = "u".into();
        let (nl, pins) = impact_netlist();
        let names = ImpactNames::new(&nl, &pins);
        assert!(swap_pins_mt_generate(&t, &names).is_empty());
        t.swap_pins_input = Some(SwapPinsInput { input_port: "A".into(), input_delay: 0.5, equivalents: vec![eq("B", 0.5), eq("C", 0.25), eq("D", 0.25), eq("E", 0.3)] });
        let c = swap_pins_mt_generate(&t, &names);
        assert_eq!(c.len(), 1);
        assert_eq!((c[0].kind, c[0].desc.as_str()), (Move::SwapPins, "C"));
        assert_eq!((c[0].impact.complete, &c[0].impact.write_instances, &c[0].impact.write_nets, &c[0].impact.reconnect_pins), (true, &vec!["u".to_string()], &vec![], &vec!["u/A".to_string(), "u/C".to_string()]));
        assert_eq!((c[0].estimate.legal, c[0].estimate.score, c[0].estimate.side_loss), (true, 0.25, None));
        t.swap_pins_input.as_mut().unwrap().equivalents = vec![eq("B", 0.5), eq("C", 0.75)];
        assert!(swap_pins_mt_generate(&t, &names).is_empty());
    }

    // SwapPinsMtCandidate::estimate with the side model: side_loss = weight * (min(slack, 0) -
    // min(slack - gain, 0)), score = gain - side_loss; a displaced slack below
    // side_slack_floor - margin * gain is side_slack_floor_violation; a score at or below zero
    // otherwise is side_loss_non_improving and tail-admissible.
    #[test]
    fn swap_pins_mt_charges_the_displaced_signal() {
        let mut ctx = crate::delay_estimator::EstimatorContext::new(1);
        (ctx.fanin_side_loss_weight, ctx.side_slack_floor, ctx.side_floor_margin) = (3.0, -10.0, 1.0);
        let input = SwapPinsInput { input_port: "A".into(), input_delay: 2.0, equivalents: vec![] };
        let sw = |slack: f32| SwapPinsEquivalent { port: "B".into(), delay: 1.0, pin: Some("u/B".into()), slack };
        // Slack 4 stays above zero after the gain of 1: no loss.
        let e = swap_pins_mt_estimate(&input, &sw(4.0), Some(&ctx));
        assert_eq!((e.legal, e.score, e.raw_gain, e.side_loss, e.reason, e.tail_admissible), (true, 1.0, Some(1.0), Some(0.0), "estimate_legal", false));
        // Slack 0.5 falls to -0.5: loss 3 * 0.5 = 1.5, score -0.5.
        let e = swap_pins_mt_estimate(&input, &sw(0.5), Some(&ctx));
        assert_eq!((e.legal, e.score, e.side_loss, e.reason, e.tail_admissible), (false, -0.5, Some(1.5), "side_loss_non_improving", true));
        // Slack -9.5 falls to -10.5, above the floor -10 - 1 * 1 = -11: the loss rules.
        let e = swap_pins_mt_estimate(&input, &sw(-9.5), Some(&ctx));
        assert_eq!(e.reason, "side_loss_non_improving");
        // Slack -10.5 falls to -11.5 < -11: the floor.
        let e = swap_pins_mt_estimate(&input, &sw(-10.5), Some(&ctx));
        assert_eq!((e.legal, e.reason, e.tail_admissible), (false, "side_slack_floor_violation", false));
        // A gain at or below zero is non-improving first.
        let slow = SwapPinsInput { input_delay: 1.0, ..input.clone() };
        let e = swap_pins_mt_estimate(&slow, &sw(-10.5), Some(&ctx));
        assert_eq!((e.legal, e.reason, e.side_loss), (false, "estimate_non_improving", Some(0.0)));
        // No side pin: the plain estimate.
        let plain = SwapPinsEquivalent { pin: None, ..sw(-10.5) };
        let e = swap_pins_mt_estimate(&slow, &plain, Some(&ctx));
        assert_eq!((e.legal, e.score, e.side_loss, e.reason), (false, 0.0, None, "estimate_non_improving"));
    }


    fn delay_estimate(legal: bool, arrival_impr: f32, reason: crate::delay_estimator::FailReason, side_paths_evaluated: i32, side_loss: f32) -> crate::delay_estimator::DelayEstimate {
        crate::delay_estimator::DelayEstimate { legal, candidate_delay: 0.0, arrival_impr, side_paths_evaluated, side_loss, reason }
    }

    // SizeUpMtCandidate::estimate: score = gain * scale. Not legal: the score kept, raw gain only
    // for estimate_non_improving, tail-admissible only for dual_rf_non_improving with a positive
    // score, no side loss. Legal: chargeCellSwapSideCosts — with side paths, side_loss is the
    // window's, score = (gain - loss) * scale, and a gain left at or below zero is
    // side_loss_non_improving, tail-admissible when the own gain is positive.
    #[test]
    fn size_up_mt_scores_the_scaled_gain_less_the_side_loss() {
        use crate::delay_estimator::FailReason as F;
        let t = with_context(target_at(0));
        let e = size_up_mt_estimate_of(&t, &delay_estimate(false, -2.0, F::EstimateNonImproving, 0, 0.0), None, 0.5).unwrap();
        assert_eq!((e.legal, e.score, e.raw_gain, e.reason, e.tail_admissible, e.side_loss), (false, -1.0, Some(-2.0), "estimate_non_improving", false, None));
        let e = size_up_mt_estimate_of(&t, &delay_estimate(false, 2.0, F::SideSlackFloorViolation, 1, 0.5), None, 0.5).unwrap();
        assert_eq!((e.legal, e.score, e.raw_gain, e.reason, e.tail_admissible, e.side_loss), (false, 1.0, None, "side_slack_floor_violation", false, None));
        let e = size_up_mt_estimate_of(&t, &delay_estimate(false, 2.0, F::DualRfNonImproving, 1, 0.5), None, 0.5).unwrap();
        assert_eq!((e.reason, e.tail_admissible), ("dual_rf_non_improving", true));
        let e = size_up_mt_estimate_of(&t, &delay_estimate(false, -2.0, F::DualRfNonImproving, 1, 0.5), None, 0.5).unwrap();
        assert!(!e.tail_admissible);
        let e = size_up_mt_estimate_of(&t, &delay_estimate(true, 2.0, F::EstimateLegal, 0, 0.0), None, 0.5).unwrap();
        assert_eq!((e.legal, e.score, e.raw_gain, e.reason, e.side_loss), (true, 1.0, Some(2.0), "estimate_legal", None));
        let e = size_up_mt_estimate_of(&t, &delay_estimate(true, 2.0, F::EstimateLegal, 2, 0.5), None, 0.5).unwrap();
        assert_eq!((e.legal, e.score, e.side_loss), (true, 0.75, Some(0.5)));
        let e = size_up_mt_estimate_of(&t, &delay_estimate(true, 2.0, F::EstimateLegal, 2, 3.0), None, 0.5).unwrap();
        assert_eq!((e.legal, e.score, e.reason, e.tail_admissible, e.side_loss), (false, -0.5, "side_loss_non_improving", true, Some(3.0)));
    }

    // VtSwapMtCandidate::estimate (makeRejectedEstimate): a non-improving estimate keeps its gain as
    // score and raw gain, any other reject scores 0 with no raw gain; legal is scored by the gain at
    // scale 1, then the side costs.
    #[test]
    fn vt_swap_mt_keeps_the_gain_only_of_a_non_improving_reject() {
        use crate::delay_estimator::FailReason as F;
        let t = with_context(target_at(0));
        let e = vt_swap_mt_estimate_of(&t, &delay_estimate(false, -2.0, F::EstimateNonImproving, 0, 0.0), None).unwrap();
        assert_eq!((e.legal, e.score, e.raw_gain, e.reason), (false, -2.0, Some(-2.0), "estimate_non_improving"));
        let e = vt_swap_mt_estimate_of(&t, &delay_estimate(false, 2.0, F::DualRfNonImproving, 1, 0.5), None).unwrap();
        assert_eq!((e.legal, e.score, e.raw_gain, e.reason, e.tail_admissible), (false, 0.0, None, "dual_rf_non_improving", false));
        let e = vt_swap_mt_estimate_of(&t, &delay_estimate(true, 2.0, F::EstimateLegal, 1, 0.5), None).unwrap();
        assert_eq!((e.legal, e.score, e.raw_gain, e.side_loss), (true, 1.5, Some(2.0), Some(0.5)));
    }

    // Rules (MoveCandidate::chargeCellSwapSideCosts with a sequential timing estimator): its
    // weighted loss comes off the gain first and is recorded with the max loss; the score is the
    // gain left times the scale. An illegal side rejects with the side's reason; a legal side
    // leaving no gain is sequential_non_improving; either is tail-admissible only for a legal side
    // with a positive own gain. A legal side with gain left goes on to the side-path loss.
    #[test]
    fn a_sequential_target_pays_its_side_checks_first() {
        use crate::delay_estimator::FailReason as F;
        let mut t = with_context(target_at(0));
        t.sequential_estimator = Some(crate::sequential_timing::SequentialTimingEstimator { candidates: Vec::new(), checks: Vec::new(), prepare_failure_reason: "", data_ports: Vec::new() });
        let side = |legal: bool, w: f32, m: f32, reason: &'static str| Some(SequentialResult { legal, weighted_side_slack_loss: w, max_side_slack_loss: m, reason });
        let e = size_up_mt_estimate_of(&t, &delay_estimate(true, 2.0, F::EstimateLegal, 0, 0.0), side(false, 0.5, 1.0, "sequential_setup_violation"), 0.5).unwrap();
        assert_eq!((e.legal, e.score, e.reason, e.tail_admissible, e.sequential_penalty, e.sequential_side_loss), (false, 0.75, "sequential_setup_violation", false, Some(0.5), Some(1.0)));
        let e = size_up_mt_estimate_of(&t, &delay_estimate(true, 2.0, F::EstimateLegal, 0, 0.0), side(true, 2.0, 2.0, "sequential_estimate_legal"), 0.5).unwrap();
        assert_eq!((e.legal, e.score, e.reason, e.tail_admissible), (false, 0.0, "sequential_non_improving", true));
        let e = size_up_mt_estimate_of(&t, &delay_estimate(true, 2.0, F::EstimateLegal, 1, 2.5), side(true, 0.0, 0.0, "sequential_estimate_legal"), 0.5).unwrap();
        assert_eq!((e.legal, e.score, e.reason, e.tail_admissible, e.side_loss, e.sequential_penalty), (false, -0.25, "side_loss_non_improving", true, Some(2.5), Some(0.0)));
        let e = vt_swap_mt_estimate_of(&t, &delay_estimate(true, 2.0, F::EstimateLegal, 0, 0.0), side(true, 0.5, 0.5, "sequential_estimate_legal")).unwrap();
        assert_eq!((e.legal, e.score, e.reason, e.sequential_penalty), (true, 1.5, "estimate_legal", Some(0.5)));
    }

    // replacementFitsInputCapBudgets: per budget, the replacement port's largest max cap (0 without
    // the port) less the current cap may grow only up to max_cap_increase.
    #[test]
    fn a_replacement_fits_unless_an_input_grows_past_its_budget() {
        use vyges_sta::liberty::{Cell, Port};
        let cell = Cell { ports: vec![Port { name: "A".into(), capacitance: [[1.0, 3.0], [2.0, 4.0]], ..Default::default() }], ..Cell::default() };
        let b = |port: &str, current_cap: f32, max_cap_increase: f32| InputCapBudget { port: port.into(), current_cap, max_cap_increase };
        assert!(cell_fits_input_cap_budgets(&cell, &[]));
        assert!(cell_fits_input_cap_budgets(&cell, &[b("A", 2.0, 2.0)]));
        assert!(!cell_fits_input_cap_budgets(&cell, &[b("A", 2.0, 1.5)]));
        assert!(cell_fits_input_cap_budgets(&cell, &[b("A", 5.0, 0.0)]));
        // A port the replacement lacks reads 0: a decrease always fits.
        assert!(cell_fits_input_cap_budgets(&cell, &[b("B", 1.0, 0.0)]));
    }


    // fitsDelayBudget (Mt): a positive budget bounds the change (inclusive); zero or a negative one
    // allows none.
    #[test]
    fn a_step_down_fits_a_positive_budget_or_no_growth() {
        assert!(fits_delay_budget(2.0, 2.0));
        assert!(!fits_delay_budget(2.0, 2.5));
        assert!(fits_delay_budget(0.0, 0.0));
        assert!(!fits_delay_budget(0.0, 0.1));
        assert!(fits_delay_budget(-1.0, -0.5));
        assert!(!fits_delay_budget(-1.0, 0.5));
    }

    // SizeDownFanoutMtCandidate::estimate: legal as the load-change estimate says, score = gain *
    // scale, raw gain when legal or non-improving only, no side terms.
    #[test]
    fn size_down_fanout_mt_scores_the_scaled_load_change_gain() {
        use crate::delay_estimator::FailReason as F;
        let e = size_down_fanout_mt_estimate_of(&delay_estimate(true, 2.0, F::EstimateLegal, 1, 0.5), 0.75);
        assert_eq!((e.legal, e.score, e.raw_gain, e.reason, e.side_loss, e.tail_admissible), (true, 1.5, Some(2.0), "estimate_legal", None, false));
        let e = size_down_fanout_mt_estimate_of(&delay_estimate(false, -2.0, F::EstimateNonImproving, 0, 0.0), 0.75);
        assert_eq!((e.legal, e.score, e.raw_gain), (false, -1.5, Some(-2.0)));
        let e = size_down_fanout_mt_estimate_of(&delay_estimate(false, 2.0, F::SideSlackFloorViolation, 1, 0.5), 0.75);
        assert_eq!((e.legal, e.score, e.raw_gain, e.reason), (false, 1.5, None, "side_slack_floor_violation"));
    }

    // SizeDownFanoutMtGenerator::generate: a load's step down is kept unless it violates the
    // output limits, misses the delay budget or has no target load delta.
    #[test]
    fn size_down_fanout_mt_keeps_a_fitting_step_down() {
        let load = |violates: bool, change: f32, delta: Option<f32>| SizeDownFanoutLoad {
            inst: "a".into(), pin: "a/A".into(), port: "A".into(), cell: "INVx2".into(), load_slack: 0.0, delay_budget: 1.0, input_cap: 0.0,
            step_down: SizeDownFanoutCellOption { cell: "INVx1".into(), input_cap: 0.0, violates_output_limits: violates, worst_delay_change: change, target_load_delta: delta },
        };
        assert_eq!(size_down_fanout_mt_step_down(&load(false, 0.5, Some(-1.0))), Some(-1.0));
        assert_eq!(size_down_fanout_mt_step_down(&load(false, 1.0, Some(-1.0))), Some(-1.0));
        assert_eq!(size_down_fanout_mt_step_down(&load(true, 0.5, Some(-1.0))), None);
        assert_eq!(size_down_fanout_mt_step_down(&load(false, 1.5, Some(-1.0))), None);
        assert_eq!(size_down_fanout_mt_step_down(&load(false, 0.5, None)), None);
    }


    // SplitLoadMtCandidate::estimate, its end: gain = own gain - side loss; legal when the estimate
    // is and gain > 0; score = gain * scale; raw gain and side loss always; a legal estimate whose
    // gain is at or below zero is side_loss_non_improving, tail-admissible for a positive own gain.
    #[test]
    fn split_load_mt_scores_the_gain_less_the_moved_loads_loss() {
        use crate::delay_estimator::FailReason as F;
        let e = split_load_mt_score(&delay_estimate(true, 2.0, F::EstimateLegal, 2, 0.5), 0.5);
        assert_eq!((e.legal, e.score, e.raw_gain, e.side_loss, e.reason, e.tail_admissible), (true, 0.75, Some(2.0), Some(0.5), "estimate_legal", false));
        let e = split_load_mt_score(&delay_estimate(true, 2.0, F::EstimateLegal, 2, 3.0), 0.5);
        assert_eq!((e.legal, e.score, e.reason, e.tail_admissible), (false, -0.5, "side_loss_non_improving", true));
        // A floor reject from the moved loads stays as it is.
        let e = split_load_mt_score(&delay_estimate(false, 2.0, F::SideSlackFloorViolation, 2, 0.5), 0.5);
        assert_eq!((e.legal, e.reason, e.tail_admissible, e.side_loss), (false, "side_slack_floor_violation", false, Some(0.5)));
    }

    /// A small network for the impacts: in -> d (INV) -> u (NAND2, input B unconnected) -> b1
    /// (BUF) -> l1 (INV) and the port out. NAND2's terminals list B first: pins follow the master.
    fn impact_netlist() -> (vyges_sta::netlist::Netlist, HashMap<String, Vec<String>>) {
        use vyges_sta::netlist::{inst, Conn, Net, Netlist, PortDir};
        let net = |name: &str, pins: Vec<Conn>| std::sync::Arc::new(Net { name: name.into(), pins });
        let pin = |k: usize, p: &str| Conn::Inst(k, p.into());
        let nl = Netlist {
            insts: vec![inst("d", "INV"), inst("u", "NAND2"), inst("b1", "BUF"), inst("l1", "INV")],
            ports: vec![("in".into(), PortDir::Input), ("out".into(), PortDir::Output)],
            nets: vec![
                net("n_in", vec![Conn::Port(0), pin(0, "A")]),
                net("n_d", vec![pin(0, "ZN"), pin(1, "A")]),
                net("n_u", vec![pin(1, "ZN"), pin(2, "A")]),
                net("n_b", vec![pin(2, "Z"), pin(3, "A"), Conn::Port(1)]),
            ],
        };
        let pins = [("INV", vec!["A", "ZN"]), ("NAND2", vec!["B", "A", "ZN"]), ("BUF", vec!["A", "Z"])];
        (nl, pins.into_iter().map(|(c, ps)| (c.to_string(), ps.into_iter().map(String::from).collect())).collect())
    }

    fn stage_at(driver_pin: &str) -> crate::delay_estimator::DelayStageState {
        crate::delay_estimator::DelayStageState {
            arc: crate::delay_estimator::SelectedArc { scene: 0, cell: String::new(), set: 0, arc: 0 },
            driver_pin: driver_pin.into(),
            input_slew: 0.0,
            load_cap: 0.0,
            current_model_delay: 0.0,
            current_delay: 0.0,
            current_wire_delay: 0.0,
            current_model_slew: 0.0,
            current_slew: 0.0,
            output_slew_merge_arcs: Vec::new(),
            sta_slew_bias: Default::default(),
            opposite_rf_arcs: Vec::new(),
            path_index: 0,
        }
    }

    // Network::instance of a top-level port is the top instance (dbNetwork), of an instance pin
    // its instance; Network::net is none for an unconnected pin; pinIterator lists an instance's
    // pins in its master's terminal order (supply terminals skipped, as master_pins holds them).
    #[test]
    fn impact_names_read_the_network_as_the_reference_does() {
        let (nl, pins) = impact_netlist();
        let names = ImpactNames::new(&nl, &pins);
        assert_eq!((names.instance("in"), names.instance("u/ZN"), names.instance("top/sub/x/Z")), (TOP_INSTANCE.to_string(), "u".to_string(), "top/sub/x".to_string()));
        assert_eq!((names.net("u/A"), names.net("u/B"), names.net("out")), (Some("n_d".to_string()), None, Some("n_b".to_string())));
        assert_eq!(names.pins("u"), ["u/B", "u/A", "u/ZN"]);
        assert!(names.pins("nowhere").is_empty());
    }

    // MoveCandidate::impact: nothing without an estimator context (not complete); else every
    // path stage driver's instance and net are timing dependencies, appendUnique (set, once, in
    // stage order) — a port driver's instance is the top instance.
    #[test]
    fn the_base_impact_depends_on_every_path_stage_driver_once() {
        let (nl, pins) = impact_netlist();
        let names = ImpactNames::new(&nl, &pins);
        let mut t = target_at(2);
        assert_eq!(move_candidate_impact(&t, &names), MoveImpact::default());
        t = with_context(t);
        t.estimator_context.as_mut().unwrap().path_stages = ["in", "d/ZN", "u/ZN", "d/ZN"].into_iter().map(stage_at).collect();
        let i = move_candidate_impact(&t, &names);
        assert!(!i.complete);
        assert_eq!(i.timing_dependency_instances, [TOP_INSTANCE, "d", "u"]);
        assert_eq!(i.timing_dependency_nets, ["n_in", "n_d", "n_u"]);
    }

    // MoveCandidate::cellSwapImpact: the base impact, complete; the instance written and in the
    // effect cone; the net of each of its pins in pin order, once, unconnected pins skipped.
    // SizeDownFanoutMtCandidate::impact: the load's cell swap, and the target driver's instance
    // in the effect cone after it.
    #[test]
    fn a_cell_swap_changes_every_net_on_the_instance() {
        let (nl, pins) = impact_netlist();
        let names = ImpactNames::new(&nl, &pins);
        let mut t = target_at(1);
        (t.inst, t.driver_pin) = ("u".into(), "u/ZN".into());
        let i = cell_swap_impact(&t, &names, "u");
        assert_eq!((i.complete, &i.write_instances, &i.timing_effect_instances, &i.timing_effect_nets), (true, &vec!["u".to_string()], &vec!["u".to_string()], &vec!["n_d".to_string(), "n_u".to_string()]));
        assert!(i.write_nets.is_empty() && i.reconnect_pins.is_empty() && i.reconnect_instances.is_empty());
        let i = size_down_fanout_mt_impact(&t, &names, "l1");
        assert_eq!((&i.write_instances, &i.timing_effect_instances, &i.timing_effect_nets), (&vec!["l1".to_string()], &vec!["l1".to_string(), "u".to_string()], &vec!["n_b".to_string()]));
    }

    // SwapPinsMtCandidate::impact: the base impact, complete; the driver written and in the
    // effect cone; for the path input port, the swap's port and the driver port in that order:
    // the pin reconnects unless it is the driver's, and its net is in the effect cone (once,
    // unconnected skipped).
    #[test]
    fn a_pin_swap_changes_the_two_input_nets_and_the_output_net() {
        let (nl, pins) = impact_netlist();
        let names = ImpactNames::new(&nl, &pins);
        let mut t = target_at(1);
        (t.inst, t.driver_pin) = ("u".into(), "u/ZN".into());
        let input = SwapPinsInput { input_port: "A".into(), input_delay: 0.5, equivalents: Vec::new() };
        let swap = SwapPinsEquivalent { port: "B".into(), delay: 0.25, pin: None, slack: 0.0 };
        let i = swap_pins_mt_impact(&t, &names, &input, &swap);
        assert_eq!((i.complete, &i.write_instances, &i.timing_effect_instances), (true, &vec!["u".to_string()], &vec!["u".to_string()]));
        assert_eq!(i.reconnect_pins, ["u/A", "u/B"]);
        assert_eq!(i.timing_effect_nets, ["n_d", "n_u"]);
        assert!(i.write_nets.is_empty() && i.reconnect_instances.is_empty());
    }

    // RebufferMtCandidate::impact: the base impact, complete; the flat net written and in the
    // effect cone; the driver's instance written and in the effect cone; per load path, in plan
    // order, appendReconnectedLoad: the load pin reconnects, its instance reconnects and is in the
    // effect cone, each once (a port load's instance is the top instance).
    #[test]
    fn a_rebuffer_reconnects_each_planned_load_once() {
        let (nl, pins) = impact_netlist();
        let names = ImpactNames::new(&nl, &pins);
        let lp = |pin: &str| crate::rebuffer::LoadPath { load_pin: pin.into(), stages: Vec::new() };
        let plan = crate::rebuffer::TreePlan { tree: 0, target_load_pin: None, load_paths: vec![lp("l1/A"), lp("out"), lp("l1/A")], tree_input_cap: 0.0, driver_slack_gain: 0.0, anchor_tree: false, buffer_count: 1, load_count: 2 };
        let i = rebuffer_impact_on(MoveImpact::default(), &names, Some("n_b".into()), "b1/Z", &plan);
        assert!(i.complete);
        assert_eq!((&i.write_nets, &i.timing_effect_nets, &i.write_instances), (&vec!["n_b".to_string()], &vec!["n_b".to_string()], &vec!["b1".to_string()]));
        assert_eq!(i.reconnect_pins, ["l1/A", "out"]);
        assert_eq!(i.reconnect_instances, ["l1", TOP_INSTANCE]);
        assert_eq!(i.timing_effect_instances, ["b1", "l1", TOP_INSTANCE]);
        // No flat net: nothing on the net lists.
        assert!(rebuffer_impact_on(MoveImpact::default(), &names, None, "b1/Z", &plan).write_nets.is_empty());
    }

    // makeBatchCandidates: one candidate per evaluation with a best, in pool order, carrying the
    // evaluation's pool index, the best estimate's score as both scores (unweighted: no stall;
    // weighted by bottleneckPriorityScore while saved batches stall WNS),
    // the target's endpoint slack and the best candidate's impact; an evaluation without a best
    // is skipped. weightsPriority: a WNS stall, at most bottleneck_max_wns_stall_batches long.
    #[test]
    fn batch_candidates_are_the_best_of_each_evaluation_in_pool_order() {
        let pe = ParallelEval::new(for_parallel_eval(Effort::High), '*');
        let cand = |score: f32, inst: &str| Candidate { kind: Move::SizeUp, estimate: Estimate { legal: true, score, raw_gain: None, reason: "estimate_legal", tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None }, desc: String::new(), impact: MoveImpact { complete: true, write_instances: vec![inst.into()], ..MoveImpact::default() }, tree: None, rebuffer_side_pins: Vec::new() };
        let mut targets = vec![target_at(0), target_at(1), target_at(2)];
        targets[2].slack = -3e-10;
        let evaluations = vec![TargetEvaluation { candidates: vec![cand(1.0, "a"), cand(2.0, "b")], best: Some(1) }, TargetEvaluation { candidates: vec![cand(5.0, "c")], best: None }, TargetEvaluation { candidates: vec![cand(3.0, "d")], best: Some(0) }];
        let bcs = pe_make_batch_candidates(&pe, &targets, &evaluations).unwrap();
        assert_eq!(bcs.iter().map(|b| (b.evaluation_index, b.local_score, b.priority_score, b.endpoint_slack, b.impact.write_instances[0].as_str())).collect::<Vec<_>>(), [(0, 2.0, 2.0, -1e-10, "b"), (2, 3.0, 3.0, -3e-10, "d")]);
        let mut stalled = pe.clone();
        stalled.stall.wns_stall_batches = 1;
        assert!(pe_stall_weights_priority(&stalled));
        // Weighted: a target outside the analyzed cone keeps its local score; one in it is scaled.
        let bcs = pe_make_batch_candidates(&stalled, &targets, &evaluations).unwrap();
        assert_eq!(bcs.iter().map(|b| b.priority_score).collect::<Vec<_>>(), [2.0, 3.0]);
        stalled.bottleneck.severity.insert(("b1/Z".to_string(), true), 1.0);
        stalled.bottleneck.max_severity = 1.0;
        let factor = 1.0 + stalled.config.parallel_eval.priority.bottleneck_scale;
        let bcs = pe_make_batch_candidates(&stalled, &targets, &evaluations).unwrap();
        assert_eq!(bcs.iter().map(|b| (b.local_score, b.priority_score)).collect::<Vec<_>>(), [(2.0, 2.0 * factor), (3.0, 3.0 * factor)]);
        stalled.stall.wns_stall_batches = pe.config.parallel_eval.priority.bottleneck_max_wns_stall_batches + 1;
        assert!(!pe_stall_weights_priority(&stalled));
        assert!(!pe_stall_weights_priority(&pe));
    }

    // SplitLoadMtCandidate::impact: the driver and its net written, the moved load pins in order.
    #[test]
    fn split_load_mt_writes_the_driver_and_its_net_and_moves_the_loads() {
        let mut t = target_at(0);
        t.inst = "u".into();
        let input = SplitLoadInput { drvr_net: "n".into(), drvr_loc: (0, 0), load_pins: vec!["b/A".into(), "a/A".into()], buffer_cell: "BUF".into(), resized_cell: "BUF".into(), moved_pin_cap: 0.0, target_wire_cap_saved: 0.0, moved_wire_cap: 0.0, target_load_delta: 0.0, moved_loads: vec![] };
        t.driver_pin = "u/ZN".into();
        let (nl, pins) = impact_netlist();
        let i = split_load_mt_impact(&t, &ImpactNames::new(&nl, &pins), &input);
        // The driver's instance and net written and in the effect cone; the moved loads'
        // instances reconnected and in the effect cone after the driver, each once.
        assert_eq!(
            i,
            MoveImpact {
                complete: true,
                write_instances: vec!["u".into()],
                write_nets: vec!["n".into()],
                reconnect_pins: vec!["b/A".into(), "a/A".into()],
                reconnect_instances: vec!["b".into(), "a".into()],
                timing_effect_instances: vec!["u".into(), "b".into(), "a".into()],
                timing_effect_nets: vec!["n".into()],
                ..MoveImpact::default()
            }
        );
    }


    // RebufferMtCandidate::estimate: raw score = gain - side loss, score = raw * scale. A reject
    // keeps its gain only for estimate_non_improving / side_slack_floor_violation; a legal
    // estimate at raw <= 0 is side_loss_non_improving, under min_score is min_score — both
    // tail-admissible for a positive own gain. side_loss only when side paths were evaluated.
    #[test]
    fn rebuffer_mt_scores_the_gain_less_the_side_loss() {
        use crate::delay_estimator::FailReason as F;
        let e = rebuffer_mt_estimate_of(&delay_estimate(true, 20e-12, F::EstimateLegal, 1, 4e-12), 0.5, 5e-12);
        assert_eq!((e.legal, e.score, e.raw_gain, e.side_loss, e.reason), (true, (20e-12f32 - 4e-12) * 0.5, Some(20e-12), Some(4e-12), "estimate_legal"));
        let e = rebuffer_mt_estimate_of(&delay_estimate(true, 6e-12, F::EstimateLegal, 0, 0.0), 0.5, 7e-12);
        assert_eq!((e.legal, e.reason, e.tail_admissible, e.side_loss), (false, "min_score", true, None));
        let e = rebuffer_mt_estimate_of(&delay_estimate(true, 2e-12, F::EstimateLegal, 1, 3e-12), 0.5, 0.0);
        assert_eq!((e.legal, e.reason, e.tail_admissible), (false, "side_loss_non_improving", true));
        let e = rebuffer_mt_estimate_of(&delay_estimate(false, 2e-12, F::SideSlackFloorViolation, 1, 1e-12), 0.5, 0.0);
        assert_eq!((e.legal, e.raw_gain, e.reason, e.tail_admissible), (false, Some(2e-12), "side_slack_floor_violation", false));
        let e = rebuffer_mt_estimate_of(&delay_estimate(false, 0.0, F::InvalidBufferInsertion, 0, 0.0), 0.5, 0.0);
        assert_eq!((e.legal, e.raw_gain, e.reason), (false, None, "invalid_buffer_insertion"));
    }

    // makeSideContexts: every plan load but the target that has load timing, in load-path order;
    // none at all without load timing (side loads off).
    #[test]
    fn rebuffer_side_contexts_are_the_other_timed_loads_in_path_order() {
        let lp = |pin: &str| crate::rebuffer::LoadPath { load_pin: pin.into(), stages: vec![] };
        let p = crate::rebuffer::TreePlan { tree: 0, target_load_pin: Some("t/A".into()), load_paths: vec![lp("b/A"), lp("t/A"), lp("a/A"), lp("u/A")], tree_input_cap: 0.0, driver_slack_gain: 0.0, anchor_tree: false, buffer_count: 1, load_count: 4 };
        let lt = |pin: &str| RebufferLoadTiming { pin: pin.into(), slack: -1.0, wire_delay: 2.0, weight: 0.5 };
        let s = rebuffer_make_side_contexts(&p, &[lt("a/A"), lt("t/A"), lt("b/A")]);
        assert_eq!(s.iter().map(|c| c.load_pin.as_str()).collect::<Vec<_>>(), ["b/A", "a/A"]);
        assert_eq!((s[0].current_driver_to_load_delay, s[0].current_slack, s[0].weight), (2.0, -1.0, 0.5));
        assert!(rebuffer_make_side_contexts(&p, &[]).is_empty());
    }

    /// `batchImprovesCheckpoint` (upstream SetupParallelEvalPolicy): a strictly better WNS
    /// accepts; else WNS fuzzily equal to the checkpoint's, or at least the best less the
    /// tolerance, AND a strictly better TNS; else not.
    #[test]
    fn batch_improves_checkpoint_follows_the_wns_band_and_tns() {
        // Strictly better WNS: accepted whatever TNS does.
        assert!(batch_improves_checkpoint(-2e-10, -1e-9, -1e-10, -5e-9, -2e-10, 0.0));
        // WNS equal, TNS better: accepted; TNS equal or worse: not.
        assert!(batch_improves_checkpoint(-2e-10, -1e-9, -2e-10, -0.5e-9, -2e-10, 0.0));
        assert!(!batch_improves_checkpoint(-2e-10, -1e-9, -2e-10, -1e-9, -2e-10, 0.0));
        // WNS slightly worse but within the tolerance of the best, TNS better: accepted.
        assert!(batch_improves_checkpoint(-2e-10, -1e-9, -2.05e-10, -0.5e-9, -2e-10, 1e-11));
        // Outside the band: not, even with a better TNS.
        assert!(!batch_improves_checkpoint(-2e-10, -1e-9, -2.5e-10, -0.5e-9, -2e-10, 1e-11));
    }

    /// `StallMonitor::onSaved` / `stall` / `moveBudget` (upstream ParallelEvalState): progress is
    /// a WNS gain over the best past `progress_wns_period_pct` of the reference period, or a TNS
    /// gain over the previous checkpoint past `progress_tns_pct` of |initial TNS|; without it the
    /// batch and its moves count as stalled; the move budget is `stall_batches` times the larger
    /// of `min_batch_cap` and the last cap.
    /// `BottleneckAnalysis::bottleneckPriorityScore` (upstream policy/BottleneckAnalysis): the
    /// rank is `local_score * (1 + bottleneck_scale * ratio^bottleneck_exponent)`, the ratio the
    /// target vertex's severity over the largest; 0 outside the cone or with no severity; the
    /// factor multiplies a negative score too (a cone-central loss ranks lower).
    #[test]
    fn the_bottleneck_priority_scales_the_local_score_by_the_cone_share() {
        let p = BatchPriorityConfig::default();
        let mut b = Bottlenecks::default();
        let key = ("u1/Z".to_string(), true);
        assert_eq!(b.priority_score(2.0, Some(&key), &p), 2.0, "no analysis");
        b.severity.insert(key.clone(), 0.5);
        b.severity.insert(("u2/Z".to_string(), true), 1.0);
        b.max_severity = 1.0;
        let factor = 1.0 + p.bottleneck_scale * 0.5f32.powf(p.bottleneck_exponent);
        assert_eq!(b.priority_score(2.0, Some(&key), &p), 2.0 * factor);
        assert_eq!(b.priority_score(-2.0, Some(&key), &p), -2.0 * factor);
        assert_eq!(b.priority_score(2.0, None, &p), 2.0, "outside the cone");
        b.max_severity = 0.0;
        assert_eq!(b.priority_score(2.0, Some(&key), &p), 2.0, "no severity anywhere");
    }

    /// Upstream gtest `ParallelEvalTailGainFollowsThePeriod`: a tail batch is WNS progress only
    /// when the best WNS gains more than `wns_tail_min_gain_period_pct` of the reference period,
    /// whatever the WNS.
    #[test]
    fn the_tail_gain_follows_the_period() {
        let cfg = ParallelEvalConfig::default();
        let period = 500e-12f32;
        let min_gain = period * cfg.wns_tail_min_gain_period_pct / 100.0;
        let mut tail = WnsTail::default();
        assert!(tail.start(&cfg, 0.0, period));
        for wns in [-100e-12f32, -0.01e-12] {
            tail.after_batch(wns, wns + 0.5 * min_gain, true);
            assert_eq!(tail.batches_without_gain, 1, "{wns}");
            tail.after_batch(wns, wns + 2.0 * min_gain, true);
            assert_eq!(tail.batches_without_gain, 0, "{wns}");
        }
    }

    /// `WnsTail::start` / `afterBatch` (upstream ParallelEvalState): the tail starts once and only
    /// with tail endpoints; its budget is `wns_tail_work_pct` of the work so far, at least
    /// `wns_tail_min_work`; the cap starts at one move, doubles after a save up to the stage's
    /// `wns_tail_batch`, and drops to one after a miss; a speculative run is not judged per batch.
    #[test]
    fn the_tail_starts_once_and_sizes_its_batches() {
        let cfg = ParallelEvalConfig::default();
        let mut tail = WnsTail::default();
        assert!(!tail.start(&ParallelEvalConfig { wns_tail_endpoints: 0, ..cfg }, 10.0, 1e-9));
        assert!(tail.start(&cfg, 10.0, 1e-9));
        assert!(!tail.start(&cfg, 10.0, 1e-9), "once per run");
        assert_eq!((tail.endpoints, tail.batch_cap, tail.max_batch), (cfg.wns_tail_endpoints, 1, cfg.wns_tail_batch));
        assert_eq!(tail.budget_work, f64::from(cfg.wns_tail_min_work), "4x of 10 is below the floor");
        let mut big = WnsTail::default();
        assert!(big.start(&cfg, 100.0, 1e-9));
        assert_eq!(big.budget_work, 100.0 * f64::from(cfg.wns_tail_work_pct) / 100.0);
        for want in [2, 4, 8, 8] {
            tail.after_batch(0.0, 0.0, true);
            assert_eq!(tail.batch_cap, want.min(cfg.wns_tail_batch));
        }
        tail.after_batch(0.0, 0.0, false);
        assert_eq!(tail.batch_cap, 1);
        let before = tail.batches_without_gain;
        tail.keep_speculative(&[InstanceRecord { name: "u1".into(), id: 3 }]);
        tail.after_batch(0.0, 0.0, false);
        assert_eq!(tail.batches_without_gain, before, "a speculative run is judged when it ends");
        assert_eq!(tail.end_speculation(), vec![InstanceRecord { name: "u1".into(), id: 3 }]);
        assert!(!tail.speculating());
    }

    /// `WnsTail::nextStep` / `widen` (upstream ParallelEvalState): the first rule that fires is
    /// the reason (no moves, stalled, max batches, work budget); the budget and the batch limit
    /// stop the tail; a stall in a greedy stage speculates first; else the tail widens (endpoints
    /// and stage cap times `wns_tail_grow`, endpoints capped) while below `wns_tail_max_endpoints`,
    /// then stops. Speculation needs a stalled greedy stage and stays within `wns_tail_speculate`.
    #[test]
    fn the_tail_steps_speculate_widen_then_stop() {
        let cfg = ParallelEvalConfig::default();
        let mut tail = WnsTail::default();
        assert!(tail.start(&cfg, 0.0, 1e-9));
        assert_eq!(tail.next_step(&cfg, 1, 0.0), (TailStep::Continue, None));
        assert_eq!(tail.next_step(&cfg, 0, 0.0), (TailStep::Widen, Some("wns_tail_no_committed_moves")));
        assert_eq!(tail.next_step(&cfg, 1, tail.budget_work + 1.0), (TailStep::Stop, Some("wns_tail_work_budget")));
        tail.batches_without_gain = cfg.wns_tail_stall;
        assert!(!tail.may_speculate(&cfg), "only after a stalled greedy stage");
        assert_eq!(tail.next_step(&cfg, 1, 0.0), (TailStep::Speculate, Some("wns_tail_stalled")));
        tail.begin_speculative_stage();
        assert_eq!((tail.batches_without_gain, tail.greedy_stalled), (0, true));
        assert!(tail.may_speculate(&cfg));
        tail.speculative_batches = cfg.wns_tail_speculate;
        assert!(!tail.may_speculate(&cfg));
        tail.batches_without_gain = cfg.wns_tail_stall;
        assert_eq!(tail.next_step(&cfg, 1, 0.0), (TailStep::Widen, Some("wns_tail_stalled")));
        let (e, m) = (tail.endpoints, tail.max_batch);
        tail.widen(&cfg);
        assert_eq!((tail.endpoints, tail.max_batch, tail.speculative_batches), (e * cfg.wns_tail_grow, m * cfg.wns_tail_grow, 0));
        while tail.endpoints < cfg.wns_tail_max_endpoints {
            tail.widen(&cfg);
        }
        assert_eq!(tail.endpoints, cfg.wns_tail_max_endpoints);
        assert_eq!(tail.next_step(&cfg, 0, 0.0), (TailStep::Stop, Some("wns_tail_no_committed_moves")));
        tail.batches = cfg.wns_tail_max_batches;
        assert_eq!(tail.next_step(&cfg, 0, 0.0), (TailStep::Stop, Some("wns_tail_no_committed_moves")), "the first rule names the reason");
        // Buffer removal runs only in the single-move batches of a stalled greedy stage.
        tail.batch_cap = 1;
        assert!(tail.unbuffer_allowed());
        tail.batch_cap = 2;
        assert!(!tail.unbuffer_allowed());
    }

    /// `acceptsBatch` under the tail (upstream SetupParallelEvalPolicy): no WNS tolerance, and a
    /// WNS gain may cost TNS only up to `wns_tail_tns_ratio` times itself.
    #[test]
    fn the_tail_bounds_the_tns_a_wns_gain_may_cost() {
        let mut pe = ParallelEval::new(for_parallel_eval(Effort::High), 'P');
        pe.reference_period = 1e-9;
        pe.checkpoint_wns = -1e-10;
        pe.best_checkpoint_wns = -1e-10;
        pe.checkpoint_tns = -1e-9;
        let ratio = pe.config.parallel_eval.wns_tail_tns_ratio;
        // WNS +1e-12: TNS may fall by up to ratio * 1e-12.
        assert!(pe_accepts_batch(&pe, -0.99e-10, -1e-9 - 0.5 * ratio * 1e-12));
        assert!(pe_accepts_batch(&pe, -0.99e-10, -1e-9 - 2.0 * ratio * 1e-12), "no bound before the tail");
        pe.tail.active = true;
        assert_eq!(pe_checkpoint_wns_tolerance(&pe), 0.0);
        assert!(pe_accepts_batch(&pe, -0.99e-10, -1e-9 - 0.5 * ratio * 1e-12));
        assert!(!pe_accepts_batch(&pe, -0.99e-10, -1e-9 - 2.0 * ratio * 1e-12));
    }

    /// `MissTabu` (upstream ParallelEvalState): `addTabu` skips an instance until batch
    /// `now + miss_tabu_batches`, and `expire(now)` drops entries ending at or before `now`;
    /// `recordUndo` also counts undos and excludes the instance once the count reaches
    /// `max_undos_per_instance` (true on that call); every entry counts only while its slot holds
    /// an instance of the recorded name (a new instance in the slot starts over);
    /// `clearTemporary` drops the tabu and keeps the exclusions.
    #[test]
    fn miss_tabu_expires_counts_undos_and_checks_the_slot_name() {
        let cfg = ParallelEvalConfig::default();
        let mut t = MissTabu::default();
        t.add_tabu(&cfg, 7, "u1", 3);
        assert!(t.is_tabu(7, "u1"));
        assert!(!t.is_tabu(7, "u2"), "another instance in the slot is not tabu");
        t.expire(3 + cfg.miss_tabu_batches - 1);
        assert!(t.is_tabu(7, "u1"));
        t.expire(3 + cfg.miss_tabu_batches);
        assert!(!t.is_tabu(7, "u1"), "an entry ending at now is dropped");
        for k in 1..cfg.max_undos_per_instance {
            assert!(!t.record_undo(&cfg, 7, "u1", k), "undo {k} of {}", cfg.max_undos_per_instance);
        }
        // A new instance in the slot starts its count over.
        assert!(!t.record_undo(&cfg, 7, "u9", 5));
        assert_eq!(t.undo_counts[&7], ("u9".to_string(), 1));
        for k in 1..cfg.max_undos_per_instance {
            assert_eq!(t.record_undo(&cfg, 7, "u9", 5 + k), k + 1 == cfg.max_undos_per_instance);
        }
        assert!(t.is_excluded(7, "u9") && !t.is_excluded(7, "u1"));
        t.clear_temporary();
        assert!(!t.is_tabu(7, "u9") && t.is_excluded(7, "u9"));
    }

    /// `guardedSidePins` (upstream move/*MtCandidate): SizeUp and VtSwap guard every fanin side
    /// net's loads (`faninSidePins`), SwapPins the displaced input's pin, SplitLoad the moved
    /// loads, Rebuffer its side contexts' load pins; the rest (Unbuffer, SizeDownFanout) none.
    #[test]
    fn each_move_guards_its_own_side_pins() {
        use crate::delay_estimator::{EstimatorContext, FaninSideNet, SideLoad};
        let load = |pin: &str| SideLoad { pin: pin.into(), slack: 0.0, weight: 1.0 };
        let mut t = target_at(2);
        let mut ctx = EstimatorContext::new(2);
        ctx.fanin_side_nets = vec![
            FaninSideNet { input_port: "A".into(), input_cap: 0.0, delay_per_cap: 0.0, selected_arc: true, loads: vec![load("x/A"), load("y/A")] },
            FaninSideNet { input_port: "B".into(), input_cap: 0.0, delay_per_cap: 0.0, selected_arc: false, loads: vec![load("z/B")] },
        ];
        t.estimator_context = Some(ctx);
        t.swap_pins_input = Some(SwapPinsInput { input_port: "A".into(), input_delay: 0.0, equivalents: vec![
            SwapPinsEquivalent { port: "B".into(), delay: 0.0, pin: Some("b1/B".into()), slack: 0.0 },
            SwapPinsEquivalent { port: "C".into(), delay: 0.0, pin: None, slack: 0.0 },
        ] });
        let c = |kind: Move, desc: &str, sides: Vec<String>| Candidate { kind, estimate: Estimate { legal: true, score: 1.0, raw_gain: None, reason: "x", tail_admissible: false, side_loss: None, sequential_penalty: None, sequential_side_loss: None }, desc: desc.into(), impact: MoveImpact::default(), tree: None, rebuffer_side_pins: sides };
        let fanin = vec!["x/A".to_string(), "y/A".into(), "z/B".into()];
        assert_eq!(pe_guarded_side_pins(&c(Move::SizeUp, "BUF_X2", Vec::new()), &t), fanin);
        assert_eq!(pe_guarded_side_pins(&c(Move::VtSwap, "BUF_X1_LVT", Vec::new()), &t), fanin);
        assert_eq!(pe_guarded_side_pins(&c(Move::SwapPins, "B", Vec::new()), &t), vec!["b1/B".to_string()]);
        assert!(pe_guarded_side_pins(&c(Move::SwapPins, "C", Vec::new()), &t).is_empty(), "no pin on the equivalent");
        assert_eq!(pe_guarded_side_pins(&c(Move::Buffer, "", vec!["s/A".into()]), &t), vec!["s/A".to_string()]);
        assert!(pe_guarded_side_pins(&c(Move::Unbuffer, "remove", Vec::new()), &t).is_empty());
        assert!(pe_guarded_side_pins(&c(Move::SizeDownFanout, "", Vec::new()), &t).is_empty());
    }

    /// `startSmallBatchRetry` (upstream SetupParallelEvalPolicy): once; misses cleared, the
    /// temporary tabu dropped (exclusions kept), and a NEW selector from the policy config's
    /// (which carries the setup slack margin) without the score floor and worst-path reserve.
    #[test]
    fn the_small_batch_retry_rebuilds_the_selector_from_the_config() {
        let mut pe = ParallelEval::new(for_parallel_eval(Effort::High), 'P');
        pe.config.parallel_eval.selector.slack_margin = 2e-11;
        pe.selector = BatchSelectorConfig { slack_margin: -1.0, ..pe.config.parallel_eval.selector };
        pe.checkpoint_misses = 8;
        let cfg = pe.config.parallel_eval;
        pe.tabu.add_tabu(&cfg, 1, "a", 0);
        pe.tabu.excluded.insert(2, "b".into());
        pe_start_small_batch_retry(&mut pe);
        assert!(pe.small_batch_retry_started);
        assert_eq!(pe.checkpoint_misses, 0);
        assert_eq!((pe.selector.min_score_ratio, pe.selector.wns_reserve_pct, pe.selector.slack_margin), (0.0, 0, 2e-11));
        assert_eq!(pe.selector.score_floor_start, cfg.selector.score_floor_start);
        assert!(!pe.tabu.is_tabu(1, "a") && pe.tabu.is_excluded(2, "b"));
    }

    #[test]
    fn stall_monitor_counts_saves_without_progress() {
        let cfg = ParallelEvalConfig::default();
        let mut m = StallMonitor::default();
        m.reset(-1e-9, 1e-9);
        m.begin_batch(64, 10);
        m.end_batch(30);
        // WNS +1e-12 (> 0.04% of 1 ns = 4e-13): progress, nothing stalled.
        assert_eq!(m.on_saved(&cfg, -1e-9, -2e-10, -1.99e-10, -1e-9, 0.0), 0);
        assert_eq!((m.stalled_batches, m.wns_stall_batches), (0, 0));
        // No WNS gain and a TNS gain below 0.5% of 1e-9: the batch's 20 moves stall.
        m.end_batch(20);
        assert_eq!(m.on_saved(&cfg, -1e-9, -1.99e-10, -1.99e-10, -0.999e-9, 0.0), 20);
        assert_eq!((m.stalled_batches, m.wns_stall_batches), (1, 1));
        assert_eq!(m.move_budget(&cfg), 6 * 64);
        assert_eq!(m.stall(&cfg), Stall::None);
        m.stalled_moves = 6 * 64;
        assert_eq!(m.stall(&cfg), Stall::Moves);
        m.stalled_moves = 0;
        m.stalled_batches = cfg.stall_max_batches;
        assert_eq!(m.stall(&cfg), Stall::Batches);
        // weightsPriority: stalled, at most bottleneck_max_wns_stall_batches saves.
        assert!(m.weights_priority(&cfg));
        m.wns_stall_batches = cfg.priority.bottleneck_max_wns_stall_batches + 1;
        assert!(!m.weights_priority(&cfg));
    }

    /// `StallMonitor::updateWindow`: once the window holds more than `stall_window` saves and
    /// spent at least `stall_window_min_work`, neither WNS nor the failing endpoints moving
    /// stalls it; a TNS-progress save never does.
    #[test]
    fn stall_window_stalls_only_a_costly_flat_window() {
        let cfg = ParallelEvalConfig::default();
        let mut m = StallMonitor::default();
        m.reset(-1e-9, 1e-9);
        m.begin_batch(64, 10);
        for k in 0..=cfg.stall_window {
            m.on_saved(&cfg, -1e-9, -2e-10, -2e-10, -1e-9, f64::from(k) * 10.0);
        }
        assert!(m.window_stalled, "flat WNS and endpoints over 60 work units stall the window");
        let mut cheap = StallMonitor::default();
        cheap.reset(-1e-9, 1e-9);
        for k in 0..=cfg.stall_window {
            cheap.on_saved(&cfg, -1e-9, -2e-10, -2e-10, -1e-9, f64::from(k));
        }
        assert!(!cheap.window_stalled, "a window under stall_window_min_work is never cut");
    }

    /// `WorkMeter::seconds`: each count times its weight, the timing updates times the vertices.
    #[test]
    fn work_meter_weighs_counts() {
        let cfg = ParallelEvalConfig::default();
        let mut w = WorkMeter::default();
        w.reset(1000);
        w.add_timing_update(10);
        w.add_estimates(4);
        w.add_plan_request();
        w.add_endpoints(3);
        let want = 5.0e-7f32 as f64 * 1000.0 + 1.93e-3f32 as f64 * 10.0 + 1.35e-5f32 as f64 * 4.0 + 1.14e-3f32 as f64 + 5.65e-4f32 as f64 * 3.0;
        assert_eq!(w.seconds(&cfg), want);
    }
}
