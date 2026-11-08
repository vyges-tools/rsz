// SPDX-License-Identifier: Apache-2.0
//! The buffer-tree part of the delay estimator (`DelayEstimatorBufferTree`): the timing of a
//! RebufferMt tree plan along the target path and its side paths.
//!
//! Stages in the reference's order: `estimateBufferTreeWithSidePaths` →
//! `estimateBufferTreeWindow` (the target path) → `estimateBranchTiming` (`branchDriverTiming`,
//! then `treeStageTiming` per stage, through the branch-prefix cache) → `estimateSidePathWindow`
//! per side load.

use super::{estimate_output_slew, estimate_receiver_input_slew, lookup_arc_delay_and_slew, lookup_buffer_delay_and_slew, make_estimate, DelayEstimate, DelayStageState, EstimatorContext, FailReason, MIN_SLEW_RATIO_DENOMINATOR};
use crate::preamble::Libs;
use crate::rebuffer::{LoadPath, PathStage, StageKind, TreePlan, WireRcStage};

/// `kElmoreSlewFactor`.
const ELMORE_SLEW_FACTOR: f32 = 1.39;
/// The model's calibration (`kBufferTree*`): a buffer stage's effective load as a share of its
/// lumped downstream cap on the target path, on a side path, and for a direct load.
const TARGET_EFFECTIVE_LOAD_CAP_FACTOR: f32 = 0.65;
const SIDE_EFFECTIVE_LOAD_CAP_FACTOR: f32 = 0.875;
const DIRECT_LOAD_CAP_FACTOR: f32 = 0.60;
/// `kLoadRatioSlewFloorExponent`.
const LOAD_RATIO_SLEW_FLOOR_EXPONENT: f32 = 0.85;
/// Elmore wire delays past the threshold shrink by the scale; shorter wires add slew at the
/// short-wire factor.
const WIRE_DELAY_SCALE_THRESHOLD: f32 = 20.0e-12;
const WIRE_DELAY_SCALE: f32 = 0.78;
const SHORT_WIRE_SLEW_FACTOR: f32 = 1.35;

/// `BufferTreeSidePathContext`: a plan load off the target path, with its current timing.
#[derive(Debug, Clone, PartialEq)]
pub struct SidePathContext {
    pub load_pin: String,
    pub current_driver_to_load_delay: f32,
    pub current_slack: f32,
    pub weight: f32,
}

/// `StageEvaluation`, the parts the printer shows: the stage's path index, its cell (none for a
/// wire), input slew, load cap, delay, output slew and wire delay.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct StageEvaluation {
    pub path_index: i32,
    pub cell: Option<String>,
    pub input_slew: f32,
    pub load_cap: f32,
    pub stage_delay: f32,
    pub output_slew: f32,
    pub wire_delay: f32,
}

/// `BufferTreeSidePathEvaluation`.
#[derive(Debug, Clone, PartialEq)]
pub struct SidePathEvaluation {
    pub load_pin: String,
    pub legal: bool,
    pub arrival_impr: f32,
    pub slack_after: f32,
    pub trace: Vec<StageEvaluation>,
}

/// `loadRatioSlewFloor`: the stage's current slew times its load ratio to the 0.85th power; 0
/// without a load or a slew.
fn load_ratio_slew_floor(stage: &DelayStageState, load_cap: f32) -> f32 {
    if stage.load_cap <= 0.0 || stage.current_slew <= 0.0 || load_cap <= 0.0 {
        return 0.0;
    }
    let load_ratio = (load_cap / stage.load_cap).clamp(0.0, 1.0);
    stage.current_slew * load_ratio.powf(LOAD_RATIO_SLEW_FLOOR_EXPONENT)
}

/// `wireOutputSlewFromDelay`: the input slew and the wire's slew (short-wire factor below the
/// threshold, else Elmore's) combined as `hypot`.
fn wire_output_slew_from_delay(wire_delay: f32, input_slew: f32) -> f32 {
    let slew_factor = if wire_delay < WIRE_DELAY_SCALE_THRESHOLD { SHORT_WIRE_SLEW_FACTOR } else { ELMORE_SLEW_FACTOR };
    let wire_slew = slew_factor * wire_delay.max(0.0);
    input_slew.max(0.0).hypot(wire_slew)
}

/// `bufferTreeEffectiveLoadCap`.
fn effective_load_cap(total_load_cap: f32, factor: f32) -> f32 {
    (total_load_cap * factor).max(0.0)
}

/// `boundBufferTreeDownstreamStageDelay`: a stage after the tree keeps at least 50% to 80% of its
/// current delay (`max(0.5, 0.45 + 0.35·sqrt(slew ratio))`).
fn bound_downstream_stage_delay(stage: &DelayStageState, input_slew: f32, stage_delay: f32) -> f32 {
    let current_input_slew = stage.input_slew.max(MIN_SLEW_RATIO_DENOMINATOR);
    let input_slew_ratio = (input_slew / current_input_slew).clamp(0.0, 1.0);
    let min_delay_ratio = 0.5f32.max(0.45 + 0.35 * input_slew_ratio.sqrt());
    stage_delay.max(stage.current_delay * min_delay_ratio)
}

/// `wireElmoreDelay`: `R·(C_load + C_wire/2)`; 0 for no resistance or a negative cap.
fn wire_elmore_delay(wire: &WireRcStage, load_cap: f32) -> f32 {
    if wire.res <= 0.0 || wire.cap < 0.0 || load_cap < 0.0 {
        return 0.0;
    }
    wire.res * (load_cap + 0.5 * wire.cap)
}

/// `bufferTreeWireDelay`: Elmore, scaled past the threshold.
fn buffer_tree_wire_delay(wire: &WireRcStage, load_cap: f32) -> f32 {
    let raw = wire_elmore_delay(wire, load_cap);
    if raw < WIRE_DELAY_SCALE_THRESHOLD {
        return raw;
    }
    raw * WIRE_DELAY_SCALE
}

/// `BranchTiming`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct BranchTiming {
    legal: bool,
    delay: f32,
    load_slew: f32,
    reason: Option<FailReason>,
}

/// `BranchTimingCacheNode`: a trie of replayed branch prefixes under a dummy root; the key has no
/// effective-cap factor, so a prefix the target path shares with a side path keeps the timing of
/// whichever was evaluated first.
#[derive(Debug, Clone)]
struct CacheNode {
    valid: bool,
    stage: Option<PathStage>,
    prefix: BranchTiming,
    trace_entry: StageEvaluation,
    children: Vec<usize>,
}

/// The cache as an arena: node 0 is the root (the driver into the tree).
struct BranchCache {
    nodes: Vec<CacheNode>,
}

impl BranchCache {
    fn new() -> BranchCache {
        BranchCache { nodes: vec![CacheNode { valid: false, stage: None, prefix: BranchTiming::default(), trace_entry: StageEvaluation::default(), children: Vec::new() }] }
    }

    /// `findBranchTimingCacheChild`: the first valid child with the same stage.
    fn find_child(&self, parent: usize, stage: &PathStage) -> Option<usize> {
        self.nodes[parent].children.iter().copied().find(|&c| self.nodes[c].valid && self.nodes[c].stage.as_ref().is_some_and(|s| same_stage(s, stage)))
    }
}

/// `sameBufferTreePathStage`: type, downstream cap and distributed flag; a buffer its cell, a
/// wire its cell (none) and RC.
fn same_stage(lhs: &PathStage, rhs: &PathStage) -> bool {
    if lhs.kind != rhs.kind || lhs.downstream_cap != rhs.downstream_cap || lhs.drives_distributed_load != rhs.drives_distributed_load {
        return false;
    }
    if lhs.kind == StageKind::Buffer {
        return lhs.buffer_cell == rhs.buffer_cell;
    }
    lhs.buffer_cell == rhs.buffer_cell && lhs.wire_rc.length == rhs.wire_rc.length && lhs.wire_rc.res == rhs.wire_rc.res && lhs.wire_rc.cap == rhs.wire_rc.cap
}

/// `branchDriverTiming`: the target driver's table delay at the tree's input cap, the slew its
/// load-ratio floor; from the cache root when it holds it. `None` when the arc lookup fails.
fn branch_driver_timing(libs: &Libs, target_stage: &DelayStageState, plan: &TreePlan, trace: &mut Option<&mut Vec<StageEvaluation>>, cache: &mut BranchCache) -> Option<BranchTiming> {
    if cache.nodes[0].valid {
        if let Some(t) = trace.as_deref_mut() {
            t.push(cache.nodes[0].trace_entry.clone());
        }
        return Some(cache.nodes[0].prefix);
    }
    let (driver_delay, _) = lookup_arc_delay_and_slew(libs, &target_stage.arc, target_stage.input_slew, plan.tree_input_cap, &target_stage.arc.cell, false, None)?;
    let propagated_slew = load_ratio_slew_floor(target_stage, plan.tree_input_cap);
    let entry = StageEvaluation { path_index: target_stage.path_index, cell: Some(target_stage.arc.cell.clone()), input_slew: target_stage.input_slew, load_cap: plan.tree_input_cap, stage_delay: driver_delay, output_slew: propagated_slew, wire_delay: 0.0 };
    let branch = BranchTiming { legal: true, delay: driver_delay, load_slew: propagated_slew, reason: Some(FailReason::EstimateLegal) };
    let root = &mut cache.nodes[0];
    root.valid = true;
    root.prefix = branch;
    root.trace_entry = entry.clone();
    if let Some(t) = trace.as_deref_mut() {
        t.push(entry);
    }
    Some(branch)
}

/// `treeStageTiming`: a wire adds its (scaled) Elmore delay and its slew; a buffer its table
/// delay on the effective load (the path's factor for a distributed load, else the direct one).
/// `None` when the buffer has no arc.
fn tree_stage_timing(libs: &Libs, target_stage: &DelayStageState, stage: &PathStage, factor: f32, branch: &BranchTiming) -> Option<(StageEvaluation, BranchTiming)> {
    let input_slew = branch.load_slew;
    match stage.kind {
        StageKind::Wire => {
            let wire_delay = buffer_tree_wire_delay(&stage.wire_rc, stage.downstream_cap);
            let output_slew = wire_output_slew_from_delay(wire_delay, input_slew);
            let entry = StageEvaluation { path_index: target_stage.path_index, cell: None, input_slew, load_cap: stage.downstream_cap, stage_delay: wire_delay, output_slew, wire_delay };
            Some((entry, BranchTiming { legal: true, delay: branch.delay + wire_delay, load_slew: output_slew, reason: Some(FailReason::EstimateLegal) }))
        }
        StageKind::Buffer => {
            let cell = stage.buffer_cell.as_deref().expect("a buffer stage has a cell");
            let gate_load_cap = effective_load_cap(stage.downstream_cap, if stage.drives_distributed_load { factor } else { DIRECT_LOAD_CAP_FACTOR });
            let (buffer_delay, buffer_output_slew) = lookup_buffer_delay_and_slew(libs, target_stage, cell, input_slew, gate_load_cap)?;
            let entry = StageEvaluation { path_index: target_stage.path_index, cell: Some(cell.to_string()), input_slew, load_cap: stage.downstream_cap, stage_delay: buffer_delay, output_slew: buffer_output_slew, wire_delay: 0.0 };
            Some((entry, BranchTiming { legal: true, delay: branch.delay + buffer_delay, load_slew: buffer_output_slew, reason: Some(FailReason::EstimateLegal) }))
        }
    }
}

/// `estimateBranchTiming`: the driver, then each stage of the load path, replayed from the cache
/// where a prefix matches.
fn estimate_branch_timing(libs: &Libs, target_stage: &DelayStageState, plan: &TreePlan, load_path: &LoadPath, factor: f32, trace: &mut Option<&mut Vec<StageEvaluation>>, cache: &mut BranchCache) -> BranchTiming {
    let Some(mut branch) = branch_driver_timing(libs, target_stage, plan, trace, cache) else {
        return BranchTiming { legal: false, reason: Some(FailReason::MissingCurrentTimingArc), ..BranchTiming::default() };
    };
    let mut parent = 0usize;
    for stage in &load_path.stages {
        if let Some(child) = cache.find_child(parent, stage) {
            branch = cache.nodes[child].prefix;
            parent = child;
            if let Some(t) = trace.as_deref_mut() {
                t.push(cache.nodes[child].trace_entry.clone());
            }
            continue;
        }
        let Some((entry, next)) = tree_stage_timing(libs, target_stage, stage, factor, &branch) else {
            return BranchTiming { legal: false, reason: Some(FailReason::MissingBufferTimingArc), ..BranchTiming::default() };
        };
        cache.nodes.push(CacheNode { valid: true, stage: Some(stage.clone()), prefix: next, trace_entry: entry.clone(), children: Vec::new() });
        let child = cache.nodes.len() - 1;
        cache.nodes[parent].children.push(child);
        parent = child;
        branch = next;
        if let Some(t) = trace.as_deref_mut() {
            t.push(entry);
        }
    }
    branch
}

/// `estimateBufferTreeWindow`: the pre-target stages replayed, the target driver and the tree's
/// branch to the target load, then each later stage at the propagated slew (bounded below by a
/// share of its current delay).
fn estimate_window(libs: &Libs, context: &EstimatorContext, plan: &TreePlan, trace: &mut Option<&mut Vec<StageEvaluation>>, cache: &mut BranchCache) -> DelayEstimate {
    if !plan.tree_input_cap.is_finite() {
        return super::rejected(FailReason::InvalidBufferInsertion);
    }
    let target_load_path = plan.target_path().expect("a RebufferMt plan reaches its target load");
    let stages = &context.path_stages;
    let target_index = context.target_stage_index;
    let target_stage = context.target();
    let current_total_delay = context.current_total_delay + target_stage.current_wire_delay;
    let mut candidate_total_delay = 0.0f32;
    for stage in &stages[..target_index] {
        candidate_total_delay += stage.current_delay;
        if let Some(t) = trace.as_deref_mut() {
            t.push(StageEvaluation { path_index: stage.path_index, cell: Some(stage.arc.cell.clone()), input_slew: stage.input_slew, load_cap: stage.load_cap, stage_delay: stage.current_delay, output_slew: stage.current_slew, wire_delay: stage.current_wire_delay });
        }
    }
    let branch = estimate_branch_timing(libs, target_stage, plan, target_load_path, TARGET_EFFECTIVE_LOAD_CAP_FACTOR, trace, cache);
    if !branch.legal {
        return super::rejected(branch.reason.unwrap_or(FailReason::None));
    }
    candidate_total_delay += branch.delay;
    let mut propagated_slew = branch.load_slew;
    for stage_index in target_index + 1..stages.len() {
        let stage = &stages[stage_index];
        let input_slew = if stage_index == target_index + 1 { propagated_slew } else { estimate_receiver_input_slew(&stages[stage_index - 1], stage, propagated_slew) };
        let Some((model_stage_delay, stage_slew)) = lookup_arc_delay_and_slew(libs, &stage.arc, input_slew, stage.load_cap, &stage.arc.cell, false, None) else {
            return super::rejected(FailReason::MissingCurrentTimingArc);
        };
        let stage_delay = bound_downstream_stage_delay(stage, input_slew, stage.current_delay + (model_stage_delay - stage.current_model_delay));
        candidate_total_delay += stage_delay;
        propagated_slew = estimate_output_slew(libs, stage, &stage.arc.cell, input_slew, stage.load_cap, stage_slew, false, None);
        if let Some(t) = trace.as_deref_mut() {
            t.push(StageEvaluation { path_index: stage.path_index, cell: Some(stage.arc.cell.clone()), input_slew, load_cap: stage.load_cap, stage_delay, output_slew: propagated_slew, wire_delay: stage.current_wire_delay });
        }
    }
    make_estimate(current_total_delay, candidate_total_delay)
}

/// `estimateSidePathWindow`: the driver and the tree's branch to the side load, against the
/// driver's current delay plus the load's current wire delay.
fn estimate_side_path(libs: &Libs, context: &EstimatorContext, plan: &TreePlan, side: &SidePathContext, trace: &mut Option<&mut Vec<StageEvaluation>>, cache: &mut BranchCache) -> DelayEstimate {
    let load_path = plan.load_path(&side.load_pin).expect("side contexts come from the plan's own load paths");
    let target_stage = context.target();
    let branch = estimate_branch_timing(libs, target_stage, plan, load_path, SIDE_EFFECTIVE_LOAD_CAP_FACTOR, trace, cache);
    if !branch.legal {
        return super::rejected(branch.reason.unwrap_or(FailReason::None));
    }
    make_estimate(target_stage.current_delay + side.current_driver_to_load_delay, branch.delay)
}

/// `DelayEstimator::estimateBufferTreeWithSidePaths`: the target window; then, when it has an
/// estimate, each side load in order — its slack below zero lost (weighted) to the side loss, a
/// drop below the side slack floor (or no estimate) rejects the plan.
pub fn estimate_buffer_tree_with_side_paths(libs: &Libs, context: &EstimatorContext, plan: &TreePlan, side_contexts: &[SidePathContext], mut trace: Option<&mut Vec<StageEvaluation>>, mut side_evaluations: Option<&mut Vec<SidePathEvaluation>>) -> DelayEstimate {
    if let Some(s) = side_evaluations.as_deref_mut() {
        s.clear();
    }
    if let Some(t) = trace.as_deref_mut() {
        t.clear();
    }
    let mut cache = BranchCache::new();
    let mut estimate = estimate_window(libs, context, plan, &mut trace, &mut cache);
    if !matches!(estimate.reason, FailReason::EstimateLegal | FailReason::EstimateNonImproving) {
        return estimate;
    }
    for side in side_contexts {
        let mut side_trace = Vec::new();
        let mut st = side_evaluations.is_some().then_some(&mut side_trace);
        let side_estimate = estimate_side_path(libs, context, plan, side, &mut st, &mut cache);
        let available = matches!(side_estimate.reason, FailReason::EstimateLegal | FailReason::EstimateNonImproving);
        let side_impr = if available { side_estimate.arrival_impr } else { f32::NEG_INFINITY };
        let slack_after = side.current_slack + side_impr;
        estimate.side_paths_evaluated += 1;
        let mut side_legal = available;
        if !available {
            estimate.legal = false;
            estimate.reason = side_estimate.reason;
        } else {
            estimate.side_loss += side.weight * (side.current_slack.min(0.0) - slack_after.min(0.0));
            if slack_after < context.side_slack_floor_for(estimate.arrival_impr) {
                side_legal = false;
                estimate.legal = false;
                estimate.reason = FailReason::SideSlackFloorViolation;
            }
        }
        if let Some(s) = side_evaluations.as_deref_mut() {
            s.push(SidePathEvaluation { load_pin: side.load_pin.clone(), legal: side_legal, arrival_impr: side_impr, slack_after, trace: side_trace });
        }
    }
    estimate
}
