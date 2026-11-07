//! `DelayEstimator`: a target stage's (and its fanin/fanout neighbours') timing captured from the
//! timer once (`buildContext`), then each candidate cell scored on the Liberty table models alone
//! (`estimate`) — the arrival improvement over the stage window. PARALLEL_EVAL's
//! `prepareTargets` builds a context per target (S3a); the estimate is not called yet.
//!
//! The functions are the reference's, in its order. The builder reads one scene's graph after a
//! timer update; the estimate reads only the libraries and the captured state.

use std::collections::{BTreeSet, HashMap, HashSet};

use vyges_sta::dcalc::{Dmp, Thresholds};
use vyges_sta::graph::{EdgeKind, Graph, NetParasitics};
use vyges_sta::liberty::{ArcSet, Cell, Model, Role, MAX};

use crate::preamble::Libs;

/// `SelectedArc`: the stage's timing arc on its CURRENT scene cell — the arc set and the arc
/// within it — and the scene (the min/max is always max here: setup).
#[derive(Debug, Clone, PartialEq)]
pub struct SelectedArc {
    pub scene: usize,
    /// `currentCell()`: the scene cell the arc belongs to.
    pub cell: String,
    pub set: usize,
    pub arc: usize,
}

/// One non-path arc whose slew merges into the driver's output transition.
#[derive(Debug, Clone, PartialEq)]
pub struct OutputSlewMergeArc {
    pub set: usize,
    pub arc: usize,
    pub input_slew: f32,
    pub current_model_slew: f32,
}

/// `SlewBiasSample`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SlewBiasSample {
    pub load_cap: f32,
    pub table_slew: f32,
    pub sta_slew: f32,
}

/// `SlewBiasModel`: the timer's driver slew against the table's at three loads, for the arc whose
/// table slew is the worst at all three.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SlewBiasModel {
    pub valid: bool,
    /// `table_worst_arc` (set, arc) on the stage's current cell.
    pub table_worst_arc: Option<(usize, usize)>,
    pub input_slew: f32,
    pub samples: [SlewBiasSample; 3],
}

/// `OppositeRfArc`: a target cell arc from the selected input to the OPPOSITE output transition,
/// with its current table delay (the dual rise/fall check).
#[derive(Debug, Clone, PartialEq)]
pub struct OppositeRfArc {
    pub set: usize,
    pub arc: usize,
    pub input_slew: f32,
    pub current_model_delay: f32,
}

/// `DelayStageState`.
#[derive(Debug, Clone, PartialEq)]
pub struct DelayStageState {
    pub arc: SelectedArc,
    pub driver_pin: String,
    pub input_slew: f32,
    pub load_cap: f32,
    pub current_model_delay: f32,
    pub current_delay: f32,
    /// The path's wire delay after the driver (`currentPathWireDelay`).
    pub current_wire_delay: f32,
    pub current_model_slew: f32,
    pub current_slew: f32,
    pub output_slew_merge_arcs: Vec<OutputSlewMergeArc>,
    pub sta_slew_bias: SlewBiasModel,
    /// Target stage only, with dual_rf_eval.
    pub opposite_rf_arcs: Vec<OppositeRfArc>,
    /// The stage's index in the target's expanded path; -1 for a latch fanin stage, which has
    /// none.
    pub path_index: i32,
}

/// `SideLoad`: a load pin whose slack pays for the delay a candidate adds to it.
#[derive(Debug, Clone, PartialEq)]
pub struct SideLoad {
    pub pin: String,
    pub slack: f32,
    pub weight: f32,
}

/// `FaninSideNet`: one input net of the target instance and its other loads.
#[derive(Debug, Clone, PartialEq)]
pub struct FaninSideNet {
    pub input_port: String,
    pub input_cap: f32,
    pub delay_per_cap: f32,
    pub selected_arc: bool,
    pub loads: Vec<SideLoad>,
}

/// `EstimatorContext`: the target stage and its window, its fanin side nets, and the side-path
/// floor.
#[derive(Debug, Clone, PartialEq)]
pub struct EstimatorContext {
    pub path_stages: Vec<DelayStageState>,
    pub target_stage_index: usize,
    pub delay_estimation_levels: i32,
    pub fanin_side_nets: Vec<FaninSideNet>,
    /// `-inf` until the policy sets the global worst slack.
    pub side_slack_floor: f32,
    pub side_floor_margin: f32,
    pub fanin_side_loss_weight: f32,
    /// `+inf` unless the opposite output transition has a finite slack.
    pub opposite_rf_slack: f32,
    pub current_total_delay: f32,
}

impl EstimatorContext {
    /// `sideSlackFloor(gain)`: the side slack floor less the margin times the gain above zero.
    pub fn side_slack_floor_for(&self, gain: f32) -> f32 {
        self.side_slack_floor - self.side_floor_margin * gain.max(0.0)
    }

    pub(crate) fn new(delay_estimation_levels: i32) -> EstimatorContext {
        EstimatorContext {
            path_stages: Vec::new(),
            target_stage_index: 0,
            delay_estimation_levels,
            fanin_side_nets: Vec::new(),
            side_slack_floor: f32::NEG_INFINITY,
            side_floor_margin: 1.0,
            fanin_side_loss_weight: 1.0,
            opposite_rf_slack: f32::INFINITY,
            current_total_delay: 0.0,
        }
    }

    pub fn target(&self) -> &DelayStageState {
        &self.path_stages[self.target_stage_index]
    }
}

/// `FailReason`, the values a context build can fail with, and `failReasonName`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailReason {
    MissingTargetInstance,
    PathIndexOutOfRange,
    MissingPrevTimingArc,
    MissingArcTransition,
    MissingSceneOrMinMax,
    MissingOutputPort,
    MissingCurrentPortMap,
    MissingCurrentTimingArc,
    /// The estimate's own reasons (`estimateWindow`, `applyDualRfCheck`, `chargeSideLoss`).
    None,
    EstimateLegal,
    EstimateNonImproving,
    MissingCandidatePort,
    MissingCandidateTimingArc,
    SideSlackFloorViolation,
    DualRfNonImproving,
}

impl FailReason {
    pub fn name(self) -> &'static str {
        match self {
            FailReason::MissingTargetInstance => "missing_target_instance",
            FailReason::PathIndexOutOfRange => "path_index_out_of_range",
            FailReason::MissingPrevTimingArc => "missing_prev_timing_arc",
            FailReason::MissingArcTransition => "missing_arc_transition",
            FailReason::MissingSceneOrMinMax => "missing_scene_or_minmax",
            FailReason::MissingOutputPort => "missing_output_port",
            FailReason::MissingCurrentPortMap => "missing_current_port_map",
            FailReason::MissingCurrentTimingArc => "missing_current_timing_arc",
            FailReason::None => "",
            FailReason::EstimateLegal => "estimate_legal",
            FailReason::EstimateNonImproving => "estimate_non_improving",
            FailReason::MissingCandidatePort => "missing_candidate_port",
            FailReason::MissingCandidateTimingArc => "missing_candidate_timing_arc",
            FailReason::SideSlackFloorViolation => "side_slack_floor_violation",
            FailReason::DualRfNonImproving => "dual_rf_non_improving",
        }
    }
}

/// `FaninSideLoadOptions`: loads on a violating worst path (`critical_load_pins`) weigh 1, the
/// others `off_path_weight`.
#[derive(Debug, Clone, Copy)]
pub struct FaninSideLoadOptions<'a> {
    pub critical_load_pins: &'a HashSet<String>,
    pub off_path_weight: f32,
}

/// The timer's infinity (`sta::INF`).
const INF: f32 = 1e30;

/// `delayInf` (`fuzzyInf`): fuzzily plus or minus the timer's infinity.
pub fn delay_inf(x: f32) -> bool {
    vyges_sta::fuzzy::equal(x, INF) || vyges_sta::fuzzy::equal(x, -INF)
}

/// `sideLoadWeight`.
pub fn side_load_weight(pin: &str, options: &FaninSideLoadOptions<'_>) -> f32 {
    if options.critical_load_pins.contains(pin) { 1.0 } else { options.off_path_weight }
}

/// What `buildContext` adds past the window: each optional stage of the reference's builder.
pub struct ContextOptions<'a> {
    pub delay_levels: i32,
    pub use_sta_slew_bias: bool,
    /// `include_latch_fanin`, and the latch D segment of the target's path (`latchDataPath`,
    /// expanded on its own) with its start index.
    pub latch_data_path: Option<(&'a [PathPin], usize)>,
    pub fanin_side_loads: Option<FaninSideLoadOptions<'a>>,
    pub dual_rf_eval: bool,
}

/// `DelayEstimate`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DelayEstimate {
    pub legal: bool,
    pub candidate_delay: f32,
    pub arrival_impr: f32,
    pub side_paths_evaluated: i32,
    /// The weighted slack loss of the side paths below zero slack; zero when none.
    pub side_loss: f32,
    pub reason: FailReason,
}

/// `{.legal = false, .reason = reason}`.
fn rejected(reason: FailReason) -> DelayEstimate {
    DelayEstimate { legal: false, candidate_delay: 0.0, arrival_impr: 0.0, side_paths_evaluated: 0, side_loss: 0.0, reason }
}

/// `kSideSlewDelayFactor`: the share of a side gate's input slew growth added to its delay.
const SIDE_SLEW_DELAY_FACTOR: f32 = 0.3;
/// `kSideSlewPerDelay`: the slew growth per delay growth of a side net off the selected arc.
const SIDE_SLEW_PER_DELAY: f32 = 1.4;

/// `kSlewMatchAbsTolerance`, `kSlewMatchRelTolerance`, `kMinSlewRatioDenominator`.
const SLEW_MATCH_ABS_TOLERANCE: f32 = 1.0e-15;
const SLEW_MATCH_REL_TOLERANCE: f32 = 1.0e-6;
const MIN_SLEW_RATIO_DENOMINATOR: f32 = 1.0e-15;

/// One stage of the expanded path as the builder reads it: the pin's vertex, and the edge and arc
/// the path took into it.
#[derive(Debug, Clone)]
pub struct PathPin {
    pub vertex: usize,
    pub in_edge: Option<(usize, usize)>,
}

/// What the builder reads: the libraries, one scene's graph and parasitics, and the equivalent
/// cells and dont_use list of `maxTargetInputCapDelta`; for the side loads and the opposite
/// transition, the timer's slacks, the clock network, the pin ids (`PinSet` order) and each
/// master's terminal order (`pinIterator`).
pub struct Reader<'a, 'g> {
    pub libs: &'a Libs,
    pub g: &'g Graph<'g>,
    pub parasitics: &'a HashMap<String, NetParasitics>,
    pub scene: usize,
    pub equiv: &'a crate::sizing::EquivCells,
    pub dont_use: &'a BTreeSet<String>,
    /// `Sta::slack(vertex[, rf], max)` of a vertex of `g`.
    pub slack: &'a dyn Fn(usize, Option<usize>) -> f32,
    /// `Sta::isClock(pin)`: the vertices of `g` in a clock network.
    pub clock_pins: &'a BTreeSet<usize>,
    pub pin_id: &'a HashMap<String, u64>,
    pub master_pins: &'a HashMap<String, Vec<String>>,
}

fn arc_set_of(cell: &Cell, set: usize) -> &ArcSet {
    &cell.arc_sets[set]
}

/// `TimingArcSet::equiv` (from, to, role, cond, the arcs' edges pairwise) — this library model has
/// no `mode`, `sdf_cond` or default-cond to compare: equal by construction.
fn sets_equiv(a: &ArcSet, b: &ArcSet) -> bool {
    a.from == b.from && a.to == b.to && a.role == b.role && a.cond == b.cond && a.arcs.len() == b.arcs.len() && a.arcs.iter().zip(&b.arcs).all(|(x, y)| x.from_rf == y.from_rf && x.to_rf == y.to_rf)
}

/// `isExactArcMatch`: the sets equivalent and the arcs' edges equal.
fn is_exact_arc_match(ref_set: &ArcSet, ref_arc: usize, cand_set: &ArcSet, cand_arc: usize) -> bool {
    let (r, c) = (&ref_set.arcs[ref_arc], &cand_set.arcs[cand_arc]);
    sets_equiv(ref_set, cand_set) && r.from_rf == c.from_rf && r.to_rf == c.to_rf
}

/// `isRelaxedArcMatch`: from an unconditional reference arc into a non-check arc of the same ports
/// and edges (the condition may differ).
fn is_relaxed_arc_match(ref_set: &ArcSet, ref_arc: usize, cand_set: &ArcSet, cand_arc: usize) -> bool {
    let (r, c) = (&ref_set.arcs[ref_arc], &cand_set.arcs[cand_arc]);
    ref_set.cond.is_none() && !cand_set.role.is_timing_check() && ref_set.from == cand_set.from && ref_set.to == cand_set.to && r.from_rf == c.from_rf && r.to_rf == c.to_rf
}

/// `findMatchingTimingArc(reference, candidate_set)` (exact): the first arc of the set that matches.
fn find_matching_arc(ref_set: &ArcSet, ref_arc: usize, cand_set: &ArcSet) -> Option<usize> {
    (0..cand_set.arcs.len()).find(|&k| is_exact_arc_match(ref_set, ref_arc, cand_set, k))
}

/// `timingArcSetsForRefPorts`: the cell's arc sets from the reference arc's from port to its to
/// port, in the cell's order (none when either port is missing).
fn arc_sets_for_ref_ports<'c>(cell: &'c Cell, ref_set: &ArcSet) -> Vec<(usize, &'c ArcSet)> {
    if cell.port(&ref_set.from).is_none() || cell.port(&ref_set.to).is_none() {
        return Vec::new();
    }
    cell.arc_sets.iter().enumerate().filter(|(_, s)| s.from == ref_set.from && s.to == ref_set.to).collect()
}

/// `gateDelayAndSlewFromTableModel` (no PVT scaling: the scene's library is its PVT).
fn gate_delay_and_slew(cell: &Cell, set: usize, arc: usize, in_slew: f32, load_cap: f32) -> Option<(f32, f32)> {
    match &cell.arc_sets[set].arcs[arc].model {
        Model::Gate(m) => Some(m.gate_delay(in_slew, load_cap)),
        _ => None,
    }
}

/// `lookupArcDelayAndSlewForArc`: the candidate cell's (scene cell's) arc matching the reference
/// arc — the first exact match over the sets between its ports; with `relaxed` and none, the
/// relaxed match with the worst output slew. `relaxed_out` is OR-set when the relaxed one is used.
#[allow(clippy::too_many_arguments)]
pub fn lookup_arc_delay_and_slew_for_arc(libs: &Libs, scene: usize, ref_cell: &Cell, ref_set_index: usize, ref_arc: usize, input_slew: f32, load_cap: f32, cell: &str, relaxed: bool, relaxed_out: Option<&mut bool>) -> Option<(f32, f32)> {
    let scene_cell = libs.scene_cell(scene, cell)?;
    let ref_set = arc_set_of(ref_cell, ref_set_index);
    let sets = arc_sets_for_ref_ports(scene_cell, ref_set);
    for (si, set) in &sets {
        if set.role.is_timing_check() {
            continue;
        }
        if let Some(k) = find_matching_arc(ref_set, ref_arc, set) {
            return gate_delay_and_slew(scene_cell, *si, k, input_slew, load_cap);
        }
    }
    if !relaxed {
        return None;
    }
    let mut found: Option<(f32, f32)> = None;
    for (si, set) in &sets {
        if set.role.is_timing_check() {
            continue;
        }
        for k in 0..set.arcs.len() {
            // `matchTimingArc(.., kRelaxedCandidate) == kRelaxed`: relaxed but not exact.
            if is_exact_arc_match(ref_set, ref_arc, set, k) || !is_relaxed_arc_match(ref_set, ref_arc, set, k) {
                continue;
            }
            let Some((d, s)) = gate_delay_and_slew(scene_cell, *si, k, input_slew, load_cap) else { continue };
            // `min_max->compare(candidate_slew, output_slew)`: max, strictly greater.
            if found.is_none_or(|(_, w)| s > w) {
                found = Some((d, s));
            }
        }
    }
    if found.is_some() {
        if let Some(r) = relaxed_out {
            *r = true;
        }
    }
    found
}

/// `lookupArcDelayAndSlew` on the stage's selected arc.
fn lookup_arc_delay_and_slew(libs: &Libs, arc: &SelectedArc, input_slew: f32, load_cap: f32, cell: &str, relaxed: bool, relaxed_out: Option<&mut bool>) -> Option<(f32, f32)> {
    let ref_cell = libs.scene_cell(arc.scene, &arc.cell)?;
    lookup_arc_delay_and_slew_for_arc(libs, arc.scene, ref_cell, arc.set, arc.arc, input_slew, load_cap, cell, relaxed, relaxed_out)
}

/// `LibertyPort::capacitance()`: the largest of its values, 0 without any.
fn port_capacitance(cell: &Cell, port: &str) -> f32 {
    let Some(p) = cell.port(port) else { return 0.0 };
    let mut max: Option<f32> = None;
    for rf in &p.capacitance {
        for &c in rf {
            max = Some(max.map_or(c, |m: f32| m.max(c)));
        }
    }
    max.unwrap_or(0.0)
}

impl Reader<'_, '_> {
    fn cell(&self, name: &str) -> Option<&Cell> {
        self.libs.scene_cell(self.scene, name)
    }

    /// The vertex of pin `name` (a driver or a load, as asked).
    fn vertex_of(&self, name: &str, driver: bool) -> Option<usize> {
        self.g.vertices.iter().position(|x| x.name == name && x.is_driver == driver)
    }

    /// `GraphDelayCalc::edgeFromSlew`: an ideal clock's slew (0) for a clock-to-output arc from an
    /// ideal clock pin, else the graph's max slew.
    fn edge_from_slew(&self, from: usize, rf: usize, role: Role) -> f32 {
        if matches!(role, Role::RegClkToQ | Role::LatchEnToQ) && self.g.ideal_clock.contains(&from) {
            0.0
        } else {
            self.g.slew[from][rf][MAX]
        }
    }

    /// The arc set of a gate edge as the graph holds it (`Edge::timingArcSet`, `Path::prevArc`'s).
    fn edge_set(&self, e: usize) -> Option<(usize, &ArcSet)> {
        let EdgeKind::Gate { set } = self.g.edges[e].kind else { return None };
        let vx = &self.g.vertices[self.g.edges[e].to];
        vx.lib?;
        vx.cell.as_deref()?;
        Some((set, self.g.arc_set(e, set)))
    }

    /// `buildSelectedArc`: the driver's output port, its current scene cell, the path's arc into
    /// the driver (`selectedPathArc`: a gate arc), its ports on that cell, and the first exactly
    /// matching arc over the sets between them.
    fn build_selected_arc(&self, path: &[PathPin], start: usize, index: usize) -> Result<SelectedArc, FailReason> {
        let v = path[index].vertex;
        let vx = &self.g.vertices[v];
        // `network->libertyPort(driver_pin)`: an instance pin of a liberty cell.
        let (Some(cell_name), Some(out_port)) = (vx.cell.as_deref(), vx.port.as_deref()) else { return Err(FailReason::MissingOutputPort) };
        if vx.lib.is_none() {
            return Err(FailReason::MissingOutputPort);
        }
        let current = self.cell(cell_name).ok_or(FailReason::MissingCurrentPortMap)?;
        // `selectedPathArc`.
        if index < start || index >= path.len() {
            return Err(FailReason::PathIndexOutOfRange);
        }
        let (e, k) = path[index].in_edge.ok_or(FailReason::MissingPrevTimingArc)?;
        // A wire arc has no liberty ports.
        let (_, path_set) = self.edge_set(e).ok_or(FailReason::MissingPrevTimingArc)?;
        if current.port(&path_set.from).is_none() || current.port(out_port).is_none() || path_set.to != out_port {
            return Err(FailReason::MissingCurrentPortMap);
        }
        for (si, set) in arc_sets_for_ref_ports(current, path_set) {
            if let Some(a) = find_matching_arc(path_set, k, set) {
                return Ok(SelectedArc { scene: self.scene, cell: current.name.clone(), set: si, arc: a });
            }
        }
        Err(FailReason::MissingCurrentTimingArc)
    }

    /// `inputSlewOrZero`: the instance's pin of the arc's input port, `edgeFromSlew` at it.
    fn input_slew(&self, driver: usize, arc: &SelectedArc) -> f32 {
        let Some(cell) = self.cell(&arc.cell) else { return 0.0 };
        let set = arc_set_of(cell, arc.set);
        let vyges_sta::netlist::Conn::Inst(k, _) = &self.g.vertices[driver].conn else { return 0.0 };
        // `network->findPin(inst, port)` → `pinDrvrVertex`: the pin's one vertex, a bidirect's
        // driver vertex — an output port too (an output-to-output arc, a half adder's CON -> SN,
        // reads the output's own slew).
        let of_pin = |x: &vyges_sta::graph::Vertex| x.port.as_deref() == Some(set.from.as_str()) && matches!(&x.conn, vyges_sta::netlist::Conn::Inst(j, _) if j == k);
        let pin = self.g.vertices.iter().position(|x| of_pin(x) && x.is_driver).or_else(|| self.g.vertices.iter().position(of_pin));
        let Some(u) = pin else { return 0.0 };
        self.edge_from_slew(u, set.arcs[arc.arc].from_rf, set.role)
    }

    /// `collectOutputSlewMergeArcs`: over the driver's in-edges (newest first, as the graph iterates
    /// them), each non-check arc into the selected output port and transition, but the selected
    /// arc: its arc on the current cell, its input slew, its table slew at the load.
    fn collect_output_slew_merge_arcs(&self, driver: usize, arc: &SelectedArc, load_cap: f32) -> Vec<OutputSlewMergeArc> {
        let mut out = Vec::new();
        let Some(current) = self.cell(&arc.cell) else { return out };
        let sel = arc_set_of(current, arc.set);
        let (output_rf, output_port) = (sel.arcs[arc.arc].to_rf, sel.to.clone());
        for &e in self.g.in_edges[driver].iter().rev() {
            let Some((_, set)) = self.edge_set(e) else { continue };
            if set.role.is_timing_check() || set.to != output_port {
                continue;
            }
            for (gk, ga) in set.arcs.iter().enumerate() {
                if ga.to_rf != output_rf {
                    continue;
                }
                // `findCellTimingArcLike(graph_arc, current_cell)`.
                let mut like = None;
                for (si, cset) in arc_sets_for_ref_ports(current, set) {
                    if cset.role.is_timing_check() {
                        continue;
                    }
                    if let Some(a) = find_matching_arc(set, gk, cset) {
                        like = Some((si, a));
                        break;
                    }
                }
                let Some((si, a)) = like else { continue };
                if si == arc.set && a == arc.arc {
                    continue;
                }
                let input_slew = self.edge_from_slew(self.g.edges[e].from, ga.from_rf, set.role);
                let Some((_, slew)) = gate_delay_and_slew(current, si, a, input_slew, load_cap) else { continue };
                out.push(OutputSlewMergeArc { set: si, arc: a, input_slew, current_model_slew: slew });
            }
        }
        out
    }

    /// `currentPathWireDelay`: the wire edge into the next pin of the path, at the arc's output
    /// transition (0 at the path's end or without a wire edge).
    fn current_path_wire_delay(&self, path: &[PathPin], index: usize, out_rf: usize) -> f32 {
        let Some(next) = path.get(index + 1) else { return 0.0 };
        let Some((e, _)) = next.in_edge else { return 0.0 };
        if !matches!(self.g.edges[e].kind, EdgeKind::Wire) {
            return 0.0;
        }
        // `wireArcDelay(edge, rf)`: a wire edge's arc per transition, rise then fall.
        self.g.delay[e].get(out_rf).map_or(0.0, |d| d[MAX])
    }

    /// `buildDelayStageState`.
    fn build_delay_stage_state(&self, path: &[PathPin], start: usize, index: usize) -> Result<DelayStageState, FailReason> {
        let arc = self.build_selected_arc(path, start, index)?;
        let driver = path[index].vertex;
        let input_slew = self.input_slew(driver, &arc);
        let load_cap = self.g.load_cap(driver, self.parasitics);
        let (current_model_delay, current_slew_table) = lookup_arc_delay_and_slew(self.libs, &arc, input_slew, load_cap, &arc.cell, false, None).ok_or(FailReason::MissingCurrentTimingArc)?;
        // `pathGraphCellDelayOrModelDelay`: the graph's delay of the path's arc.
        let current_delay = match path[index].in_edge {
            Some((e, k)) => self.g.delay[e][k][MAX],
            None => current_model_delay,
        };
        let out_rf = arc_set_of(self.cell(&arc.cell).ok_or(FailReason::MissingCurrentTimingArc)?, arc.set).arcs[arc.arc].to_rf;
        let current_wire_delay = self.current_path_wire_delay(path, index, out_rf);
        let merges = self.collect_output_slew_merge_arcs(driver, &arc, load_cap);
        // The merged model slew over every arc to the output transition (`min_max->compare`: max,
        // strictly greater).
        let mut current_model_slew = current_slew_table;
        for m in &merges {
            if m.current_model_slew > current_model_slew {
                current_model_slew = m.current_model_slew;
            }
        }
        Ok(DelayStageState {
            driver_pin: self.g.vertices[driver].name.clone(),
            input_slew,
            load_cap,
            current_model_delay,
            current_delay,
            current_wire_delay,
            current_model_slew,
            current_slew: self.g.slew[driver][out_rf][MAX],
            output_slew_merge_arcs: merges,
            sta_slew_bias: SlewBiasModel::default(),
            opposite_rf_arcs: Vec::new(),
            path_index: index as i32,
            arc,
        })
    }

    /// `buildDelayStageStateFromPath`: the path's index in range and its pin on an instance.
    fn build_delay_stage_state_from_path(&self, path: &[PathPin], start: usize, index: usize) -> Result<DelayStageState, FailReason> {
        if index < start || index >= path.len() {
            return Err(FailReason::PathIndexOutOfRange);
        }
        if !matches!(self.g.vertices[path[index].vertex].conn, vyges_sta::netlist::Conn::Inst(..)) {
            return Err(FailReason::MissingTargetInstance);
        }
        self.build_delay_stage_state(path, start, index)
    }

    /// The instance of a vertex (`network->instance(pin)`), `None` on a top-level port.
    fn inst_of(&self, v: usize) -> Option<usize> {
        match self.g.vertices[v].conn {
            vyges_sta::netlist::Conn::Inst(k, _) => Some(k),
            vyges_sta::netlist::Conn::Port(_) => None,
        }
    }

    fn inst_of_pin(&self, name: &str) -> Option<usize> {
        self.vertex_of(name, true).or_else(|| self.vertex_of(name, false)).and_then(|v| self.inst_of(v))
    }

    /// `collectPathStages`: the target, with up to `levels` valid stages before it (nearest first
    /// found, kept in path order) and after it — a later stage on the target's own instance (an
    /// output-to-output arc) is skipped; the total is the sum of the stages' delays in order.
    fn collect_path_stages(&self, path: &[PathPin], start: usize, target_index: usize, levels: i32, target: DelayStageState) -> EstimatorContext {
        let mut context = EstimatorContext::new(levels);
        if levels == 0 {
            context.current_total_delay = target.current_delay;
            context.path_stages = vec![target];
            return context;
        }
        let levels = levels as usize;
        let mut fanin = Vec::new();
        let mut i = target_index;
        while i > start && fanin.len() < levels {
            i -= 1;
            if let Ok(s) = self.build_delay_stage_state_from_path(path, start, i) {
                fanin.push(s);
            }
        }
        fanin.reverse();
        let target_inst = self.inst_of_pin(&target.driver_pin);
        let mut stages = fanin;
        context.target_stage_index = stages.len();
        stages.push(target);
        let mut found = 0;
        let mut i = target_index + 1;
        while i < path.len() && found < levels {
            if let Ok(s) = self.build_delay_stage_state_from_path(path, start, i) {
                let fanout_inst = self.inst_of(path[i].vertex);
                if !(fanout_inst.is_some() && target_inst.is_some() && fanout_inst == target_inst) {
                    stages.push(s);
                    found += 1;
                }
            }
            i += 1;
        }
        let mut total = 0.0f32;
        for s in &stages {
            total += s.current_delay;
        }
        context.path_stages = stages;
        context.current_total_delay = total;
        context
    }

    /// `includeLatchFaninStages`: a target whose arc is a latch D -> Q at the path's start takes
    /// the stages that drive the latch D (its data path expanded on its own) in front of its
    /// window. The D pin's driver must reach it by a wire and sit on another instance; the
    /// stages, none on the target's instance, have no index in the target's path.
    fn include_latch_fanin_stages(&self, path: &[PathPin], start: usize, latch_data_path: Option<(&[PathPin], usize)>, context: &mut EstimatorContext) -> Result<(), FailReason> {
        let target = context.target();
        let role = self.cell(&target.arc.cell).map(|c| arc_set_of(c, target.arc.set).role);
        if role != Some(Role::LatchDtoQ) || target.path_index != start as i32 {
            return Ok(());
        }
        let _ = path;
        // `latchDataPathMatches`: the Q path's previous path is the latch D path.
        let Some((data, data_start)) = latch_data_path else { return Err(FailReason::MissingPrevTimingArc) };
        let target_inst = self.inst_of_pin(&target.driver_pin);
        if data.len() < 2 {
            return Err(FailReason::MissingPrevTimingArc);
        }
        let driver_index = data.len() - 2;
        let d = &data[data.len() - 1];
        let wire = d.in_edge.is_some_and(|(e, _)| matches!(self.g.edges[e].kind, EdgeKind::Wire) && self.g.edges[e].from == data[driver_index].vertex && self.g.edges[e].to == d.vertex);
        if driver_index < data_start || !wire || self.inst_of(data[driver_index].vertex) == target_inst {
            return Err(FailReason::MissingPrevTimingArc);
        }
        let driver_stage = self.build_delay_stage_state_from_path(data, data_start, driver_index)?;
        let additional_levels = context.delay_estimation_levels.max(1) - 1;
        let mut fanin = self.collect_path_stages(data, data_start, driver_index, additional_levels, driver_stage);
        for stage in &mut fanin.path_stages {
            // A feedback segment must not bring the resized driver back.
            if self.inst_of_pin(&stage.driver_pin) == target_inst {
                return Err(FailReason::MissingPrevTimingArc);
            }
            stage.path_index = -1;
        }
        let n = fanin.path_stages.len();
        let mut stages = fanin.path_stages;
        stages.append(&mut context.path_stages);
        context.path_stages = stages;
        context.target_stage_index += n;
        context.current_total_delay += fanin.current_total_delay;
        Ok(())
    }

    /// `maxTargetInputCapDelta`: over the link cell's equivalent cells (not dont_use, link cells),
    /// the largest input capacitance of the arc's input port beyond the current one.
    fn max_target_input_cap_delta(&self, target: &DelayStageState) -> f32 {
        let Some(current) = self.cell(&target.arc.cell) else { return 0.0 };
        let input = arc_set_of(current, target.arc.set).from.clone();
        if self.libs.link_cell(&current.name).is_none() {
            return 0.0;
        }
        let current_input_cap = port_capacitance(current, &input);
        let mut max_input_cap = current_input_cap;
        if let Some(&class) = self.equiv.class_of.get(&current.name) {
            for name in &self.equiv.classes[class] {
                if self.dont_use.contains(name) {
                    continue;
                }
                // `candidateInputCap`: the scene cell's port of the same name.
                if let Some(c) = self.cell(name) {
                    if c.port(&input).is_some() {
                        max_input_cap = max_input_cap.max(port_capacitance(c, &input));
                    }
                }
            }
        }
        (max_input_cap - current_input_cap).max(0.0)
    }

    /// `findTableWorstArcAtLoad`: over the selected arc and the merge arcs (in that order), the one
    /// with the strictly largest table slew at `load` (the first on a tie).
    fn find_table_worst_arc_at_load(&self, stage: &DelayStageState, load: f32) -> Option<((usize, usize), f32)> {
        let current = self.cell(&stage.arc.cell)?;
        let arcs = std::iter::once(((stage.arc.set, stage.arc.arc), stage.input_slew)).chain(stage.output_slew_merge_arcs.iter().map(|m| ((m.set, m.arc), m.input_slew)));
        let mut worst: Option<((usize, usize), f32, f32)> = None;
        for (a, in_slew) in arcs {
            let Some((_, s)) = lookup_arc_delay_and_slew_for_arc(self.libs, self.scene, current, a.0, a.1, in_slew, load, &current.name, false, None) else { continue };
            if worst.is_none_or(|(_, _, w)| s > w) {
                worst = Some((a, in_slew, s));
            }
        }
        worst.map(|(a, i, _)| (a, i))
    }

    /// `buildSlewBiasModel`: at the stage load and up to `max_load_delta` more, the table-worst arc
    /// stable at all three; the timer's driver slew on a synthetic pi model of each load (the net's
    /// reduced pi scaled, c2's share kept) against the table's.
    fn build_slew_bias_model(&self, stage: &DelayStageState, max_load_delta: f32) -> SlewBiasModel {
        if max_load_delta <= 0.0 {
            return SlewBiasModel::default();
        }
        let loads = [stage.load_cap, stage.load_cap + max_load_delta * 0.5, stage.load_cap + max_load_delta];
        // `findStableTableWorstArc`: the same arc and input slew at every load.
        let Some((worst, in_slew)) = self.find_table_worst_arc_at_load(stage, loads[0]) else { return SlewBiasModel::default() };
        for &l in &loads[1..] {
            match self.find_table_worst_arc_at_load(stage, l) {
                Some((w, i)) if w == worst && i == in_slew => {}
                _ => return SlewBiasModel::default(),
            }
        }
        let Some(driver) = self.vertex_of(&stage.driver_pin, true) else { return SlewBiasModel::default() };
        let Some(current) = self.cell(&stage.arc.cell) else { return SlewBiasModel::default() };
        let out_rf = arc_set_of(current, stage.arc.set).arcs[stage.arc.arc].to_rf;
        // `findParasitic` and `isPiModel`.
        let Some((c2, rpi, c1)) = find_parasitic_pi(self.g, driver, out_rf, self.parasitics) else { return SlewBiasModel::default() };
        let Some(lib) = self.g.vertices[driver].lib.and_then(|l| self.g.libs.get(l)) else { return SlewBiasModel::default() };
        let Model::Gate(model) = &current.arc_sets[worst.0].arcs[worst.1].model else { return SlewBiasModel::default() };
        let th = Thresholds { vth: lib.output_threshold[out_rf], vl: lib.slew_lower_threshold[out_rf], vh: lib.slew_upper_threshold[out_rf], slew_derate: lib.slew_derate };
        let mut m = SlewBiasModel { valid: false, table_worst_arc: Some(worst), input_slew: in_slew, samples: [SlewBiasSample::default(); 3] };
        for (k, &load) in loads.iter().enumerate() {
            // `fillSlewBiasSample`.
            let Some((_, table_slew)) = lookup_arc_delay_and_slew_for_arc(self.libs, self.scene, current, worst.0, worst.1, in_slew, load, &current.name, false, None) else { return SlewBiasModel::default() };
            // `syntheticPiForLoad`.
            let total = c1 + c2;
            if total <= 0.0 || stage.load_cap <= 0.0 || load < 0.0 {
                return SlewBiasModel::default();
            }
            let c2_fraction = c2 / total;
            let load_delta = load - stage.load_cap;
            let s_c2 = c2 + load_delta * c2_fraction;
            let s_c1 = c1 + load_delta * (1.0 - c2_fraction);
            if !(s_c2 >= 0.0 && s_c1 >= 0.0 && s_c2.is_finite() && rpi.is_finite() && s_c1.is_finite()) {
                return SlewBiasModel::default();
            }
            // `staDriverSlewForSyntheticPi`: the active delay calculator (DMP) on the sample.
            let sta_slew = Dmp::new(model, &th, in_slew, s_c2, rpi, s_c1).gate_delay_slew().1 as f32;
            if !(sta_slew.is_finite() && sta_slew >= 0.0) || !table_slew.is_finite() {
                return SlewBiasModel::default();
            }
            m.samples[k] = SlewBiasSample { load_cap: load, table_slew, sta_slew };
        }
        m.valid = true;
        m
    }

    /// `prepareFaninNeighborSlewBias`: the fanin neighbour's slew bias over the target's input
    /// cap growth. (`DelayEstimatorCache` memoizes both reads on every input they depend on, so a
    /// recomputation gives what a hit returns.)
    fn prepare_fanin_neighbor_slew_bias(&self, context: &mut EstimatorContext) {
        if context.target_stage_index == 0 {
            return;
        }
        let delta = self.max_target_input_cap_delta(context.target());
        if delta <= 0.0 {
            return;
        }
        let k = context.target_stage_index - 1;
        context.path_stages[k].sta_slew_bias = self.build_slew_bias_model(&context.path_stages[k], delta);
    }

    /// `collectFaninSideLoads`: each input net of the target, in the instance's pin order — an
    /// input pin (not a clock) with a liberty port and a net. The selected-arc net (the target's
    /// selected input, driven by the fanin stage) leaves out the target pin; any other net needs
    /// its first driver's (`PinSet` order) drive resistance > 0. Its loads in `PinSet` order, each
    /// with a finite slack; a net without loads is dropped.
    fn collect_fanin_side_loads(&self, target_inst: usize, options: &FaninSideLoadOptions<'_>, context: &mut EstimatorContext) {
        let target = context.target();
        let Some(target_cell) = self.cell(&target.arc.cell) else { return };
        let selected_input_port = arc_set_of(target_cell, target.arc.set).from.clone();
        let selected_fanin_pin = (context.target_stage_index >= 1).then(|| context.path_stages[context.target_stage_index - 1].driver_pin.clone());
        let nl: &vyges_sta::netlist::Netlist = &self.g.netlist;
        let (inst_name, link_name) = (&nl.insts[target_inst].0, &nl.insts[target_inst].1);
        let Some(link) = self.libs.link_cell(link_name) else { return };
        let id = |name: &str| self.pin_id.get(name).copied().unwrap_or(u64::MAX);
        // `network->net(pin)`: the vertex's net (a load vertex for an input, else the driver's).
        let net_of = |pin: &str| self.vertex_of(pin, false).or_else(|| self.vertex_of(pin, true)).and_then(|v| self.g.vertex_net[v]);
        let ports: Vec<String> = self.master_pins.get(link_name).cloned().unwrap_or_default();
        let mut nets = Vec::new();
        for port_name in &ports {
            let Some(port) = link.port(port_name) else { continue };
            if port.direction != vyges_sta::liberty::Direction::Input {
                continue;
            }
            let input_pin = format!("{inst_name}/{port_name}");
            if self.vertex_of(&input_pin, false).is_some_and(|v| self.clock_pins.contains(&v)) {
                continue;
            }
            let Some(net) = net_of(&input_pin) else { continue };
            let selected_arc = *port_name == selected_input_port && selected_fanin_pin.as_deref().is_some_and(|f| net_of(f) == Some(net));
            let mut delay_per_cap = 0.0;
            if !selected_arc {
                // `network->drivers(net)`, a `PinSet`: its first pin by id.
                let mut drivers: Vec<String> = nl.nets[net].pins.iter().map(|c| nl.pin_name(c)).filter(|n| self.vertex_of(n, true).is_some()).collect();
                drivers.sort_by_key(|n| id(n));
                let Some(driver) = drivers.first() else { continue };
                let Some(dv) = self.vertex_of(driver, true) else { continue };
                let dx = &self.g.vertices[dv];
                let Some((dcell, dport)) = dx.cell.as_deref().zip(dx.port.as_deref()) else { continue };
                let Some(dlink) = self.libs.link_cell(dcell) else { continue };
                if dlink.port(dport).is_none() {
                    continue;
                }
                delay_per_cap = dlink.drive_resistance(dport);
                if delay_per_cap <= 0.0 {
                    continue;
                }
            }
            let mut pins: Vec<String> = nl.nets[net].pins.iter().map(|c| nl.pin_name(c)).collect();
            pins.sort_by_key(|n| id(n));
            let mut loads = Vec::new();
            for pin in pins {
                if selected_arc && pin == input_pin {
                    continue;
                }
                // `isLoad` → `pinLoadVertex`.
                let Some(v) = self.vertex_of(&pin, false) else { continue };
                let slack = (self.slack)(v, None);
                if delay_inf(slack) {
                    continue;
                }
                loads.push(SideLoad { weight: side_load_weight(&pin, options), pin, slack });
            }
            if loads.is_empty() {
                continue;
            }
            nets.push(FaninSideNet { input_port: port_name.clone(), input_cap: port_capacitance(link, port_name), delay_per_cap, selected_arc, loads });
        }
        context.fanin_side_nets = nets;
    }

    /// `collectOppositeRfArcs`: over the driver's in-edges (as the graph iterates them) from the
    /// selected input to the selected output, each non-check arc to the OPPOSITE transition with
    /// its match on the current cell and its table delay at the stage load; with any, the
    /// driver's worst slack at that transition when finite.
    fn collect_opposite_rf_arcs(&self, context: &mut EstimatorContext) {
        let k = context.target_stage_index;
        let target = &context.path_stages[k];
        let Some(driver) = self.vertex_of(&target.driver_pin, true) else { return };
        let Some(current) = self.cell(&target.arc.cell) else { return };
        let sel = arc_set_of(current, target.arc.set);
        let (input_name, output_name) = (sel.from.clone(), sel.to.clone());
        let opposite_rf = 1 - sel.arcs[target.arc.arc].to_rf;
        let load_cap = target.load_cap;
        let mut arcs = Vec::new();
        for &e in self.g.in_edges[driver].iter().rev() {
            let Some((_, set)) = self.edge_set(e) else { continue };
            if set.role.is_timing_check() || set.to != output_name || set.from != input_name {
                continue;
            }
            for (gk, ga) in set.arcs.iter().enumerate() {
                if ga.to_rf != opposite_rf {
                    continue;
                }
                // `findCellTimingArcLike(graph_arc, current_cell)`.
                let mut like = None;
                for (si, cset) in arc_sets_for_ref_ports(current, set) {
                    if cset.role.is_timing_check() {
                        continue;
                    }
                    if let Some(a) = find_matching_arc(set, gk, cset) {
                        like = Some((si, a));
                        break;
                    }
                }
                let Some((si, a)) = like else { continue };
                let input_slew = self.edge_from_slew(self.g.edges[e].from, ga.from_rf, set.role);
                let Some((delay, _)) = gate_delay_and_slew(current, si, a, input_slew, load_cap) else { continue };
                arcs.push(OppositeRfArc { set: si, arc: a, input_slew, current_model_delay: delay });
            }
        }
        if arcs.is_empty() {
            return;
        }
        context.path_stages[k].opposite_rf_arcs = arcs;
        let slack = (self.slack)(driver, Some(opposite_rf));
        if !delay_inf(slack) {
            context.opposite_rf_slack = slack;
        }
    }

    /// `DelayEstimator::buildContext`: the target stage (it must be valid), its window, the latch
    /// fanin stages, the fanin neighbour's slew bias, the fanin side loads and the opposite
    /// transition's arcs, in that order.
    pub fn build_context(&self, path: &[PathPin], start: usize, index: usize, options: &ContextOptions<'_>) -> Result<EstimatorContext, FailReason> {
        let target_inst = self.inst_of(path[index].vertex).ok_or(FailReason::MissingTargetInstance)?;
        let target = self.build_delay_stage_state(path, start, index)?;
        let mut context = self.collect_path_stages(path, start, index, options.delay_levels.max(0), target);
        self.include_latch_fanin_stages(path, start, options.latch_data_path, &mut context)?;
        if options.use_sta_slew_bias {
            self.prepare_fanin_neighbor_slew_bias(&mut context);
        }
        if let Some(side) = &options.fanin_side_loads {
            self.collect_fanin_side_loads(target_inst, side, &mut context);
        }
        if options.dual_rf_eval {
            self.collect_opposite_rf_arcs(&mut context);
        }
        Ok(context)
    }
}

/// `ArcDelayCalc::findParasitic(driver, rf, scene, max)` as the slew bias reads it
/// (`LumpedCapDelayCalc::findParasitic`, which the DMP calculator inherits): no parasitic when
/// the driver's net has a `set_load` (it takes precedence over parasitics), else the net's
/// reduced pi model.
fn find_parasitic_pi(g: &Graph<'_>, driver: usize, rf: usize, parasitics: &HashMap<String, NetParasitics>) -> Option<(f32, f32, f32)> {
    if g.sdc.net_wire_cap.contains_key(&g.vertices[driver].name) {
        return None;
    }
    g.reduced_pi(driver, rf, MAX, parasitics)
}

/// `slewBias`, `interpolate`, `interpolateStaSlewBias`.
fn interpolate(x0: f32, y0: f32, x1: f32, y1: f32, x: f32) -> f32 {
    if x1 <= x0 {
        return y0;
    }
    let ratio = (x - x0) / (x1 - x0);
    y0 + ratio * (y1 - y0)
}

fn interpolate_sta_slew_bias(m: &SlewBiasModel, load_cap: f32) -> f32 {
    let bias = |s: &SlewBiasSample| s.sta_slew - s.table_slew;
    let [low, mid, high] = &m.samples;
    let clamped = load_cap.clamp(low.load_cap, high.load_cap);
    if clamped <= mid.load_cap {
        interpolate(low.load_cap, bias(low), mid.load_cap, bias(mid), clamped)
    } else {
        interpolate(mid.load_cap, bias(mid), high.load_cap, bias(high), clamped)
    }
}

/// `slewsMatch`.
fn slews_match(a: f32, b: f32) -> bool {
    let tolerance = SLEW_MATCH_ABS_TOLERANCE.max(SLEW_MATCH_REL_TOLERANCE * a.abs().max(b.abs()));
    (a - b).abs() <= tolerance
}

/// `estimateOutputSlew`: with the stage's slew bias (its current cell, its input slew), the
/// table-worst arc's slew plus the interpolated bias; else the merged table slew calibrated to the
/// timer's current slew.
#[allow(clippy::too_many_arguments)]
fn estimate_output_slew(libs: &Libs, stage: &DelayStageState, cell: &str, path_input_slew: f32, load_cap: f32, selected_output_slew: f32, relaxed: bool, mut relaxed_out: Option<&mut bool>) -> f32 {
    let Some(ref_cell) = libs.scene_cell(stage.arc.scene, &stage.arc.cell) else { return 0.0 };
    if stage.sta_slew_bias.valid && cell == stage.arc.cell && slews_match(path_input_slew, stage.input_slew) {
        if let Some((s, a)) = stage.sta_slew_bias.table_worst_arc {
            if let Some((_, table_slew)) = lookup_arc_delay_and_slew_for_arc(libs, stage.arc.scene, ref_cell, s, a, stage.sta_slew_bias.input_slew, load_cap, cell, false, None) {
                let bias = interpolate_sta_slew_bias(&stage.sta_slew_bias, load_cap);
                return (table_slew + bias).max(0.0);
            }
        }
    }
    let sel = arc_set_of(ref_cell, stage.arc.set);
    let (sel_from, sel_from_rf) = (sel.from.clone(), sel.arcs[stage.arc.arc].from_rf);
    let mut model_output_slew = selected_output_slew;
    for m in &stage.output_slew_merge_arcs {
        let mset = arc_set_of(ref_cell, m.set);
        let uses_path_input = mset.from == sel_from && mset.arcs[m.arc].from_rf == sel_from_rf;
        let input_slew = if uses_path_input { path_input_slew } else { m.input_slew };
        let Some((_, s)) = lookup_arc_delay_and_slew_for_arc(libs, stage.arc.scene, ref_cell, m.set, m.arc, input_slew, load_cap, cell, relaxed, relaxed_out.as_deref_mut()) else { continue };
        if s > model_output_slew {
            model_output_slew = s;
        }
    }
    let calibrated = stage.current_slew + (model_output_slew - stage.current_model_slew);
    calibrated.max(0.0)
}

/// `estimateReceiverInputSlew`: the candidate driver slew scaled by the net's current
/// receiver/driver slew ratio.
fn estimate_receiver_input_slew(driver: &DelayStageState, receiver: &DelayStageState, candidate_driver_output_slew: f32) -> f32 {
    let current_driver_output_slew = driver.current_slew.max(MIN_SLEW_RATIO_DENOMINATOR);
    let current_receiver_input_slew = receiver.input_slew.max(0.0);
    let candidate_output_slew = candidate_driver_output_slew.max(0.0);
    candidate_output_slew * current_receiver_input_slew / current_driver_output_slew
}

/// `SelectedArc::inputPort()` and `outputPort()` names: the arc set's from and to ports on the
/// arc's scene cell.
pub fn arc_ports(libs: &Libs, arc: &SelectedArc) -> Option<(String, String)> {
    let cell = libs.scene_cell(arc.scene, &arc.cell)?;
    let set = arc_set_of(cell, arc.set);
    Some((set.from.clone(), set.to.clone()))
}

/// `makeEstimate`.
fn make_estimate(current_delay: f32, candidate_delay: f32) -> DelayEstimate {
    let arrival_impr = current_delay - candidate_delay;
    let legal = arrival_impr > 0.0;
    DelayEstimate { legal, candidate_delay, arrival_impr, side_paths_evaluated: 0, side_loss: 0.0, reason: if legal { FailReason::EstimateLegal } else { FailReason::EstimateNonImproving } }
}

/// `oppositeRfSideLoss`: the opposite transition's slack lost below zero to a delay growth.
fn opposite_rf_side_loss(opposite_slack: f32, delay_delta: f32) -> f32 {
    if !opposite_slack.is_finite() || delay_delta <= 0.0 {
        return 0.0;
    }
    opposite_slack.min(0.0) - (opposite_slack - delay_delta).min(0.0)
}

/// `applyDualRfCheck`: the opposite output transition charged like a side path — the worst
/// delay growth over its arcs the candidate also has (relaxed match), when positive and the
/// opposite slack is finite, costs its loss below zero slack; a slack after below
/// `sideSlackFloor(arrival_impr)` rejects (`dual_rf_non_improving`).
fn apply_dual_rf_check(libs: &Libs, context: &EstimatorContext, candidate_cell: &str, estimate: &mut DelayEstimate) {
    let target = context.target();
    if target.opposite_rf_arcs.is_empty() {
        return;
    }
    let Some(ref_cell) = libs.scene_cell(target.arc.scene, &target.arc.cell) else { return };
    let mut worst_delta = f32::NEG_INFINITY;
    for opposite in &target.opposite_rf_arcs {
        let Some((delay, _)) = lookup_arc_delay_and_slew_for_arc(libs, target.arc.scene, ref_cell, opposite.set, opposite.arc, opposite.input_slew, target.load_cap, candidate_cell, true, None) else { continue };
        worst_delta = worst_delta.max(delay - opposite.current_model_delay);
    }
    if !worst_delta.is_finite() {
        return;
    }
    if worst_delta <= 0.0 || !context.opposite_rf_slack.is_finite() {
        return;
    }
    let slack_after = context.opposite_rf_slack - worst_delta;
    estimate.side_loss += opposite_rf_side_loss(context.opposite_rf_slack, worst_delta);
    estimate.side_paths_evaluated += 1;
    if slack_after < context.side_slack_floor_for(estimate.arrival_impr) {
        estimate.legal = false;
        estimate.reason = FailReason::DualRfNonImproving;
    }
}

/// `chargeSideLoss`: `side_paths` more side paths, their loss weighted by
/// `fanin_side_loss_weight` (added: the dual check may have charged one), and a worst slack after
/// below `sideSlackFloor(arrival_impr)` rejects (`side_slack_floor_violation`).
fn charge_side_loss(context: &EstimatorContext, side_paths: i32, loss: f32, worst_slack_after: f32, estimate: &mut DelayEstimate) {
    estimate.side_paths_evaluated += side_paths;
    estimate.side_loss += loss * context.fanin_side_loss_weight;
    if worst_slack_after < context.side_slack_floor_for(estimate.arrival_impr) {
        estimate.legal = false;
        estimate.reason = FailReason::SideSlackFloorViolation;
    }
}

/// `applyFaninSideLoss`: the fanin drivers' delay growth charged to the recorded side loads. On
/// the selected arc's net the window's fanin delay and slew deltas; off it, `delay_per_cap` times
/// the candidate's input cap change and `kSideSlewPerDelay` of that as slew (a port the candidate
/// lacks is skipped). A delta at or below zero is skipped; else `kSideSlewDelayFactor` of the
/// positive slew growth is added, and each load loses `weight * (min(slack, 0) - min(slack -
/// delta, 0))`.
fn apply_fanin_side_loss(context: &EstimatorContext, candidate_scene_cell: &Cell, selected_fanin_delay_delta: f32, selected_fanin_slew_delta: f32, estimate: &mut DelayEstimate) {
    if context.fanin_side_nets.is_empty() {
        return;
    }
    let mut loss = 0.0f32;
    let mut worst_slack_after = f32::INFINITY;
    let mut side_paths = 0;
    for side_net in &context.fanin_side_nets {
        let mut delay_delta = selected_fanin_delay_delta;
        let mut slew_delta = selected_fanin_slew_delta;
        if !side_net.selected_arc {
            if candidate_scene_cell.port(&side_net.input_port).is_none() {
                continue;
            }
            delay_delta = side_net.delay_per_cap * (port_capacitance(candidate_scene_cell, &side_net.input_port) - side_net.input_cap);
            slew_delta = SIDE_SLEW_PER_DELAY * delay_delta;
        }
        if delay_delta <= 0.0 {
            continue;
        }
        delay_delta += SIDE_SLEW_DELAY_FACTOR * slew_delta.max(0.0);
        for side in &side_net.loads {
            let slack_after = side.slack - delay_delta;
            loss += side.weight * (side.slack.min(0.0) - slack_after.min(0.0));
            worst_slack_after = worst_slack_after.min(slack_after);
            side_paths += 1;
        }
    }
    if side_paths == 0 {
        return;
    }
    charge_side_loss(context, side_paths, loss, worst_slack_after, estimate);
}

/// `estimateWindow`: the window's stages from the fanin neighbour on table-modelled — the
/// candidate at the target (with `target_load_delta` on its load, floored at zero), its input cap
/// change on the neighbour's load — each stage's slew carried to the next; the improvement
/// against the current total delay; then, when legal, the dual transition check, and the fanin
/// side loss from the neighbour's delay and slew deltas.
fn estimate_window(libs: &Libs, context: &EstimatorContext, candidate_cell: &str, target_load_delta: f32) -> DelayEstimate {
    let stages = &context.path_stages;
    let target_index = context.target_stage_index;
    let target = &stages[target_index];
    let Some(ref_cell) = libs.scene_cell(target.arc.scene, &target.arc.cell) else { return rejected(FailReason::MissingCandidatePort) };
    let input_port = arc_set_of(ref_cell, target.arc.set).from.clone();
    let Some(candidate_scene_cell) = libs.scene_cell(target.arc.scene, candidate_cell) else { return rejected(FailReason::MissingCandidatePort) };
    if candidate_scene_cell.port(&input_port).is_none() {
        return rejected(FailReason::MissingCandidatePort);
    }
    let target_input_cap_delta = port_capacitance(candidate_scene_cell, &input_port) - port_capacitance(ref_cell, &input_port);
    let current_total_delay = context.current_total_delay;
    let mut candidate_total_delay = 0.0f32;
    let mut fanin_delay_delta = 0.0f32;
    let mut fanin_slew_delta = 0.0f32;
    let mut propagated: Option<f32> = None;
    for (i, stage) in stages.iter().enumerate() {
        if i + 1 < target_index {
            candidate_total_delay += stage.current_delay;
            continue;
        }
        let is_target = i == target_index;
        let is_fanin_neighbor = i + 1 == target_index;
        let cell = if is_target { candidate_cell } else { stage.arc.cell.as_str() };
        let load_cap = if is_fanin_neighbor {
            stage.load_cap + target_input_cap_delta
        } else if is_target {
            (stage.load_cap + target_load_delta).max(0.0)
        } else {
            stage.load_cap
        };
        let input_slew = match propagated {
            Some(p) => estimate_receiver_input_slew(&stages[i - 1], stage, p),
            None => stage.input_slew,
        };
        let fail_reason = if is_target { FailReason::MissingCandidateTimingArc } else { FailReason::MissingCurrentTimingArc };
        let mut relaxed_flag = false;
        let Some((model_stage_delay, stage_slew)) = lookup_arc_delay_and_slew(libs, &stage.arc, input_slew, load_cap, cell, is_target, if is_target { Some(&mut relaxed_flag) } else { None }) else { return rejected(fail_reason) };
        let stage_delay = stage.current_delay + (model_stage_delay - stage.current_model_delay);
        candidate_total_delay += stage_delay;
        let output_slew = estimate_output_slew(libs, stage, cell, input_slew, load_cap, stage_slew, is_target, if is_target { Some(&mut relaxed_flag) } else { None });
        propagated = Some(output_slew);
        if is_fanin_neighbor {
            fanin_delay_delta = stage_delay - stage.current_delay;
            fanin_slew_delta = output_slew - stage.current_slew;
        }
    }
    let mut estimate = make_estimate(current_total_delay, candidate_total_delay);
    if estimate.legal {
        apply_dual_rf_check(libs, context, candidate_cell, &mut estimate);
    }
    apply_fanin_side_loss(context, candidate_scene_cell, fanin_delay_delta, fanin_slew_delta, &mut estimate);
    estimate
}

/// `DelayEstimator::estimate`: [`estimate_window`] with the candidate at the target.
pub fn estimate(libs: &Libs, context: &EstimatorContext, candidate_cell: &str) -> DelayEstimate {
    estimate_window(libs, context, candidate_cell, 0.0)
}

/// `DelayEstimator::estimateLoadChange`: [`estimate_window`] with the current cell and a load
/// change on the target net.
pub fn estimate_load_change(libs: &Libs, context: &EstimatorContext, target_load_delta: f32) -> DelayEstimate {
    let cell = context.target().arc.cell.clone();
    estimate_window(libs, context, &cell, target_load_delta)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule (estimateReceiverInputSlew): the receiver's input slew keeps the net's current
    // receiver/driver ratio; a zero driver slew divides by 1e-15, not by zero.
    #[test]
    fn receiver_slew_keeps_the_nets_ratio() {
        let st = |input_slew: f32, current_slew: f32| DelayStageState {
            arc: SelectedArc { scene: 0, cell: String::new(), set: 0, arc: 0 },
            driver_pin: String::new(),
            input_slew,
            load_cap: 0.0,
            current_model_delay: 0.0,
            current_delay: 0.0,
            current_wire_delay: 0.0,
            current_model_slew: 0.0,
            current_slew,
            output_slew_merge_arcs: Vec::new(),
            sta_slew_bias: SlewBiasModel::default(),
            opposite_rf_arcs: Vec::new(),
            path_index: 0,
        };
        assert_eq!(estimate_receiver_input_slew(&st(0.0, 2e-11), &st(3e-11, 0.0), 4e-11), 4e-11 * 3e-11 / 2e-11);
        assert_eq!(estimate_receiver_input_slew(&st(0.0, 0.0), &st(1e-11, 0.0), 1e-11), 1e-11 * 1e-11 / 1e-15);
    }

    // Rule (LumpedCapDelayCalc::findParasitic, through buildSlewBiasModel's "missing Pi
    // parasitic"): a driver whose net has a set_load has no parasitic for the slew bias even when
    // the estimator made a pi model for the net; without the set_load the pi model is found.
    // Witness: repair_setup8 under PARALLEL_EVAL, where `set_load 1.0 l2q` makes the reference
    // skip the latch Q stage's bias.
    #[test]
    fn a_net_set_load_hides_the_pi_model_from_the_slew_bias() {
        use vyges_sta::netlist::{inst, Conn, Net, Netlist, PortDir};
        const D: &str = r#"cell_rise (t) { values ("0.01, 0.01", "0.01, 0.01"); } rise_transition (t) { values ("0.01, 0.01", "0.01, 0.01"); }
          cell_fall (t) { values ("0.01, 0.01", "0.01, 0.01"); } fall_transition (t) { values ("0.01, 0.01", "0.01, 0.01"); }"#;
        let lib = format!(
            r#"library (t) {{ time_unit : "1ns"; capacitive_load_unit (1, pf);
          lu_table_template (t) {{ variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; index_1 ("0, 1"); index_2 ("0, 1"); }}
          cell (buf) {{ pin (A) {{ direction : input; capacitance : 0.001; }}
            pin (Z) {{ direction : output; function : "A"; timing () {{ related_pin : "A"; timing_sense : positive_unate; {D} }} }} }} }}"#
        );
        let libs = vec![vyges_sta::liberty::Library::read(&vyges_sta::liberty_parse::parse(&lib).unwrap()).unwrap()];
        let i = |k: usize, p: &str| Conn::Inst(k, p.into());
        let netlist = Netlist {
            insts: vec![inst("u1", "buf"), inst("u2", "buf")],
            ports: vec![("a".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![Conn::Port(0), i(0, "A")] }.into(),
                Net { name: "n1".into(), pins: vec![i(0, "Z"), i(1, "A")] }.into(),
                Net { name: "y".into(), pins: vec![i(1, "Z"), Conn::Port(1)] }.into(),
            ],
        };
        let mut g = Graph::build(&libs, &netlist).unwrap();
        let pi = vyges_sta::parasitics::PiElmore { c2: 1e-15, rpi: 10.0, c1: 2e-15, elmore: Vec::new() };
        let mut par = HashMap::new();
        par.insert("n1".to_string(), NetParasitics { node_names: vec!["u1/Z".into(), "u2/A".into()], pi: Some(pi), ..Default::default() });
        let u1z = g.vertices.iter().position(|v| v.name == "u1/Z").unwrap();
        assert_eq!(find_parasitic_pi(&g, u1z, 0, &par), Some((1e-15, 10.0, 2e-15)));
        g.sdc.net_wire_cap.insert("u1/Z".into(), [Some((1e-12, false)); 2]);
        assert_eq!(find_parasitic_pi(&g, u1z, 0, &par), None);
    }

    // Rule (sideLoadWeight): a side load on a violating endpoint's worst path (a critical path load
    // pin) moves its endpoint one for one and weighs 1; any other load weighs off_path_weight.
    #[test]
    fn a_side_load_weighs_one_on_a_worst_path_else_the_off_path_weight() {
        let crit: HashSet<String> = ["u1/A".to_string()].into_iter().collect();
        let o = FaninSideLoadOptions { critical_load_pins: &crit, off_path_weight: 0.25 };
        assert_eq!(side_load_weight("u1/A", &o), 1.0);
        assert_eq!(side_load_weight("u2/A", &o), 0.25);
    }

    // Rule (delayInf = fuzzyInf): a slack fuzzily at +-INF (1e30) is infinite and a side load
    // with it is skipped; a large finite slack is not.
    #[test]
    fn an_unconstrained_slack_is_infinite_to_the_side_load_collector() {
        assert!(delay_inf(1e30) && delay_inf(-1e30) && delay_inf(1e30 * (1.0 + 1e-7)));
        assert!(!delay_inf(1e20) && !delay_inf(0.0));
    }

    // Rule (failReasonName): the names the prepare_reject_<reason> events and the printer use.
    #[test]
    fn the_fail_reasons_print_as_failreasonname_does() {
        assert_eq!(FailReason::MissingPrevTimingArc.name(), "missing_prev_timing_arc");
        assert_eq!(FailReason::MissingSceneOrMinMax.name(), "missing_scene_or_minmax");
        assert_eq!(FailReason::MissingCurrentPortMap.name(), "missing_current_port_map");
    }

    // A fresh context (`EstimatorContext{}`): no side floor (-inf), margin and weight 1, the
    // opposite slack +inf.
    #[test]
    fn a_new_context_has_no_floor_and_no_opposite_slack() {
        let c = EstimatorContext::new(1);
        assert_eq!((c.side_slack_floor, c.side_floor_margin, c.fanin_side_loss_weight, c.opposite_rf_slack), (f32::NEG_INFINITY, 1.0, 1.0, f32::INFINITY));
        assert_eq!(c.delay_estimation_levels, 1);
    }

    // Rule (interpolateStaSlewBias): the bias is linear between the samples, the load clamped to
    // their range; an empty segment (x1 <= x0) gives the low sample's bias.
    #[test]
    fn slew_bias_interpolates_between_its_samples() {
        let s = |load_cap: f32, table_slew: f32, sta_slew: f32| SlewBiasSample { load_cap, table_slew, sta_slew };
        let m = SlewBiasModel { valid: true, table_worst_arc: Some((0, 0)), input_slew: 0.0, samples: [s(1.0, 1.0, 2.0), s(2.0, 1.0, 4.0), s(3.0, 1.0, 1.0)] };
        assert_eq!(interpolate_sta_slew_bias(&m, 0.0), 1.0);
        assert_eq!(interpolate_sta_slew_bias(&m, 1.5), 2.0);
        assert_eq!(interpolate_sta_slew_bias(&m, 2.5), 1.5);
        assert_eq!(interpolate_sta_slew_bias(&m, 9.0), 0.0);
        assert_eq!(interpolate(1.0, 5.0, 1.0, 9.0, 3.0), 5.0);
    }

    // oppositeRfSideLoss: min(slack, 0) - min(slack - delta, 0); zero for a delta at or below zero
    // or a slack that is not finite (either infinity).
    #[test]
    fn the_opposite_transition_loses_only_below_zero_slack() {
        assert_eq!(opposite_rf_side_loss(2.0, 3.0), 1.0);
        assert_eq!(opposite_rf_side_loss(-1.0, 3.0), 3.0);
        assert_eq!(opposite_rf_side_loss(5.0, 3.0), 0.0);
        assert_eq!(opposite_rf_side_loss(-1.0, 0.0), 0.0);
        assert_eq!(opposite_rf_side_loss(f32::INFINITY, 3.0), 0.0);
        assert_eq!(opposite_rf_side_loss(f32::NEG_INFINITY, 3.0), 0.0);
    }

    // chargeSideLoss: the paths and the weighted loss accumulate onto what is there; a worst
    // slack after below side_slack_floor - margin * max(arrival_impr, 0) rejects with
    // side_slack_floor_violation.
    #[test]
    fn charge_side_loss_accumulates_and_checks_the_floor() {
        let mut ctx = EstimatorContext::new(1);
        (ctx.fanin_side_loss_weight, ctx.side_slack_floor, ctx.side_floor_margin) = (3.0, -10.0, 1.0);
        let mut e = make_estimate(5.0, 3.0);
        e.side_paths_evaluated = 1;
        e.side_loss = 0.5;
        charge_side_loss(&ctx, 2, 1.0, -11.5, &mut e);
        assert_eq!((e.side_paths_evaluated, e.side_loss, e.legal, e.reason), (3, 3.5, true, FailReason::EstimateLegal));
        charge_side_loss(&ctx, 1, 0.0, -12.5, &mut e);
        assert_eq!((e.legal, e.reason), (false, FailReason::SideSlackFloorViolation));
    }

    // applyFaninSideLoss: off the selected arc, delay = delay_per_cap * cap change and slew = 1.4 *
    // delay; a delta at or below zero is skipped; else 0.3 * max(slew, 0) is added and each load
    // loses weight * (min(slack, 0) - min(slack - delta, 0)).
    #[test]
    fn fanin_side_loss_charges_each_side_load() {
        let mut ctx = EstimatorContext::new(1);
        ctx.fanin_side_loss_weight = 2.0;
        let load = |slack: f32, weight: f32| SideLoad { pin: "x/A".into(), slack, weight };
        ctx.fanin_side_nets = vec![
            FaninSideNet { input_port: "A".into(), input_cap: 0.0, delay_per_cap: 0.0, selected_arc: true, loads: vec![load(0.5, 1.0), load(-1.0, 0.5)] },
            FaninSideNet { input_port: "B".into(), input_cap: 0.0, delay_per_cap: 0.0, selected_arc: true, loads: vec![load(-5.0, 1.0)] },
        ];
        let cell = Cell::default();
        let mut e = make_estimate(5.0, 3.0);
        // Delay 1, slew 2: delta 1 + 0.3 * 2 = 1.6 per load.
        apply_fanin_side_loss(&ctx, &cell, 1.0, 2.0, &mut e);
        let loss = 1.0 * (0.0 - (0.5f32 - 1.6).min(0.0)) + 0.5 * (-1.0 - (-1.0f32 - 1.6)) + 1.0 * (-5.0 - (-5.0f32 - 1.6));
        assert_eq!(e.side_paths_evaluated, 3);
        assert!((e.side_loss - 2.0 * loss).abs() < 1e-6, "{} vs {}", e.side_loss, 2.0 * loss);
        // A delay at or below zero skips the net.
        let mut e = make_estimate(5.0, 3.0);
        apply_fanin_side_loss(&ctx, &cell, 0.0, 2.0, &mut e);
        assert_eq!((e.side_paths_evaluated, e.side_loss), (0, 0.0));
        // A negative slew adds nothing: delta 1 per load.
        let mut e = make_estimate(5.0, 3.0);
        apply_fanin_side_loss(&ctx, &cell, 1.0, -2.0, &mut e);
        let loss = 1.0 * (0.0 - (0.5f32 - 1.0).min(0.0)) + 0.5 * (-1.0 - (-1.0f32 - 1.0)) + 1.0 * (-5.0 - (-5.0f32 - 1.0));
        assert!((e.side_loss - 2.0 * loss).abs() < 1e-6, "{} vs {}", e.side_loss, 2.0 * loss);
        // Off the selected arc a port the candidate lacks is skipped.
        ctx.fanin_side_nets[0].selected_arc = false;
        ctx.fanin_side_nets[1].selected_arc = false;
        let mut e = make_estimate(5.0, 3.0);
        apply_fanin_side_loss(&ctx, &cell, 1.0, 2.0, &mut e);
        assert_eq!(e.side_paths_evaluated, 0);
        // Off the arc with the port: delay 0.5 * (2 - 1) = 0.5, slew 1.4 * 0.5 = 0.7, delta 0.5 +
        // 0.3 * 0.7 = 0.71; the selected-arc deltas are ignored. Only net A's two loads count.
        let port = vyges_sta::liberty::Port { name: "A".into(), capacitance: [[2.0; 2]; 2], ..Default::default() };
        let with_a = Cell { ports: vec![port], ..Cell::default() };
        (ctx.fanin_side_nets[0].input_cap, ctx.fanin_side_nets[0].delay_per_cap) = (1.0, 0.5);
        // Net B's port is not on the candidate: skipped, though a zero cap would read as growth.
        (ctx.fanin_side_nets[1].input_cap, ctx.fanin_side_nets[1].delay_per_cap) = (1.0, -0.5);
        let mut e = make_estimate(5.0, 3.0);
        apply_fanin_side_loss(&ctx, &with_a, 9.0, 9.0, &mut e);
        let d = 0.5f32 + 0.3 * (1.4 * 0.5);
        let loss = 1.0 * (0.0 - (0.5f32 - d).min(0.0)) + 0.5 * (-1.0 - (-1.0f32 - d));
        assert_eq!(e.side_paths_evaluated, 2);
        assert!((e.side_loss - 2.0 * loss).abs() < 1e-6, "{} vs {}", e.side_loss, 2.0 * loss);
    }

}
