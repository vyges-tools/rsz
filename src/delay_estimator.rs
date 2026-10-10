//! `DelayEstimator`: a target stage's (and its fanin/fanout neighbours') timing captured from the
//! timer once (`buildContext`), then each candidate cell scored on the Liberty table models alone
//! (`estimate`) — the arrival improvement over the stage window. The MT policies' (MT1) scoring.
//!
//! The functions are the reference's, in its order. The builder reads one scene's graph after a
//! timer update; the estimate reads only the libraries and the captured state.

use std::collections::HashMap;

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
#[derive(Debug, Clone)]
pub struct OutputSlewMergeArc {
    pub set: usize,
    pub arc: usize,
    pub input_slew: f32,
    pub current_model_slew: f32,
}

/// `SlewBiasSample`.
#[derive(Debug, Clone, Copy, Default)]
pub struct SlewBiasSample {
    pub load_cap: f32,
    pub table_slew: f32,
    pub sta_slew: f32,
}

/// `SlewBiasModel`: the timer's driver slew against the table's at three loads, for the arc whose
/// table slew is the worst at all three.
#[derive(Debug, Clone, Default)]
pub struct SlewBiasModel {
    pub valid: bool,
    /// `table_worst_arc` (set, arc) on the stage's current cell.
    pub table_worst_arc: Option<(usize, usize)>,
    pub input_slew: f32,
    pub samples: [SlewBiasSample; 3],
}

/// `DelayStageState`.
#[derive(Debug, Clone)]
pub struct DelayStageState {
    pub arc: SelectedArc,
    pub driver_pin: String,
    pub input_slew: f32,
    pub load_cap: f32,
    pub current_model_delay: f32,
    pub current_delay: f32,
    pub current_model_slew: f32,
    pub current_slew: f32,
    pub output_slew_merge_arcs: Vec<OutputSlewMergeArc>,
    pub sta_slew_bias: SlewBiasModel,
    pub path_index: usize,
}

/// `ArcDelayState`.
#[derive(Debug, Clone)]
pub struct ArcDelayState {
    pub path_stages: Vec<DelayStageState>,
    pub target_stage_index: usize,
    pub delay_estimation_levels: i32,
    pub current_total_delay: f32,
}

impl ArcDelayState {
    pub fn target(&self) -> &DelayStageState {
        &self.path_stages[self.target_stage_index]
    }
}

/// `DelayEstimate`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DelayEstimate {
    pub legal: bool,
    pub candidate_delay: f32,
    pub arrival_impr: f32,
    /// `reason == kEstimateNonImproving`: computed, not improving (its score is kept).
    pub non_improving: bool,
}

const REJECTED: DelayEstimate = DelayEstimate { legal: false, candidate_delay: 0.0, arrival_impr: 0.0, non_improving: false };

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
/// cells and dont_use list of `maxTargetInputCapDelta`.
pub struct Reader<'a, 'g> {
    pub libs: &'a Libs,
    pub g: &'g Graph<'g>,
    pub parasitics: &'a HashMap<String, NetParasitics>,
    pub scene: usize,
    pub equiv: &'a crate::sizing::EquivCells,
    pub dont_use: &'a std::collections::BTreeSet<String>,
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

    /// The arc set of a gate edge (the to vertex's cell).
    fn edge_set(&self, e: usize) -> Option<(usize, &ArcSet)> {
        let EdgeKind::Gate { set } = self.g.edges[e].kind else { return None };
        let vx = &self.g.vertices[self.g.edges[e].to];
        let cell = self.g.libs.get(vx.lib?)?.cells.get(vx.cell.as_deref()?)?;
        Some((set, &cell.arc_sets[set]))
    }

    /// `buildSelectedArc`: the path's arc into the driver (`selectedPathArc`: a gate arc with both
    /// edges), its ports on the current scene cell, and the first exactly matching arc over the
    /// sets between them.
    fn build_selected_arc(&self, path: &[PathPin], start: usize, index: usize) -> Option<SelectedArc> {
        if index < start || index >= path.len() {
            return None;
        }
        let v = path[index].vertex;
        let vx = &self.g.vertices[v];
        let (cell_name, out_port) = (vx.cell.as_deref()?, vx.port.as_deref()?);
        let current = self.cell(cell_name)?;
        let (e, k) = path[index].in_edge?;
        let (_, path_set) = self.edge_set(e)?;
        if current.port(&path_set.from).is_none() || current.port(out_port).is_none() || path_set.to != out_port {
            return None;
        }
        for (si, set) in arc_sets_for_ref_ports(current, path_set) {
            if let Some(a) = find_matching_arc(path_set, k, set) {
                return Some(SelectedArc { scene: self.scene, cell: current.name.clone(), set: si, arc: a });
            }
        }
        None
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

    /// `buildDelayStageState`.
    fn build_delay_stage_state(&self, path: &[PathPin], start: usize, index: usize) -> Option<DelayStageState> {
        let arc = self.build_selected_arc(path, start, index)?;
        let driver = path[index].vertex;
        let input_slew = self.input_slew(driver, &arc);
        let load_cap = self.g.load_cap(driver, self.parasitics);
        let (current_model_delay, current_slew_table) = lookup_arc_delay_and_slew(self.libs, &arc, input_slew, load_cap, &arc.cell, false, None)?;
        // `pathGraphCellDelayOrModelDelay`: the graph's delay of the path's arc.
        let (e, k) = path[index].in_edge?;
        let current_delay = self.g.delay[e][k][MAX];
        let merges = self.collect_output_slew_merge_arcs(driver, &arc, load_cap);
        let mut current_model_slew = current_slew_table;
        for m in &merges {
            if m.current_model_slew > current_model_slew {
                current_model_slew = m.current_model_slew;
            }
        }
        let out_rf = arc_set_of(self.cell(&arc.cell)?, arc.set).arcs[arc.arc].to_rf;
        Some(DelayStageState {
            driver_pin: self.g.vertices[driver].name.clone(),
            input_slew,
            load_cap,
            current_model_delay,
            current_delay,
            current_model_slew,
            current_slew: self.g.slew[driver][out_rf][MAX],
            output_slew_merge_arcs: merges,
            sta_slew_bias: SlewBiasModel::default(),
            path_index: index,
            arc,
        })
    }

    /// `buildDelayStageStateFromPath`: a stage with an instance (not a top-level port).
    fn build_delay_stage_state_from_path(&self, path: &[PathPin], start: usize, index: usize) -> Option<DelayStageState> {
        if index < start || index >= path.len() {
            return None;
        }
        if !matches!(self.g.vertices[path[index].vertex].conn, vyges_sta::netlist::Conn::Inst(..)) {
            return None;
        }
        self.build_delay_stage_state(path, start, index)
    }

    /// `collectPathStages`: the target, with up to `levels` valid stages before it (nearest first
    /// found, kept in path order) and after it.
    fn collect_path_stages(&self, path: &[PathPin], start: usize, target_index: usize, levels: i32, target: DelayStageState) -> ArcDelayState {
        if levels == 0 {
            return ArcDelayState { current_total_delay: target.current_delay, path_stages: vec![target], target_stage_index: 0, delay_estimation_levels: 0 };
        }
        let levels = levels as usize;
        let mut fanin = Vec::new();
        let mut i = target_index;
        while i > start && fanin.len() < levels {
            i -= 1;
            if let Some(s) = self.build_delay_stage_state_from_path(path, start, i) {
                fanin.push(s);
            }
        }
        fanin.reverse();
        let mut stages = fanin;
        let target_stage_index = stages.len();
        stages.push(target);
        let mut found = 0;
        let mut i = target_index + 1;
        while i < path.len() && found < levels {
            if let Some(s) = self.build_delay_stage_state_from_path(path, start, i) {
                stages.push(s);
                found += 1;
            }
            i += 1;
        }
        let mut total = 0.0f32;
        for s in &stages {
            total += s.current_delay;
        }
        ArcDelayState { path_stages: stages, target_stage_index, delay_estimation_levels: levels as i32, current_total_delay: total }
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
        let Some((c2, rpi, c1)) = self.g.reduced_pi(driver, out_rf, MAX, self.parasitics) else { return SlewBiasModel::default() };
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

    /// `DelayEstimator::buildContext`: the target stage (it must be valid), its window, and the
    /// fanin neighbour's slew bias.
    pub fn build_context(&self, path: &[PathPin], start: usize, index: usize, levels: i32, slew_bias: bool) -> Option<ArcDelayState> {
        let target = self.build_delay_stage_state(path, start, index)?;
        let mut context = self.collect_path_stages(path, start, index, levels.max(0), target);
        if slew_bias && context.target_stage_index > 0 {
            // `prepareFaninNeighborSlewBias`.
            let delta = self.max_target_input_cap_delta(context.target());
            if delta > 0.0 {
                let k = context.target_stage_index - 1;
                context.path_stages[k].sta_slew_bias = self.build_slew_bias_model(&context.path_stages[k], delta);
            }
        }
        Some(context)
    }
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

/// `makeEstimate`.
fn make_estimate(current_delay: f32, candidate_delay: f32) -> DelayEstimate {
    let arrival_impr = current_delay - candidate_delay;
    DelayEstimate { legal: arrival_impr > 0.0, candidate_delay, arrival_impr, non_improving: arrival_impr <= 0.0 }
}

/// `DelayEstimator::estimate` → `estimateWindow`: the window's stages from the fanin neighbour
/// on table-modelled with the candidate at the target (its input cap on the neighbour's load),
/// each stage's slew carried to the next; the improvement against the current total delay.
pub fn estimate(libs: &Libs, context: &ArcDelayState, candidate_cell: &str) -> DelayEstimate {
    let stages = &context.path_stages;
    let target_index = context.target_stage_index;
    let target = &stages[target_index];
    let Some(ref_cell) = libs.scene_cell(target.arc.scene, &target.arc.cell) else { return REJECTED };
    let input_port = arc_set_of(ref_cell, target.arc.set).from.clone();
    let Some(candidate_scene_cell) = libs.scene_cell(target.arc.scene, candidate_cell) else { return REJECTED };
    if candidate_scene_cell.port(&input_port).is_none() {
        return REJECTED;
    }
    let target_input_cap_delta = port_capacitance(candidate_scene_cell, &input_port) - port_capacitance(ref_cell, &input_port);
    let current_total_delay = context.current_total_delay;
    let mut candidate_total_delay = 0.0f32;
    let mut propagated: Option<f32> = None;
    for (i, stage) in stages.iter().enumerate() {
        if i + 1 < target_index {
            candidate_total_delay += stage.current_delay;
            continue;
        }
        let is_target = i == target_index;
        let is_fanin_neighbor = i + 1 == target_index;
        let cell = if is_target { candidate_cell } else { stage.arc.cell.as_str() };
        let load_cap = if is_fanin_neighbor { stage.load_cap + target_input_cap_delta } else { stage.load_cap };
        let input_slew = match propagated {
            Some(p) => estimate_receiver_input_slew(&stages[i - 1], stage, p),
            None => stage.input_slew,
        };
        let mut relaxed_flag = false;
        let Some((model_stage_delay, stage_slew)) = lookup_arc_delay_and_slew(libs, &stage.arc, input_slew, load_cap, cell, is_target, if is_target { Some(&mut relaxed_flag) } else { None }) else { return REJECTED };
        let stage_delay = stage.current_delay + (model_stage_delay - stage.current_model_delay);
        candidate_total_delay += stage_delay;
        propagated = Some(estimate_output_slew(libs, stage, cell, input_slew, load_cap, stage_slew, is_target, if is_target { Some(&mut relaxed_flag) } else { None }));
    }
    make_estimate(current_total_delay, candidate_total_delay)
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
            current_model_slew: 0.0,
            current_slew,
            output_slew_merge_arcs: Vec::new(),
            sta_slew_bias: SlewBiasModel::default(),
            path_index: 0,
        };
        assert_eq!(estimate_receiver_input_slew(&st(0.0, 2e-11), &st(3e-11, 0.0), 4e-11), 4e-11 * 3e-11 / 2e-11);
        assert_eq!(estimate_receiver_input_slew(&st(0.0, 0.0), &st(1e-11, 0.0), 1e-11), 1e-11 * 1e-11 / 1e-15);
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
}
