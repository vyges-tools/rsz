//! `SequentialTimingEstimator`: the setup and hold checks at a flip-flop target's constrained
//! inputs, captured from the timer once (`prepare`), then each candidate cell's effect on them
//! scored from the Liberty tables alone (`estimate`): the candidate's input capacitance moves the
//! delay and slew of the gate driving each input, and its check tables move the margin. A Q-path
//! size or VT swap pays for the slack the checks lose.
//!
//! The functions are the reference's, in its order. `prepare` reads every scene's graph and
//! search after a timer update; `estimate` reads only the libraries and the prepared state.

use std::collections::{BTreeSet, HashMap};

use vyges_sta::graph::{EdgeKind, Graph, NetParasitics};
use vyges_sta::liberty::{ArcSet, Cell, Model, Role};
use vyges_sta::search::{CheckEnd, Path, Search};
use vyges_sta::table::{AxisVar, Table};

use crate::delay_estimator::{arc_sets_for_ref_ports, find_matching_arc, gate_delay_and_slew, min_max_compare, ContextOptions, DelayStageState, PathPin, Reader};
use crate::preamble::Libs;

/// A side check's slack loss counts fully at or below zero slack and fades out over this much
/// slack (`kSideSlackWeightCutoff`).
pub const SIDE_SLACK_WEIGHT_CUTOFF: f64 = 10.0e-12;
/// The rounding allowance before a side check counts as a new violation
/// (`kSideSlackViolationTolerance`).
pub const SIDE_SLACK_VIOLATION_TOLERANCE: f64 = 0.001e-12;

/// A gate timing model: the arc (set, arc) of a scene cell.
#[derive(Debug, Clone, PartialEq)]
pub struct GateModelRef {
    pub scene: usize,
    pub cell: String,
    pub set: usize,
    pub arc: usize,
}

/// A check timing model: the check arc (set, arc) of a scene cell.
pub type CheckModelRef = GateModelRef;

/// `PreparedInputTiming`: one constrained input (the data or the clock side of a check) — its
/// slew, its connected input capacitance now and per candidate, and the gate driving it.
#[derive(Debug, Clone, PartialEq)]
pub struct PreparedInputTiming {
    pub mm: usize,
    pub driver: Option<DelayStageState>,
    /// The driver's selected arc's model, then each output-slew merge arc's.
    pub driver_gate_models: Vec<Option<GateModelRef>>,
    pub candidate_connected_input_caps: Vec<f32>,
    pub current_connected_input_cap: f32,
    pub slew: f32,
    pub delay_derate: f32,
    pub ideal_clock: bool,
}

impl PreparedInputTiming {
    fn new(mm: usize, slew: f32, candidates: usize) -> PreparedInputTiming {
        PreparedInputTiming { mm, driver: None, driver_gate_models: Vec::new(), candidate_connected_input_caps: vec![0.0; candidates], current_connected_input_cap: 0.0, slew, delay_derate: 1.0, ideal_clock: false }
    }
}

/// `PreparedSetupHoldCheck`.
#[derive(Debug, Clone, PartialEq)]
pub struct PreparedSetupHoldCheck {
    pub mm: usize,
    pub data: PreparedInputTiming,
    pub clock: PreparedInputTiming,
    pub clock_check_slew_input: Option<PreparedInputTiming>,
    pub candidate_check_models: Vec<Option<CheckModelRef>>,
    pub related_out_cap: f32,
    pub baseline_check_margin: f32,
    pub check_derate: f32,
    pub baseline_slack: f64,
    pub setup: bool,
    /// The end's data and clock pins and transitions and its scene (for the trace).
    pub data_pin: String,
    pub data_rf: usize,
    pub clock_pin: String,
    pub clock_rf: usize,
    pub scene: usize,
}

/// `SequentialTimingEstimator`: the prepared flip-flop timing (`PreparedFlipFlopTiming`).
#[derive(Debug, Clone, PartialEq)]
pub struct SequentialTimingEstimator {
    /// The candidate cells: the target's own, and its swappable and VT-equivalent flip-flops, by
    /// name (the reference keeps a pointer-ordered set; its order reaches no value — `estimate`
    /// finds a cell in it).
    pub candidates: Vec<String>,
    pub checks: Vec<PreparedSetupHoldCheck>,
    pub prepare_failure_reason: &'static str,
    /// The constrained data ports, in the order they were visited (for the trace).
    pub data_ports: Vec<String>,
}

/// `SequentialTimingEstimator::Result`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SequentialResult {
    pub legal: bool,
    pub weighted_side_slack_loss: f32,
    pub max_side_slack_loss: f32,
    pub reason: &'static str,
}

impl SequentialResult {
    fn reject(reason: &'static str) -> SequentialResult {
        SequentialResult { legal: false, weighted_side_slack_loss: 0.0, max_side_slack_loss: 0.0, reason }
    }
}

/// What `prepare` reads: the libraries, every scene's graph, search and parasitics, and what the
/// driver's `buildContext` needs (the equivalent cells, the dont_use list, pin ids, master pins).
pub struct Timer<'a, 's, 'g> {
    pub libs: &'a Libs,
    pub gs: &'a [Graph<'g>],
    pub ss: &'a [Search<'s, 'g>],
    pub parasitics: &'a dyn Fn(usize) -> &'a HashMap<String, NetParasitics>,
    pub equiv: &'a crate::sizing::EquivCells,
    pub dont_use: &'a BTreeSet<String>,
    pub clock_pins: &'a [BTreeSet<usize>],
    pub pin_id: &'a HashMap<String, u64>,
    pub master_pins: &'a HashMap<String, Vec<String>>,
}

/// `isFlipFlop`: a cell with sequentials, every one a register.
pub fn is_flip_flop(cell: Option<&Cell>) -> bool {
    cell.is_some_and(|c| !c.sequentials.is_empty() && c.sequentials.iter().all(|(_, is_register, _)| *is_register))
}

/// `matchingArc(reference, cell)`: over the cell's arc sets between the reference's ports, the
/// first exactly matching arc whose set has the same related output (this library model has none
/// on any set, so every set agrees).
pub fn matching_arc(ref_set: &ArcSet, ref_arc: usize, cell: &Cell) -> Option<(usize, usize)> {
    arc_sets_for_ref_ports(cell, ref_set).into_iter().find_map(|(si, set)| find_matching_arc(ref_set, ref_arc, set).map(|a| (si, a)))
}

/// `pinCapacitance(cell, port, scene, rf, min_max, pvt)`: the scene cell's port capacitance (no
/// PVT scaling: a scene's library is its PVT); NaN without the port, or when the value is not a
/// finite non-negative number. (This library model keeps no "capacitance exists" flag: a port
/// with none reads 0, which the reference reads as missing.)
pub fn pin_capacitance(libs: &Libs, cell: &str, port: &str, scene: usize, rf: usize, mm: usize) -> f32 {
    let Some(p) = libs.scene_cell(scene, cell).and_then(|c| c.port(port)) else { return f32::NAN };
    let cap = p.capacitance[rf][mm];
    if cap.is_finite() && cap >= 0.0 {
        cap
    } else {
        f32::NAN
    }
}

/// `tableDelay(model, pvt, slew, cap)`: the gate table's delay and slew; false without a model,
/// or with a delay or slew that is not finite, or a negative slew.
pub fn table_delay(libs: &Libs, model: Option<&GateModelRef>, slew: f32, cap: f32) -> Option<(f32, f32)> {
    let m = model?;
    let cell = libs.scene_cell(m.scene, &m.cell)?;
    let (delay, out_slew) = gate_delay_and_slew(cell, m.set, m.arc, slew, cap)?;
    (delay.is_finite() && out_slew.is_finite() && out_slew >= 0.0).then_some((delay, out_slew))
}

/// The pins of the net holding vertex `v`, in the net's order, each with whether it drives and
/// whether it loads the net (`Network::isDriver` / `isLoad`: a vertex of each kind).
fn net_pins(g: &Graph<'_>, v: usize) -> Vec<(String, bool, bool)> {
    let Some(net) = g.vertex_net[v] else { return Vec::new() };
    g.netlist.nets[net]
        .pins
        .iter()
        .map(|c| {
            let name = g.netlist.pin_name(c);
            let driver = g.vertices.iter().any(|x| x.name == name && x.is_driver);
            let load = g.vertices.iter().any(|x| x.name == name && !x.is_driver);
            (name, driver, load)
        })
        .collect()
}

/// `prepareInputCaps`: over the input's net, every load pin of the target instance — not just
/// the selected D or CK — its capacitance on the current cell and on each candidate, at the
/// path's transition; usable with one driver on the net and a load port found.
#[allow(clippy::too_many_arguments)]
fn prepare_input_caps(t: &Timer<'_, '_, '_>, scene: usize, inst: &str, cell: &str, v: usize, path: &Path, candidates: &[String], context: &mut PreparedInputTiming) -> bool {
    let g = &t.gs[scene];
    let rf = path.tag.rf;
    let mut drivers = 0;
    let mut found_load_port = false;
    for (pin, is_driver, is_load) in net_pins(g, v) {
        drivers += i32::from(is_driver);
        let Some((pin_inst, port)) = pin.rsplit_once('/') else { continue };
        if pin_inst != inst || !is_load {
            continue;
        }
        found_load_port = true;
        context.current_connected_input_cap += pin_capacitance(t.libs, cell, port, scene, rf, context.mm);
        for (k, candidate) in candidates.iter().enumerate() {
            context.candidate_connected_input_caps[k] += pin_capacitance(t.libs, candidate, port, scene, rf, context.mm);
        }
    }
    drivers == 1 && found_load_port && context.current_connected_input_cap.is_finite()
}

/// `PathExpanded(path)`: the path's pins root first, and its start index (the pin after a
/// clock-to-output arc; a latch's D -> Q keeps the latch's D path and stops there).
fn expanded(search: &Search<'_, '_>, g: &Graph<'_>, v: usize, path: &Path) -> (Vec<PathPin>, usize) {
    let mut chain = vec![(v, *path)];
    let mut p = *path;
    while let Some((pv, prev)) = search.prev_path(&p) {
        chain.push((pv, *prev));
        p = *prev;
    }
    let role = |e: usize| match g.edges[e].kind {
        EdgeKind::Gate { set } => Some(g.arc_set(e, set).role),
        EdgeKind::Wire => None,
    };
    let mut start_from_end = None;
    for (i, (_, p)) in chain.iter().enumerate() {
        let Some(prev) = p.prev else { continue };
        match role(prev.edge) {
            Some(Role::RegClkToQ | Role::LatchEnToQ) => {
                start_from_end = Some(i);
                break;
            }
            Some(Role::LatchDtoQ) => {
                start_from_end = Some(i);
                chain.truncate(i + 2);
                break;
            }
            _ => {}
        }
    }
    let n = chain.len();
    let start = n - 1 - start_from_end.unwrap_or(n - 1);
    chain.reverse();
    (chain.iter().map(|(v, p)| PathPin { vertex: *v, in_edge: p.prev.map(|x| (x.edge, x.arc)) }).collect(), start)
}

/// `prepareInputDriver`: the gate driving the input — `DelayEstimator::buildContext` on the
/// driver path at the input's min/max with no window (levels 0) — its table models at the
/// input transition's load and its cell-delay derate, so `estimate` can replay a load change.
fn prepare_input_driver(t: &Timer<'_, '_, '_>, scene: usize, path: &Path, dv: usize, driver_path: &Path, context: &mut PreparedInputTiming) -> bool {
    let (g, search) = (&t.gs[scene], &t.ss[scene]);
    let mm = context.mm;
    let (pins, start) = expanded(search, g, dv, driver_path);
    let no_slack = |_: usize, _: Option<usize>| f32::INFINITY;
    let reader = Reader { libs: t.libs, g, parasitics: (t.parasitics)(scene), scene, equiv: t.equiv, dont_use: t.dont_use, slack: &no_slack, clock_pins: &t.clock_pins[scene], pin_id: t.pin_id, master_pins: t.master_pins, mm };
    let options = ContextOptions { delay_levels: 0, use_sta_slew_bias: false, latch_data_path: None, fanin_side_loads: None, dual_rf_eval: false };
    let Ok(prepared) = reader.build_context(&pins, start, pins.len() - 1, &options) else { return false };
    let mut stage = prepared.target().clone();
    stage.load_cap = g.load_cap_rf(dv, (t.parasitics)(scene), path.tag.rf, mm);
    let model = |set: usize, arc: usize| Some(GateModelRef { scene, cell: stage.arc.cell.clone(), set, arc });
    context.driver_gate_models.push(model(stage.arc.set, stage.arc.arc));
    let Some((delay, slew)) = table_delay(t.libs, context.driver_gate_models[0].as_ref(), stage.input_slew, stage.load_cap) else { return false };
    stage.current_model_delay = delay;
    stage.current_model_slew = slew;
    for k in 0..stage.output_slew_merge_arcs.len() {
        let (set, arc, input_slew) = (stage.output_slew_merge_arcs[k].set, stage.output_slew_merge_arcs[k].arc, stage.output_slew_merge_arcs[k].input_slew);
        let m = model(set, arc);
        let Some((_, merge_slew)) = table_delay(t.libs, m.as_ref(), input_slew, stage.load_cap) else { return false };
        stage.output_slew_merge_arcs[k].current_model_slew = merge_slew;
        context.driver_gate_models.push(m);
        if min_max_compare(mm, merge_slew, stage.current_model_slew) {
            stage.current_model_slew = merge_slew;
        }
    }
    // `timingDerateInstance(driver input pin, cell_delay, clk or data, the arc's input
    // transition, min_max)`: the derate of the driver path's own arc.
    let (e, k) = pins[pins.len() - 1].in_edge.expect("buildContext found the driver's arc");
    context.delay_derate = g.timing_derate(e, k, driver_path.tag.is_clock, mm);
    let finite = stage.load_cap.is_finite() && context.delay_derate.is_finite();
    context.driver = Some(stage);
    finite
}

/// `prepareInput`: the input's check slew; an ideal clock input needs nothing more; else the
/// connected input caps, then — through a wire from another instance's gate — that driver.
#[allow(clippy::too_many_arguments)]
fn prepare_input(t: &Timer<'_, '_, '_>, scene: usize, inst: &str, cell: &str, v: usize, path: &Path, check_slew: f32, mm: usize, candidates: &[String]) -> Option<PreparedInputTiming> {
    let (g, search) = (&t.gs[scene], &t.ss[scene]);
    let mut context = PreparedInputTiming::new(mm, check_slew, candidates.len());
    if !check_slew.is_finite() || check_slew < 0.0 {
        return None;
    }
    context.ideal_clock = path.tag.is_clock && g.ideal_clock.contains(&v);
    if context.ideal_clock {
        return Some(context);
    }
    if !prepare_input_caps(t, scene, inst, cell, v, path, candidates, &mut context) {
        return None;
    }
    // A primary input without a gate model is usable only for an unchanged load.
    let (Some(prev), Some((dv, driver_path))) = (path.prev, search.prev_path(path)) else { return Some(context) };
    if !matches!(g.edges[prev.edge].kind, EdgeKind::Wire) {
        return Some(context);
    }
    let driver_inst = g.vertices[dv].name.rsplit_once('/').map(|(i, _)| i);
    if matches!(g.vertices[dv].conn, vyges_sta::netlist::Conn::Inst(..)) && driver_inst == Some(inst) {
        return None;
    }
    if driver_path.prev.is_none() {
        return Some(context);
    }
    let driver_path = *driver_path;
    prepare_input_driver(t, scene, path, dv, &driver_path, &mut context).then_some(context)
}

/// `CheckTableModel::checkDelay(pvt, clock_slew, data_slew, related_out_cap)`: the check table at
/// the related (clock) and constrained (data) pin transitions and the related output's load.
fn check_delay(table: &Table, clock_slew: f32, data_slew: f32, related_out_cap: f32) -> f32 {
    let pick = |a: usize| match table.axes.get(a).map(|x| x.var) {
        Some(AxisVar::RelatedPinTransition) => clock_slew,
        Some(AxisVar::ConstrainedPinTransition) => data_slew,
        Some(AxisVar::RelatedOutTotalOutputNetCapacitance) => related_out_cap,
        _ => 0.0,
    };
    table.find_value(pick(0), pick(1), pick(2))
}

/// `checkMargin(model, context, data_slew, clock_slew)`: the check model's margin; false without
/// a model or with a margin that is not finite.
fn check_margin(libs: &Libs, model: Option<&CheckModelRef>, related_out_cap: f32, data_slew: f32, clock_slew: f32) -> Option<f32> {
    let m = model?;
    let cell = libs.scene_cell(m.scene, &m.cell)?;
    let Model::Check(table) = &cell.arc_sets[m.set].arcs[m.arc].model else { return None };
    let margin = check_delay(table, clock_slew, data_slew, related_out_cap);
    margin.is_finite().then_some(margin)
}

/// `prepareSetupHoldCheck`: one setup or hold end at the instance — its check arc on the scene
/// cell and on each candidate, its slack, derate and margin now, and its data and clock inputs.
#[allow(clippy::too_many_arguments)]
fn prepare_setup_hold_check(t: &Timer<'_, '_, '_>, scene: usize, inst: &str, cell: &str, d_v: usize, end: &CheckEnd, candidates: &[String]) -> Option<PreparedSetupHoldCheck> {
    let (g, search) = (&t.gs[scene], &t.ss[scene]);
    let data = search.paths[d_v][end.path];
    let clock = search.paths[end.tgt_vertex][end.tgt_path];
    let mm = data.tag.mm;
    let scene_cell = t.libs.scene_cell(scene, cell)?;
    let EdgeKind::Gate { set } = g.edges[end.edge].kind else { return None };
    let (cs, ca) = matching_arc(g.arc_set(end.edge, set), end.arc, scene_cell)?;
    let check_set = &scene_cell.arc_sets[cs];
    let setup = check_set.role == Role::Setup;
    let baseline_slack = f64::from(end.slack(&data));
    // `timingDerateInstance(clock pin, cell_check, data, the data transition, min_max)`.
    let check_derate = g.timing_derate(end.edge, end.arc, false, mm);
    let candidate_check_models = candidates
        .iter()
        .map(|c| {
            let cand = t.libs.scene_cell(scene, c)?;
            matching_arc(check_set, ca, cand).map(|(set, arc)| CheckModelRef { scene, cell: c.clone(), set, arc })
        })
        .collect();
    // The check's clock slew: an ideal clock's, else the graph's, at `checkClkSlewMinMax` (the
    // opposite min/max under OCV, as the graph times its checks).
    let clock_rf = check_set.arcs[ca].from_rf;
    let tgt_v = end.tgt_vertex;
    let clock_slew = if g.ideal_clock.contains(&tgt_v) { g.ideal_clock_slew.get(&tgt_v).map_or(0.0, |s| s[clock_rf][1 - mm]) } else { g.slew[tgt_v][clock_rf][1 - mm] };
    let data_slew = g.slew[d_v][data.tag.rf][mm];
    let clock_mm = clock.tag.mm;
    let waveform_mm = 1 - mm;
    let data_input = prepare_input(t, scene, inst, cell, d_v, &data, data_slew, mm, candidates)?;
    let clock_input = prepare_input(t, scene, inst, cell, tgt_v, &clock, clock_slew, clock_mm, candidates)?;
    // The check-slew analysis reuses the clock's topology at the waveform's min/max; its delay
    // delta never replaces the capture arrival's.
    let clock_check_slew_input = if waveform_mm != clock_mm && !clock_input.ideal_clock { Some(prepare_input(t, scene, inst, cell, tgt_v, &clock, clock_slew, waveform_mm, candidates)?) } else { None };
    // No set of this library model has a related output pin: the related output load is 0.
    let related_out_cap = 0.0f32;
    let mut check = PreparedSetupHoldCheck {
        mm,
        data: data_input,
        clock: clock_input,
        clock_check_slew_input,
        candidate_check_models,
        related_out_cap,
        baseline_check_margin: 0.0,
        check_derate,
        baseline_slack,
        setup,
        data_pin: g.vertices[d_v].name.clone(),
        data_rf: data.tag.rf,
        clock_pin: g.vertices[tgt_v].name.clone(),
        clock_rf: clock.tag.rf,
        scene,
    };
    if !(baseline_slack.is_finite() && check_derate.is_finite() && related_out_cap.is_finite()) {
        return None;
    }
    let own = CheckModelRef { scene, cell: scene_cell.name.clone(), set: cs, arc: ca };
    check.baseline_check_margin = check_margin(t.libs, Some(&own), related_out_cap, data_slew, clock_slew)?;
    Some(check)
}

/// `SetupHoldCheckCollector::visit` over `VisitPathEnds::visitPathEnds(vertex)`: every scene's
/// check ends at the data pin, scene by scene (the reference's path order at the vertex), each
/// setup or hold end prepared; the first that cannot be is the failure and ends the visit.
fn collect_setup_hold_checks(t: &Timer<'_, '_, '_>, inst: &str, cell: &str, pin: &str, candidates: &[String], checks: &mut Vec<PreparedSetupHoldCheck>) -> Result<(), &'static str> {
    for (scene, (g, search)) in t.gs.iter().zip(t.ss).enumerate() {
        let Some(d_v) = g.vertices.iter().position(|x| x.name == pin && !x.is_driver) else { continue };
        for end in search.check_ends(d_v) {
            let EdgeKind::Gate { set } = g.edges[end.edge].kind else { continue };
            if !matches!(g.arc_set(end.edge, set).role, Role::Setup | Role::Hold) {
                continue;
            }
            match prepare_setup_hold_check(t, scene, inst, cell, d_v, &end, candidates) {
                Some(check) => checks.push(check),
                None => return Err("sequential_missing_check_context"),
            }
        }
    }
    Ok(())
}

/// `estimateInputTimingForCap`: the input's arrival delta and slew with a candidate's connected
/// input cap — unchanged for an ideal clock or an equal cap; else the driver's table delay at the
/// load changed by the difference (the measured wire contribution kept), its merged slew moving
/// the input slew by the table's change.
pub fn estimate_input_timing_for_cap(libs: &Libs, context: &PreparedInputTiming, candidate_connected_input_cap: f32) -> Option<(f64, f32)> {
    let slew = context.slew;
    if context.ideal_clock {
        return Some((0.0, slew));
    }
    if !candidate_connected_input_cap.is_finite() {
        return None;
    }
    let delta_cap = candidate_connected_input_cap - context.current_connected_input_cap;
    if delta_cap == 0.0 {
        return Some((0.0, slew));
    }
    let driver = context.driver.as_ref()?;
    let load_cap = driver.load_cap + delta_cap;
    if !load_cap.is_finite() || load_cap < 0.0 {
        return None;
    }
    let (delay, mut model_slew) = table_delay(libs, context.driver_gate_models.first().and_then(Option::as_ref), driver.input_slew, load_cap)?;
    for (k, arc) in driver.output_slew_merge_arcs.iter().enumerate() {
        let (_, merge_slew) = table_delay(libs, context.driver_gate_models.get(k + 1).and_then(Option::as_ref), arc.input_slew, load_cap)?;
        if min_max_compare(context.mm, merge_slew, model_slew) {
            model_slew = merge_slew;
        }
    }
    let arrival_delta = (f64::from(delay) - f64::from(driver.current_model_delay)) * f64::from(context.delay_derate);
    let slew = (context.slew + model_slew - driver.current_model_slew).max(0.0);
    (arrival_delta.is_finite() && slew.is_finite()).then_some((arrival_delta, slew))
}

impl SequentialTimingEstimator {
    /// `SequentialTimingEstimator::prepare`: none for a target that is not a flip-flop; else the
    /// estimator, its failure reason set when the target has no clock-to-Q arc to its output, a
    /// constrained input has no pin, a check cannot be prepared, or the cell constrains no input.
    /// The candidates are the cell itself and its swappable and VT-equivalent flip-flops.
    pub fn prepare(t: &Timer<'_, '_, '_>, inst: &str, cell: &str, output_port: &str, swappable: &[String], vt_equiv: &[String]) -> Option<SequentialTimingEstimator> {
        let link = t.libs.link_cell(cell);
        if !is_flip_flop(link) {
            return None;
        }
        let link = link.expect("a flip-flop has a cell");
        let mut out = SequentialTimingEstimator { candidates: Vec::new(), checks: Vec::new(), prepare_failure_reason: "", data_ports: Vec::new() };
        let has_clock_to_q = link.arc_sets.iter().any(|s| s.to == output_port && s.role == Role::RegClkToQ);
        // No POCV here: the timer is scalar.
        if !has_clock_to_q {
            out.prepare_failure_reason = "sequential_unsupported_target_context";
            return Some(out);
        }
        let mut candidates: BTreeSet<String> = BTreeSet::from([cell.to_string()]);
        candidates.extend(swappable.iter().chain(vt_equiv).filter(|c| is_flip_flop(t.libs.link_cell(c))).cloned());
        out.candidates = candidates.into_iter().collect();
        // Liberty roles identify the constrained inputs, scan data included. The reference's set
        // is pointer-ordered; this one keeps the cell's port order.
        for port in &link.ports {
            if link.arc_sets.iter().any(|s| matches!(s.role, Role::Setup | Role::Hold) && s.to == port.name) {
                out.data_ports.push(port.name.clone());
            }
        }
        for port in out.data_ports.clone() {
            let pin = format!("{inst}/{port}");
            if !t.gs.first().is_some_and(|g| g.vertices.iter().any(|x| x.name == pin && !x.is_driver)) {
                out.prepare_failure_reason = "sequential_missing_data_pin";
                break;
            }
            if let Err(reason) = collect_setup_hold_checks(t, inst, cell, &pin, &out.candidates, &mut out.checks) {
                out.prepare_failure_reason = reason;
                break;
            }
        }
        if out.data_ports.is_empty() && out.prepare_failure_reason.is_empty() {
            out.prepare_failure_reason = "sequential_missing_check_context";
        }
        Some(out)
    }

    /// `SequentialTimingEstimator::estimate`: a candidate swap's effect on each check — the data
    /// and clock arrival deltas and slews at its input caps, its check margin at those slews —
    /// as a slack change; the largest weighted and unweighted losses, and illegal when a check
    /// would fall below the lesser of zero and its slack now (setup = clock − data − margin, hold
    /// = data − clock − margin).
    pub fn estimate(&self, libs: &Libs, candidate_cell: &str) -> SequentialResult {
        if !self.prepare_failure_reason.is_empty() {
            return SequentialResult::reject(self.prepare_failure_reason);
        }
        let Some(index) = self.candidates.iter().position(|c| c == candidate_cell) else { return SequentialResult::reject("sequential_missing_candidate_cell") };
        let mut result = SequentialResult { legal: true, weighted_side_slack_loss: 0.0, max_side_slack_loss: 0.0, reason: "sequential_estimate_legal" };
        for check in &self.checks {
            let Some((data_delta, data_slew)) = estimate_input_timing_for_cap(libs, &check.data, check.data.candidate_connected_input_caps[index]) else { return SequentialResult::reject("sequential_missing_candidate_model") };
            let Some((clock_delta, mut clock_slew)) = estimate_input_timing_for_cap(libs, &check.clock, check.clock.candidate_connected_input_caps[index]) else { return SequentialResult::reject("sequential_missing_candidate_model") };
            if let Some(input) = &check.clock_check_slew_input {
                let Some((_, slew)) = estimate_input_timing_for_cap(libs, input, input.candidate_connected_input_caps[index]) else { return SequentialResult::reject("sequential_missing_candidate_model") };
                clock_slew = slew;
            }
            let Some(candidate_check_margin) = check_margin(libs, check.candidate_check_models[index].as_ref(), check.related_out_cap, data_slew, clock_slew) else { return SequentialResult::reject("sequential_missing_candidate_model") };
            let margin_delta = (f64::from(candidate_check_margin) - f64::from(check.baseline_check_margin)) * f64::from(check.check_derate);
            let slack_delta = (if check.setup { clock_delta - data_delta } else { data_delta - clock_delta }) - margin_delta;
            let after = check.baseline_slack + slack_delta;
            let loss = (-slack_delta).max(0.0);
            let weight = (1.0 - check.baseline_slack.min(after) / SIDE_SLACK_WEIGHT_CUTOFF).clamp(0.0, 1.0);
            result.weighted_side_slack_loss = result.weighted_side_slack_loss.max((weight * loss) as f32);
            result.max_side_slack_loss = result.max_side_slack_loss.max(loss as f32);
            if after < 0.0f64.min(check.baseline_slack) - SIDE_SLACK_VIOLATION_TOLERANCE {
                result.legal = false;
                result.reason = if check.setup { "sequential_setup_violation" } else { "sequential_hold_violation" };
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_sta::liberty::MAX;

    fn input(slew: f32) -> PreparedInputTiming {
        PreparedInputTiming::new(MAX, slew, 2)
    }

    /// Rules (`estimateInputTimingForCap`): an ideal clock input, or a candidate whose connected
    /// cap equals the current one, keeps the input's slew and moves no arrival — no driver model
    /// is read; a different cap with no driver stage cannot be scored; a cap that is not finite
    /// cannot be scored even for an equal-looking change.
    #[test]
    fn an_unchanged_load_moves_nothing_and_a_changed_one_needs_a_driver() {
        let libs = Libs { libs: Vec::new(), scenes: vec!["default".into()], scene_libs: Vec::new(), link_scene: None };
        let mut ideal = input(3e-12);
        ideal.ideal_clock = true;
        assert_eq!(estimate_input_timing_for_cap(&libs, &ideal, f32::NAN), Some((0.0, 3e-12)));
        let mut data = input(4e-12);
        data.current_connected_input_cap = 1e-15;
        assert_eq!(estimate_input_timing_for_cap(&libs, &data, 1e-15), Some((0.0, 4e-12)));
        assert_eq!(estimate_input_timing_for_cap(&libs, &data, 2e-15), None);
        assert_eq!(estimate_input_timing_for_cap(&libs, &data, f32::NAN), None);
    }

    fn estimator(checks: Vec<PreparedSetupHoldCheck>) -> SequentialTimingEstimator {
        SequentialTimingEstimator { candidates: vec!["A".into(), "B".into()], checks, prepare_failure_reason: "", data_ports: vec!["D".into()] }
    }

    /// Rules (`estimate`): a prepare failure is the reason for every candidate; a cell that is not
    /// a candidate is `sequential_missing_candidate_cell`; with no checks every candidate is legal
    /// with no loss (`sequential_estimate_legal`).
    #[test]
    fn with_no_checks_every_candidate_is_legal_and_failures_name_their_reason() {
        let libs = Libs { libs: Vec::new(), scenes: vec!["default".into()], scene_libs: Vec::new(), link_scene: None };
        let e = estimator(Vec::new());
        assert_eq!(e.estimate(&libs, "B"), SequentialResult { legal: true, weighted_side_slack_loss: 0.0, max_side_slack_loss: 0.0, reason: "sequential_estimate_legal" });
        assert_eq!(e.estimate(&libs, "C"), SequentialResult::reject("sequential_missing_candidate_cell"));
        let mut failed = estimator(Vec::new());
        failed.prepare_failure_reason = "sequential_missing_check_context";
        assert_eq!(failed.estimate(&libs, "A"), SequentialResult::reject("sequential_missing_check_context"));
    }

    /// Rule (`estimate`): a check whose candidate has no check model cannot be scored
    /// (`sequential_missing_candidate_model`), even with its inputs unchanged.
    #[test]
    fn a_candidate_without_a_check_model_is_unscorable() {
        let libs = Libs { libs: Vec::new(), scenes: vec!["default".into()], scene_libs: Vec::new(), link_scene: None };
        let mut clock = input(0.0);
        clock.ideal_clock = true;
        let check = PreparedSetupHoldCheck { mm: MAX, data: input(1e-12), clock, clock_check_slew_input: None, candidate_check_models: vec![None, None], related_out_cap: 0.0, baseline_check_margin: 0.0, check_derate: 1.0, baseline_slack: 0.0, setup: true, data_pin: "r/D".into(), data_rf: 0, clock_pin: "r/CK".into(), clock_rf: 0, scene: 0 };
        assert_eq!(estimator(vec![check]).estimate(&libs, "A"), SequentialResult::reject("sequential_missing_candidate_model"));
    }

    /// Rule (`estimate`'s weighting): the loss counts fully at or below zero slack (before or
    /// after) and fades linearly to nothing over 10 ps of slack; legality fails only below the
    /// lesser of zero and the slack now, less 0.001 ps.
    #[test]
    fn the_side_weight_fades_over_ten_picoseconds_of_slack() {
        let weight = |baseline: f64, after: f64| (1.0 - baseline.min(after) / SIDE_SLACK_WEIGHT_CUTOFF).clamp(0.0, 1.0);
        assert_eq!(weight(-1e-12, 5e-12), 1.0);
        assert_eq!(weight(20e-12, 15e-12), 0.0);
        assert!((weight(8e-12, 5e-12) - 0.5).abs() < 1e-12);
        let violates = |baseline: f64, after: f64| after < 0.0f64.min(baseline) - SIDE_SLACK_VIOLATION_TOLERANCE;
        assert!(!violates(5e-12, 0.0));
        assert!(violates(5e-12, -0.002e-12));
        assert!(!violates(-3e-12, -3.0005e-12));
        assert!(violates(-3e-12, -3.002e-12));
    }
}
