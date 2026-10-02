// SPDX-License-Identifier: Apache-2.0
//! `repair_timing -setup`'s legacy repair — the LEGACY phase, SizeUp moves only — from the progress
//! table's row 0 to the summary lines, after the preamble [`crate::repair_timing`] prints.
//!
//! One function per stage, named after the reference's and in its call order:
//! - [`repair_setup`] — `Optimizer::run`: the LEGACY phase, then the final report (LAST_GASP is
//!   skipped by the caller's `-skip_last_gasp`; CRIT_VT_SWAP finds no VT cells);
//! - [`Repair::iterate`] — `SetupLegacyPolicy::iterate`: `initializeMainRepair`, then
//!   `runMainRepairLoop`;
//! - [`Repair::initialize_main_repair`] — the violating endpoints, row 0, the violation range;
//! - [`Repair::run_main_repair_loop`] — per violating endpoint in slack order:
//!   `beginEndpointRepair`, `repairEndpoint`; then the forced progress row;
//! - [`Repair::repair_endpoint`] — the pass loop: the margin test, `repairPath`, re-timing, the
//!   journal kept on an improvement and restored after too many worse passes or a pass with no move;
//! - [`Repair::repair_path`] — `SetupLegacyBase::repairPath`: the path's drivers ranked by
//!   load-dependent delay, each tried in turn until the pass budget is spent;
//! - [`size_up`] — `SizeUpGenerator::generate`: the first weaker-first equivalent cell that drives
//!   no worse and estimates a faster stage, if no fanin net's max capacitance suffers;
//! - [`Repair::finalize_and_report`] — `OptimizationPolicy::finalizeAndReport`.
//!
//! The timer is rebuilt over the design after each edit and timed in full ([`Snapshot`]): the
//! reference times incrementally, which reaches the same values. The journal is a stack of the
//! cells swapped (`MoveCommitter`'s, over odb's eco journal): a restore swaps them back, last first.

use std::collections::{BTreeSet, HashMap};

use vyges_sta::fuzzy;
use vyges_sta::graph::{EdgeKind, Graph, SdcEnv};
use vyges_sta::liberty::{Cell, Model, Role, MAX};
use vyges_sta::search::Search;

use crate::design::Design;
use crate::preamble::Libs;
use crate::repair_timing::{collect_violating, delay_as_string, progress_header, progress_row, startpoint_tns, timing_points, total_negative_slack, worst_slack, Args, Point, Row};
use crate::sizing::Sizing;
use crate::Stop;

/// `decreasing_slack_max_passes_`, `print_interval_`, `opto_small_interval_`,
/// `opto_large_interval_`, `inc_fix_rate_threshold_`.
const DECREASING_SLACK_MAX_PASSES: i64 = 50;
const PRINT_INTERVAL: i64 = 10;
const OPTO_SMALL_INTERVAL: i64 = 100;
const OPTO_LARGE_INTERVAL: i64 = 1000;
const INC_FIX_RATE_THRESHOLD: f32 = 0.0001;
/// `MinMax::min()->initValue()`.
const INF: f32 = 1e30;
/// The LEGACY phase is phase 0: its marker is the first of `"*+^&@!-="`.
const PHASE: &str = "LEGACY*";

/// What the repair reads besides the design.
pub struct Ctx<'a> {
    pub libs: &'a Libs,
    pub sizing: &'a Sizing<'a>,
    pub env: &'a SdcEnv,
    pub master_pins: &'a HashMap<String, Vec<String>>,
    pub ssdc: &'a vyges_sta::sdc::Sdc,
    pub clock_sources: &'a [String],
    /// The clock is ideal (no `set_propagated_clock`).
    pub ideal_clock: bool,
    /// `-setup_margin`, seconds.
    pub margin: f32,
    pub time_scale: f32,
}

/// A driver's `checkCapacitance`: its load, limit and slack, and whether it has a limit at all.
#[derive(Debug, Clone, Copy)]
struct CapCheck {
    cap: f32,
    max_cap: f32,
    slack: f32,
    limited: bool,
}

/// One pin of an expanded path, with what the repair reads about it there.
#[derive(Debug, Clone)]
struct Stage {
    pin: String,
    is_driver: bool,
    top_port: bool,
    inst: Option<String>,
    cell: Option<String>,
    port: Option<String>,
    /// `arcDelay(prev_edge, prev_arc) − prev_arc->intrinsicDelay()`, for a pin reached by a gate arc.
    load_delay: Option<f32>,
    /// The from-port of the arc the path took into this pin.
    in_port: Option<String>,
    /// Wire edges out of the pin.
    fanout: usize,
    /// `GraphDelayCalc::loadCap` on a driver.
    load_cap: f32,
    /// Each input pin of the instance: its port, and its net's drivers' capacitance checks.
    fanin_caps: Vec<(String, Vec<CapCheck>)>,
}

/// `PathExpanded`: the path from its root, and `startIndex`.
#[derive(Debug, Clone)]
struct PathView {
    stages: Vec<Stage>,
    start: usize,
}

/// The timer over the design as it is now: every endpoint's and startpoint's slack, and the worst
/// path of the endpoints asked for (and of the worst endpoint).
pub struct Snapshot {
    ends: Vec<Point>,
    starts: Vec<Point>,
    paths: HashMap<String, PathView>,
}

impl Snapshot {
    /// `Sta::slack(vertex, max)` of an endpoint.
    fn slack(&self, pin: &str) -> f32 {
        self.ends.iter().find(|p| p.pin == pin).map_or(INF, |p| p.slack)
    }

    /// `Sta::worstSlack(max, wns, worst_vertex)`.
    fn worst(&self) -> (f32, Option<String>) {
        let (wns, at) = worst_slack(&self.ends);
        (wns, at.map(|p| p.pin.clone()))
    }

    fn tns(&self) -> f32 {
        total_negative_slack(&self.ends)
    }
}

fn timer_stop(e: String) -> Stop {
    Stop::refused("RSZ-TIMER", e)
}

/// The arc set of a gate edge.
fn edge_arc_set<'g>(g: &'g Graph<'_>, e: usize) -> Option<&'g vyges_sta::liberty::ArcSet> {
    let EdgeKind::Gate { set } = g.edges[e].kind else { return None };
    let vx = &g.vertices[g.edges[e].to];
    Some(&g.libs[vx.lib?].cells[vx.cell.as_deref()?].arc_sets[set])
}

/// `TimingArc::intrinsicDelay`: the gate delay at slew 0 and load 0.
fn arc_intrinsic(model: &Model) -> f32 {
    match model {
        Model::Gate(m) => m.gate_delay(0.0, 0.0).0,
        _ => 0.0,
    }
}

/// The timer rebuilt over the design and timed: `updateParasitics` has run, so the parasitics
/// are the estimator's current ones. `want`: the endpoints whose worst path the repair will read.
fn snapshot(ctx: &Ctx<'_>, design: &dyn Design, want: &[String]) -> Result<Snapshot, Stop> {
    let netlist = design.netlist();
    let mut g = crate::repair_design::timer_graph(ctx.libs, 0, netlist, ctx.env, ctx.master_pins)?;
    let clocks = crate::timing::clock_pins(&g, ctx.clock_sources);
    if ctx.ideal_clock {
        g.ideal_clock = clocks.iter().copied().collect();
    }
    g.find_delays(design.parasitics(0), None).map_err(timer_stop)?;
    let mut search = Search::in_graph_order(&g, ctx.ssdc);
    search.find_arrivals().map_err(timer_stop)?;
    search.find_requireds().map_err(timer_stop)?;
    let (ends, starts) = timing_points(&g, &search, ctx.ssdc, ctx.libs, &clocks).map_err(timer_stop)?;
    let ideal = if ctx.ideal_clock { clocks } else { BTreeSet::new() };
    let mut names: Vec<String> = want.to_vec();
    if let (_, Some(w)) = worst_slack(&ends) {
        names.push(w.pin.clone());
    }
    let mut paths = HashMap::new();
    for n in names {
        if paths.contains_key(&n) {
            continue;
        }
        let Some(v) = g.vertices.iter().position(|x| x.name == n) else { continue };
        if let Some(view) = expand(&g, &search, design, v, &ideal)? {
            paths.insert(n, view);
        }
    }
    Ok(Snapshot { ends, starts, paths })
}

/// `Sta::vertexWorstSlackPath(end, max)`, then `PathExpanded`: the max path with the fuzzily
/// least slack (the first, in tag order), walked back through its prev paths to the root.
/// `startIndex`: the pin reached by the clock-to-output arc nearest the end, else the root.
fn expand(g: &Graph<'_>, search: &Search<'_, '_>, design: &dyn Design, end: usize, ideal: &BTreeSet<usize>) -> Result<Option<PathView>, Stop> {
    let mut worst = None;
    let mut min_slack = INF;
    for p in search.paths[end].iter().filter(|p| p.tag.mm == MAX) {
        let slack = p.required - p.arrival;
        if fuzzy::less(slack, min_slack) {
            min_slack = slack;
            worst = Some(*p);
        }
    }
    let Some(mut p) = worst else { return Ok(None) };
    // From the end back to the root.
    let mut chain = vec![(end, p)];
    while let Some(prev) = p.prev {
        p = *search.paths[prev.vertex].iter().find(|q| q.tag == prev.tag).ok_or_else(|| timer_stop(format!("{}: a path's prev path is not at its vertex", g.vertices[prev.vertex].name)))?;
        chain.push((prev.vertex, p));
    }
    let mut start_from_end = None;
    for (i, (_, p)) in chain.iter().enumerate() {
        let Some(prev) = p.prev else { continue };
        match edge_arc_set(g, prev.edge).map(|s| s.role) {
            Some(Role::RegClkToQ | Role::LatchEnToQ) => {
                start_from_end = Some(i);
                break;
            }
            Some(Role::LatchDtoQ) => return Err(Stop::refused("RSZ-LATCH", "a path through a latch D->Q arc: not modelled".into())),
            _ => {}
        }
    }
    let n = chain.len();
    let start = n - 1 - start_from_end.unwrap_or(n - 1);
    chain.reverse();
    let sc = crate::timing::Scenes::new(std::slice::from_ref(g));
    let parasitics = std::slice::from_ref(design.parasitics(0));
    let mut stages = Vec::with_capacity(n);
    for (i, (v, p)) in chain.iter().enumerate() {
        let vx = &g.vertices[*v];
        let inst_index = match &vx.conn {
            vyges_sta::netlist::Conn::Inst(k, _) => Some(*k),
            vyges_sta::netlist::Conn::Port(_) => None,
        };
        let top_port = vx.lib.is_none();
        let mut st = Stage {
            pin: vx.name.clone(),
            is_driver: vx.is_driver,
            top_port,
            inst: inst_index.map(|k| g.netlist.insts[k].0.clone()),
            cell: vx.cell.clone(),
            port: vx.port.clone(),
            load_delay: None,
            in_port: None,
            fanout: g.out_edges[*v].iter().filter(|&&e| matches!(g.edges[e].kind, EdgeKind::Wire)).count(),
            load_cap: 0.0,
            fanin_caps: Vec::new(),
        };
        if let Some(prev) = p.prev {
            if let Some(set) = edge_arc_set(g, prev.edge) {
                st.in_port = Some(set.from.clone());
                st.load_delay = Some(g.delay[prev.edge][prev.arc][MAX] - arc_intrinsic(&set.arcs[prev.arc].model));
            }
        }
        if i > 0 && vx.is_driver && !top_port {
            st.load_cap = g.load_cap(*v, design.parasitics(0));
            // Each input pin of the instance, its net's drivers' capacitance checks.
            for (u, ux) in g.vertices.iter().enumerate() {
                let same = matches!(&ux.conn, vyges_sta::netlist::Conn::Inst(k, _) if Some(*k) == inst_index);
                if !same || ux.is_driver {
                    continue;
                }
                let checks = crate::timing::net_drivers(g, u)
                    .into_iter()
                    .map(|d| {
                        let (cap, max_cap, slack, limited, _) = crate::timing::check_capacitance(&sc, d, parasitics, ideal);
                        CapCheck { cap, max_cap, slack, limited }
                    })
                    .collect();
                st.fanin_caps.push((ux.port.clone().unwrap_or_default(), checks));
            }
        }
        stages.push(st);
    }
    Ok(Some(PathView { stages, start }))
}

/// `SetupLegacyBase::rankPathDrivers`: from the start index, every driver that is not a top-level
/// port, with its load-dependent delay; by delay, larger first, a tie by the LATER index first.
fn rank_path_drivers(view: &PathView) -> Vec<(usize, f32)> {
    let mut load_delays = Vec::new();
    for (i, st) in view.stages.iter().enumerate().skip(view.start) {
        if i > 0 && st.is_driver && !st.top_port {
            load_delays.push((i, st.load_delay.unwrap_or(0.0)));
        }
    }
    load_delays.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal).then(b.0.cmp(&a.0)));
    load_delays
}

/// `LibertyPort::capacitance()`: the largest of its rise/fall, min/max values.
fn port_capacitance(cell: &Cell, port: &str) -> Option<f32> {
    let p = cell.port(port)?;
    Some(p.capacitance.iter().flatten().fold(f32::MIN, |a, &b| a.max(b)))
}

/// `inputPinCapacitance(pin, cell, max)`: the larger of the port's rise and fall max values, 0
/// when the cell has no such port.
fn input_pin_capacitance(cell: &Cell, port: &str) -> f32 {
    cell.port(port).map_or(0.0, |p| p.capacitance[0][MAX].max(p.capacitance[1][MAX]).max(0.0))
}

/// `LibertyPort::intrinsicDelay`: over the non-check arcs into the port, the largest intrinsic
/// delay fuzzily above 0; 0 when none is.
fn port_intrinsic_delay(cell: &Cell, port: &str) -> f32 {
    let mut max_delay = -INF;
    let mut found = false;
    for set in cell.arc_sets.iter().filter(|s| s.to == port && !s.role.is_timing_check()) {
        for arc in &set.arcs {
            let d = arc_intrinsic(&arc.model);
            if fuzzy::greater(d, 0.0) {
                if fuzzy::greater(d, max_delay) {
                    max_delay = d;
                }
                found = true;
            }
        }
    }
    if found { max_delay } else { 0.0 }
}

/// `Resizer::gateDelay(drvr_port, load_cap)`: over the non-check arcs into the port, each at the
/// target slew of its input transition and the lumped load, the larger delay per output
/// transition (from −INF); then the larger of rise and fall.
fn gate_delay(sizing: &Sizing<'_>, cell: &Cell, port: &str, load_cap: f32) -> f32 {
    let mut delays = [-INF; 2];
    for set in cell.arc_sets.iter().filter(|s| s.to == port && !s.role.is_timing_check()) {
        for arc in &set.arcs {
            if let Model::Gate(m) = &arc.model {
                let (delay, _) = m.gate_delay(sizing.tgt_slews[arc.from_rf], load_cap);
                delays[arc.to_rf] = delays[arc.to_rf].max(delay);
            }
        }
    }
    delays[0].max(delays[1])
}

/// `MoveGenerator::weakerCellFirst`: cells with the driver port first, by (drive resistance,
/// intrinsic delay) LARGER first; cells without it by name. (The reference's sort is not stable;
/// equal keys keep their order here.)
fn weaker_cell_first(libs: &Libs, a: &str, b: &str, port: &str) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    let (ca, cb) = (libs.link_cell(a), libs.link_cell(b));
    let pa = ca.filter(|c| c.port(port).is_some());
    let pb = cb.filter(|c| c.port(port).is_some());
    match (pa, pb) {
        (Some(_), None) => Ordering::Less,
        (None, Some(_)) => Ordering::Greater,
        (None, None) => a.cmp(b),
        (Some(x), Some(y)) => {
            let kx = (x.drive_resistance(port), port_intrinsic_delay(x, port));
            let ky = (y.drive_resistance(port), port_intrinsic_delay(y, port));
            ky.partial_cmp(&kx).unwrap_or(Ordering::Equal)
        }
    }
}

/// `SizeUpGenerator::upsizeCell`: the swappable cells weaker first; the first whose port drives no
/// worse and whose stage delay — `gateDelay` at the load plus the previous driver's resistance
/// times its input capacitance — is smaller than the current cell's.
fn upsize_cell(sizing: &Sizing<'_>, in_port: &str, cell: &str, drvr_port: &str, load_cap: f32, prev_drive: f32) -> Result<Option<String>, Stop> {
    let libs = sizing.libs;
    let mut swappable = sizing.swappable_cells(cell)?;
    if swappable.is_empty() {
        return Ok(None);
    }
    swappable.sort_by(|a, b| weaker_cell_first(libs, a, b, drvr_port));
    let Some(c) = libs.link_cell(cell) else { return Ok(None) };
    let (Some(_), Some(in_cap)) = (c.port(drvr_port), port_capacitance(c, in_port)) else { return Ok(None) };
    let drive_r = c.drive_resistance(drvr_port);
    let delay = gate_delay(sizing, c, drvr_port, load_cap) + prev_drive * in_cap;
    for name in swappable {
        let Some(s) = libs.link_cell(&name) else { continue };
        let (Some(_), Some(s_in_cap)) = (s.port(drvr_port), port_capacitance(s, in_port)) else { continue };
        let s_drive_r = s.drive_resistance(drvr_port);
        let s_delay = gate_delay(sizing, s, drvr_port, load_cap) + prev_drive * s_in_cap;
        if s_drive_r <= drive_r && s_delay < delay {
            return Ok(Some(name));
        }
    }
    Ok(None)
}

/// `Resizer::replacementPreservesMaxCap`: each input pin whose capacitance grows must leave every
/// driver of its net within its limit (`checkMaxCapOK`: a driver already over it may not grow).
fn replacement_preserves_max_cap(libs: &Libs, cell: &str, replacement: &str, fanin_caps: &[(String, Vec<CapCheck>)]) -> bool {
    let (Some(cur), Some(rep)) = (libs.link_cell(cell), libs.link_cell(replacement)) else { return true };
    for (port, checks) in fanin_caps {
        let cap_delta = input_pin_capacitance(rep, port) - input_pin_capacitance(cur, port);
        if cap_delta <= 0.0 {
            continue;
        }
        for c in checks {
            if c.max_cap > 0.0 && c.limited {
                let new_cap = c.cap + cap_delta;
                let ok = if c.slack < 0.0 { new_cap <= c.cap } else { new_cap <= c.max_cap };
                if !ok {
                    return false;
                }
            }
        }
    }
    true
}

/// A size-up the generator proposes: the instance, its cell now, the replacement.
struct SizeUp {
    inst: String,
    from: String,
    to: String,
}

/// `SizeUpGenerator::generate` on the path's driver at `index`: `resolveDriverContext` (an
/// instance pin, not dont_touch, a logic standard cell), `loadStageContext` (the driver's load,
/// the input port the path entered by, the previous driver's drive resistance), `upsizeCell`, and
/// the max-capacitance guard.
fn size_up(ctx: &Ctx<'_>, design: &dyn Design, view: &PathView, index: usize) -> Result<Option<SizeUp>, Stop> {
    let st = &view.stages[index];
    let (Some(inst), Some(cell), Some(port)) = (&st.inst, &st.cell, &st.port) else { return Ok(None) };
    if design.net_info().dont_touch_insts.contains(inst) || !ctx.sizing.masters.get(cell).is_some_and(|m| m.logic_std) {
        return Ok(None);
    }
    let Some(in_port) = &st.in_port else { return Ok(None) };
    // The driver path's prev path's prev path: the previous driver (a top-level port has no
    // liberty port, so no drive).
    let prev_drive = match index.checked_sub(2).map(|k| &view.stages[k]) {
        Some(Stage { cell: Some(c), port: Some(p), .. }) => ctx.libs.link_cell(c).map_or(0.0, |c| c.drive_resistance(p)),
        _ => 0.0,
    };
    let Some(to) = upsize_cell(ctx.sizing, in_port, cell, port, st.load_cap, prev_drive)? else { return Ok(None) };
    if !replacement_preserves_max_cap(ctx.libs, cell, &to, &st.fanin_caps) {
        return Ok(None);
    }
    Ok(Some(SizeUp { inst: inst.clone(), from: cell.clone(), to }))
}

/// `MoveCommitter`'s SizeUp accounting and its journal: per open level, the cells swapped.
#[derive(Default)]
struct Committer {
    committed: i64,
    pending: i64,
    levels: Vec<Vec<(String, String)>>,
}

/// `SetupLegacyBase::EndpointRepairState`.
struct EndpointState {
    end: String,
    end_slack: f32,
    worst_slack: f32,
    worst_vertex: Option<String>,
    prev_end_slack: f32,
    prev_worst_slack: f32,
    pass: i64,
    decreasing_slack_passes: i64,
    force_single_repair: bool,
    journal_open: bool,
}

/// What the repair printed: the report lines (the table and the summary), and every line in order
/// with the `repair_setup` (to level 3) and `size_up_move` debug lines between them.
#[derive(Debug, Clone, Default)]
pub struct Outcome {
    pub lines: Vec<String>,
    pub trace: Vec<String>,
    /// SizeUp moves kept.
    pub resized: i64,
}

struct Repair<'c, 'd> {
    ctx: &'c Ctx<'c>,
    args: &'c Args,
    design: &'d mut dyn SetupDesign,
    timing: Snapshot,
    committer: Committer,
    out: Outcome,
    initial_design_area: f64,
    // MainRepairState
    opto_iteration: i64,
    initial_tns: f32,
    prev_tns: f32,
    num_viols: i64,
    max_end_count: i64,
    end_index: i64,
    fix_rate_threshold: f32,
    prev_termination: bool,
    two_cons_terminations: bool,
    // RepairSetupContext
    min_viol: f32,
    max_viol: f32,
    /// The collector's violating endpoint count as its last `init` found it (the Viol column).
    collector_violating: usize,
}

/// The design as setup repair edits it: also `computeDesignArea`.
pub trait SetupDesign: Design {
    /// `Resizer::computeDesignArea`: over the block's instances, each non-filler master's area
    /// (m², core-autoplaceable masters only).
    fn design_area(&self) -> f64;
    fn as_design(&self) -> &dyn Design;
}

/// `Optimizer::run` for the LEGACY phase: the phase, then the final report.
pub fn repair_setup(ctx: &Ctx<'_>, design: &mut dyn SetupDesign, args: &Args) -> Result<Outcome, Stop> {
    if args.max_utilization.is_some() {
        return Err(Stop::refused("RSZ-ABSENT", "repair_timing -max_utilization: not modelled".into()));
    }
    let timing = snapshot(ctx, design.as_design(), &[])?;
    // RepairSetupContext: the area and TNS before any move.
    let initial_design_area = design.design_area();
    let initial_tns = timing.tns();
    let mut r = Repair {
        ctx,
        args,
        design,
        timing,
        committer: Committer::default(),
        out: Outcome::default(),
        initial_design_area,
        opto_iteration: 0,
        initial_tns,
        prev_tns: initial_tns,
        num_viols: 0,
        max_end_count: 0,
        end_index: 0,
        fix_rate_threshold: INC_FIX_RATE_THRESHOLD,
        prev_termination: false,
        two_cons_terminations: false,
        min_viol: 0.0,
        max_viol: 0.0,
        collector_violating: 0,
    };
    r.iterate()?;
    r.finalize_and_report()?;
    r.out.resized = r.committer.committed;
    Ok(r.out)
}

impl Repair<'_, '_> {
    fn report(&mut self, line: String) {
        self.out.trace.push(line.clone());
        self.out.lines.push(line);
    }

    fn debug(&mut self, group: &str, line: String) {
        self.out.trace.push(format!("[DEBUG RSZ-{group}] {line}"));
    }

    fn ds(&self, v: f32, digits: usize) -> String {
        delay_as_string(v, digits, self.ctx.time_scale)
    }

    /// The timer again over the design (`updateParasitics`, `findRequireds`), with the worst path
    /// of `want` ready.
    fn retime(&mut self, want: &[String]) -> Result<(), Stop> {
        self.design.update_parasitics().map_err(timer_stop)?;
        self.timing = snapshot(self.ctx, self.design.as_design(), want)?;
        Ok(())
    }

    /// `SetupLegacyPolicy::iterate`.
    fn iterate(&mut self) -> Result<(), Stop> {
        let violating_ends = self.initialize_main_repair()?;
        if !violating_ends.is_empty() {
            self.run_main_repair_loop(&violating_ends)?;
        }
        Ok(())
    }

    /// `initializeMainRepair`: the violating endpoints, row 0, and the violation range.
    fn initialize_main_repair(&mut self) -> Result<Vec<Point>, Stop> {
        let violating_ends = collect_violating(&self.timing.ends, self.ctx.margin);
        self.collector_violating = violating_ends.len();
        // A diagnostic: the endpoints in the order the loop visits them, each slack's bits.
        if let Ok(path) = std::env::var("VYGES_RSZ_ENDS_DUMP") {
            let text: String = violating_ends.iter().map(|p| format!("{} {:e}\n", p.pin, p.slack)).collect();
            let _ = std::fs::write(path, text);
        }
        if violating_ends.is_empty() {
            self.debug("repair_setup", format!("{PHASE} Phase: No violating endpoints, exiting"));
            return Ok(violating_ends);
        }
        self.debug("repair_setup", format!("{PHASE} Phase: {} violating endpoints found", violating_ends.len()));
        self.max_end_count = ((violating_ends.len() as f64 * self.args.repair_tns_end_percent) as i64).max(1);
        self.initial_tns = self.timing.tns();
        self.prev_tns = self.initial_tns;
        self.num_viols = violating_ends.len() as i64;
        self.fix_rate_threshold = INC_FIX_RATE_THRESHOLD;
        self.print_progress(self.opto_iteration, false)?;
        self.min_viol = -violating_ends.last().expect("non-empty").slack;
        self.max_viol = -violating_ends[0].slack;
        Ok(violating_ends)
    }

    /// `runMainRepairLoop`.
    fn run_main_repair_loop(&mut self, violating_ends: &[Point]) -> Result<(), Stop> {
        for end in violating_ends {
            if self.args.max_iterations > 0 && self.opto_iteration >= self.args.max_iterations {
                break;
            }
            let Some(mut es) = self.begin_endpoint_repair(&end.pin) else { break };
            self.repair_endpoint(&mut es)?;
            if self.args.verbose || self.opto_iteration == 1 {
                self.print_progress(self.opto_iteration, true)?;
            }
            if self.two_cons_terminations {
                self.debug("repair_setup", format!("{PHASE} Phase: Exiting due to no TNS progress for two opto cycles"));
                break;
            }
        }
        self.print_progress(self.opto_iteration, true)?;
        let (wns, _) = self.timing.worst();
        let tns = self.timing.tns();
        let line = format!("{PHASE} Phase complete. WNS: {}, TNS: {}", self.ds(wns, 3), self.ds(tns, 1));
        self.debug("repair_setup", line);
        Ok(())
    }

    /// `beginEndpointRepair` → `beginJournaledEndpointSearch`.
    fn begin_endpoint_repair(&mut self, end: &str) -> Option<EndpointState> {
        let mut es = EndpointState {
            end: end.to_string(),
            end_slack: 0.0,
            worst_slack: 0.0,
            worst_vertex: None,
            prev_end_slack: 0.0,
            prev_worst_slack: 0.0,
            pass: 1,
            decreasing_slack_passes: 0,
            force_single_repair: false,
            journal_open: false,
        };
        self.refresh_endpoint_slacks(&mut es);
        self.end_index += 1;
        if self.end_index > self.max_end_count {
            return None;
        }
        self.begin_journal();
        es.journal_open = true;
        let line = format!(
            "{PHASE} Phase: Doing endpoint {} ({}/{}) WNS = {}, endpoint slack = {}, TNS = {}",
            es.end,
            self.end_index,
            self.max_end_count,
            self.ds(es.worst_slack, 3),
            self.ds(es.end_slack, 3),
            self.ds(self.prev_tns, 1)
        );
        self.debug("repair_setup", line);
        es.prev_end_slack = es.end_slack;
        es.prev_worst_slack = es.worst_slack;
        Some(es)
    }

    /// `refreshEndpointSlacks`.
    fn refresh_endpoint_slacks(&self, es: &mut EndpointState) {
        es.end_slack = self.timing.slack(&es.end);
        let (wns, at) = self.timing.worst();
        es.worst_slack = wns;
        es.worst_vertex = at;
    }

    /// `SetupLegacyPolicy::repairEndpoint`: the pass loop.
    fn repair_endpoint(&mut self, es: &mut EndpointState) -> Result<(), Stop> {
        let margin = self.ctx.margin;
        while es.pass <= self.args.max_passes {
            self.opto_iteration += 1;
            if self.args.verbose || self.opto_iteration == 1 {
                self.print_progress(self.opto_iteration, false)?;
            }
            if self.terminate_progress() {
                if self.prev_termination {
                    self.two_cons_terminations = true;
                } else {
                    self.prev_termination = true;
                }
                let line = format!("{PHASE} Phase: Restoring best slack; endpoint slack = {}, WNS = {}", self.ds(es.prev_end_slack, 3), self.ds(es.prev_worst_slack, 3));
                self.debug("repair_setup", line);
                self.restore_endpoint_state(es)?;
                break;
            }
            if self.opto_iteration % OPTO_SMALL_INTERVAL == 0 {
                self.prev_termination = false;
            }
            // fuzzyGreaterEqual.
            if !fuzzy::less(es.end_slack, margin) {
                self.num_viols -= 1;
                let line = format!("{PHASE} Phase: Endpoint slack {} meets slack margin {}, done", self.ds(es.worst_slack, 3), self.ds(margin, 3));
                self.debug("repair_setup", line);
                self.finish_endpoint_search(es)?;
                break;
            }
            let prev_tns_local = self.timing.tns();
            let end = es.end.clone();
            let changed = self.repair_path(&end, es.end_slack, es.force_single_repair)?;
            if !changed {
                if es.pass != 1 {
                    self.debug("repair_setup", format!("{PHASE} Phase: No change after {} decreasing slack passes.", es.decreasing_slack_passes));
                    let line = format!("{PHASE} Phase: Restoring best slack; endpoint slack = {}, WNS = {}", self.ds(es.prev_end_slack, 3), self.ds(es.prev_worst_slack, 3));
                    self.debug("repair_setup", line);
                }
                self.debug("repair_setup", format!("{PHASE} Phase: No change possible for endpoint {} ", es.end));
                self.finish_endpoint_search(es)?;
                break;
            }
            // updateParasitics, findRequireds; the next pass reads this endpoint's path, or the
            // worst endpoint's.
            self.retime(std::slice::from_ref(&es.end))?;
            self.refresh_endpoint_slacks(es);
            let better = path_improved(es.end_slack, es.worst_slack, es.prev_end_slack, es.prev_worst_slack);
            let new_tns = self.timing.tns();
            let line = format!(
                "{PHASE} Phase: {} after changes: WNS ({} -> {}) TNS ({} -> {}) Endpoint slack ({} -> {})",
                if better { "Improved" } else { "Worsened" },
                self.ds(es.prev_worst_slack, 3),
                self.ds(es.worst_slack, 3),
                self.ds(prev_tns_local, 1),
                self.ds(new_tns, 1),
                self.ds(es.prev_end_slack, 3),
                self.ds(es.end_slack, 3)
            );
            self.debug("repair_setup", line);
            if better {
                if !fuzzy::less(es.end_slack, margin) {
                    self.num_viols -= 1;
                }
                es.prev_end_slack = es.end_slack;
                es.prev_worst_slack = es.worst_slack;
                es.decreasing_slack_passes = 0;
                self.save_improved_checkpoint(es);
            } else {
                es.force_single_repair = true;
                es.decreasing_slack_passes += 1;
                if es.decreasing_slack_passes > DECREASING_SLACK_MAX_PASSES {
                    let line = format!("{PHASE} Phase: Endpoint {} stuck after {} non-improving passes (limit {DECREASING_SLACK_MAX_PASSES})", es.end, es.decreasing_slack_passes);
                    self.debug("repair_setup", line);
                    let line = format!("{PHASE} Phase: Restoring best slack; endpoint slack = {}, WNS = {}", self.ds(es.prev_end_slack, 3), self.ds(es.prev_worst_slack, 3));
                    self.debug("repair_setup", line);
                    self.restore_endpoint_state(es)?;
                    break;
                }
                self.debug("repair_setup", format!("{PHASE} Phase: Allowing decreasing slack for {}/{DECREASING_SLACK_MAX_PASSES} passes", es.decreasing_slack_passes));
            }
            // overMaxArea: -max_utilization is refused, so never.
            if self.end_index == 1 {
                if let Some(w) = es.worst_vertex.clone() {
                    es.end = w;
                }
            }
            es.pass += 1;
            if self.args.max_iterations > 0 && self.opto_iteration >= self.args.max_iterations {
                self.accept_endpoint_state(es);
                break;
            }
        }
        self.accept_endpoint_state(es);
        Ok(())
    }

    /// `terminateProgress`: every `opto_small_interval_` iterations the incremental fix rate,
    /// which past iteration 1000 must reach a threshold doubled every `opto_large_interval_`.
    fn terminate_progress(&mut self) -> bool {
        let iteration = self.opto_iteration;
        if iteration % OPTO_LARGE_INTERVAL == 0 {
            self.fix_rate_threshold *= 2.0;
        }
        if iteration % OPTO_SMALL_INTERVAL == 0 {
            let curr_tns = self.timing.tns();
            let inc_fix_rate = (self.prev_tns - curr_tns) / self.initial_tns;
            self.prev_tns = curr_tns;
            if iteration > 1000 && inc_fix_rate < self.fix_rate_threshold {
                let line = format!(
                    "{PHASE} Phase: Exiting at iteration {iteration} because incr fix rate {:.2}% is < {:.2}% [endpt {}/{}]",
                    inc_fix_rate * 100.0,
                    self.fix_rate_threshold * 100.0,
                    self.end_index,
                    self.max_end_count
                );
                self.debug("repair_setup", line);
                return true;
            }
        }
        false
    }

    /// `repairBudget`: one repair, or more for a slack nearer the worst violation.
    fn repair_budget(&self, path_slack: f32, force_single_repair: bool) -> i64 {
        let mut repairs_per_pass = 1i64;
        if self.max_viol - self.min_viol != 0.0 {
            // round() of a float expression, added to an int.
            repairs_per_pass += ((self.args.max_repairs_per_pass - 1) as f32 * (-path_slack - self.min_viol) / (self.max_viol - self.min_viol)).round() as i64;
        }
        if force_single_repair { 1 } else { repairs_per_pass }
    }

    /// `SetupLegacyBase::repairPath`.
    fn repair_path(&mut self, end: &str, path_slack: f32, force_single_repair: bool) -> Result<bool, Stop> {
        if !self.timing.paths.contains_key(end) {
            self.timing = snapshot(self.ctx, self.design.as_design(), &[end.to_string()])?;
        }
        let Some(view) = self.timing.paths.get(end).cloned() else { return Ok(false) };
        if view.stages.len() <= 1 {
            return Ok(false);
        }
        let repairs_per_pass = self.repair_budget(path_slack, force_single_repair);
        if repairs_per_pass > 1 {
            return Err(Stop::refused("RSZ-ABSENT", format!("{repairs_per_pass} repairs in one pass: a target after the first move would read a timer the move changed, which is not modelled")));
        }
        let ranked = rank_path_drivers(&view);
        let line = format!("Path slack: {}, repairs: {repairs_per_pass}, ranked_targets: {}", self.ds(path_slack, 3), ranked.len());
        self.debug("repair_setup", line);
        let mut changed = 0i64;
        for (index, _) in ranked {
            if changed >= repairs_per_pass {
                break;
            }
            self.try_repair_path_target(&view, index, &mut changed)?;
        }
        Ok(changed > 0)
    }

    /// `tryRepairPathTarget` → `logRepairTarget`, `tryRepairTarget` over the move sequence
    /// (SizeUp alone): `tryCandidateSequence` applies the candidate and counts it.
    fn try_repair_path_target(&mut self, view: &PathView, index: usize, changed: &mut i64) -> Result<bool, Stop> {
        let st = &view.stages[index];
        let line = format!("{} {} fanout = {} drvr_index = {index}", st.pin, st.cell.as_deref().unwrap_or("none"), st.fanout);
        self.debug("repair_setup", line);
        self.debug("repair_setup", format!("Considering SizeUpMove for {}", st.pin));
        let Some(m) = size_up(self.ctx, self.design.as_design(), view, index)? else { return Ok(false) };
        // SizeUpCandidate::apply → Resizer::replaceCell.
        self.design.swap_master(&m.inst, &m.to).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        self.debug("size_up_move", format!("ACCEPT SizeUpMove {}: {} -> {}", st.pin, m.from, m.to));
        self.committer.pending += 1;
        if let Some(level) = self.committer.levels.last_mut() {
            level.push((m.inst, m.from));
        }
        *changed += 1;
        Ok(true)
    }

    /// `MoveCommitter::beginJournal`.
    fn begin_journal(&mut self) {
        self.committer.levels.push(Vec::new());
    }

    /// `MoveCommitter::commitJournal` (its `updateParasiticsAndTiming` finds nothing invalid: the
    /// pass re-timed after its edits).
    fn commit_journal(&mut self) {
        let Some(top) = self.committer.levels.pop() else { return };
        match self.committer.levels.last_mut() {
            Some(parent) => parent.extend(top),
            None => {
                self.committer.committed += self.committer.pending;
                self.committer.pending = 0;
            }
        }
    }

    /// `MoveCommitter::restoreJournal`: this level's swaps undone, last first, and its moves
    /// unrecorded; then the timer again when anything was undone.
    fn restore_journal(&mut self, want: &[String]) -> Result<(), Stop> {
        let Some(top) = self.committer.levels.pop() else { return Ok(()) };
        for (inst, cell) in top.iter().rev() {
            self.design.swap_master(inst, cell).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        }
        self.committer.pending -= top.len() as i64;
        if !top.is_empty() {
            self.retime(want)?;
        }
        Ok(())
    }

    /// `acceptEndpointState`.
    fn accept_endpoint_state(&mut self, es: &mut EndpointState) {
        if es.journal_open {
            self.commit_journal();
            es.journal_open = false;
        }
    }

    /// `restoreEndpointState`.
    fn restore_endpoint_state(&mut self, es: &mut EndpointState) -> Result<(), Stop> {
        if es.journal_open {
            self.restore_journal(std::slice::from_ref(&es.end))?;
            es.journal_open = false;
        }
        Ok(())
    }

    /// `finishEndpointSearch`: the first pass's state is kept, a later one's restored.
    fn finish_endpoint_search(&mut self, es: &mut EndpointState) -> Result<(), Stop> {
        if es.pass == 1 {
            self.accept_endpoint_state(es);
            Ok(())
        } else {
            self.restore_endpoint_state(es)
        }
    }

    /// `saveImprovedCheckpoint`.
    fn save_improved_checkpoint(&mut self, es: &mut EndpointState) {
        self.accept_endpoint_state(es);
        self.begin_journal();
        es.journal_open = true;
    }

    /// The area growth since the repair began, in percent (infinite from an empty design).
    fn area_growth_percent(&self) -> f64 {
        let area = self.design.design_area();
        if self.initial_design_area.abs() > 0.0 {
            (area - self.initial_design_area) / self.initial_design_area * 100.0
        } else {
            f64::INFINITY
        }
    }

    /// `SetupLegacyBase::printProgress`: the header at iteration 0; a row every
    /// `print_interval_` iterations, or when forced. StTNS over the startpoints' slacks now; Viol
    /// as the collector last counted.
    fn print_progress(&mut self, iteration: i64, force: bool) -> Result<(), Stop> {
        if iteration == 0 {
            for l in progress_header() {
                self.report(l);
            }
        }
        if iteration % PRINT_INTERVAL != 0 && !force {
            return Ok(());
        }
        let starts = collect_violating(&self.timing.starts, self.ctx.margin);
        let (wns, worst) = self.timing.worst();
        let field = format!("{iteration}*");
        let row = progress_row(
            &Row {
                iter: &field,
                removed: 0,
                resized: self.committer.committed + self.committer.pending,
                inserted: 0,
                cloned: 0,
                swaps: 0,
                area_growth_percent: self.area_growth_percent(),
                wns,
                st_tns: startpoint_tns(&starts),
                en_tns: self.timing.tns(),
                viol: self.collector_violating,
                worst: worst.as_deref().unwrap_or(""),
            },
            self.ctx.time_scale,
        );
        self.report(row);
        Ok(())
    }

    /// `OptimizationPolicy::finalizeAndReport`: a fresh collector's row, the closing rule, then
    /// `reportRepairSummary`.
    fn finalize_and_report(&mut self) -> Result<(), Stop> {
        let violating = collect_violating(&self.timing.ends, self.ctx.margin).len();
        let starts = collect_violating(&self.timing.starts, self.ctx.margin);
        let (wns, worst) = self.timing.worst();
        let row = progress_row(
            &Row {
                iter: "final",
                removed: 0,
                resized: self.committer.committed + self.committer.pending,
                inserted: 0,
                cloned: 0,
                swaps: 0,
                area_growth_percent: self.area_growth_percent(),
                wns,
                st_tns: startpoint_tns(&starts),
                en_tns: self.timing.tns(),
                viol: violating,
                worst: worst.as_deref().unwrap_or(""),
            },
            self.ctx.time_scale,
        );
        self.report(row);
        self.report("-".repeat(126));
        let size_up = self.committer.committed;
        if size_up > 0 {
            self.report(format!("[INFO RSZ-0051] Resized {size_up} instances: {size_up} up, 0 up match, 0 down, 0 VT"));
        }
        if fuzzy::less(wns, self.ctx.margin) {
            self.report("[WARNING RSZ-0062] Unable to repair all setup violations.".to_string());
        }
        Ok(())
    }
}

/// `SetupLegacyPolicy::pathImproved`: a better WNS, or the same WNS and a better endpoint slack.
fn path_improved(end_slack: f32, worst_slack: f32, prev_end_slack: f32, prev_worst_slack: f32) -> bool {
    fuzzy::greater(worst_slack, prev_worst_slack) || (fuzzy::equal(worst_slack, prev_worst_slack) && fuzzy::greater(end_slack, prev_end_slack))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stage(pin: &str, is_driver: bool, top_port: bool, load_delay: f32) -> Stage {
        Stage {
            pin: pin.into(),
            is_driver,
            top_port,
            inst: None,
            cell: None,
            port: None,
            load_delay: Some(load_delay),
            in_port: None,
            fanout: 1,
            load_cap: 0.0,
            fanin_caps: Vec::new(),
        }
    }

    // Rule (rankPathDrivers): from the start index, drivers that are not top-level ports, by
    // load-dependent delay larger first; equal delays put the LATER path index first.
    #[test]
    fn drivers_rank_by_load_delay_then_later_index() {
        let view = PathView {
            stages: vec![
                stage("clk", true, true, 0.0),
                stage("r1/CK", false, false, 0.0),
                stage("r1/Q", true, false, 2.0),
                stage("u1/A", false, false, 0.0),
                stage("u1/Z", true, false, 5.0),
                stage("u2/A", false, false, 0.0),
                stage("u2/Z", true, false, 2.0),
            ],
            start: 2,
        };
        assert_eq!(rank_path_drivers(&view), vec![(4, 5.0), (6, 2.0), (2, 2.0)]);
    }

    use crate::preamble::Master;
    use std::collections::BTreeMap;
    use vyges_sta::liberty::Library;
    use vyges_sta::liberty_parse::parse as lparse;

    /// A buffer: input cap `cin`, delay `d0 + (d1 − d0)·load` over a 0..1 load axis, and a fall
    /// slew of `r` at load 1 (its drive resistance, `r` over the unit load).
    fn buf(name: &str, cin: f32, d0: f32, d1: f32, r: f32) -> String {
        format!(
            r#"cell ({name}) {{ pin (A) {{ direction : input ; capacitance : {cin} ; }} pin (Z) {{ direction : output ; function : "A" ;
              timing () {{ related_pin : "A" ; timing_sense : positive_unate ;
                cell_rise (t) {{ values ("{d0}, {d1}", "{d0}, {d1}") ; }} cell_fall (t) {{ values ("{d0}, {d1}", "{d0}, {d1}") ; }}
                rise_transition (t) {{ values ("0, {r}", "0, {r}") ; }} fall_transition (t) {{ values ("0, {r}", "0, {r}") ; }} }} }} }}"#
        )
    }

    fn libs(cells: &[String]) -> Libs {
        let text = format!(
            r#"library (l) {{ lu_table_template (t) {{ variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ;
              index_1 ("0, 1") ; index_2 ("0, 1") ; }} {} }}"#,
            cells.join(" ")
        );
        Libs { libs: vec![Library::read(&lparse(&text).unwrap()).unwrap()], ..Default::default() }
    }

    fn with_sizing<R>(l: &Libs, f: impl FnOnce(&Sizing<'_>) -> R) -> R {
        let masters: BTreeMap<String, Master> = l.libs[0].cells.keys().map(|n| (n.clone(), Master { site: "s".into(), area: 1, is_core: true, logic_std: true, implant_obs: vec![] })).collect();
        let equiv = crate::sizing::make_equiv_cells(l);
        let (dont_use, loads) = (BTreeSet::new(), BTreeMap::new());
        f(&Sizing { libs: l, masters: &masters, dont_use: &dont_use, equiv: &equiv, target_loads: &loads, tgt_slews: [0.0; 2], tgt_scene: 0 })
    }

    // Rule (weakerCellFirst): cells with the driver port first, the LARGER drive resistance
    // first; cells without the port after them, by name.
    #[test]
    fn swappable_cells_are_tried_weakest_first() {
        let l = libs(&[buf("B4", 0.01, 0.1, 0.2, 1.0), buf("B1", 0.01, 0.1, 0.5, 4.0), buf("B2", 0.01, 0.1, 0.3, 2.0)]);
        let mut cells = vec!["B4".to_string(), "B2".into(), "NOPE".into(), "B1".into()];
        cells.sort_by(|a, b| weaker_cell_first(&l, a, b, "Z"));
        assert_eq!(cells, vec!["B1", "B2", "B4", "NOPE"]);
    }

    // Rule (upsizeCell): the first weaker-first cell that drives no worse and whose stage delay —
    // gateDelay at the load plus the previous driver's resistance times ITS input capacitance — is
    // smaller. A larger input capacitance can make the next size up lose to the one after it.
    #[test]
    fn size_up_takes_the_first_cell_with_a_faster_stage() {
        let l = libs(&[buf("B1", 0.01, 0.1, 0.5, 4.0), buf("B2", 0.2, 0.12, 0.32, 2.0), buf("B4", 0.05, 0.15, 0.25, 1.0)]);
        let unit = l.libs[0].cells["B1"].port("A").unwrap().capacitance[0][MAX] / 0.01;
        with_sizing(&l, |s| {
            // No previous driver: B2 is faster at half the unit load.
            assert_eq!(upsize_cell(s, "A", "B1", "Z", 0.5 * unit, 0.0).unwrap().as_deref(), Some("B2"));
            // A previous driver of 1 (time per unit cap) pays for B2's input cap: B4 wins.
            let r = l.libs[0].cells["B1"].drive_resistance("Z") / 4.0;
            assert_eq!(upsize_cell(s, "A", "B1", "Z", 0.5 * unit, r).unwrap().as_deref(), Some("B4"));
            // The strongest cell has nothing stronger.
            assert_eq!(upsize_cell(s, "A", "B4", "Z", 0.5 * unit, 0.0).unwrap(), None);
        });
    }

    // Rule (replacementPreservesMaxCap → checkMaxCapOK): only an input whose capacitance GROWS is
    // checked; a driver already over its limit may not grow at all, else it must stay within it;
    // a driver with no limit passes. (In one scene "may not grow" and "must stay within" agree for
    // an over-limit driver — its load already exceeds the limit — so a mutant swapping them lives.)
    #[test]
    fn a_size_up_may_not_overload_a_fanin_driver() {
        let l = libs(&[buf("B1", 0.01, 0.1, 0.5, 4.0), buf("B2", 0.2, 0.12, 0.32, 2.0)]);
        let unit = l.libs[0].cells["B1"].port("A").unwrap().capacitance[0][MAX] / 0.01;
        let check = |cap: f32, max_cap: f32| CapCheck { cap: cap * unit, max_cap: max_cap * unit, slack: (max_cap - cap) * unit, limited: true };
        let fanin = |c: CapCheck| vec![("A".to_string(), vec![c])];
        assert!(replacement_preserves_max_cap(&l, "B1", "B2", &fanin(check(0.5, 1.0))));
        assert!(!replacement_preserves_max_cap(&l, "B1", "B2", &fanin(check(0.9, 1.0))), "0.9 + 0.19 > 1");
        assert!(!replacement_preserves_max_cap(&l, "B1", "B2", &fanin(check(1.1, 1.0))), "already over");
        assert!(replacement_preserves_max_cap(&l, "B2", "B1", &fanin(check(1.1, 1.0))), "the input shrinks");
        assert!(replacement_preserves_max_cap(&l, "B1", "B2", &fanin(CapCheck { cap: 0.9, max_cap: -1e30, slack: 1e30, limited: false })));
    }

    // Rule (pathImproved): a fuzzily better WNS wins; an equal WNS needs a better endpoint slack.
    #[test]
    fn a_pass_improves_on_wns_then_on_the_endpoint() {
        assert!(path_improved(-0.3, -0.1, -0.3, -0.2));
        assert!(!path_improved(-0.1, -0.3, -0.3, -0.2));
        assert!(path_improved(-0.1, -0.2, -0.3, -0.2));
        assert!(!path_improved(-0.3, -0.2, -0.3, -0.2));
    }
}
