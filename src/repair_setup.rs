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

use std::collections::{BTreeMap, BTreeSet, HashMap};

use vyges_sta::fuzzy;
use vyges_sta::graph::{EdgeKind, Graph, SdcEnv};
use vyges_sta::liberty::{Cell, Model, Role, MAX};
use vyges_sta::incr::{EventCtx, IncTimer};
use vyges_sta::search::Search;

use crate::design::{Design, TimerEdits};
use crate::preamble::Libs;
use crate::repair_timing::{network_name, collect_violating, delay_as_string, progress_header, progress_row, startpoint_tns, timing_points, total_negative_slack, worst_slack, Args, Move, Point, Row};
use crate::sizing::Sizing;
use crate::timing::Limits;
use crate::{clone, rebuffer, swap_pins, unbuffer};
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
    /// The move sequence (`buildMainMoveSequence`), in order.
    pub sequence: &'a [Move],
    /// What the buffer-removal estimate reads: the constraints' limits, the database units, the
    /// signal wire RC, the slew shape factor.
    pub limits: &'a Limits,
    pub dbu: i32,
    pub wire_rc: crate::buffered_net::WireRc,
    pub slew_shape_factor: f32,
    /// `set_debug_level` per (tool, group): a debug line is traced when its group's level reaches it.
    pub debug: &'a BTreeMap<(String, String), i64>,
    /// BufferMove's characterized buffer sizes and units (`Rebuffer::init`), when the sequence has it.
    pub rebuffer: Option<&'a rebuffer::Ctx<'a>>,
    /// `buffer_lowest_drive_` (SplitLoadMove's buffer).
    pub lowest_buffer: &'a str,
    /// The reference's pin addresses, when the gate supplies them ([`unbuffer::PinAddr`]).
    pub pin_addr: Option<&'a unbuffer::PinAddr>,
    /// `dbNetwork::hasHierarchy`: instance and net names print without their parent prefix.
    pub hierarchy: bool,
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
    /// The path's transition at this pin.
    rf: usize,
    /// UnbufferMove's timer verdict on this driver (`None`: not asked, or `validTarget` fails).
    unbuffer: Option<unbuffer::Verdict>,
    /// On a driver: its instance's input pins' timed slews, rise and fall, by port
    /// (`annotateInputSlews`).
    in_slews: Vec<(String, [f32; 2])>,
    /// BufferMove's view of this driver (`None`: not asked).
    rebuffer: Option<RebufProbe>,
    /// CloneMove's fanouts by slack delta (`collectFanoutSlacks`; `None`: not asked).
    fanout_slacks: Option<Vec<clone::FanoutSlack>>,
    /// SplitLoadMove's (`collectRankedFanoutSlacks`: the driver's slack over both transitions,
    /// wire edges only).
    split_slacks: Option<Vec<clone::FanoutSlack>>,
}

/// What `rebufferPin` finds before it buffers.
#[derive(Debug, Clone)]
enum RebufProbe {
    /// The net has a top-level output port, or the driver no liberty port: nothing inserted.
    Skip,
    /// A warning, then nothing inserted (RSZ-2020 top port driver, RSZ-0075 no buffered net).
    Warn(String),
    /// The annotated net and the driver's timing.
    Ready(Box<rebuffer::Probe>),
}

/// `PathExpanded`: the path from its root, and `startIndex`.
#[derive(Debug, Clone)]
struct PathView {
    stages: Vec<Stage>,
    start: usize,
    /// The scene of the path (`Path::scene`): its delays, loads and library cells.
    scene: usize,
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

/// The timer's state across the repair: the reference's incremental timer, the database's edits
/// replayed into it at each update — a value it does not recompute stays as it was.
struct Timer {
    /// One per scene: every scene is timed on its own libraries and parasitics.
    inc: Vec<IncTimer>,
    /// The decision trace's length when the update ran (for `VYGES_RSZ_INC_TRACE`).
    trace_at: usize,
}

impl Timer {
    fn new(design: &mut dyn SetupDesign, scenes: usize) -> Result<Timer, Stop> {
        design.start_timer_edits().map_err(timer_stop)?;
        let mut inc = IncTimer::default();
        inc.track_netlist(design.netlist());
        Ok(Timer { inc: vec![inc; scenes], trace_at: 0 })
    }
}

/// The timer over the design and timed: `updateParasitics` has run, so the parasitics are the
/// estimator's current ones; `edits`, what the timer has not seen. `want`: the endpoints whose
/// worst path the repair will read.
fn snapshot(ctx: &Ctx<'_>, design: &dyn Design, want: &[String], timer: &mut Timer, edits: TimerEdits) -> Result<Snapshot, Stop> {
    timed_all(ctx, design, timer, edits, |gs, ss, cs| {
        // Each point's slack: the least over the scenes (`Sta::slack` over every path).
        let mut per_scene = Vec::with_capacity(gs.len());
        for k in 0..gs.len() {
            per_scene.push(timing_points(&gs[k], &ss[k], ctx.ssdc, ctx.libs, &cs[k]).map_err(timer_stop)?);
        }
        let (ends, starts) = if gs.len() == 1 {
            per_scene.pop().expect("one scene")
        } else {
            (
                crate::repair_timing::least_over_scenes(per_scene.iter().map(|(e, _)| e.as_slice()).collect()).map_err(timer_stop)?,
                crate::repair_timing::least_over_scenes(per_scene.iter().map(|(_, s)| s.as_slice()).collect()).map_err(timer_stop)?,
            )
        };
        let mut names: Vec<String> = want.to_vec();
        if let (_, Some(w)) = worst_slack(&ends) {
            names.push(w.pin.clone());
        }
        let mut paths = HashMap::new();
        for n in names {
            if paths.contains_key(&n) {
                continue;
            }
            // `vertexWorstSlackPath(end, max)` over every scene's paths: the fuzzily least slack,
            // the earlier scene on a tie; the path is expanded in its own scene.
            let mut best: Option<(usize, usize, f32)> = None;
            for k in 0..gs.len() {
                let Some(v) = gs[k].vertices.iter().position(|x| x.name == n) else { continue };
                let worst = ss[k].paths[v].iter().filter(|p| p.tag.mm == MAX).map(|p| p.required - p.arrival).fold(None, |m: Option<f32>, s| match m {
                    Some(m) if !fuzzy::less(s, m) => Some(m),
                    _ => Some(s),
                });
                if let Some(s) = worst {
                    if best.is_none_or(|(_, _, b)| fuzzy::less(s, b)) {
                        best = Some((k, v, s));
                    }
                }
            }
            let Some((k, v, _)) = best else { continue };
            let ideal = if ctx.ideal_clock { cs[k].clone() } else { BTreeSet::new() };
            if let Some(view) = expand(ctx, gs, ss, cs, design, k, v, &ideal)? {
                paths.insert(n, view);
            }
        }
        Ok(Snapshot { ends, starts, paths })
    })
}

/// [`timed_all`] over scene 0 alone (the hold repair: one corner).
fn timed<R>(ctx: &Ctx<'_>, design: &dyn Design, timer: &mut Timer, edits: TimerEdits, read: impl FnOnce(&Graph<'_>, &Search<'_, '_>, &BTreeSet<usize>) -> Result<R, Stop>) -> Result<R, Stop> {
    timed_all(ctx, design, timer, edits, |gs, ss, cs| read(&gs[0], &ss[0], &cs[0]))
}

/// The incremental timers brought up to date with the design (`edits` replayed, then delays,
/// arrivals and requireds in full — the reference's `findRequireds`), every scene on its own
/// libraries and parasitics; `read` run over the scenes' graphs, searches and clock pins.
fn timed_all<R>(ctx: &Ctx<'_>, design: &dyn Design, timer: &mut Timer, edits: TimerEdits, read: impl FnOnce(&[Graph<'_>], &[Search<'_, '_>], &[BTreeSet<usize>]) -> Result<R, Stop>) -> Result<R, Stop> {
    let netlist = design.netlist();
    let mut graphs = Vec::with_capacity(timer.inc.len());
    let mut all_clocks = Vec::with_capacity(timer.inc.len());
    for k in 0..timer.inc.len() {
        let mut gk = crate::repair_design::timer_graph(ctx.libs, k, netlist, ctx.env, ctx.master_pins)?;
        let ck = crate::timing::clock_pins(&gk, ctx.clock_sources);
        if ctx.ideal_clock {
            gk.ideal_clock = ck.iter().copied().collect();
        }
        inc_prepare(ctx, design, &mut gk, &mut timer.inc[k], &edits)?;
        graphs.push(gk);
        all_clocks.push(ck);
    }
    // `findDelays()`: one queue over every scene (the reference's one graph).
    let pars: Vec<&HashMap<String, vyges_sta::graph::NetParasitics>> = (0..graphs.len()).map(|k| design.parasitics(k)).collect();
    vyges_sta::incr::find_delays_scenes(&mut timer.inc, &mut graphs, &pars, None).map_err(timer_stop)?;
    for (inc, gk) in timer.inc.iter_mut().zip(&graphs) {
        inc.export(gk);
    }
    let g = &graphs[0];
    // A diagnostic: one pin's slews at every snapshot (`VYGES_RSZ_SNAP_PIN=pin:path`).
    if let Some((pin, path)) = std::env::var("VYGES_RSZ_SNAP_PIN").ok().as_deref().and_then(|s| s.split_once(':')).map(|(a, b)| (a.to_string(), b.to_string())) {
        if let Some(v) = g.vertices.iter().position(|x| x.name == pin) {
            use std::io::Write;
            let net = g.vertex_net[v].map(|n| g.netlist.nets[n].name.clone()).unwrap_or_default();
            let has_par = design.parasitics(0).contains_key(&net);
            if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                let _ = writeln!(f, "SNAP {pin} slew={:e},{:e} net={net} parasitic={has_par} ideal={}", g.slew[v][0][MAX], g.slew[v][1][MAX], g.ideal_clock.contains(&v));
            }
        }
    }
    let mut searches: Vec<Search<'_, '_>> = graphs.iter().map(|gk| Search::in_graph_order(gk, ctx.ssdc)).collect();
    for (inc, search) in timer.inc.iter_mut().zip(searches.iter_mut()) {
        inc.import_paths(search);
    }
    // One queue over every scene for arrivals, then for requireds (a vertex's tag group holds
    // every scene's paths).
    vyges_sta::incr::find_arrivals_scenes(&mut timer.inc, &mut searches, None).map_err(timer_stop)?;
    vyges_sta::incr::find_requireds_scenes(&mut timer.inc, &mut searches, None).map_err(timer_stop)?;
    inc_trace(&timer.inc[0], timer.trace_at);
    let search = &searches[0];
    // A diagnostic: every instance pin's max slack per transition after this update, in fs
    // (`VYGES_RSZ_SLACKS=path`, rewritten each update — the last is the state the repair left).
    if let Some(path) = std::env::var_os("VYGES_RSZ_SLACKS") {
        let mut s = String::new();
        for (v, vx) in g.vertices.iter().enumerate() {
            if vx.lib.is_none() {
                continue;
            }
            let slack = |rf: usize| {
                let mut m = INF;
                for p in search.paths[v].iter().filter(|p| p.tag.mm == MAX && p.tag.rf == rf) {
                    let x = p.required - p.arrival;
                    if fuzzy::less(x, m) {
                        m = x;
                    }
                }
                if m == INF { "INF".to_string() } else { format!("{:.9}", f64::from(m) * 1e15) }
            };
            s.push_str(&format!("{} {} {}\n", vx.name, slack(0), slack(1)));
        }
        let _ = std::fs::write(path, s);
    }
    read(&graphs, &searches, &all_clocks)
}

/// The edits replayed into one scene's incremental timer (`dbStaCbk`), the estimator's
/// `delaysInvalidFromFanin` on each net it estimated — but a skip net, driven by an ideal clock
/// pin (`isSkipNet`) — and the stored state imported into the scene's graph; `findDelays()`
/// follows over every scene at once.
fn inc_prepare(ctx: &Ctx<'_>, design: &dyn Design, g: &mut Graph<'_>, inc: &mut IncTimer, edits: &TimerEdits) -> Result<(), Stop> {
    let cx = EventCtx { libs: &ctx.libs.libs, ideal_clock_mode: ctx.ideal_clock };
    for ev in &edits.events {
        inc.apply_event(&cx, ev).map_err(timer_stop)?;
    }
    let netlist = design.netlist();
    let index = g.pin_index();
    for net in &edits.updated_nets {
        let Some(n) = netlist.nets.iter().find(|n| &n.name == net) else { continue };
        let pins: Vec<String> = n.pins.iter().map(|c| netlist.pin_name(c)).collect();
        let skip = pins.iter().filter_map(|p| index.get(p)).any(|&v| g.vertices[v].is_driver && g.ideal_clock.contains(&v));
        if skip {
            continue;
        }
        for p in &pins {
            inc.delays_invalid_from(p);
        }
    }
    inc.import(g);
    Ok(())
}

/// What the last update visited (`VYGES_RSZ_INC_TRACE=path`), as the instrumented reference
/// prints it: `DV|pin`, `AV|pin|changed`, `RV|pin|changed`, after a `SNAP|<trace length>` line.
fn inc_trace(inc: &IncTimer, trace_at: usize) {
    let Some(path) = std::env::var_os("VYGES_RSZ_INC_TRACE") else { return };
    use std::io::Write;
    let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) else { return };
    let _ = writeln!(f, "SNAP|{trace_at}");
    for v in &inc.visited {
        let _ = writeln!(f, "DV|{v}");
    }
    for (v, c) in &inc.arrival_visits {
        let _ = writeln!(f, "AV|{v}|{}", u8::from(*c));
    }
    for (v, c) in &inc.required_visits {
        let _ = writeln!(f, "RV|{v}|{}", u8::from(*c));
    }
}

/// `Sta::vertexWorstSlackPath(end, max)`, then `PathExpanded`: the max path with the fuzzily
/// least slack (the first, in tag order), walked back through its prev paths to the root.
/// `startIndex`: the pin reached by the clock-to-output arc nearest the end, else the root.
#[allow(clippy::too_many_arguments)]
fn expand(ctx: &Ctx<'_>, gs: &[Graph<'_>], ss: &[Search<'_, '_>], cs: &[BTreeSet<usize>], design: &dyn Design, scene: usize, end: usize, ideal: &BTreeSet<usize>) -> Result<Option<PathView>, Stop> {
    let (g, search) = (&gs[scene], &ss[scene]);
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
    // `checkMaxCapOK` (`checkCapacitance` over every scene), in scene 0's vertices.
    let sc = crate::timing::Scenes::new(gs);
    let all_parasitics: Vec<HashMap<String, vyges_sta::graph::NetParasitics>>;
    let parasitics: &[HashMap<String, vyges_sta::graph::NetParasitics>] = if gs.len() == 1 {
        std::slice::from_ref(design.parasitics(0))
    } else {
        all_parasitics = (0..gs.len()).map(|k| design.parasitics(k).clone()).collect();
        &all_parasitics
    };
    let ideal0: BTreeSet<usize> = if scene == 0 { ideal.clone() } else { ideal.iter().filter_map(|&u| gs[0].vertices.iter().position(|x| x.name == g.vertices[u].name && x.is_driver == g.vertices[u].is_driver)).collect() };
    let to_scene0 = |u: usize| if scene == 0 { Some(u) } else { gs[0].vertices.iter().position(|x| x.name == g.vertices[u].name && x.is_driver == g.vertices[u].is_driver) };
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
            rf: p.tag.rf,
            unbuffer: None,
            in_slews: Vec::new(),
            rebuffer: None,
            fanout_slacks: None,
            split_slacks: None,
        };
        if let Some(prev) = p.prev {
            if let Some(set) = edge_arc_set(g, prev.edge) {
                st.in_port = Some(set.from.clone());
                st.load_delay = Some(g.delay[prev.edge][prev.arc][MAX] - arc_intrinsic(&set.arcs[prev.arc].model));
            }
        }
        if i > 0 && vx.is_driver && !top_port {
            st.load_cap = g.load_cap(*v, design.parasitics(scene));
            // Each input pin of the instance, its net's drivers' capacitance checks.
            for (u, ux) in g.vertices.iter().enumerate() {
                let same = matches!(&ux.conn, vyges_sta::netlist::Conn::Inst(k, _) if Some(*k) == inst_index);
                if !same || ux.is_driver {
                    continue;
                }
                let checks = crate::timing::net_drivers(g, u)
                    .into_iter()
                    .filter_map(to_scene0)
                    .map(|d| {
                        let (cap, max_cap, slack, limited, _) = crate::timing::check_capacitance(&sc, d, parasitics, &ideal0);
                        CapCheck { cap, max_cap, slack, limited }
                    })
                    .collect();
                st.fanin_caps.push((ux.port.clone().unwrap_or_default(), checks));
                // annotateInputSlews: an input pin with a liberty port.
                let is_input = ux.cell.as_deref().and_then(|c| ctx.libs.link_cell(c)).and_then(|c| c.port(ux.port.as_deref()?)).is_some_and(|p| p.direction == vyges_sta::liberty::Direction::Input);
                if is_input {
                    st.in_slews.push((ux.port.clone().unwrap_or_default(), [g.slew[u][0][MAX], g.slew[u][1][MAX]]));
                }
            }
        }
        stages.push(st);
    }
    if ctx.sequence.contains(&Move::Unbuffer) {
        probe_unbuffer(ctx, gs, ss, cs, design, scene, ideal, &chain, &mut stages);
    }
    if ctx.sequence.contains(&Move::Clone) {
        for (i, (v, _)) in chain.iter().enumerate() {
            let st = &stages[i];
            if i > 0 && st.is_driver && !st.top_port && st.fanout > clone::CLONE_MIN_FANOUT {
                stages[i].fanout_slacks = Some(fanout_slacks(g, search, *v, stages[i].rf, false));
            }
        }
    }
    if ctx.sequence.contains(&Move::SplitLoad) {
        for (i, (v, _)) in chain.iter().enumerate() {
            let st = &stages[i];
            if i > 0 && st.is_driver && !st.top_port && st.fanout > clone::CLONE_MIN_FANOUT {
                stages[i].split_slacks = Some(fanout_slacks(g, search, *v, stages[i].rf, true));
            }
        }
    }
    if ctx.sequence.contains(&Move::Buffer) {
        for (i, (v, _)) in chain.iter().enumerate() {
            let st = &stages[i];
            if i > 0 && st.is_driver && !st.top_port && st.fanout > 1 && st.fanout < rebuffer::REBUFFER_MAX_FANOUT {
                stages[i].rebuffer = Some(probe_rebuffer(ctx, gs, ss, cs, design, &g.vertices[*v].name));
            }
        }
    }
    Ok(Some(PathView { stages, start, scene }))
}

/// `rebufferPin` up to its buffering: the driver's checks, `makeBufferedNet`, `annotateLoadSlacks`
/// (each load's worst max path walked back to the driver: its slack, its transition, and the
/// first such path per transition as `arrival_paths_`), `drvrPinTiming`'s arcs and `setPin`.
/// `corner_` is `cmdScene()` (scene 0): the buffered net and `setPin`'s limits are read there;
/// each load's path is its worst over EVERY scene (`vertexWorstSlackPath`), and a transition's
/// driver arc is timed in its arrival path's scene.
fn probe_rebuffer(ctx: &Ctx<'_>, gs: &[Graph<'_>], ss: &[Search<'_, '_>], cs: &[BTreeSet<usize>], design: &dyn Design, drvr_name: &str) -> RebufProbe {
    let g = &gs[0];
    let ideal: BTreeSet<usize> = if ctx.ideal_clock { cs[0].clone() } else { BTreeSet::new() };
    let Some(drvr) = g.vertices.iter().position(|x| x.name == drvr_name && x.is_driver) else { return RebufProbe::Skip };
    let vx = &g.vertices[drvr];
    if vx.lib.is_none() {
        return RebufProbe::Warn(format!("[WARNING RSZ-2020] rebuffering does not support top port as the driver pin: {}", vx.name));
    }
    let Some(net) = g.vertex_net[drvr] else { return RebufProbe::Skip };
    let (Some(cell_name), Some(port)) = (vx.cell.as_deref(), vx.port.as_deref()) else { return RebufProbe::Skip };
    let Some(cell) = g.libs[vx.lib.expect("an instance pin")].cells.get(cell_name) else { return RebufProbe::Skip };
    // hasTopLevelOutputPort.
    let top_out = g.netlist.nets[net].pins.iter().any(|c| matches!(c, vyges_sta::netlist::Conn::Port(k) if g.netlist.ports[*k].1 == vyges_sta::netlist::PortDir::Output));
    if top_out {
        return RebufProbe::Skip;
    }
    let net_name = &g.netlist.nets[net].name;
    let bctx = crate::buffered_net::Ctx { graph: g, libs: ctx.libs, sdc: ctx.env, limits: ctx.limits, dbu: ctx.dbu, rc: ctx.wire_rc };
    let Some((bn, root)) = design.steiner(net_name, &vx.name).and_then(|tree| crate::buffered_net::make_buffered_net_steiner(&bctx, &tree)) else {
        return RebufProbe::Warn(format!("[WARNING RSZ-0075] makeBufferedNet failed for driver {}", vx.name));
    };
    // The buffered net as rebuffer nodes, its loads annotated in visit order.
    let mut nodes: Vec<rebuffer::Node> = Vec::with_capacity(bn.nodes.len());
    for n in &bn.nodes {
        let kind = match n.kind {
            crate::buffered_net::Kind::Load { pin } => rebuffer::Kind::Load { pin: g.vertices[pin].name.clone() },
            crate::buffered_net::Kind::Junction { r, r2 } => rebuffer::Kind::Junction { r, r2 },
            crate::buffered_net::Kind::Wire { r } => rebuffer::Kind::Wire { r },
        };
        nodes.push(rebuffer::Node { kind, x: n.x, y: n.y, cap: n.cap, fanout: n.fanout, max_load_slew: n.max_load_slew, area: 0.0, slack_rf: None, slack: rebuffer::FixedDelay::ZERO, delay: rebuffer::FixedDelay::ZERO, arrival: rebuffer::FixedDelay::ZERO });
    }
    let mut arrival_paths: [Option<(usize, vyges_sta::search::Path)>; 2] = [None, None];
    let mut warnings = Vec::new();
    annotate_load_slacks(gs, ss, &bn, root, &vx.name, &mut nodes, &mut arrival_paths, &mut warnings);
    let mut arcs: [Option<Option<rebuffer::DriverArc>>; 2] = [None, None];
    let mut arc_scenes = [0usize; 2];
    for rf in 0..2 {
        let Some((k, ap)) = arrival_paths[rf] else { continue };
        arc_scenes[rf] = k;
        let (gk, sk) = (&gs[k], &ss[k]);
        arcs[rf] = Some(ap.prev.and_then(|prev| {
            let dp = sk.paths[prev.vertex].iter().find(|q| q.tag == prev.tag)?;
            let set = edge_arc_set(gk, prev.edge)?;
            let arc = &set.arcs[prev.arc];
            // edgeFromSlew: an ideal clock pin's clock-to-output edge reads the ideal slew, 0.
            let ideal_clk = set.role == Role::RegClkToQ && gk.ideal_clock.contains(&prev.vertex);
            let from_slew = if ideal_clk { 0.0 } else { gk.slew[prev.vertex][arc.from_rf][MAX] };
            // clkPathArrival for a clock path: the ideal clock's edge, else its arrival.
            let prev_arrival = if dp.tag.is_clock && !ctx.ssdc.clock.propagated { ctx.ssdc.clock.edge_time(dp.tag.clk_edge.unwrap_or(dp.tag.rf)) } else { dp.arrival };
            Some(rebuffer::DriverArc { model: arc.model.clone(), from_slew, prev_arrival, arrival: ap.arrival })
        }));
    }
    let link = ctx.libs.link_cell(cell_name).unwrap_or(cell);
    // setPin: the fanout limit (the resizer's check, with its backstop); the driver's slew limit
    // at `corner_` (`findSlewLimit(drvr_port_, corner_)`: the scene port and its library).
    let (_, max_fanout, _) = crate::timing::check_fanout(g, design.net_info(), drvr, ctx.limits, &ideal);
    let fanout_limit = if max_fanout > 0.0 { max_fanout } else { INF };
    let lib = ctx.libs.scene_library(0, cell_name).unwrap_or(&g.libs[vx.lib.expect("an instance pin")]);
    let p = ctx.libs.scene_cell(0, cell_name).unwrap_or(link).port(port);
    let (max_slew, exists) = crate::timing::find_slew_limit(lib, p.map_or(vyges_sta::liberty::Direction::Output, |p| p.direction), p.and_then(|p| p.max_transition), ctx.limits);
    let drvr_pin_max_slew = if exists { (f64::from(max_slew) * (1.0 - 20.0 / 100.0)) as f32 } else { INF };
    RebufProbe::Ready(Box::new(rebuffer::Probe {
        nodes,
        root,
        arcs,
        arc_scenes,
        drvr_port_cap: port_capacitance(link, port).unwrap_or(0.0),
        drvr_resistance: link.drive_resistance(port),
        fanout_limit,
        drvr_pin_max_slew,
        pin: vx.name.clone(),
        warnings,
    }))
}

/// `collectFanoutSlacks` (CloneMove) / `collectRankedFanoutSlacks` (SplitLoadMove, `split`): at
/// the path's transition, each out-edge's vertex's slack (`Sta::slack(vertex, rf, max)`) less the
/// driver's — at that transition for a clone, over both for a split, which also skips a non-wire
/// edge — in the reference's order.
fn fanout_slacks(g: &Graph<'_>, search: &Search<'_, '_>, drvr: usize, rf: usize, split: bool) -> Vec<clone::FanoutSlack> {
    let slack_at = |v: usize, rf: Option<usize>| {
        let mut s = INF;
        for p in search.paths[v].iter().filter(|p| p.tag.mm == MAX && rf.is_none_or(|r| p.tag.rf == r)) {
            let x = p.required - p.arrival;
            if fuzzy::less(x, s) {
                s = x;
            }
        }
        s
    };
    let drvr_slack = slack_at(drvr, if split { None } else { Some(rf) });
    let fanouts = g.out_edges[drvr]
        .iter()
        .filter(|&&e| !split || matches!(g.edges[e].kind, EdgeKind::Wire))
        .map(|&e| {
            let to = g.edges[e].to;
            clone::FanoutSlack { pin: g.vertices[to].name.clone(), slack: slack_at(to, Some(rf)) - drvr_slack, top_port: g.vertices[to].lib.is_none() }
        })
        .collect();
    clone::collect_fanout_slacks(fanouts)
}

/// `annotateLoadSlacks` over the buffered net: per load, `vertexWorstSlackPath(max)` — the
/// fuzzily least slack over every scene's paths, the earlier scene on a tie — walked back along
/// its previous paths (in its scene) to the driver; its slack `required − arrival(at the driver)`
/// and transition, or INF with none when the walk does not reach the driver.
#[allow(clippy::too_many_arguments)]
fn annotate_load_slacks(gs: &[Graph<'_>], ss: &[Search<'_, '_>], bn: &crate::buffered_net::BufferedNet, n: usize, drvr: &str, nodes: &mut [rebuffer::Node], arrival_paths: &mut [Option<(usize, vyges_sta::search::Path)>; 2], warnings: &mut Vec<String>) {
    match bn.nodes[n].kind {
        crate::buffered_net::Kind::Wire { r } => annotate_load_slacks(gs, ss, bn, r, drvr, nodes, arrival_paths, warnings),
        crate::buffered_net::Kind::Junction { r, r2 } => {
            annotate_load_slacks(gs, ss, bn, r, drvr, nodes, arrival_paths, warnings);
            annotate_load_slacks(gs, ss, bn, r2, drvr, nodes, arrival_paths, warnings);
        }
        crate::buffered_net::Kind::Load { pin } => {
            let name = gs[0].vertices[pin].name.as_str();
            let mut req: Option<(usize, usize, vyges_sta::search::Path)> = None;
            let mut min_slack = INF;
            for (k, (g, search)) in gs.iter().zip(ss).enumerate() {
                let Some(v) = g.vertices.iter().position(|x| x.name == name && !x.is_driver) else { continue };
                for p in search.paths[v].iter().filter(|p| p.tag.mm == MAX) {
                    let s = p.required - p.arrival;
                    if fuzzy::less(s, min_slack) {
                        min_slack = s;
                        req = Some((k, v, *p));
                    }
                }
            }
            let mut arrival = req.map(|(_, _, p)| p);
            if let Some((k, v, _)) = req {
                let (g, search) = (&gs[k], &ss[k]);
                let mut at = v;
                while let Some(a) = arrival {
                    if g.vertices[at].name == drvr && g.vertices[at].is_driver {
                        break;
                    }
                    arrival = a.prev.and_then(|pv| {
                        at = pv.vertex;
                        search.paths[pv.vertex].iter().find(|q| q.tag == pv.tag).copied()
                    });
                    if arrival.is_none() {
                        warnings.push(format!("[WARNING RSZ-2006] failed to trace timing path for load {} when buffering {}", name, drvr));
                    }
                }
            }
            match (req, arrival) {
                (Some((k, _, r)), Some(a)) => {
                    nodes[n].slack_rf = Some(rebuffer::Rfs::of(r.tag.rf));
                    nodes[n].slack = rebuffer::FixedDelay::from_secs(r.required - a.arrival);
                    if arrival_paths[r.tag.rf].is_none() {
                        arrival_paths[r.tag.rf] = Some((k, a));
                    }
                }
                _ => {
                    nodes[n].slack_rf = None;
                    nodes[n].slack = rebuffer::FixedDelay::inf();
                }
            }
        }
    }
}

/// UnbufferMove's timer verdict on each driver of the path that may be ranked
/// ([`unbuffer::probe`]), taken while the timer is at hand.
#[allow(clippy::too_many_arguments)]
fn probe_unbuffer(ctx: &Ctx<'_>, gs: &[Graph<'_>], ss: &[Search<'_, '_>], cs: &[BTreeSet<usize>], design: &dyn Design, scene: usize, ideal: &BTreeSet<usize>, chain: &[(usize, vyges_sta::search::Path)], stages: &mut [Stage]) {
    let (g, search) = (&gs[scene], &ss[scene]);
    let index: HashMap<String, usize> = g.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect();
    let steiner = |net: &str, drvr: &str| design.steiner(net, drvr);
    // Several corners: every scene's timer, for the reads made over all of them.
    let several = gs.len() > 1;
    let all_parasitics: Vec<HashMap<String, vyges_sta::graph::NetParasitics>> =
        if several { (0..gs.len()).map(|k| design.parasitics(k).clone()).collect() } else { Vec::new() };
    let ideals: Vec<BTreeSet<usize>> =
        if several { cs.iter().map(|c| if ctx.ideal_clock { c.clone() } else { BTreeSet::new() }).collect() } else { Vec::new() };
    let indices: Vec<HashMap<String, usize>> = if several {
        gs.iter().map(|gk| gk.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect()).collect()
    } else {
        Vec::new()
    };
    let multi = unbuffer::Multi { graphs: gs, searches: ss, parasitics: &all_parasitics, ideals: &ideals, indices: &indices };
    let t = unbuffer::Timer { g, search, parasitics: design.parasitics(scene), ideal, info: design.net_info(), index: &index, steiner: &steiner, multi: several.then_some(&multi) };
    let uctx = unbuffer::Ctx {
        libs: ctx.libs,
        limits: ctx.limits,
        sdc: ctx.env,
        dbu: ctx.dbu,
        wire_rc: ctx.wire_rc,
        slew_shape_factor: ctx.slew_shape_factor,
        tgt_slews: ctx.sizing.tgt_slews,
        margin: ctx.margin,
        master_pins: ctx.master_pins,
        pin_addr: ctx.pin_addr,
        hierarchy: ctx.hierarchy,
    };
    let path: Vec<unbuffer::PathStage<'_>> = chain
        .iter()
        .zip(stages.iter())
        .map(|((v, _), st)| unbuffer::PathStage { vertex: *v, rf: st.rf, inst: st.inst.as_deref(), cell: st.cell.as_deref(), in_port: st.in_port.as_deref() })
        .collect();
    let verdicts: Vec<Option<unbuffer::Verdict>> = (0..stages.len())
        .map(|i| {
            let st = &stages[i];
            (i > 0 && st.is_driver && !st.top_port).then(|| unbuffer::probe(&uctx, &t, &path, i, st.fanout)).flatten()
        })
        .collect();
    for (st, v) in stages.iter_mut().zip(verdicts) {
        st.unbuffer = v;
    }
}

/// `SetupLegacyBase::rankPathDrivers`: from the start index, every driver that is not a top-level
/// port, with its load-dependent delay; by delay, larger first, a tie by the LATER index first.
fn rank_path_drivers(view: &PathView) -> Vec<(usize, f32)> {
    let mut load_delays = Vec::new();
    for (i, st) in view.stages.iter().enumerate().skip(view.start) {
        if i > 0 && st.is_driver && !st.top_port {
            // A diagnostic: the key's raw bits (`VYG_RANK`), as the instrumented reference
            // prints them (`VYGC|rankld`).
            if std::env::var_os("VYG_RANK").is_some() {
                eprintln!("VYGR|rankld|{}|{:08x}", st.pin, st.load_delay.unwrap_or(0.0).to_bits());
            }
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
                delays[arc.to_rf] = crate::rebuffer::std_max(delays[arc.to_rf], delay);
            }
        }
    }
    crate::rebuffer::std_max(delays[0], delays[1])
}

/// `MoveGenerator::weakerCellFirst(lhs, rhs, port, lib_ap)` as a strict "lhs first": the cells
/// with the driver port at the scene (`scenePort`) first; among those, `std::tie(drive
/// resistance, intrinsic delay)` of lhs GREATER than rhs's; cells without it by name.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn weaker_cell_first(libs: &Libs, scene: usize, a: &str, b: &str, port: &str) -> bool {
    let (ca, cb) = (libs.scene_cell(scene, a), libs.scene_cell(scene, b));
    let pa = ca.filter(|c| c.port(port).is_some());
    let pb = cb.filter(|c| c.port(port).is_some());
    match (pa, pb) {
        (Some(_), None) => true,
        (None, Some(_)) => false,
        (None, None) => a < b,
        (Some(x), Some(y)) => {
            let (xr, xi) = (x.drive_resistance(port), port_intrinsic_delay(x, port));
            let (yr, yi) = (y.drive_resistance(port), port_intrinsic_delay(y, port));
            // `std::tie(...) > std::tie(...)`: lexicographic, each `<` on `float`.
            yr < xr || (!(xr < yr) && yi < xi)
        }
    }
}

/// `SizeUpGenerator::upsizeCell`: the swappable cells weaker first; the first whose port drives no
/// worse and whose stage delay — `gateDelay` at the load plus the previous driver's resistance
/// times its input capacitance — is smaller than the current cell's.
fn upsize_cell(sizing: &Sizing<'_>, scene: usize, in_port: &str, cell: &str, drvr_port: &str, load_cap: f32, prev_drive: f32) -> Result<Option<String>, Stop> {
    let libs = sizing.libs;
    let mut swappable = sizing.swappable_cells(cell)?;
    if swappable.is_empty() {
        return Ok(None);
    }
    crate::order::libcxx_sort_by(&mut swappable, |a, b| weaker_cell_first(libs, scene, a, b, drvr_port)).map_err(|h| Stop::refused("RSZ-ORDER", format!("{} swappable cells reach the sort's heap fallback, which is not modelled", h.len)))?;
    // The current cell's ports at the scene (`scenePort`); the gate delay of its arcs there.
    let Some(c) = libs.scene_cell(scene, cell) else { return Ok(None) };
    let (Some(_), Some(in_cap)) = (c.port(drvr_port), port_capacitance(c, in_port)) else { return Ok(None) };
    let drive_r = c.drive_resistance(drvr_port);
    let delay = gate_delay(sizing, c, drvr_port, load_cap) + prev_drive * in_cap;
    for name in swappable {
        let Some(s) = libs.scene_cell(scene, &name) else { continue };
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
    let Some(to) = upsize_cell(ctx.sizing, view.scene, in_port, cell, port, st.load_cap, prev_drive)? else { return Ok(None) };
    if !replacement_preserves_max_cap(ctx.libs, cell, &to, &st.fanin_caps) {
        return Ok(None);
    }
    Ok(Some(SizeUp { inst: inst.clone(), from: cell.clone(), to }))
}

/// `MoveResult`: an accepted move's type, count and the instances it touched.
#[derive(Debug, Clone)]
struct MoveResult {
    kind: Move,
    count: i64,
    insts: Vec<String>,
}

/// `MoveCommitter`'s accounting per move type, and its journal: per open level, the results
/// accepted in it (the design keeps the edits' own journal). An instance is remembered by its odb
/// id: the reference keys its sets by `Instance*` — the odb object's address — and odb reuses a
/// destroyed object's slot, so an instance made after an undo inherits whatever its slot's
/// previous owner had been through.
#[derive(Default)]
struct Committer {
    committed: HashMap<Move, i64>,
    pending: HashMap<Move, i64>,
    /// The instances each type touched: once touched, a later guard sees it even after the
    /// journal level is restored (the reference keeps this "legacy" set on purpose).
    committed_insts: HashMap<Move, BTreeSet<u32>>,
    pending_insts: HashMap<Move, Vec<u32>>,
    levels: Vec<Vec<(MoveResult, Vec<u32>)>>,
}

impl Committer {
    /// `totalMoves(type)`: committed and pending.
    fn total(&self, m: Move) -> i64 {
        self.committed.get(&m).copied().unwrap_or(0) + self.pending.get(&m).copied().unwrap_or(0)
    }

    fn committed(&self, m: Move) -> i64 {
        self.committed.get(&m).copied().unwrap_or(0)
    }

    /// `recordAcceptedResult` (the touched instances by odb id).
    fn record(&mut self, r: &MoveResult, ids: &[u32]) {
        *self.pending.entry(r.kind).or_default() += r.count;
        for &i in ids {
            self.pending_insts.entry(r.kind).or_default().push(i);
            self.committed_insts.entry(r.kind).or_default().insert(i);
        }
    }

    /// `unrecordAcceptedResult`: one instance entry removed per touch.
    fn unrecord(&mut self, r: &MoveResult, ids: &[u32]) {
        *self.pending.entry(r.kind).or_default() -= r.count;
        let list = self.pending_insts.entry(r.kind).or_default();
        for i in ids {
            if let Some(k) = list.iter().position(|x| x == i) {
                list.remove(k);
            }
        }
    }

    /// `acceptPendingMoves`.
    fn accept_pending(&mut self) {
        for (m, n) in self.pending.drain() {
            *self.committed.entry(m).or_default() += n;
        }
        for (m, insts) in self.pending_insts.drain() {
            self.committed_insts.entry(m).or_default().extend(insts);
        }
    }

    /// `hasMoves(type, inst)`.
    fn has_moves(&self, m: Move, inst: u32) -> bool {
        self.committed_insts.get(&m).is_some_and(|s| s.contains(&inst)) || self.pending_insts.get(&m).is_some_and(|l| l.contains(&inst))
    }

    /// `hasBlockingBufferRemovalMove`: the first reason, in the reference's order.
    fn blocking_buffer_removal(&self, inst: u32) -> Option<&'static str> {
        [
            (Move::SwapPins, "its pins have been swapped"),
            (Move::Clone, "it has been cloned"),
            (Move::SplitLoad, "it was from split load buffering"),
            (Move::Buffer, "it was from rebuffering"),
            (Move::SizeUp, "it has been resized"),
        ]
        .into_iter()
        .find(|(m, _)| self.has_moves(*m, inst))
        .map(|(_, why)| why)
    }
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
/// with the debug lines `set_debug_level` asked for between them.
#[derive(Debug, Clone, Default)]
pub struct Outcome {
    pub lines: Vec<String>,
    pub trace: Vec<String>,
    /// SizeUp moves kept.
    pub resized: i64,
    /// Buffers removed (UnbufferMove).
    pub removed: i64,
    /// Buffers inserted (BufferMove).
    pub inserted: i64,
}

struct Repair<'c, 'd> {
    ctx: &'c Ctx<'c>,
    args: &'c Args,
    design: &'d mut dyn SetupDesign,
    timing: Snapshot,
    /// The phase's label with its marker (`LEGACY*`, `LAST_GASP+`) and its move sequence.
    phase: &'static str,
    sequence: Vec<Move>,
    /// The timer's state across the repair ([`snapshot`]).
    timer: Timer,
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

/// What `Resizer::swapPins` did.
#[derive(Debug, Clone, PartialEq)]
pub enum PinSwap {
    /// The pins traded nets.
    Swapped,
    /// A pin has no net: nothing to do, and the move still counts.
    NoNet,
    /// A pin's net is dont_touch: rejected (the first such net, pin 1's first).
    DontTouch(String),
}

/// A buffer removed: the nets either side as they were named before, and the survivor's name now.
#[derive(Debug, Clone, PartialEq)]
pub struct RemovedBuffer {
    pub in_net: String,
    pub out_net: String,
    pub survivor: String,
}

/// The design as setup repair edits it: also `computeDesignArea`, the journal, buffer removal.
pub trait SetupDesign: Design {
    /// `Resizer::computeDesignArea`: over the block's instances, each non-filler master's area
    /// (m², core-autoplaceable masters only).
    fn design_area(&self) -> f64;
    /// `Resizer::area(dbMaster)`: `dbuToMeters(width) × dbuToMeters(height)`, 0 for a master that
    /// is not core autoplaceable.
    fn master_area(&self, cell: &str) -> f64;
    /// `Resizer::coreArea`: `dbuToMeters(core dx) × dbuToMeters(core dy)` (0 without a core).
    fn core_area(&self) -> f64;
    fn as_design(&self) -> &dyn Design;
    /// `odb::dbDatabase::beginEco`: a nested journal level.
    fn begin_journal(&mut self) -> Result<(), String>;
    /// `commitEco`: the level merged into its parent, permanent at the outermost.
    fn commit_journal(&mut self) -> Result<(), String>;
    /// `undoEco`: the level's edits undone, the estimator's state with them. Returns whether the
    /// level had any edit (`ecoHasPendingChanges` before the undo).
    fn restore_journal(&mut self) -> Result<bool, String>;
    /// `Resizer::canRemoveBuffer(buffer, honor_dont_touch_fixed = true)`'s database half: neither
    /// the buffer nor its nets dont_touch, the buffer not FIXED, and the net that would be removed
    /// mergeable into the survivor (`canMergeNet`).
    fn can_remove_buffer(&self, inst: &str, in_pin: &str, out_pin: &str) -> bool;
    /// `Resizer::removeBuffer`'s edit: the nets merged, the instance destroyed, the estimator's
    /// cache following the odb callbacks. `Err` when the input is undriven (RSZ-0168).
    fn remove_buffer(&mut self, inst: &str, in_pin: &str, out_pin: &str) -> Result<RemovedBuffer, String>;
    /// `Resizer::swapPins(inst, port1, port2)`: the two pins trade nets, unless either has none or
    /// is on a dont_touch net; the estimator's cache follows (both nets invalid).
    fn swap_pins(&mut self, inst: &str, pin1: &str, pin2: &str) -> Result<PinSwap, String>;
    /// `Resizer::okToBufferNet(driver)`: not a tristate driver; its net exists, is not dont_touch,
    /// not connected by abutment, not special.
    fn ok_to_buffer_net(&self, drvr_pin: &str) -> bool;
    /// `Resizer::dontTouch(pin)`: its instance or its net is dont_touch.
    fn pin_dont_touch(&self, pin: &str) -> bool;
    /// The instance's odb id (the slot its `Instance*` addresses).
    fn inst_id(&self, inst: &str) -> u32;
    /// `CloneCandidate::applyClone`: `makeInstance(cell, "clone", …, loc)` (odb's name, TIMING
    /// source, placed and clamped to the core), its inputs on the driver's input nets in pin
    /// order, its output on a new net (`makeNet`: "net", always uniquified), the moved loads
    /// onto it. Returns the clone's name.
    fn clone_instance(&mut self, drvr_inst: &str, cell: &str, loc: (i32, i32), moved_loads: &[String]) -> Result<String, String>;
    /// `Resizer::insertBufferBeforeLoads(net, loads, cell, loc, base, "net", ALWAYS, diff_nets)`.
    #[allow(clippy::too_many_arguments)]
    fn insert_buffer_before_loads(&mut self, net: Option<&str>, loads: &[String], cell: &str, loc: (i32, i32), reason: &str, loads_on_diff_nets: bool, uniquify: &str) -> Result<crate::design::Repeater, String>;
}

/// `Resizer::repairHold` → `RepairHold::repairHold`: the hold buffer, then the passes over the
/// endpoints' hold violations, on the incremental timer — every timer read is a full update of
/// it (the reference's lazy reads, measured: one level-limited delay pass per inserted buffer,
/// with no edit before the full pass that follows it).
pub fn repair_hold(ctx: &Ctx<'_>, hctx: &crate::repair_hold::HoldCtx<'_>, design: &mut dyn SetupDesign, ha: &crate::repair_hold::HoldArgs) -> Result<crate::repair_hold::HoldOutcome, Stop> {
    let timer = Timer::new(design, 1)?;
    let initial_area = design.design_area();
    // `setMaxUtilization`: the core area times the fraction; 0 (no limit) without one.
    let max_area = ha.max_utilization.map_or(0.0, |u| design.core_area() * u);
    let mut h = HoldRepair { ctx, hctx, ha, design, timer, out: crate::repair_hold::HoldOutcome::default(), inserted: 0, resized: 0, initial_area, max_buffer_count: 0, design_area: initial_area, max_area };
    h.run()?;
    h.out.inserted = h.inserted;
    h.out.resized = h.resized;
    Ok(h.out)
}

/// `RepairHold`'s state over one call.
struct HoldRepair<'c, 'd> {
    ctx: &'c Ctx<'c>,
    hctx: &'c crate::repair_hold::HoldCtx<'c>,
    ha: &'c crate::repair_hold::HoldArgs,
    design: &'d mut dyn SetupDesign,
    timer: Timer,
    out: crate::repair_hold::HoldOutcome,
    /// `inserted_buffer_count_`, `resize_count_`.
    inserted: i64,
    resized: i64,
    initial_area: f64,
    max_buffer_count: i64,
    /// `design_area_`: recomputed by `Resizer::init` (the preamble, every `journalRestore`), else
    /// moved by `designAreaIncr(float)` — each master's area narrowed to `float` and added to the
    /// `double`.
    design_area: f64,
    /// `max_area_`.
    max_area: f64,
}

/// A fanout of a driver on a hold path, as `repairEndHold` reads it.
struct HoldFanout {
    pin: String,
    hold_slack: f32,
    /// `Sta::slacks`: `[rf][min/max]`, each from `INF`.
    slacks: [[f32; 2]; 2],
    /// An input (or bidirect) pin, or a top-level port.
    input_or_port: bool,
    /// A top-level output port.
    out_port: bool,
    /// Its liberty port's capacitance, 0 for a port.
    cap: f32,
}

/// The worst hold path to an endpoint, as `repairEndHold` copies it out.
struct HoldPath {
    slack: f32,
    /// `PathExpanded::size()`.
    len: usize,
    /// From `startIndex()` to the end: each pin and whether it is a driver.
    vertices: Vec<(String, bool)>,
}

const MIN: usize = 0;

impl HoldRepair<'_, '_> {
    fn debug(&mut self, group: &str, level: i64, line: String) {
        if self.ctx.debug.get(&("RSZ".to_string(), group.to_string())).is_some_and(|&l| l >= level) {
            self.out.trace.push(format!("[DEBUG RSZ-{group}] {line}"));
        }
    }

    fn report(&mut self, line: String) {
        self.out.trace.push(line.clone());
        self.out.lines.push(line);
    }

    fn ds(&self, v: f32) -> String {
        delay_as_string(v, 3, self.ctx.time_scale)
    }

    /// A read of the timer: the edits so far replayed, then a full update ([`timed`]).
    fn query<R>(&mut self, read: impl FnOnce(&Graph<'_>, &Search<'_, '_>, &BTreeSet<usize>) -> Result<R, Stop>) -> Result<R, Stop> {
        let edits = self.design.take_timer_edits();
        self.timer.trace_at = self.out.trace.len();
        timed(self.ctx, self.design.as_design(), &mut self.timer, edits, read)
    }

    /// The endpoints in vertex order with their min and max slacks (`findRequireds`, then
    /// `Sta::slack` of each).
    fn hold_ends(&mut self) -> Result<Vec<crate::repair_hold::HoldEnd>, Stop> {
        let ctx = self.ctx;
        self.query(|g, s, clocks| {
            let (ends, _) = timing_points(g, s, ctx.ssdc, ctx.libs, clocks).map_err(timer_stop)?;
            Ok(ends
                .iter()
                .map(|p| {
                    let v = g.vertices.iter().position(|x| x.name == p.pin).expect("an endpoint vertex");
                    crate::repair_hold::HoldEnd { pin: p.pin.clone(), hold_slack: s.slack_of(v, MIN, None), setup_slack: p.slack, is_clock: clocks.contains(&v) }
                })
                .collect())
        })
    }

    /// `Sta::worstSlack(max)`.
    fn setup_wns(&mut self) -> Result<f32, Stop> {
        let ctx = self.ctx;
        self.query(|g, s, clocks| {
            let (ends, _) = timing_points(g, s, ctx.ssdc, ctx.libs, clocks).map_err(timer_stop)?;
            Ok(worst_slack(&ends).0)
        })
    }

    /// `Sta::worstSlack(max)` and `Sta::slew(pin, riseFall, scenes, max)` (the larger of the max
    /// slews) together.
    fn setup_wns_and_slew(&mut self, pin: &str) -> Result<(f32, f32), Stop> {
        let ctx = self.ctx;
        let pin = pin.to_string();
        self.query(move |g, s, clocks| {
            let (ends, _) = timing_points(g, s, ctx.ssdc, ctx.libs, clocks).map_err(timer_stop)?;
            let v = g.vertices.iter().position(|x| x.name == pin).ok_or_else(|| timer_stop(format!("{pin}: not in the timing graph")))?;
            let slew = crate::rebuffer::std_max(g.slew[v][0][MAX], g.slew[v][1][MAX]);
            Ok((worst_slack(&ends).0, slew))
        })
    }

    /// `printProgress(iteration, force, end)`; the timer is read only when a row prints.
    fn progress(&mut self, iteration: i64, force: bool, end: bool) -> Result<(), Stop> {
        let prints = iteration == 0 || iteration % 10 == 0 || force || end;
        let ends = if prints { self.hold_ends()? } else { Vec::new() };
        let p = crate::repair_hold::Progress { resized: self.resized, inserted: self.inserted, cloned: 0, initial_area: self.initial_area, design_area: if prints { self.design.design_area() } else { self.initial_area } };
        crate::repair_hold::print_progress(self.hctx, &ends, &p, iteration, force, end, &mut self.out);
        Ok(())
    }

    /// `Resizer::overMaxArea`: a limit, and `fuzzyGreaterEqual` of the two (`float` arguments).
    fn over_max_area(&self) -> bool {
        self.max_area != 0.0 && {
            let (a, m) = (self.design_area as f32, self.max_area as f32);
            a > m || fuzzy::equal(a, m)
        }
    }

    /// `designAreaIncr(area(master))`.
    fn design_area_incr(&mut self, delta: f64) {
        self.design_area += f64::from(delta as f32);
    }

    /// `logger_->error`: the line, then the call ends (it throws).
    fn error(&mut self, code: &str, msg: &str) {
        self.report(format!("[ERROR {code}] {msg}"));
        self.out.error = Some(format!("{code}: {msg}"));
    }

    /// `RepairHold::repairHold(ends, buffer, …)`.
    fn run(&mut self) -> Result<(), Stop> {
        let buffer = crate::repair_hold::find_hold_buffer(self.hctx, &mut self.out)?.ok_or_else(|| Stop::refused("RSZ-HOLD", "no hold buffer with an area: not modelled".into()))?;
        let ends = self.hold_ends()?;
        // `max_buffer_percent * instanceCount()` (`float`), truncated; at least 100.
        let instances = self.design.as_design().netlist().insts.len();
        self.max_buffer_count = i64::from(((self.ha.max_buffer_percent * instances as f32) as i32).max(100));
        let margin = self.ha.hold_margin;
        let (mut worst, viol) = crate::repair_hold::find_hold_violations(self.hctx, &ends, margin, &mut self.out);
        let mut fails: Vec<String> = viol.iter().map(|&i| ends[i].pin.clone()).collect();
        self.out.violating = fails.len();
        if fails.is_empty() {
            self.report("[INFO RSZ-0033] No hold violations found.".into());
            return Ok(());
        }
        self.report(format!("[INFO RSZ-0046] Found {} endpoints with hold violations.", fails.len()));
        self.progress(0, true, false)?;
        let mut progress = true;
        let mut pass: i64 = 1;
        while f64::from(worst) < margin && progress && !self.over_max_area() && self.inserted <= self.max_buffer_count && pass <= self.ha.max_passes && (self.ha.max_iterations < 0 || pass <= self.ha.max_iterations) {
            if self.ha.verbose || pass == 1 {
                self.progress(pass, false, false)?;
            }
            let wns = self.setup_wns()?;
            self.debug("repair_hold", 1, format!("pass {} hold slack {} setup slack {}", pass, self.ds(worst), self.ds(wns)));
            let before = self.inserted;
            self.repair_hold_pass(&mut fails, &buffer, &mut pass)?;
            self.debug("repair_hold", 1, format!("inserted {}", self.inserted - before));
            let ends = self.hold_ends()?;
            let (w, viol) = crate::repair_hold::find_hold_violations(self.hctx, &ends, margin, &mut self.out);
            worst = w;
            fails = viol.iter().map(|&i| ends[i].pin.clone()).collect();
            progress = self.inserted > before;
        }
        self.progress(pass, true, true)?;
        // `fuzzyLess(float, float)`: the margin narrowed to `float` as the call passes it.
        if margin == 0.0 && fuzzy::less(worst, 0.0) {
            self.report("[WARNING RSZ-0066] Unable to repair all hold violations.".into());
        } else if fuzzy::less(worst, margin as f32) {
            self.report("[WARNING RSZ-0064] Unable to repair all hold checks within margin.".into());
        }
        if self.resized > 0 {
            self.report(format!("[INFO RSZ-0132] Resized {} instances.", self.resized));
        }
        if self.inserted > 0 {
            self.report(format!("[INFO RSZ-0032] Inserted {} hold buffers.", self.inserted));
        }
        if self.inserted > self.max_buffer_count {
            self.error("RSZ-0060", "Max buffer count reached.");
        } else if self.over_max_area() {
            self.error("RSZ-0050", "Max utilization reached.");
        }
        Ok(())
    }

    /// `repairHoldPass`: the estimates brought up to date, the failing endpoints sorted by hold
    /// slack (libc++ `std::sort`), each repaired in turn, `pass` counting them.
    fn repair_hold_pass(&mut self, fails: &mut [String], buffer: &str, pass: &mut i64) -> Result<(), Stop> {
        self.design.update_parasitics().map_err(|e| Stop::error("RSZ-EST", e))?;
        let ends = self.hold_ends()?;
        let slack: HashMap<String, f32> = ends.iter().map(|e| (e.pin.clone(), e.hold_slack)).collect();
        crate::order::libcxx_sort_by(fails, |a, b| slack[a] < slack[b]).map_err(|h| Stop::refused("RSZ-ORDER", format!("{} hold failures reach the sort's heap fallback, which is not modelled", h.len)))?;
        for end in fails.iter() {
            if self.ha.verbose {
                self.progress(*pass, false, false)?;
            }
            self.repair_end_hold(end, buffer)?;
            *pass += 1;
            if self.inserted > self.max_buffer_count {
                break;
            }
        }
        Ok(())
    }

    /// `vertexWorstSlackPath(end, min)` expanded (`PathExpanded`, from `startIndex`), with the
    /// end's setup slack.
    fn worst_hold_path(&mut self, end: &str) -> Result<Option<(HoldPath, f32)>, Stop> {
        let end = end.to_string();
        self.query(move |g, s, _| {
            let Some(v) = g.vertices.iter().position(|x| x.name == end) else { return Ok(None) };
            let mut worst = None;
            let mut min_slack = INF;
            for p in s.paths[v].iter().filter(|p| p.tag.mm == MIN) {
                let slack = p.arrival - p.required;
                if fuzzy::less(slack, min_slack) {
                    min_slack = slack;
                    worst = Some(*p);
                }
            }
            let Some(mut p) = worst else { return Ok(None) };
            let mut chain = vec![v];
            let mut start_from_end = None;
            let mut i = 0;
            while let Some(prev) = p.prev {
                if start_from_end.is_none() {
                    match edge_arc_set(g, prev.edge).map(|a| a.role) {
                        Some(Role::RegClkToQ | Role::LatchEnToQ) => start_from_end = Some(i),
                        Some(Role::LatchDtoQ) => return Err(Stop::refused("RSZ-LATCH", "a hold path through a latch D->Q arc: not modelled".into())),
                        _ => {}
                    }
                }
                p = *s.paths[prev.vertex].iter().find(|q| q.tag == prev.tag).ok_or_else(|| timer_stop(format!("{}: a path's prev path is not at its vertex", g.vertices[prev.vertex].name)))?;
                chain.push(prev.vertex);
                i += 1;
            }
            let n = chain.len();
            let start = n - 1 - start_from_end.unwrap_or(n - 1);
            chain.reverse();
            let vertices = chain[start..].iter().map(|&u| (g.vertices[u].name.clone(), g.vertices[u].is_driver)).collect();
            Ok(Some((HoldPath { slack: min_slack, len: n, vertices }, s.slack_of(v, MAX, None))))
        })
    }

    /// The driver's fanouts through its out edges (`RepairHoldPredicate`: every edge but timing
    /// checks and latch D→Q), with what `repairEndHold` reads of each.
    fn driver_fanouts(&mut self, drvr: &str) -> Result<Vec<HoldFanout>, Stop> {
        let drvr = drvr.to_string();
        let libs = self.ctx.libs;
        self.query(move |g, s, _| {
            let v = g.vertices.iter().position(|x| x.name == drvr).ok_or_else(|| timer_stop(format!("{drvr}: not in the timing graph")))?;
            let mut out = Vec::new();
            for &e in &g.out_edges[v] {
                if g.is_check(e) || matches!(edge_arc_set(g, e).map(|a| a.role), Some(Role::LatchDtoQ)) {
                    continue;
                }
                let to = g.edges[e].to;
                let vx = &g.vertices[to];
                let port = vx.lib.is_none();
                let mut slacks = [[INF; 2]; 2];
                for (rf, row) in slacks.iter_mut().enumerate() {
                    for (mm, cell) in row.iter_mut().enumerate() {
                        *cell = s.slack_of(to, mm, Some(rf));
                    }
                }
                let cell = vx.cell.as_deref().and_then(|c| libs.link_cell(c));
                let direction = cell.and_then(|c| vx.port.as_deref().and_then(|pn| c.port(pn))).map(|p| p.direction);
                let input_or_port = port || matches!(direction, Some(vyges_sta::liberty::Direction::Input | vyges_sta::liberty::Direction::Bidirect));
                let out_port = port && !vx.is_driver;
                let cap = match (cell, vx.port.as_deref()) {
                    (Some(c), Some(pn)) => crate::rebuffer::port_cap(c, pn),
                    _ => 0.0,
                };
                out.push(HoldFanout { pin: vx.name.clone(), hold_slack: s.slack_of(to, MIN, None), slacks, input_or_port, out_port, cap });
            }
            Ok(out)
        })
    }

    /// `repairEndHold`.
    fn repair_end_hold(&mut self, end: &str, buffer: &str) -> Result<(), Stop> {
        let Some((path, end_setup)) = self.worst_hold_path(end)? else { return Ok(()) };
        self.debug("repair_hold", 3, format!("repair end {} hold_slack={} setup_slack={}", end, self.ds(path.slack), self.ds(end_setup)));
        if path.len <= 1 {
            return Ok(());
        }
        let (sm, hm) = (self.ha.setup_margin, self.ha.hold_margin);
        for i in 0..path.vertices.len().saturating_sub(1) {
            let (pin, is_driver) = path.vertices[i].clone();
            if !is_driver || !self.design.ok_to_buffer_net(&pin) {
                continue;
            }
            let mut loads: Vec<String> = Vec::new();
            // mergeInit / mergeInto: min of the min slacks, max of the max slacks (`std::min`,
            // `std::max`).
            let mut slacks = [[INF, -INF], [INF, -INF]];
            let mut loads_have_out_port = false;
            let mut load_cap = 0.0f32;
            for f in self.driver_fanouts(&pin)? {
                if !f.input_or_port {
                    continue;
                }
                if f64::from(f.hold_slack) < hm {
                    for (merged, from) in slacks.iter_mut().zip(&f.slacks) {
                        merged[MIN] = crate::rebuffer::std_min(merged[MIN], from[MIN]);
                        merged[MAX] = crate::rebuffer::std_max(merged[MAX], from[MAX]);
                    }
                    if f.out_port {
                        loads_have_out_port = true;
                    }
                    load_cap += f.cap;
                    loads.push(f.pin);
                }
            }
            let _ = loads_have_out_port;
            if loads.is_empty() {
                continue;
            }
            self.debug("repair_hold", 3, format!(" {} hold_slack={}/{} setup_slack={}/{} fanouts={}", pin, self.ds(slacks[0][MIN]), self.ds(slacks[1][MIN]), self.ds(slacks[0][MAX]), self.ds(slacks[1][MAX]), loads.len()));
            let cell = self.ctx.libs.link_cell(buffer).ok_or_else(|| Stop::error("RSZ-HOLD", format!("{buffer}: not a liberty cell")))?;
            let (_, out) = cell.buffer_ports().ok_or_else(|| Stop::error("RSZ-HOLD", format!("{buffer}: not a buffer")))?;
            let (bd, _) = crate::rebuffer::gate_delays(cell, &out.name, load_cap, self.hctx.tgt_slews);
            // In `double`: `float` slacks less the `double` margins.
            let s = |rf: usize, mm: usize| f64::from(slacks[rf][mm]);
            let fits = self.ha.allow_setup_violations
                || (s(0, MAX) - sm > -(s(0, MIN) - hm) && s(1, MAX) - sm > -(s(1, MIN) - hm) && (s(0, MAX) - sm) > f64::from(bd[0]) && (s(1, MAX) - sm) > f64::from(bd[1]));
            if !fits {
                continue;
            }
            let path_load = path.vertices[i + 1].0.clone();
            let d = self.design.as_design();
            let (lx, ly) = d.pin_location(&path_load);
            let (dx, dy) = d.pin_location(&pin);
            let loc = ((dx + lx) / 2, (dy + ly) / 2);
            self.design.begin_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
            let (setup_before, slew_before) = self.setup_wns_and_slew(&pin)?;
            let (buffers, resizes) = self.make_hold_delay(&pin, &loads, buffer, loc)?;
            let (setup_after, slew_after) = self.setup_wns_and_slew(&pin)?;
            let slew_factor: f32 = if slew_before > 0.0 { slew_after / slew_before } else { 1.0 };
            if f64::from(slew_factor) > 1.20 || (!self.ha.allow_setup_violations && fuzzy::less(setup_after, setup_before) && f64::from(setup_after) < sm) {
                // `journalRestore`: undone; the estimates and the timer brought up to date (the
                // next read updates the timer).
                // `init()` first: the design area recomputed.
                self.design_area = self.design.design_area();
                if self.design.restore_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))? {
                    self.design.update_parasitics().map_err(|e| Stop::error("RSZ-EST", e))?;
                }
            } else {
                // `journalEnd`.
                self.design.update_parasitics().map_err(|e| Stop::error("RSZ-EST", e))?;
                self.design.commit_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
                self.inserted += buffers;
                self.resized += resizes;
            }
        }
        Ok(())
    }

    /// `makeHoldDelay`: a buffer before the loads (not dont_touch, in pin-id order) on the
    /// driver's net, named `hold<n>`; the estimates; a resize to the target slew when its output
    /// fails `checkMaxSlewCap`. Returns the buffers and resizes made.
    fn make_hold_delay(&mut self, drvr: &str, loads: &[String], buffer: &str, loc: (i32, i32)) -> Result<(i64, i64), Stop> {
        let info = self.design.net_info().clone();
        let mut set: Vec<String> = loads.iter().filter(|l| !self.design.pin_dont_touch(l)).cloned().collect();
        set.sort_by_key(|p| info.pin_id.get(p).copied().unwrap_or(u64::MAX));
        set.dedup();
        if set.is_empty() {
            return Ok((0, 0));
        }
        let d = self.design.as_design();
        let net = d.netlist().nets.iter().find(|n| n.pins.iter().any(|c| d.netlist().pin_name(c) == drvr)).map(|n| n.name.clone());
        let r = self.design.insert_buffer_before_loads(net.as_deref(), &set, buffer, loc, "hold", false, "ALWAYS").map_err(|_| Stop::error("RSZ-3009", format!("insert_buffer failed on drvr_pin '{drvr}'.")))?;
        // `insertBufferPostProcess`: the design area grows by the buffer.
        let a = self.design.master_area(buffer);
        self.design_area_incr(a);
        self.debug("repair_hold", 3, format!(" insert {}", r.inst));
        self.design.update_parasitics().map_err(|e| Stop::error("RSZ-EST", e))?;
        let mut resizes = 0;
        if !self.check_max_slew_cap(&r.output)? {
            let before = self.design.as_design().netlist().insts.iter().find(|(n, _)| n == &r.inst).map(|(_, c)| c.clone());
            if resize_to_target_slew(self.ctx, self.design, &r, buffer)? {
                // `replaceCell`: the old master's area out, the new one's in.
                let after = self.design.as_design().netlist().insts.iter().find(|(n, _)| n == &r.inst).map(|(_, c)| c.clone());
                if let (Some(old), Some(new)) = (before, after) {
                    let (ao, an) = (self.design.master_area(&old), self.design.master_area(&new));
                    self.design_area_incr(-ao);
                    self.design_area_incr(an);
                }
                self.design.update_parasitics().map_err(|e| Stop::error("RSZ-EST", e))?;
                resizes = 1;
            }
        }
        Ok((1, resizes))
    }

    /// `checkMaxSlewCap(drvr)`: the capacitance check, then the slew check, then the loads'
    /// slews — each `slack / limit` (`float`) at least 0.2, the first two only when a limit exists.
    fn check_max_slew_cap(&mut self, drvr: &str) -> Result<bool, Stop> {
        let drvr = drvr.to_string();
        let ctx = self.ctx;
        let info = self.design.net_info().clone();
        let parasitics = vec![self.design.as_design().parasitics(0).clone()];
        self.query(move |g, _, clocks| {
            let v = g.vertices.iter().position(|x| x.name == drvr).ok_or_else(|| timer_stop(format!("{drvr}: not in the timing graph")))?;
            let sc = crate::timing::Scenes::new(std::slice::from_ref(g));
            const RATIO: f32 = 0.2;
            let (_, limit, slack, _, corner) = crate::timing::check_capacitance(&sc, v, &parasitics, clocks);
            if corner.is_some() && slack / limit < RATIO {
                return Ok(false);
            }
            if let Some(c) = crate::timing::check_slew(&sc, v, ctx.limits, clocks) {
                if c.slack / c.limit < RATIO {
                    return Ok(false);
                }
            }
            let l = crate::timing::check_load_slews(&sc, &info, v, ctx.limits, clocks, 0.0);
            Ok(l.slack / l.limit >= RATIO)
        })
    }
}

/// `Resizer::resizeToTargetSlew(buffer output)` under placement parasitics: the new net's
/// parasitic ensured, its load at the target-slew corner, `findTargetCell`, a swap if it differs.
/// Returns whether it swapped.
fn resize_to_target_slew(ctx: &Ctx<'_>, design: &mut dyn SetupDesign, r: &crate::design::Repeater, cell: &str) -> Result<bool, Stop> {
    design.ensure_wire_parasitic(&r.out_net).map_err(|e| Stop::error("RSZ-EST", e))?;
    let d = design.as_design();
    let nl = d.netlist().clone();
    let k = ctx.sizing.tgt_scene;
    let par = d.parasitics(k).clone();
    let graph = crate::repair_design::timer_graph(ctx.libs, k, &nl, ctx.env, ctx.master_pins)?;
    let v = graph.vertices.iter().position(|x| x.name == r.output).ok_or_else(|| Stop::error("RSZ-INSERT", format!("{}: not in the timing graph", r.output)))?;
    let load_cap = graph.load_cap(v, &par);
    if load_cap > 0.0 {
        let target = ctx.sizing.find_target_cell(cell, load_cap, false)?;
        if target != cell {
            design.swap_master(&r.inst, &target).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
            return Ok(true);
        }
    }
    Ok(false)
}

/// `Optimizer::run` for the LEGACY phase: the phase, then the final report.
pub fn repair_setup(ctx: &Ctx<'_>, design: &mut dyn SetupDesign, args: &Args) -> Result<Outcome, Stop> {
    if args.max_utilization.is_some() {
        return Err(Stop::refused("RSZ-ABSENT", "repair_timing -max_utilization: not modelled".into()));
    }
    let mut timer = Timer::new(design, ctx.libs.scene_count())?;
    let timing = snapshot(ctx, design.as_design(), &[], &mut timer, TimerEdits::default())?;
    // RepairSetupContext: the area and TNS before any move.
    let initial_design_area = design.design_area();
    let initial_tns = timing.tns();
    let mut r = Repair {
        ctx,
        args,
        design,
        timing,
        phase: PHASE,
        sequence: ctx.sequence.to_vec(),
        timer,
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
    // The phases after LEGACY: LAST_GASP unless skipped (or -phases LEGACY), then the implicit
    // CRIT_VT_SWAP, which finds no VT cells (a VT library is refused) and does nothing.
    if !args.skip_last_gasp && args.phases.is_none() {
        r.last_gasp()?;
    }
    r.finalize_and_report()?;
    r.out.resized = r.committer.committed(Move::SizeUp);
    r.out.removed = r.committer.committed(Move::Unbuffer);
    r.out.inserted = r.committer.committed(Move::Buffer);
    Ok(r.out)
}

impl Repair<'_, '_> {
    fn report(&mut self, line: String) {
        self.out.trace.push(line.clone());
        self.out.lines.push(line);
    }

    /// `debugPrint(logger, RSZ, group, level, …)`: traced when the group's level reaches it.
    fn debug(&mut self, group: &str, level: i64, line: String) {
        if self.ctx.debug.get(&("RSZ".to_string(), group.to_string())).is_some_and(|&l| l >= level) {
            self.out.trace.push(format!("[DEBUG RSZ-{group}] {line}"));
        }
    }

    fn ds(&self, v: f32, digits: usize) -> String {
        delay_as_string(v, digits, self.ctx.time_scale)
    }

    /// The timer again over the design (`updateParasitics`, `findRequireds`), with the worst path
    /// of `want` ready.
    fn retime(&mut self, want: &[String]) -> Result<(), Stop> {
        self.design.update_parasitics().map_err(timer_stop)?;
        let edits = self.design.take_timer_edits();
        self.timer.trace_at = self.out.trace.len();
        self.timing = snapshot(self.ctx, self.design.as_design(), want, &mut self.timer, edits)?;
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
            self.debug("repair_setup", 1, format!("{PHASE} Phase: No violating endpoints, exiting"));
            return Ok(violating_ends);
        }
        self.debug("repair_setup", 1, format!("{PHASE} Phase: {} violating endpoints found", violating_ends.len()));
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
            let Some(mut es) = self.begin_endpoint_repair(&end.pin)? else { break };
            self.repair_endpoint(&mut es)?;
            if self.args.verbose || self.opto_iteration == 1 {
                self.print_progress(self.opto_iteration, true)?;
            }
            if self.two_cons_terminations {
                self.debug("repair_setup", 1, format!("{PHASE} Phase: Exiting due to no TNS progress for two opto cycles"));
                break;
            }
        }
        self.print_progress(self.opto_iteration, true)?;
        let (wns, _) = self.timing.worst();
        let tns = self.timing.tns();
        let line = format!("{PHASE} Phase complete. WNS: {}, TNS: {}", self.ds(wns, 3), self.ds(tns, 1));
        self.debug("repair_setup", 1, line);
        Ok(())
    }

    /// `beginEndpointRepair` → `beginJournaledEndpointSearch`.
    fn begin_endpoint_repair(&mut self, end: &str) -> Result<Option<EndpointState>, Stop> {
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
            return Ok(None);
        }
        self.begin_journal()?;
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
        self.debug("repair_setup", 1, line);
        es.prev_end_slack = es.end_slack;
        es.prev_worst_slack = es.worst_slack;
        Ok(Some(es))
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
                self.debug("repair_setup", 2, line);
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
                self.debug("repair_setup", 1, line);
                self.finish_endpoint_search(es)?;
                break;
            }
            let prev_tns_local = self.timing.tns();
            let end = es.end.clone();
            let changed = self.repair_path(&end, es.end_slack, es.force_single_repair)?;
            if !changed {
                if es.pass != 1 {
                    self.debug("repair_setup", 2, format!("{PHASE} Phase: No change after {} decreasing slack passes.", es.decreasing_slack_passes));
                    let line = format!("{PHASE} Phase: Restoring best slack; endpoint slack = {}, WNS = {}", self.ds(es.prev_end_slack, 3), self.ds(es.prev_worst_slack, 3));
                    self.debug("repair_setup", 2, line);
                }
                self.debug("repair_setup", 1, format!("{PHASE} Phase: No change possible for endpoint {} ", es.end));
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
            self.debug("repair_setup", 3, line);
            if better {
                if !fuzzy::less(es.end_slack, margin) {
                    self.num_viols -= 1;
                }
                es.prev_end_slack = es.end_slack;
                es.prev_worst_slack = es.worst_slack;
                es.decreasing_slack_passes = 0;
                self.save_improved_checkpoint(es)?;
            } else {
                es.force_single_repair = true;
                es.decreasing_slack_passes += 1;
                if es.decreasing_slack_passes > DECREASING_SLACK_MAX_PASSES {
                    let line = format!("{PHASE} Phase: Endpoint {} stuck after {} non-improving passes (limit {DECREASING_SLACK_MAX_PASSES})", es.end, es.decreasing_slack_passes);
                    self.debug("repair_setup", 2, line);
                    let line = format!("{PHASE} Phase: Restoring best slack; endpoint slack = {}, WNS = {}", self.ds(es.prev_end_slack, 3), self.ds(es.prev_worst_slack, 3));
                    self.debug("repair_setup", 2, line);
                    self.restore_endpoint_state(es)?;
                    break;
                }
                self.debug("repair_setup", 3, format!("{PHASE} Phase: Allowing decreasing slack for {}/{DECREASING_SLACK_MAX_PASSES} passes", es.decreasing_slack_passes));
            }
            // overMaxArea: -max_utilization is refused, so never.
            if self.end_index == 1 {
                if let Some(w) = es.worst_vertex.clone() {
                    es.end = w;
                }
            }
            es.pass += 1;
            if self.args.max_iterations > 0 && self.opto_iteration >= self.args.max_iterations {
                self.accept_endpoint_state(es)?;
                break;
            }
        }
        self.accept_endpoint_state(es)?;
        Ok(())
    }

    /// `SetupLastGaspPolicy::iterate`: the sequence narrowed to VtSwap, SizeUpMatch, SizeUp and
    /// SwapPins; the violating endpoints again; per endpoint up to 10 passes, a pass kept only
    /// when WNS and TNS both hold (fuzzily), else the journal restored.
    fn last_gasp(&mut self) -> Result<(), Stop> {
        self.phase = "LAST_GASP+";
        let phase = self.phase;
        // initializeLastGaspRepair.
        self.sequence.clear();
        if !self.args.skip_vt_swap {
            self.sequence.push(Move::VtSwap);
        }
        self.sequence.push(Move::SizeUpMatch);
        self.sequence.push(Move::SizeUp);
        if !self.args.skip_pin_swap {
            self.sequence.push(Move::SwapPins);
        }
        let violating_ends = collect_violating(&self.timing.ends, self.ctx.margin);
        self.collector_violating = violating_ends.len();
        self.num_viols = violating_ends.len() as i64;
        let curr_tns = self.timing.tns();
        if !fuzzy::less(curr_tns, 0.0) {
            self.debug("repair_setup", 1, format!("{phase} Phase: TNS is {}, exiting", self.ds(curr_tns, 1)));
            return Ok(());
        }
        if violating_ends.is_empty() {
            self.debug("repair_setup", 1, format!("{phase} Phase: No violating endpoints found"));
            return Ok(());
        }
        self.max_end_count = violating_ends.len() as i64;
        self.end_index = 0;
        self.prev_tns = curr_tns;
        let mut prev_worst_slack = violating_ends[0].slack;
        self.fix_rate_threshold = INC_FIX_RATE_THRESHOLD;
        self.prev_termination = false;
        self.two_cons_terminations = false;
        self.debug("repair_setup", 1, format!("{phase} Phase: {} violating endpoints remain", self.max_end_count));
        self.print_progress(self.opto_iteration, false)?;
        // runLastGaspLoop.
        for end in &violating_ends {
            if self.last_gasp_should_stop() {
                break;
            }
            // beginLastGaspEndpoint → beginJournaledEndpointSearch.
            let mut es = EndpointState {
                end: end.pin.clone(),
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
                self.debug("repair_setup", 1, format!("{phase} Phase: Hit maximum endpoint repairs of {}", self.max_end_count));
                break;
            }
            self.begin_journal()?;
            es.journal_open = true;
            es.prev_end_slack = es.end_slack;
            es.prev_worst_slack = es.worst_slack;
            let line = format!("{phase} Phase: Doing endpoint {} ({}/{}) endpoint slack = {}, WNS = {}", es.end, self.end_index, self.max_end_count, self.ds(es.end_slack, 3), self.ds(es.worst_slack, 3));
            self.debug("repair_setup", 1, line);
            self.repair_last_gasp_endpoint(&mut es, &mut prev_worst_slack)?;
            if self.args.verbose || self.opto_iteration == 1 {
                self.print_progress(self.opto_iteration, true)?;
            }
            if self.last_gasp_should_stop() {
                self.debug("repair_setup", 1, format!("{phase} Phase: No TNS progress for two opto cycles, exiting"));
                break;
            }
        }
        let (wns, _) = self.timing.worst();
        let tns = self.timing.tns();
        let line = format!("{phase} Phase complete. WNS: {}, TNS: {}", self.ds(wns, 3), self.ds(tns, 1));
        self.debug("repair_setup", 1, line);
        Ok(())
    }

    /// `shouldStopLastGasp`.
    fn last_gasp_should_stop(&self) -> bool {
        self.two_cons_terminations || (self.args.max_iterations > 0 && self.opto_iteration >= self.args.max_iterations)
    }

    /// `repairLastGaspEndpoint` with `advanceLastGaspProgress`.
    fn repair_last_gasp_endpoint(&mut self, es: &mut EndpointState, prev_worst_slack: &mut f32) -> Result<(), Stop> {
        let phase = self.phase;
        let margin = self.ctx.margin;
        while es.pass <= 10 {
            self.opto_iteration += 1;
            if self.terminate_progress() {
                if self.prev_termination {
                    self.two_cons_terminations = true;
                } else {
                    self.prev_termination = true;
                }
                self.accept_endpoint_state(es)?;
                break;
            }
            if self.opto_iteration % OPTO_SMALL_INTERVAL == 0 {
                self.prev_termination = false;
            }
            if self.args.verbose || self.opto_iteration == 1 {
                self.print_progress(self.opto_iteration, false)?;
            }
            if !fuzzy::less(es.end_slack, margin) {
                self.num_viols -= 1;
                self.accept_endpoint_state(es)?;
                break;
            }
            let end = es.end.clone();
            let changed = self.repair_path(&end, es.end_slack, false)?;
            if !changed {
                self.finish_endpoint_search(es)?;
                break;
            }
            self.retime(std::slice::from_ref(&es.end))?;
            self.refresh_endpoint_slacks(es);
            let curr_tns = self.timing.tns();
            // advanceLastGaspProgress.
            let improved = !fuzzy::less(es.worst_slack, *prev_worst_slack) && !fuzzy::less(curr_tns, self.prev_tns);
            if !improved {
                let line = format!(
                    "{phase} Phase: Move rejected for endpoint {} pass {} because WNS worsened {} -> {} and TNS worsened {} -> {}",
                    self.end_index,
                    es.pass,
                    self.ds(*prev_worst_slack, 3),
                    self.ds(es.worst_slack, 3),
                    self.ds(self.prev_tns, 1),
                    self.ds(curr_tns, 1)
                );
                self.debug("repair_setup", 2, line);
                self.restore_endpoint_state(es)?;
                break;
            }
            let line = format!(
                "{phase} Phase: Move accepted for endpoint {} pass {} because WNS improved {} -> {} and TNS improved {} -> {}",
                self.end_index,
                es.pass,
                self.ds(*prev_worst_slack, 3),
                self.ds(es.worst_slack, 3),
                self.ds(self.prev_tns, 1),
                self.ds(curr_tns, 1)
            );
            self.debug("repair_setup", 2, line);
            *prev_worst_slack = es.worst_slack;
            self.prev_tns = curr_tns;
            if !fuzzy::less(es.end_slack, margin) {
                self.num_viols -= 1;
                self.accept_endpoint_state(es)?;
                break;
            }
            self.save_improved_checkpoint(es)?;
            if self.end_index == 1 {
                if let Some(w) = es.worst_vertex.clone() {
                    es.end = w;
                }
            }
            es.pass += 1;
            if self.args.max_iterations > 0 && self.opto_iteration >= self.args.max_iterations {
                self.accept_endpoint_state(es)?;
                break;
            }
        }
        self.accept_endpoint_state(es)
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
                    "{} Phase: Exiting at iteration {iteration} because incr fix rate {:.2}% is < {:.2}% [endpt {}/{}]",
                    self.phase,
                    inc_fix_rate * 100.0,
                    self.fix_rate_threshold * 100.0,
                    self.end_index,
                    self.max_end_count
                );
                self.debug("repair_setup", 1, line);
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
            let edits = self.design.take_timer_edits();
            self.timer.trace_at = self.out.trace.len();
            self.timing = snapshot(self.ctx, self.design.as_design(), &[end.to_string()], &mut self.timer, edits)?;
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
        self.debug("repair_setup", 3, line);
        let mut changed = 0i64;
        for (index, _) in ranked {
            if changed >= repairs_per_pass {
                break;
            }
            self.try_repair_path_target(&view, index, &mut changed, repairs_per_pass)?;
        }
        Ok(changed > 0)
    }

    /// `tryRepairPathTarget` → `logRepairTarget`, then `tryRepairTarget`: each generator of the
    /// move sequence in order, until one's candidate is committed (`tryCandidateSequence`).
    fn try_repair_path_target(&mut self, view: &PathView, index: usize, changed: &mut i64, repairs_per_pass: i64) -> Result<bool, Stop> {
        let st = &view.stages[index];
        let line = format!("{} {} fanout = {} drvr_index = {index}", st.pin, st.cell.as_deref().unwrap_or("none"), st.fanout);
        self.debug("repair_setup", 3, line);
        for m in self.sequence.clone() {
            if !self.is_applicable(m, st, index) {
                continue;
            }
            self.debug("repair_setup", 1, format!("Considering {} for {}", m.name(), st.pin));
            let result = match m {
                Move::SizeUp => self.size_up_move(view, index)?,
                Move::Unbuffer => self.unbuffer_move(view, index)?,
                Move::SwapPins => self.swap_pins_move(view, index)?,
                Move::Buffer => self.buffer_move(view, index)?,
                Move::Clone => self.clone_move(view, index)?,
                Move::SplitLoad => self.split_load_move(view, index)?,
                Move::SizeUpMatch => self.size_up_match_move(view, index)?,
                // VtSwapGenerator: a candidate needs two VT categories, and a VT library is
                // refused before the repair — so it never has one.
                Move::VtSwap => None,
                other => return Err(Stop::refused("RSZ-ABSENT", format!("{}: not modelled", other.name()))),
            };
            if let Some(r) = result {
                // repairProgressIncrement: an unbuffer spends the pass's whole budget.
                *changed += if r.kind == Move::Unbuffer { repairs_per_pass } else { 1 };
                self.commit(r);
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// `MoveGenerator::isApplicable` on a path driver target: SwapPins also needs a path index
    /// above 0; Buffer a fanout of 2 to 19 on a net it may buffer (`okToBufferNet`); SizeUpMatch a
    /// path index of 2 or more.
    fn is_applicable(&self, m: Move, st: &Stage, index: usize) -> bool {
        match m {
            Move::Buffer => st.fanout > 1 && st.fanout < rebuffer::REBUFFER_MAX_FANOUT && self.design.ok_to_buffer_net(&st.pin),
            Move::SizeUpMatch => index >= 2,
            _ => true,
        }
    }

    /// CloneGenerator (`resolveDriverTarget` with its rejections, the pending Buffer / SplitLoad
    /// guards, the moved loads, the half-driving cell, the centroid) → `CloneCandidate::applyClone`.
    fn clone_move(&mut self, view: &PathView, index: usize) -> Result<Option<MoveResult>, Stop> {
        let st = &view.stages[index];
        if st.fanout <= clone::CLONE_MIN_FANOUT {
            self.debug("clone_move", 2, format!("REJECT CloneMove {}: Fanout {} <= {} min fanout", st.pin, st.fanout, clone::CLONE_MIN_FANOUT));
            return Ok(None);
        }
        if !self.design.ok_to_buffer_net(&st.pin) {
            self.debug("clone_move", 2, format!("REJECT CloneMove {}: Not OK to buffer net", st.pin));
            return Ok(None);
        }
        let (Some(inst), Some(cell_name)) = (&st.inst, &st.cell) else { return Ok(None) };
        let Some(cell) = self.ctx.libs.link_cell(cell_name) else { return Ok(None) };
        if !clone::is_single_output_combinational(cell).map_err(|e| Stop::refused("RSZ-ABSENT", e))? {
            self.debug("clone_move", 2, format!("REJECT CloneMove {}: Not single output combinational", st.pin));
            return Ok(None);
        }
        let id = self.design.inst_id(inst);
        if self.committer.pending_insts.get(&Move::Buffer).is_some_and(|l| l.contains(&id)) {
            self.debug("clone_move", 2, format!("REJECT CloneMove {}: Has pending BufferMove", st.pin));
            return Ok(None);
        }
        if self.committer.pending_insts.get(&Move::SplitLoad).is_some_and(|l| l.contains(&id)) {
            self.debug("clone_move", 2, format!("REJECT CloneMove {}: Has pending SplitLoadMove", st.pin));
            return Ok(None);
        }
        let Some(fanouts) = st.fanout_slacks.clone() else { return Ok(None) };
        if fanouts.is_empty() {
            return Ok(None);
        }
        let info = self.design.net_info().clone();
        let moved = clone::select_moved_loads(&fanouts, &|i| info.dont_touch_insts.contains(i));
        if moved.is_empty() {
            return Ok(None);
        }
        let swappable = self.ctx.sizing.swappable_cells(cell_name)?;
        let cands: Vec<&Cell> = swappable.iter().filter_map(|n| self.ctx.libs.link_cell(n)).collect();
        let dont_use = |n: &str| self.ctx.sizing.dont_use.contains(n);
        let clone_cell = clone::choose_clone_cell(cell, &cands, &dont_use).map_err(|e| Stop::refused("RSZ-ABSENT", e))?;
        let d = self.design.as_design();
        let loc = clone::compute_clone_location(d.pin_location(&st.pin), &fanouts, &|p| d.pin_location(p));
        let clone_inst = self.design.clone_instance(inst, &clone_cell, loc, &moved).map_err(|e| Stop::error("RSZ-CLONE", e))?;
        self.debug("clone_move", 1, format!("ACCEPT CloneMove {}: ({cell_name}) -> {clone_inst} ({clone_cell})", st.pin));
        Ok(Some(MoveResult { kind: Move::Clone, count: 1, insts: vec![clone_inst, inst.clone()] }))
    }

    /// SplitLoadGenerator (`resolveDriverPin` with its rejections, the first half of the ranked
    /// fanouts but top-level ports, the lowest-drive buffer at the driver) →
    /// `SplitLoadCandidate::applySplitBuffer` (inserted before those loads, "split", IF_NEEDED;
    /// then `resizeToTargetSlew` on its output).
    fn split_load_move(&mut self, view: &PathView, index: usize) -> Result<Option<MoveResult>, Stop> {
        let st = &view.stages[index];
        if st.fanout <= clone::CLONE_MIN_FANOUT {
            self.debug("split_load_move", 2, format!("REJECT SplitLoadMove {}: Fanout {} <= {} min fanout", st.pin, st.fanout, clone::CLONE_MIN_FANOUT));
            return Ok(None);
        }
        if !self.design.ok_to_buffer_net(&st.pin) {
            self.debug("split_load_move", 2, format!("REJECT SplitLoadMove {}: Not OK to buffer net", st.pin));
            return Ok(None);
        }
        let Some(fanouts) = st.split_slacks.clone() else { return Ok(None) };
        if fanouts.is_empty() {
            return Ok(None);
        }
        // chooseSplitLoads: a PinSet (pin-id order).
        let info = self.design.net_info().clone();
        let mut loads: Vec<String> = fanouts[..fanouts.len() / 2].iter().filter(|f| !f.top_port).map(|f| f.pin.clone()).collect();
        loads.sort_by_key(|p| info.pin_id.get(p).copied().unwrap_or(u64::MAX));
        loads.dedup();
        if loads.is_empty() {
            return Ok(None);
        }
        let d = self.design.as_design();
        let Some(net) = d.netlist().nets.iter().find(|n| n.pins.iter().any(|c| d.netlist().pin_name(c) == st.pin)).map(|n| n.name.clone()) else { return Ok(None) };
        let loc = d.pin_location(&st.pin);
        let cell = self.ctx.lowest_buffer.to_string();
        let rep = match self.design.insert_buffer_before_loads(Some(&net), &loads, &cell, loc, "split", false, "IF_NEEDED") {
            Ok(r) => r,
            Err(_) => {
                self.debug("split_load_move", 2, format!("REJECT SplitLoadMove {}: Couldn't insert buffer", st.pin));
                return Ok(None);
            }
        };
        self.debug("split_load_move", 1, format!("ACCEPT SplitLoadMove {}: Inserted buffer {}", st.pin, rep.inst));
        self.resize_to_target_slew(&rep, &cell)?;
        Ok(Some(MoveResult { kind: Move::SplitLoad, count: 1, insts: vec![rep.inst] }))
    }

    fn resize_to_target_slew(&mut self, r: &crate::design::Repeater, cell: &str) -> Result<(), Stop> {
        resize_to_target_slew(self.ctx, self.design, r, cell).map(|_| ())
    }

    /// BufferGenerator → `BufferCandidate::apply`: `rebufferPin` (the passes in
    /// [`rebuffer::Rebuf::rebuffer_pin`], then `exportBufferTree`), accepted when it inserted
    /// anything.
    fn buffer_move(&mut self, view: &PathView, index: usize) -> Result<Option<MoveResult>, Stop> {
        let st = &view.stages[index];
        let count = match st.rebuffer.clone() {
            None | Some(RebufProbe::Skip) => 0,
            Some(RebufProbe::Warn(w)) => {
                self.report(w);
                0
            }
            Some(RebufProbe::Ready(probe)) => self.rebuffer_pin(&probe)?,
        };
        if count <= 0 {
            self.debug("buffer_move", 2, format!("REJECT BufferMove {}: Couldn't insert any buffers", st.pin));
            return Ok(None);
        }
        self.debug("buffer_move", 1, format!("ACCEPT BufferMove {}: Inserted {count} buffers", st.pin));
        let inst = st.inst.clone().unwrap_or_default();
        Ok(Some(MoveResult { kind: Move::Buffer, count, insts: vec![inst] }))
    }

    /// `Rebuffer::rebufferPin` with the net annotated: the passes, then the export.
    fn rebuffer_pin(&mut self, probe: &rebuffer::Probe) -> Result<i64, Stop> {
        let rctx = self.ctx.rebuffer.ok_or_else(|| Stop::refused("RSZ-ABSENT", "BufferMove without characterized buffers".into()))?;
        self.debug("rebuffer", 2, format!("driver {}", probe.pin));
        for w in &probe.warnings {
            self.report(w.clone());
        }
        let mut rb = rebuffer::Rebuf::new(rctx, probe);
        let chosen = rb.rebuffer_pin();
        for (level, line) in std::mem::take(&mut rb.trace) {
            match level {
                Some(l) => self.debug("rebuffer", l, line),
                None => self.report(line),
            }
        }
        if let Some(r) = rb.refused.take() {
            return Err(Stop::refused("RSZ-ORDER", r));
        }
        let chosen = chosen.map_err(|e| Stop::error("RSZ-REBUFFER", e))?;
        let Some(root) = chosen else { return Ok(0) };
        let net = self.design.as_design().netlist().nets.iter().find(|n| n.pins.iter().any(|c| self.design.as_design().netlist().pin_name(c) == probe.pin)).map(|n| n.name.clone()).ok_or_else(|| Stop::error("RSZ-REBUFFER", format!("{}: no net", probe.pin)))?;
        let mut count = 0i64;
        let mut inputs: HashMap<usize, String> = HashMap::new();
        let mut loads = Vec::new();
        self.export_buffer_tree(&rb, root, &net, &mut loads, &mut count, &mut inputs)?;
        self.debug("rebuffer", 2, "-------------------------------".into());
        Ok(count)
    }

    /// `exportBufferTree`'s bottom-up insertion: a load (not dont_touch) is a terminal of the
    /// parent; a buffer is inserted before its subtree's loads (on the ORIGINAL driver net, loads
    /// on other nets allowed) and its input becomes the parent's load.
    fn export_buffer_tree(&mut self, rb: &rebuffer::Rebuf<'_>, n: usize, net: &str, current: &mut Vec<String>, count: &mut i64, inputs: &mut HashMap<usize, String>) -> Result<(), Stop> {
        match rb.nodes[n].kind.clone() {
            rebuffer::Kind::Wire { r } => self.export_buffer_tree(rb, r, net, current, count, inputs),
            rebuffer::Kind::Junction { r, r2 } => {
                self.export_buffer_tree(rb, r, net, current, count, inputs)?;
                self.export_buffer_tree(rb, r2, net, current, count, inputs)
            }
            rebuffer::Kind::Load { pin } => {
                if self.design.pin_dont_touch(&pin) {
                    self.debug("rebuffer", 3, format!("exportBufferTree: skipped load {pin} due to dont touch"));
                } else if !current.contains(&pin) {
                    current.push(pin);
                }
                Ok(())
            }
            rebuffer::Kind::Buffer { cell, r } => {
                let mut child = Vec::new();
                self.export_buffer_tree(rb, r, net, &mut child, count, inputs)?;
                if child.is_empty() {
                    return Ok(());
                }
                // A PinSet: pin-id order.
                let id = |p: &String, info: &crate::timing::NetInfo| info.pin_id.get(p).copied().unwrap_or(u64::MAX);
                let info = self.design.net_info().clone();
                child.sort_by_key(|p| id(p, &info));
                let at = (rb.nodes[n].x, rb.nodes[n].y);
                let rep = self.design.insert_buffer_before_loads(Some(net), &child, &cell, at, "rebuffer", true, "ALWAYS").map_err(|e| Stop::error("RSZ-INSERT", e))?;
                *count += 1;
                self.debug("rebuffer", 3, format!("insert {} ({cell}) -> {} loads", rep.inst, child.len()));
                for p in &child {
                    self.debug("rebuffer", 3, format!("  load pin: {p}"));
                }
                inputs.insert(n, rep.input.clone());
                if !current.contains(&rep.input) {
                    current.push(rep.input);
                }
                Ok(())
            }
        }
    }

    /// SizeUpMatchGenerator (`resolveDriverTarget`, the previous driver with ONE wire fanout, a
    /// same-family stronger cell there) → `SizeUpMatchCandidate::apply`: the driver takes the
    /// previous driver's cell, if no fanin net's max capacitance suffers.
    fn size_up_match_move(&mut self, view: &PathView, index: usize) -> Result<Option<MoveResult>, Stop> {
        let st = &view.stages[index];
        let Some(inst) = &st.inst else {
            self.debug("size_up_match_move", 2, format!("REJECT SizeUpMatchMove {}: No driver instance", st.pin));
            return Ok(None);
        };
        if self.design.net_info().dont_touch_insts.contains(inst) {
            self.debug("size_up_match_move", 2, format!("REJECT SizeUpMatchMove {}: {inst} is \"don't touch\"", st.pin));
            return Ok(None);
        }
        let cell = st.cell.clone().unwrap_or_default();
        if !self.ctx.sizing.masters.get(&cell).is_some_and(|m| m.logic_std) {
            self.debug("size_up_match_move", 2, format!("REJECT SizeUpMatchMove {}: {inst} isn't logic std cell", st.pin));
            return Ok(None);
        }
        let Some(curr) = self.ctx.libs.link_cell(&cell) else {
            self.debug("size_up_match_move", 2, format!("REJECT SizeUpMatchMove {}: No liberty cell found for {inst}", st.pin));
            return Ok(None);
        };
        // loadPreviousDriverPin: the path two stages back.
        let Some(prev) = index.checked_sub(2).map(|k| &view.stages[k]) else {
            self.debug("size_up_match_move", 2, format!("REJECT SizeUpMatchMove {}: No previous driver pin", st.pin));
            return Ok(None);
        };
        // hasSingleStageFanout: its wire fanout — counted only until it passes 1, so a rejection
        // always reports 2.
        if prev.fanout != 1 {
            let line = if prev.fanout > 1 { format!("REJECT SizeUpMatchMove {}: Previous driver fanout 2 > 1", st.pin) } else { format!("REJECT SizeUpMatchMove {}: No previous driver vertex", st.pin) };
            self.debug("size_up_match_move", 2, line);
            return Ok(None);
        }
        // selectReplacement: the previous driver's cell when it is the same family (buffer or
        // inverter) and drives stronger (`bufferDriveResistance`).
        let Some(prev_cell) = prev.cell.as_deref().filter(|c| *c != cell).and_then(|c| self.ctx.libs.link_cell(c)) else { return Ok(None) };
        let same_family = (prev_cell.is_buffer() && curr.is_buffer()) || (prev_cell.is_inverter() && curr.is_inverter());
        let r = |c: &Cell| c.buffer_ports().map_or(0.0, |(_, o)| c.drive_resistance(&o.name));
        if !same_family || r(prev_cell) >= r(curr) {
            return Ok(None);
        }
        let to = prev_cell.name.clone();
        if !replacement_preserves_max_cap(self.ctx.libs, &cell, &to, &st.fanin_caps) {
            self.debug("size_up_match_move", 2, format!("REJECT SizeUpMatchMove {}: Couldn't replace {cell} -> {to}", st.pin));
            return Ok(None);
        }
        self.design.swap_master(inst, &to).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        self.debug("size_up_match_move", 1, format!("ACCEPT SizeUpMatchMove {}: Replaced {cell} -> {to}", st.pin));
        Ok(Some(MoveResult { kind: Move::SizeUpMatch, count: 1, insts: vec![inst.clone()] }))
    }

    /// `MoveCommitter::commit` of an accepted candidate: recorded, and kept in the open level.
    fn commit(&mut self, r: MoveResult) {
        let ids: Vec<u32> = r.insts.iter().map(|i| self.design.inst_id(i)).collect();
        self.committer.record(&r, &ids);
        if let Some(level) = self.committer.levels.last_mut() {
            level.push((r, ids));
        }
    }

    /// SizeUpGenerator → SizeUpCandidate::apply (`Resizer::replaceCell`).
    fn size_up_move(&mut self, view: &PathView, index: usize) -> Result<Option<MoveResult>, Stop> {
        let st = &view.stages[index];
        let Some(m) = size_up(self.ctx, self.design.as_design(), view, index)? else { return Ok(None) };
        self.design.swap_master(&m.inst, &m.to).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        self.debug("size_up_move", 1, format!("ACCEPT SizeUpMove {}: {} -> {}", st.pin, m.from, m.to));
        Ok(Some(MoveResult { kind: Move::SizeUp, count: 1, insts: vec![m.inst] }))
    }

    /// `UnbufferGenerator::generate` (`isEligible`, its guards in order) → `UnbufferCandidate::apply`
    /// (`Resizer::removeBuffer`).
    fn unbuffer_move(&mut self, view: &PathView, index: usize) -> Result<Option<MoveResult>, Stop> {
        let st = &view.stages[index];
        self.debug("unbuffer_move", 4, format!("checking unbuffer eligibility for {}", st.pin));
        let r = match &st.unbuffer {
            None => return Ok(None),
            Some(unbuffer::Verdict::NoDriverContext) => {
                self.debug("unbuffer_move", 4, format!("buffer target {} is not removable because driver context did not resolve", st.pin));
                return Ok(None);
            }
            Some(unbuffer::Verdict::NoPathContext) => {
                let inst = st.inst.clone().unwrap_or_default();
                self.debug("unbuffer_move", 4, format!("buffer {} is not removed because path context did not resolve", network_name(&inst, self.ctx.hierarchy)));
                return Ok(None);
            }
            Some(unbuffer::Verdict::Resolved(r)) => r.clone(),
        };
        if let Some(why) = self.committer.blocking_buffer_removal(self.design.inst_id(&r.inst)) {
            self.debug("unbuffer_move", 4, format!("buffer {} is not removed because {why}", network_name(&r.inst, self.ctx.hierarchy)));
            return Ok(None);
        }
        // passesFanoutGuard, then passesCapGuard.
        if let Some(reject) = r.fanout_reject.as_ref().or(r.cap_reject.as_ref()) {
            self.debug("unbuffer_move", 2, reject.clone());
            return Ok(None);
        }
        for l in &r.slack_lines {
            self.debug("remove_buffer", 1, l.clone());
        }
        if !r.slack_ok {
            self.debug("unbuffer_move", 4, format!("buffer {} is not removed because estimated slack is not OK", network_name(&r.inst, self.ctx.hierarchy)));
            return Ok(None);
        }
        if !self.can_remove_buffer(&r) {
            self.debug("unbuffer_move", 4, format!("buffer {} is not removed because canRemoveBuffer rejected it", network_name(&r.inst, self.ctx.hierarchy)));
            return Ok(None);
        }
        // removeBuffer.
        self.debug("repair_setup", 3, format!("remove_buffer {} ({})", r.inst, r.cell));
        let removed = self.design.remove_buffer(&r.inst, &r.in_port, &r.out_port).map_err(|e| Stop::error("RSZ-0168", e))?;
        // `name(net)`, `pathName(buffer)`, `name(net)`.
        let h = self.ctx.hierarchy;
        self.debug("remove_buffer", 1, format!("remove_buffer {} (input net) - {} ({}) - {} (output net)", network_name(&removed.in_net, h), r.inst, r.cell, network_name(&removed.out_net, h)));
        Ok(Some(MoveResult { kind: Move::Unbuffer, count: 1, insts: vec![r.inst] }))
    }

    /// `SwapPinsGenerator::generate` (`resolveDriverContext`: a liberty driver port of a cell that
    /// is neither buffer nor inverter; the dont_touch and already-swapped guards; `loadInputPort`;
    /// [`swap_pins::select_swap_port`]), the estimate (legal when strictly faster), then
    /// `SwapPinsCandidate::apply` → `Resizer::swapPins`.
    fn swap_pins_move(&mut self, view: &PathView, index: usize) -> Result<Option<MoveResult>, Stop> {
        let st = &view.stages[index];
        let (Some(inst), Some(cell_name), Some(drvr_port)) = (&st.inst, &st.cell, &st.port) else { return Ok(None) };
        let Some(cell) = self.ctx.libs.link_cell(cell_name).filter(|c| c.port(drvr_port).is_some()) else { return Ok(None) };
        if cell.is_buffer() || cell.is_inverter() {
            return Ok(None);
        }
        if self.design.net_info().dont_touch_insts.contains(inst) {
            self.debug("swap_pins_move", 2, format!("REJECT SwapPinsMove {}: {inst} is \"don't touch\"", st.pin));
            return Ok(None);
        }
        if self.committer.has_moves(Move::SwapPins, self.design.inst_id(inst)) {
            self.debug("swap_pins_move", 2, format!("REJECT SwapPinsMove {}: Already swapped {inst}", st.pin));
            return Ok(None);
        }
        // loadInputPort: the arc's from-port, not an output.
        let Some(input_port) = st.in_port.as_deref().filter(|p| cell.port(p).is_some_and(|q| q.direction != vyges_sta::liberty::Direction::Output)) else { return Ok(None) };
        let tgt = self.ctx.sizing.tgt_slews;
        let in_slew = |port: &str, rf: usize| st.in_slews.iter().find(|(p, _)| p == port).map_or(tgt[rf], |(_, s)| s[rf]);
        // The target's scene (`activeScene`): the stage's load and input slews are read there
        // (the path's scene), and each arc's delay model is that scene's.
        let scene_cell = self.ctx.libs.scene_cell(view.scene, cell_name).unwrap_or(cell);
        let picked = swap_pins::select_swap_port(cell, scene_cell, drvr_port, input_port, st.load_cap, &in_slew).map_err(|e| Stop::refused("RSZ-ABSENT", e))?;
        let Some((swap_port, current_delay, swap_delay)) = picked else { return Ok(None) };
        // SwapPinsCandidate::estimate: legal when the swap is faster. (Redundant after a strict
        // selection — a different port is only ever strictly faster — so a mutant relaxing it to
        // `< 0` is equivalent and survives; kept as the reference has it.)
        if current_delay - swap_delay <= 0.0 {
            return Ok(None);
        }
        self.debug("swap_pins_move", 1, format!("ACCEPT SwapPinsMove {}: Cell {cell_name}, pins {input_port} <-> {swap_port}", st.pin));
        match self.design.swap_pins(inst, input_port, &swap_port).map_err(|e| Stop::error("RSZ-SWAP", e))? {
            PinSwap::DontTouch(net) => {
                self.debug("swap_pins_move", 2, format!("REJECT SwapPinsMove {inst}: Net {net} is \"don't touch\""));
                Ok(None)
            }
            PinSwap::Swapped | PinSwap::NoNet => Ok(Some(MoveResult { kind: Move::SwapPins, count: 1, insts: vec![inst.clone()] })),
        }
    }

    /// `Resizer::canRemoveBuffer(buffer, true)`: a logic standard cell that is a buffer (the probe
    /// resolved a buffer), then the database's checks. No SDC command names an instance pin, an
    /// instance or a net here (`isConstrained`): those commands are refused when the SDC is read.
    fn can_remove_buffer(&self, r: &unbuffer::Resolved) -> bool {
        self.ctx.sizing.masters.get(&r.cell).is_some_and(|m| m.logic_std) && self.design.can_remove_buffer(&r.inst, &r.in_port, &r.out_port)
    }

    /// `MoveCommitter::beginJournal`.
    fn begin_journal(&mut self) -> Result<(), Stop> {
        self.design.begin_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        self.committer.levels.push(Vec::new());
        Ok(())
    }

    /// `MoveCommitter::commitJournal` (its `updateParasiticsAndTiming` finds nothing invalid: the
    /// pass re-timed after its edits): the level merged into its parent, or at the outermost the
    /// pending moves committed.
    fn commit_journal(&mut self) -> Result<(), Stop> {
        let Some(top) = self.committer.levels.pop() else { return Ok(()) };
        self.design.commit_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        match self.committer.levels.last_mut() {
            Some(parent) => parent.extend(top),
            None => self.committer.accept_pending(),
        }
        Ok(())
    }

    /// `MoveCommitter::restoreJournal`: this level's edits undone and its moves unrecorded; then
    /// the timer again when the level had any edit.
    fn restore_journal(&mut self, want: &[String]) -> Result<(), Stop> {
        let Some(top) = self.committer.levels.pop() else { return Ok(()) };
        let had_changes = self.design.restore_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        for (r, ids) in &top {
            self.committer.unrecord(r, ids);
        }
        if had_changes {
            self.retime(want)?;
        }
        Ok(())
    }

    /// `acceptEndpointState`.
    fn accept_endpoint_state(&mut self, es: &mut EndpointState) -> Result<(), Stop> {
        if es.journal_open {
            self.commit_journal()?;
            es.journal_open = false;
        }
        Ok(())
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
            self.accept_endpoint_state(es)
        } else {
            self.restore_endpoint_state(es)
        }
    }

    /// `saveImprovedCheckpoint`.
    fn save_improved_checkpoint(&mut self, es: &mut EndpointState) -> Result<(), Stop> {
        self.accept_endpoint_state(es)?;
        self.begin_journal()?;
        es.journal_open = true;
        Ok(())
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
        let field = format!("{iteration}{}", self.phase.chars().last().unwrap_or('*'));
        let row = progress_row(
            &Row {
                iter: &field,
                removed: self.committer.total(Move::Unbuffer),
                resized: self.committer.total(Move::SizeUp) + self.committer.total(Move::SizeUpMatch),
                inserted: self.committer.total(Move::Buffer) + self.committer.total(Move::SplitLoad),
                cloned: self.committer.total(Move::Clone),
                swaps: self.committer.total(Move::SwapPins),
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
                removed: self.committer.total(Move::Unbuffer),
                resized: self.committer.total(Move::SizeUp) + self.committer.total(Move::SizeUpMatch),
                inserted: self.committer.total(Move::Buffer) + self.committer.total(Move::SplitLoad),
                cloned: self.committer.total(Move::Clone),
                swaps: self.committer.total(Move::SwapPins),
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
        let unbuffer = self.committer.committed(Move::Unbuffer);
        if unbuffer > 0 {
            self.report(format!("[INFO RSZ-0059] Removed {unbuffer} buffers."));
        }
        let (buffers, splits) = (self.committer.committed(Move::Buffer), self.committer.committed(Move::SplitLoad));
        if buffers > 0 || splits > 0 {
            if splits == 0 {
                self.report(format!("[INFO RSZ-0040] Inserted {buffers} buffers."));
            } else {
                self.report(format!("[INFO RSZ-0045] Inserted {} buffers, {splits} to split loads.", buffers + splits));
            }
        }
        let (size_up, up_match) = (self.committer.committed(Move::SizeUp), self.committer.committed(Move::SizeUpMatch));
        if size_up + up_match > 0 {
            self.report(format!("[INFO RSZ-0051] Resized {} instances: {size_up} up, {up_match} up match, 0 down, 0 VT", size_up + up_match));
        }
        let swaps = self.committer.committed(Move::SwapPins);
        if swaps > 0 {
            self.report(format!("[INFO RSZ-0043] Swapped pins on {swaps} instances."));
        }
        let clones = self.committer.committed(Move::Clone);
        if clones > 0 {
            self.report(format!("[INFO RSZ-0049] Cloned {clones} instances."));
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
            rf: 0,
            unbuffer: None,
            in_slews: Vec::new(),
            rebuffer: None,
            fanout_slacks: None,
            split_slacks: None,
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
            scene: 0,
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
        f(&Sizing { libs: l, masters: &masters, dont_use: &dont_use, equiv: &equiv, target_loads: &loads, tgt_slews: [0.0; 2], tgt_scene: 0, limits: Default::default() })
    }

    // Rule (weakerCellFirst): cells with the driver port first, the LARGER drive resistance
    // first; cells without the port after them, by name.
    #[test]
    fn swappable_cells_are_tried_weakest_first() {
        let l = libs(&[buf("B4", 0.01, 0.1, 0.2, 1.0), buf("B1", 0.01, 0.1, 0.5, 4.0), buf("B2", 0.01, 0.1, 0.3, 2.0)]);
        let mut cells = vec!["B4".to_string(), "B2".into(), "NOPE".into(), "B1".into()];
        crate::order::libcxx_sort_by(&mut cells, |a, b| weaker_cell_first(&l, 0, a, b, "Z")).unwrap();
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
            assert_eq!(upsize_cell(s, 0, "A", "B1", "Z", 0.5 * unit, 0.0).unwrap().as_deref(), Some("B2"));
            // A previous driver of 1 (time per unit cap) pays for B2's input cap: B4 wins.
            let r = l.libs[0].cells["B1"].drive_resistance("Z") / 4.0;
            assert_eq!(upsize_cell(s, 0, "A", "B1", "Z", 0.5 * unit, r).unwrap().as_deref(), Some("B4"));
            // The strongest cell has nothing stronger.
            assert_eq!(upsize_cell(s, 0, "A", "B4", "Z", 0.5 * unit, 0.0).unwrap(), None);
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
