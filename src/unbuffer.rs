// SPDX-License-Identifier: Apache-2.0
//! UnbufferMove: a buffer on the repair path removed when its previous driver can take its loads.
//!
//! One function per stage, named after the reference's and in its call order:
//! - [`probe`] — what `UnbufferGenerator::isEligible` reads from the timer, taken while the path is
//!   expanded (nothing the generator reads changes before a move commits, and an accepted removal
//!   ends the pass): `resolveDriverContext`, `loadExpandedPath`, then [`passes_fanout_guard`],
//!   [`passes_cap_guard`] and [`estimated_slack_ok`] (`passesSlackGuard`);
//! - [`estimated_slack_ok`] — `Resizer::estimatedSlackOK`: [`compute_new_delays_slews`], the max
//!   capacitance check, [`estimate_slews_after_buffer_removal`] and [`estimate_input_slew_impact`]
//!   over the buffer's loads, then over the previous driver's side loads;
//! - [`estimate_slews_after_buffer_removal`] — both nets' buffered nets, [`stitch_trees`], the
//!   calibrated estimate ([`estimate_slews_in_tree`]).
//!
//! The move-conflict guard (`hasBlockingBufferRemovalMove`) and `canRemoveBuffer` read the
//! committer and the database, so the sequencer evaluates them when the target is tried, in the
//! generator's order around the timer verdicts kept here.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use vyges_sta::fuzzy;
use vyges_sta::graph::{Graph, NetParasitics};
use vyges_sta::liberty::{Cell, Model, MAX};
use vyges_sta::search::Search;

use crate::buffered_net::{self, BufferedNet, Kind};
use crate::repair_timing::{fmt_float, network_name};
use crate::timing::{self, Limits, NetInfo, INF};

/// `kBufferRemovalMaxFanout`.
const BUFFER_REMOVAL_MAX_FANOUT: f32 = 10.0;
/// `kSlackDegradRatioLimit`.
const SLACK_DEGRAD_RATIO_LIMIT: f32 = 0.1;

/// The reference's pin addresses (a capture's `post-1.pinaddr`): per odb table page, its base
/// address and the slot stride. A dbTable slot never moves, so a terminal's address is its page's
/// base plus its slot — for every terminal the run ever had, from the ones alive at its end.
#[derive(Debug, Clone, Default)]
pub struct PinAddr {
    /// (is a block terminal, page) → (base, stride).
    pages: HashMap<(bool, u64), (u64, u64)>,
}

impl PinAddr {
    /// `I|B id address-hex-little-endian` lines. Instance terminals live in 1024-slot pages, block
    /// terminals in 128-slot pages (odb's dbTable page sizes).
    pub fn parse(text: &str) -> Result<PinAddr, String> {
        let mut seen: HashMap<(bool, u64), Vec<(u64, u64)>> = HashMap::new();
        // `P I|B page 0x…`: a page base the instrumented reference printed when it allocated it.
        let mut page_bases: HashMap<(bool, u64), u64> = HashMap::new();
        for l in text.lines() {
            let w: Vec<&str> = l.split_whitespace().collect();
            if let ["P", kind, page, addr] = w.as_slice() {
                let page: u64 = page.parse().map_err(|_| format!("pinaddr: bad page {page}"))?;
                let base = u64::from_str_radix(addr.trim_start_matches("0x"), 16).map_err(|_| format!("pinaddr: bad page address {addr}"))?;
                page_bases.insert((*kind == "B", page), base);
                continue;
            }
            let [kind, id, hex] = w.as_slice() else { continue };
            let port = *kind == "B";
            let id: u64 = id.parse().map_err(|_| format!("pinaddr: bad id {id}"))?;
            let bytes: Vec<u8> = (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16)).collect::<Result<_, _>>().map_err(|_| format!("pinaddr: bad address {hex}"))?;
            let addr = bytes.iter().rev().fold(0u64, |a, &b| (a << 8) | u64::from(b));
            let shift = if port { 7 } else { 10 };
            seen.entry((port, id >> shift)).or_default().push((id & ((1 << shift) - 1), addr));
        }
        // The stride: from two slots of one page (the same in every page of a table).
        let mut stride: HashMap<bool, u64> = HashMap::new();
        for ((port, _), slots) in &seen {
            if let [(i1, a1), (i2, a2), ..] = slots.as_slice() {
                if i2 != i1 {
                    stride.entry(*port).or_insert((a2.wrapping_sub(*a1)) / (i2.wrapping_sub(*i1)));
                }
            }
        }
        // Else from a printed page base and one terminal past its slot 0: a table whose only live
        // terminals share one page gives no two-slot stride (one block port is common).
        for ((port, page), slots) in &seen {
            if let (None, Some(&base)) = (stride.get(port), page_bases.get(&(*port, *page))) {
                if let Some(&(i, a)) = slots.iter().find(|(i, _)| *i > 0) {
                    stride.insert(*port, (a.wrapping_sub(base)) / i);
                }
            }
        }
        let mut pages = HashMap::new();
        for ((port, page), slots) in seen {
            let s = stride.get(&port).copied().unwrap_or(1);
            let (i, a) = slots[0];
            pages.insert((port, page), (a - i * s, s));
        }
        for ((port, page), base) in page_bases {
            let s = stride.get(&port).copied().unwrap_or(1);
            if let Some(&(known, _)) = pages.get(&(port, page)) {
                if known != base {
                    return Err(format!("pinaddr: page {page} base {base:#x} disagrees with its terminals' {known:#x}"));
                }
            }
            pages.insert((port, page), (base, s));
        }
        Ok(PinAddr { pages })
    }

    /// A terminal's address by its `NetInfo::pin_id` (ITerm id × 2, BTerm id × 2 + 1); `None`
    /// when its page had no live terminal at the end.
    pub fn addr(&self, pin_id: u64) -> Option<u64> {
        let (port, id) = (pin_id & 1 == 1, pin_id >> 1);
        let shift = if port { 7 } else { 10 };
        let (base, stride) = self.pages.get(&(port, id >> shift))?;
        Some(base + (id & ((1 << shift) - 1)) * stride)
    }
}

/// What the probe reads besides the timer.
pub struct Ctx<'a> {
    pub libs: &'a crate::preamble::Libs,
    pub limits: &'a Limits,
    pub sdc: &'a vyges_sta::graph::SdcEnv,
    pub dbu: i32,
    pub wire_rc: buffered_net::WireRc,
    pub slew_shape_factor: f32,
    /// The target slews (`tgt_slews_`), rise and fall.
    pub tgt_slews: [f32; 2],
    /// `-setup_margin`, seconds.
    pub margin: f32,
    /// Each master's signal terminals in the database's order (an instance's pin iterator).
    pub master_pins: &'a HashMap<String, Vec<String>>,
    /// The reference's pin addresses, when the gate supplies them: a `map<const Pin*>` loop then
    /// runs in their order (debug order only — the verdict is every load's).
    pub pin_addr: Option<&'a PinAddr>,
    /// `dbNetwork::hasHierarchy`: the buffer prints by `name()`, without its parent prefix.
    pub hierarchy: bool,
}

/// The timer the probe reads: the graph, its search, the parasitics, the ideal clock pins.
pub struct Timer<'t, 'g> {
    pub g: &'t Graph<'g>,
    pub search: &'t Search<'t, 'g>,
    pub parasitics: &'t HashMap<String, NetParasitics>,
    pub ideal: &'t BTreeSet<usize>,
    pub info: &'t NetInfo,
    /// Vertex by pin name.
    pub index: &'t HashMap<String, usize>,
    /// `est::makeSteinerTree(drvr_pin)` for a net and its driver.
    pub steiner: &'t dyn Fn(&str, &str) -> Option<buffered_net::Tree>,
    /// Several corners: every scene's timer (`g` … `index` are the path's scene).
    pub multi: Option<&'t Multi<'t, 'g>>,
}

/// Every scene's timer, for what the reference reads over all scenes (`Sta::slack(vertex, max)`,
/// `checkCapacitance(pin, scenes())`), and to time the slack guard at its own corner.
pub struct Multi<'t, 'g> {
    pub graphs: &'t [Graph<'g>],
    pub searches: &'t [Search<'t, 'g>],
    pub parasitics: &'t [HashMap<String, NetParasitics>],
    pub ideals: &'t [BTreeSet<usize>],
    /// Each scene's vertex by pin name.
    pub indices: &'t [HashMap<String, usize>],
}

impl<'t, 'g> Timer<'t, 'g> {
    /// The same timer at scene `k` (several corners).
    fn at_scene(&self, k: usize) -> Timer<'t, 'g> {
        let m = self.multi.expect("several scenes");
        Timer { g: &m.graphs[k], search: &m.searches[k], parasitics: &m.parasitics[k], ideal: &m.ideals[k], info: self.info, index: &m.indices[k], steiner: self.steiner, multi: self.multi }
    }
}

/// `Sta::slack(vertex, max)`: over every scene's paths, the fuzzily least (the earlier scene on a
/// tie); one scene, that scene's.
fn slack_all(t: &Timer<'_, '_>, v: usize) -> f32 {
    let Some(m) = t.multi else { return t.search.vertex_slack(v) };
    let name = &t.g.vertices[v].name;
    let mut best: Option<f32> = None;
    for (k, search) in m.searches.iter().enumerate() {
        if let Some(&u) = m.indices[k].get(name) {
            let s = search.vertex_slack(u);
            if best.is_none_or(|b| fuzzy::less(s, b)) {
                best = Some(s);
            }
        }
    }
    best.unwrap_or(INF)
}

/// `Sta::checkCapacitance(pin, scenes(), max)`: over every scene (in scene 0's vertices), or the
/// one scene. `(cap, limit, slack, limited, scene)`.
fn check_capacitance_all(t: &Timer<'_, '_>, v: usize) -> (f32, f32, f32, bool, Option<usize>) {
    match t.multi {
        None => {
            let sc = timing::Scenes::new(std::slice::from_ref(t.g));
            timing::check_capacitance(&sc, v, std::slice::from_ref(t.parasitics), t.ideal)
        }
        Some(m) => {
            let sc = timing::Scenes::new(m.graphs);
            let Some(&v0) = m.indices[0].get(&t.g.vertices[v].name) else { return (0.0, 0.0, 0.0, false, None) };
            timing::check_capacitance(&sc, v0, m.parasitics, &m.ideals[0])
        }
    }
}

/// What `isEligible` found from the timer, in its order.
#[derive(Debug, Clone, PartialEq)]
pub enum Verdict {
    /// `resolveDriverContext` failed (not an instance pin of a buffer).
    NoDriverContext,
    /// `loadExpandedPath` failed.
    NoPathContext,
    /// The context resolved: the buffer, the pins, and each timer guard's outcome.
    Resolved(Resolved),
}

#[derive(Debug, Clone, PartialEq)]
pub struct Resolved {
    pub inst: String,
    pub cell: String,
    pub in_port: String,
    pub out_port: String,
    pub prev_pin: String,
    /// The fanout guard's rejection line (`unbuffer_move` level 2), if it rejects.
    pub fanout_reject: Option<String>,
    /// The capacitance guard's rejection line (level 2), if it rejects.
    pub cap_reject: Option<String>,
    /// `estimatedSlackOK`'s `remove_buffer` debug lines (level 1) and its verdict.
    pub slack_lines: Vec<String>,
    pub slack_ok: bool,
}

/// A stage of the expanded path as the probe reads it.
pub struct PathStage<'s> {
    pub vertex: usize,
    pub rf: usize,
    pub inst: Option<&'s str>,
    pub cell: Option<&'s str>,
    pub in_port: Option<&'s str>,
}

/// `UnbufferGenerator::isEligible`'s timer half on the driver at `index` of the path — `None`
/// when `validTarget` fails (a silent rejection). `fanout`: the target's wire fanout.
pub fn probe(ctx: &Ctx<'_>, t: &Timer<'_, '_>, stages: &[PathStage<'_>], index: usize, fanout: usize) -> Option<Verdict> {
    // validTarget: one upstream driver stage and one buffer stage on the path.
    if index < 2 {
        return None;
    }
    let st = &stages[index];
    // resolveDriverContext: an instance pin of a liberty buffer.
    let (Some(inst), Some(cell_name)) = (st.inst, st.cell) else { return Some(Verdict::NoDriverContext) };
    let Some(cell) = ctx.libs.link_cell(cell_name).filter(|c| c.is_buffer()) else { return Some(Verdict::NoDriverContext) };
    // loadExpandedPath: the arc into the driver, the previous driver's path.
    let Some(in_port) = st.in_port else { return Some(Verdict::NoPathContext) };
    let prev = &stages[index - 2];
    let (_, out) = cell.buffer_ports()?;
    let mut r = Resolved {
        inst: inst.to_string(),
        cell: cell_name.to_string(),
        in_port: in_port.to_string(),
        out_port: out.name.clone(),
        prev_pin: t.g.vertices[prev.vertex].name.clone(),
        fanout_reject: None,
        cap_reject: None,
        slack_lines: Vec::new(),
        slack_ok: false,
    };
    let buffer_in = t.index.get(&format!("{inst}/{in_port}")).copied();
    let Some(buffer_in) = buffer_in else { return Some(Verdict::NoPathContext) };
    r.fanout_reject = passes_fanout_guard(ctx, t, inst, prev.vertex, fanout);
    let (cap_reject, slack_scene) = passes_cap_guard(ctx, t, cell, cell_name, inst, prev.vertex, st.vertex);
    r.cap_reject = cap_reject;
    let p = SlackParams { driver: st.vertex, driver_rf: st.rf, prev: prev.vertex, prev_rf: prev.rf, driver_input: buffer_in, inst, cell };
    let (ok, lines) = match (t.multi, slack_scene) {
        // Several corners: the slack guard times everything at the capacitance guard's scene
        // (`passesCapGuard` sets `slack_scene`), the buffer's ports there (`scenePort`).
        (Some(_), Some(k)) => {
            let tk = t.at_scene(k);
            let at = |v: usize| tk.index.get(&t.g.vertices[v].name).copied();
            match (at(p.driver), at(p.prev), at(p.driver_input), ctx.libs.scene_cell(k, cell_name)) {
                (Some(driver), Some(prev_k), Some(driver_input), Some(scene_cell)) => {
                    let pk = SlackParams { driver, prev: prev_k, driver_input, cell: scene_cell, ..p };
                    estimated_slack_ok(ctx, &tk, &pk, slack_scene)
                }
                _ => (false, Vec::new()),
            }
        }
        _ => estimated_slack_ok(ctx, t, &p, slack_scene),
    };
    r.slack_ok = ok;
    r.slack_lines = lines;
    Some(Verdict::Resolved(r))
}

/// `passesFanoutGuard`: the previous driver's fanout after it takes the buffer's loads, against
/// the timer's limit when it has one (`0 < limit < INF`), else against 10. The timer reports
/// fanout 0 when there is no limit, so then only the buffer's fanout counts.
fn passes_fanout_guard(ctx: &Ctx<'_>, t: &Timer<'_, '_>, inst: &str, prev: usize, target_fanout: usize) -> Option<String> {
    let (fanout, limit, _) = timing::sta_check_fanout(t.g, t.info, prev, ctx.limits, t.ideal);
    let prev_name = &t.g.vertices[prev].name;
    match fanout_rejects(fanout, limit, target_fanout) {
        None => None,
        Some(FanoutLimit::Timer(l)) => Some(format!("buffer {} is not removed because of max fanout limit of {} at {prev_name}", network_name(inst, ctx.hierarchy), fmt_float(l))),
        Some(FanoutLimit::Default) => Some(format!("buffer {} is not removed because of default fanout limit of {} at {prev_name}", network_name(inst, ctx.hierarchy), BUFFER_REMOVAL_MAX_FANOUT as i32)),
    }
}

/// Which fanout limit a removal would break.
#[derive(Debug, PartialEq)]
enum FanoutLimit {
    Timer(f32),
    Default,
}

/// The fanout guard's rule: the previous driver's fanout, less the buffer's input, plus the
/// buffer's loads — within the timer's limit when it has one, else within 10.
fn fanout_rejects(fanout: f32, limit: f32, target_fanout: usize) -> Option<FanoutLimit> {
    let new_fanout = fanout + target_fanout as f32 - 1.0;
    if limit > 0.0 && limit < INF {
        return (new_fanout > limit).then_some(FanoutLimit::Timer(limit));
    }
    (new_fanout > BUFFER_REMOVAL_MAX_FANOUT).then_some(FanoutLimit::Default)
}

/// `passesCapGuard`: the previous driver's load plus the buffer's, less the buffer's input, within
/// the previous driver's max capacitance. Also returns the scene the check found
/// (`slack_scene`): none when the driver has no limit — and then the slack guard refuses.
fn passes_cap_guard(ctx: &Ctx<'_>, t: &Timer<'_, '_>, cell: &Cell, cell_name: &str, inst: &str, prev: usize, drvr: usize) -> (Option<String>, Option<usize>) {
    let (cap, max_cap, _, _, scene) = check_capacitance_all(t, prev);
    if max_cap <= 0.0 || scene.is_none() {
        return (None, scene);
    }
    let (input, _) = cell.buffer_ports().expect("a buffer");
    // `loadCap(drvr, corner)` and `portCapacitance(buffer input, corner)`: at the check's scene.
    let (drvr_cap, in_cap) = match (t.multi, scene) {
        (Some(_), Some(k)) => {
            let tk = t.at_scene(k);
            let d = tk.index.get(&t.g.vertices[drvr].name).map_or(0.0, |&u| tk.g.load_cap(u, tk.parasitics));
            (d, ctx.libs.scene_cell(k, cell_name).map_or(0.0, |c| port_capacitance(c, &input.name)))
        }
        _ => (t.g.load_cap(drvr, t.parasitics), port_capacitance(cell, &input.name)),
    };
    let new_cap = cap + drvr_cap - in_cap;
    if new_cap <= max_cap {
        return (None, scene);
    }
    let line = format!("buffer {} is not removed because of max cap limit of {} at {}", network_name(inst, ctx.hierarchy), fmt_float(max_cap), t.g.vertices[prev].name);
    (Some(line), scene)
}

/// `LibertyPort::capacitance()`: the largest of its rise/fall, min/max values.
fn port_capacitance(cell: &Cell, port: &str) -> f32 {
    cell.port(port).map_or(0.0, |p| p.capacitance.iter().flatten().fold(f32::MIN, |a, &b| a.max(b)))
}

/// `SlackEstimatorParams` as `passesSlackGuard` fills it.
struct SlackParams<'p> {
    driver: usize,
    driver_rf: usize,
    prev: usize,
    prev_rf: usize,
    driver_input: usize,
    inst: &'p str,
    cell: &'p Cell,
}

/// The cell of a vertex's instance, at the graph's scene.
fn vertex_cell<'g>(g: &'g Graph<'_>, v: usize) -> Option<&'g Cell> {
    let vx = &g.vertices[v];
    g.libs[vx.lib?].cells.get(vx.cell.as_deref()?)
}

/// `Resizer::gateDelays(drvr_port, load_cap, …)`: over the non-check arcs into the port, each at
/// its input transition's slew — an annotated input pin's own (`annotateInputSlews`), else the
/// target slew — the larger delay and slew per output transition (from −INF).
fn gate_delays(cell: &Cell, port: &str, load_cap: f32, in_slew: &dyn Fn(&str, usize) -> f32) -> ([f32; 2], [f32; 2]) {
    let mut delays = [-INF; 2];
    let mut slews = [-INF; 2];
    for set in cell.arc_sets.iter().filter(|s| s.to == port && !s.role.is_timing_check()) {
        for arc in &set.arcs {
            if let Model::Gate(m) = &arc.model {
                let (delay, slew) = m.gate_delay(in_slew(&set.from, arc.from_rf), load_cap);
                delays[arc.to_rf] = delays[arc.to_rf].max(delay);
                slews[arc.to_rf] = slews[arc.to_rf].max(slew);
            }
        }
    }
    (delays, slews)
}

/// A diagnostic line (`VYGES_RSZ_UNBUF_RAW`), set against the reference's instrumented build.
fn raw_dump(line: String) {
    if let Ok(path) = std::env::var("VYGES_RSZ_UNBUF_RAW") {
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = f.write_all(line.as_bytes());
        }
    }
}

fn bits(f: f32) -> String {
    format!("{:08x}", f.to_bits())
}

/// `Resizer::computeNewDelaysSlews(prev_pin, buffer)`: the previous driver's delays and slews at
/// its load now and with the buffer's load moved onto it (less the buffer's input capacitance),
/// its instance's input pins at their timed slews. `None` when the previous driver has no liberty
/// port (a top-level port).
#[allow(clippy::type_complexity)]
fn compute_new_delays_slews(ctx: &Ctx<'_>, t: &Timer<'_, '_>, p: &SlackParams<'_>) -> Option<([f32; 2], [f32; 2], [f32; 2], [f32; 2], f32, f32)> {
    let g = t.g;
    let vx = &g.vertices[p.prev];
    let prev_cell = vertex_cell(g, p.prev)?;
    let prev_port = vx.port.as_deref()?;
    let (input, output) = p.cell.buffer_ports()?;
    let buffer_out = *t.index.get(&format!("{}/{}", p.inst, output.name))?;
    // annotateInputSlews(instance(prev_pin)): each input pin's slew, rise and fall.
    let prev_inst = vx.name.rsplit_once('/').map(|(i, _)| i).unwrap_or("");
    let mut annotated: HashMap<&str, [f32; 2]> = HashMap::new();
    if let Some(pins) = vx.cell.as_deref().and_then(|c| ctx.master_pins.get(c)) {
        for pin in pins {
            if let Some(&u) = t.index.get(&format!("{prev_inst}/{pin}")) {
                if !g.vertices[u].is_driver {
                    annotated.insert(pin.as_str(), [g.slew[u][0][MAX], g.slew[u][1][MAX]]);
                }
            }
        }
    }
    for (port, s) in &annotated {
        raw_dump(format!("VYGA|{}|{port}|{},{}\n", vx.name, bits(s[0]), bits(s[1])));
    }
    let in_slew = |from: &str, rf: usize| annotated.get(from).map_or(ctx.tgt_slews[rf], |s| s[rf]);
    let old_load_cap = g.load_cap(p.prev, t.parasitics);
    let (old_delay, old_slew) = gate_delays(prev_cell, prev_port, old_load_cap, &in_slew);
    let new_load_cap = old_load_cap + g.load_cap(buffer_out, t.parasitics) - port_capacitance(p.cell, &input.name);
    let (new_delay, new_slew) = gate_delays(prev_cell, prev_port, new_load_cap, &in_slew);
    Some((old_delay, new_delay, old_slew, new_slew, old_load_cap, new_load_cap))
}

/// `Resizer::checkMaxCapOK(drvr, cap_delta)`: with a limit, a driver already over it may not
/// grow, else it must stay within it.
fn check_max_cap_ok(t: &Timer<'_, '_>, drvr: usize, cap_delta: f32) -> bool {
    let (cap, max_cap, slack, _, scene) = check_capacitance_all(t, drvr);
    if max_cap > 0.0 && scene.is_some() {
        let new_cap = cap + cap_delta;
        return if slack < 0.0 { new_cap <= cap } else { new_cap <= max_cap };
    }
    true
}

/// `Sta::slew(vertex, riseFall, scene, max)`: the larger of its rise and fall max slews.
fn vertex_slew(g: &Graph<'_>, v: usize) -> f32 {
    g.slew[v][0][MAX].max(g.slew[v][1][MAX])
}

/// `Resizer::estimatedSlackOK`. Returns the verdict and the `remove_buffer` debug lines it
/// printed, in order. `scene`: the capacitance guard's (none refuses).
fn estimated_slack_ok(ctx: &Ctx<'_>, t: &Timer<'_, '_>, p: &SlackParams<'_>, scene: Option<usize>) -> (bool, Vec<String>) {
    let mut lines = Vec::new();
    if scene.is_none() {
        return (false, lines);
    }
    let g = t.g;
    let Some((old_delay, new_delay, old_drvr_slew, new_drvr_slew, old_cap, new_cap)) = compute_new_delays_slews(ctx, t, p) else { return (false, lines) };
    if !check_max_cap_ok(t, p.prev, new_cap - old_cap) {
        lines.push(format!("buffer {} is not removed because of max cap violation", network_name(p.inst, ctx.hierarchy)));
        return (false, lines);
    }
    let delay_degrad = new_delay[p.prev_rf] - old_delay[p.prev_rf];
    // bufferDelay(cell, driver transition, loadCap(driver)): at the target slews.
    let (_, output) = p.cell.buffer_ports().expect("a buffer");
    let tgt = |_: &str, rf: usize| ctx.tgt_slews[rf];
    let scene_cell = vertex_cell(g, p.driver).unwrap_or(p.cell);
    let delay_imp = gate_delays(scene_cell, &output.name, g.load_cap(p.driver, t.parasitics), &tgt).0[p.driver_rf];
    raw_dump(format!(
        "VYGE|{}|prev={}|prf={}|drf={}|oc={}|nc={}|od={},{}|nd={},{}|os={},{}|ns={},{}|dlc={}|degrad={}|imp={}\n",
        p.inst,
        g.vertices[p.prev].name,
        p.prev_rf,
        p.driver_rf,
        bits(old_cap),
        bits(new_cap),
        bits(old_delay[0]),
        bits(old_delay[1]),
        bits(new_delay[0]),
        bits(new_delay[1]),
        bits(old_drvr_slew[0]),
        bits(old_drvr_slew[1]),
        bits(new_drvr_slew[0]),
        bits(new_drvr_slew[1]),
        bits(g.load_cap(p.driver, t.parasitics)),
        bits(delay_degrad),
        bits(delay_imp)
    ));
    let Some(load_pin_slew) = estimate_slews_after_buffer_removal(ctx, t, p, new_drvr_slew[p.prev_rf]) else { return (false, lines) };
    for &(load, estimated) in &load_pin_slew {
        let old_load_slew = [g.slew[load][0][MAX], g.slew[load][1][MAX]];
        let new_load_slew = [estimated, estimated];
        lines.push(format!("estimated in slew at fanout pin {} is {}, prev drvr out slew={}", g.vertices[load].name, fmt_float(estimated), fmt_float(new_drvr_slew[p.prev_rf])));
        if !estimate_input_slew_impact(ctx, t, load, old_load_slew, new_load_slew, delay_degrad - delay_imp, p.inst, true, &mut lines) {
            return (false, lines);
        }
    }
    // The previous driver's side loads: `connectedPinIterator(net)` gathers the net's pins into a
    // `PinSet`, which iterates by pin id (instance pins and ports interleaved).
    let Some(net) = g.vertex_net[p.prev] else { return (false, lines) };
    let mut side_pins: Vec<(u64, String)> = g.netlist.nets[net].pins.iter().map(|c| g.netlist.pin_name(c)).map(|n| (t.info.pin_id.get(&n).copied().unwrap_or(u64::MAX), n)).collect();
    side_pins.sort();
    for (_, name) in side_pins {
        let Some(&side) = t.index.get(&name) else { continue };
        if side == p.prev || side == p.driver_input {
            continue;
        }
        let old_slack = slack_all(t, side);
        let new_slack = old_slack - delay_degrad - ctx.margin;
        if new_slack < 0.0 {
            let slack_degrad = old_slack - new_slack;
            if old_slack >= 0.0 || (old_slack < 0.0 && slack_degrad > SLACK_DEGRAD_RATIO_LIMIT * old_slack.abs()) {
                lines.push(format!(
                    "buffer {} is not removed because side input pin {name} will have a violating slack of {}: old slack={}, slack margin={}, delay_degrad={}",
                    network_name(p.inst, ctx.hierarchy),
                    fmt_float(new_slack),
                    fmt_float(old_slack),
                    fmt_float(ctx.margin),
                    fmt_float(delay_degrad)
                ));
                return (false, lines);
            }
        }
        if !estimate_input_slew_impact(ctx, t, side, old_drvr_slew, new_drvr_slew, delay_degrad, p.inst, false, &mut lines) {
            return (false, lines);
        }
    }
    lines.push(format!("buffer {} can be removed because direct fanouts and side fanouts can absorb delay/slew degradation", network_name(p.inst, ctx.hierarchy)));
    (true, lines)
}

/// `Resizer::estimateInputSlewImpact(instance(pin), …)`: each output pin of the load's instance,
/// at its load, timed at the old and the new input slews (all arcs at their input transition's);
/// the worse delay change and `delay_adjust` taken off its slack, the margin off twice (as the
/// reference does). A top-level port's "instance" is the top: its output ports have no liberty
/// port and refuse.
#[allow(clippy::too_many_arguments)]
fn estimate_input_slew_impact(ctx: &Ctx<'_>, t: &Timer<'_, '_>, load: usize, old_in: [f32; 2], new_in: [f32; 2], delay_adjust: f32, inst: &str, accept_if_slack_improves: bool, lines: &mut Vec<String>) -> bool {
    let g = t.g;
    let vx = &g.vertices[load];
    if vx.lib.is_none() {
        // The top instance: its ports in order, the first output one has no liberty port.
        for (port, _) in &g.netlist.ports {
            let Some(&u) = t.index.get(port) else { continue };
            if !g.vertices[u].is_driver {
                lines.push(format!("buffer {} is not removed because pin {port} has no liberty port", network_name(inst, ctx.hierarchy)));
                return false;
            }
        }
        return true;
    }
    let load_inst = vx.name.rsplit_once('/').map_or("", |(i, _)| i);
    let Some(cell) = vertex_cell(g, load) else { return true };
    let Some(pins) = vx.cell.as_deref().and_then(|c| ctx.master_pins.get(c)) else { return true };
    for pin in pins {
        let name = format!("{load_inst}/{pin}");
        let Some(&u) = t.index.get(&name) else { continue };
        let is_output = cell.port(pin).is_some_and(|p| p.direction == vyges_sta::liberty::Direction::Output);
        if !is_output {
            continue;
        }
        let load_cap = g.load_cap(u, t.parasitics);
        let (old_delay, _) = gate_delays(cell, pin, load_cap, &|_, rf| old_in[rf]);
        let (new_delay, _) = gate_delays(cell, pin, load_cap, &|_, rf| new_in[rf]);
        let delay_diff = (new_delay[0] - old_delay[0]).max(new_delay[1] - old_delay[1]);
        let old_slack = slack_all(t, u) - ctx.margin;
        let new_slack = old_slack - delay_diff - delay_adjust - ctx.margin;
        // A diagnostic: every input of this slack as raw float bits (set against the reference's
        // `VYGU|` lines from its instrumented build).
        if let Ok(path) = std::env::var("VYGES_RSZ_UNBUF_RAW") {
            use std::io::Write;
            let b = |f: f32| format!("{:08x}", f.to_bits());
            let line = format!(
                "VYGU|{inst}|{name}|old={}|diff={}|adj={}|m={}|lc={}|oi={},{}|ni={},{}|new={}\n",
                b(old_slack), b(delay_diff), b(delay_adjust), b(ctx.margin), b(load_cap), b(old_in[0]), b(old_in[1]), b(new_in[0]), b(new_in[1]), b(new_slack)
            );
            if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                let _ = f.write_all(line.as_bytes());
            }
        }
        if (accept_if_slack_improves && fuzzy::greater(old_slack, new_slack)) || (!accept_if_slack_improves && new_slack < 0.0) {
            lines.push(format!("buffer {} is not removed because pin {name} will have a violating or worse slack of {}", network_name(inst, ctx.hierarchy), fmt_float(new_slack)));
            return false;
        }
    }
    true
}

/// `Resizer::makeBufferedNet(drvr_pin)` under placement parasitics: the driver's net's Steiner tree
/// as a buffered net.
fn make_buffered_net(ctx: &Ctx<'_>, t: &Timer<'_, '_>, drvr: usize) -> Option<(BufferedNet, usize)> {
    let net = t.g.vertex_net[drvr]?;
    let tree = (t.steiner)(&t.g.netlist.nets[net].name, &t.g.vertices[drvr].name)?;
    let bctx = buffered_net::Ctx { graph: t.g, libs: ctx.libs, sdc: ctx.sdc, limits: ctx.limits, dbu: ctx.dbu, rc: ctx.wire_rc };
    buffered_net::make_buffered_net_steiner(&bctx, &tree)
}

/// `Resizer::stitchTrees(outer, stitching_load, inner)`: the outer tree with the load node of
/// `stitching_load` replaced by the inner tree — spliced in at the same location, else reached by
/// a wire from the load's location (every node is on no layer). Unchanged nodes are shared; an
/// ancestor of the change is made anew. The arena holds both trees; returns the new root, or the
/// outer root itself when no load matched.
fn stitch_trees(ctx: &buffered_net::Ctx<'_, '_>, bn: &mut BufferedNet, node: usize, stitching_load: usize, inner: usize) -> usize {
    match bn.nodes[node].kind.clone() {
        Kind::Wire { r } => {
            let new_ref = stitch_trees(ctx, bn, r, stitching_load, inner);
            if new_ref == r {
                return node;
            }
            let at = bn.location(node);
            bn.wire(ctx, at, new_ref)
        }
        Kind::Junction { r, r2 } => {
            let new_ref = stitch_trees(ctx, bn, r, stitching_load, inner);
            let new_ref2 = stitch_trees(ctx, bn, r2, stitching_load, inner);
            if new_ref == r && new_ref2 == r2 {
                return node;
            }
            let at = bn.location(node);
            bn.junction(at, new_ref, new_ref2)
        }
        Kind::Load { pin } => {
            if pin != stitching_load {
                return node;
            }
            let at = bn.location(node);
            if at == bn.location(inner) {
                return inner;
            }
            bn.wire(ctx, at, inner)
        }
    }
}

/// `Resizer::estimateSlewsInTree(drvr, drvr_slew, tree)`: the slew walked down the tree in double —
/// each wire adds `length · r · (ref cap + length · c / 2) · shape factor` — recorded (as a float)
/// at each load pin.
fn estimate_slews_in_tree(bctx: &buffered_net::Ctx<'_, '_>, bn: &BufferedNet, root: usize, drvr_slew: f32, factor: f32) -> BTreeMap<usize, f32> {
    fn visit(bctx: &buffered_net::Ctx<'_, '_>, bn: &BufferedNet, n: usize, upstream: f64, factor: f32, out: &mut Vec<(usize, f32)>) {
        match bn.nodes[n].kind {
            Kind::Wire { r } => {
                let length = bctx.dbu_to_meters(bn.length(n));
                let (unit_res, unit_cap) = bn.wire_rc(n, bctx);
                let t_wire = length * unit_res * (f64::from(bn.nodes[r].cap) + length * unit_cap / 2.0) * f64::from(factor);
                visit(bctx, bn, r, upstream + t_wire, factor, out);
            }
            Kind::Junction { r, r2 } => {
                visit(bctx, bn, r, upstream, factor, out);
                visit(bctx, bn, r2, upstream, factor, out);
            }
            Kind::Load { pin } => out.push((pin, upstream as f32)),
        }
    }
    let mut v = Vec::new();
    visit(bctx, bn, root, f64::from(drvr_slew), factor, &mut v);
    v.into_iter().collect()
}

/// `Resizer::estimateSlewsAfterBufferRemoval(prev_pin, buffer, drvr_slew)`: the slew at each of
/// the buffer's loads once the previous driver drives them directly — estimated over the stitched
/// tree from `drvr_slew`, then scaled by how the estimate missed the timed slews on the way in
/// (driver to buffer input) and on the way out (buffer output to that load). Keyed by load pin,
/// in pin-id order (the reference's map is keyed by pin pointer).
fn estimate_slews_after_buffer_removal(ctx: &Ctx<'_>, t: &Timer<'_, '_>, p: &SlackParams<'_>, drvr_slew: f32) -> Option<Vec<(usize, f32)>> {
    let g = t.g;
    let (_, output) = p.cell.buffer_ports()?;
    let buffer_drvr = *t.index.get(&format!("{}/{}", p.inst, output.name))?;
    let (tree1, root1) = make_buffered_net(ctx, t, p.prev)?;
    let (tree2, root2) = make_buffered_net(ctx, t, buffer_drvr)?;
    let bctx = buffered_net::Ctx { graph: g, libs: ctx.libs, sdc: ctx.sdc, limits: ctx.limits, dbu: ctx.dbu, rc: ctx.wire_rc };
    // One arena: the outer tree, then the inner tree's nodes shifted after it.
    let mut stitched = tree1.clone();
    let offset = stitched.nodes.len();
    for node in &tree2.nodes {
        let mut node = node.clone();
        node.kind = match node.kind {
            Kind::Wire { r } => Kind::Wire { r: r + offset },
            Kind::Junction { r, r2 } => Kind::Junction { r: r + offset, r2: r2 + offset },
            k @ Kind::Load { .. } => k,
        };
        stitched.nodes.push(node);
    }
    let stitched_root = stitch_trees(&bctx, &mut stitched, root1, p.driver_input, root2 + offset);
    if stitched_root == root1 {
        return None;
    }
    let factor = ctx.slew_shape_factor;
    // Calibration on the way in: the driver's timed slew down the outer tree, against the buffer
    // input's timed slew.
    let drv2buf = estimate_slews_in_tree(&bctx, &tree1, root1, vertex_slew(g, p.prev), factor);
    let estimated_drv2buf = *drv2buf.get(&p.driver_input)?;
    let actual_drv2buf = vertex_slew(g, p.driver_input);
    let in_calib = if !fuzzy::equal(estimated_drv2buf, 0.0) { actual_drv2buf / estimated_drv2buf } else { 1.0 };
    // On the way out: per load, the buffer's timed slew down the inner tree against the load's.
    let buf2load = estimate_slews_in_tree(&bctx, &tree2, root2, vertex_slew(g, buffer_drvr), factor);
    let mut out_calib = BTreeMap::new();
    for (&load, &estimated) in &buf2load {
        let actual = vertex_slew(g, load);
        let out = if !fuzzy::equal(estimated, 0.0) { actual / estimated } else { 1.0 };
        out_calib.insert(load, in_calib * out);
    }
    let mut load_pin_slew = estimate_slews_in_tree(&bctx, &stitched, stitched_root, drvr_slew, factor);
    for (load, slew) in load_pin_slew.iter_mut() {
        if let Some(c) = out_calib.get(load) {
            *slew *= c;
        }
    }
    let mut ordered: Vec<(usize, f32)> = load_pin_slew.into_iter().collect();
    let id = |v: usize| t.info.pin_id.get(&g.vertices[v].name).copied().unwrap_or(u64::MAX);
    for &(v, _) in &ordered {
        raw_dump(format!("VYGO|{}|{}|id={}|addr={:?}\n", p.inst, g.vertices[v].name, id(v), ctx.pin_addr.and_then(|pa| pa.addr(id(v)))));
    }
    match ctx.pin_addr.filter(|pa| ordered.iter().all(|&(v, _)| pa.addr(id(v)).is_some())) {
        Some(pa) => ordered.sort_by_key(|&(v, _)| pa.addr(id(v))),
        None => ordered.sort_by_key(|&(v, _)| pointer_order(t, v)),
    }
    Some(ordered)
}

/// The order of a `std::map<const Pin*, …>`: a pin is its odb object's address, so instance pins
/// and ports sit in two tables. Probed on the reference over the 88 repair_timing designs (pin
/// handles printed by address): every run of a design gives the same order, and in 64 of them every
/// instance pin precedes every port, each table in id order — the rule here. In the other 24 the
/// heap put a port page first or between instance pins; that changes only which loads print before
/// the first rejection, never the verdict (every load must pass).
fn pointer_order(t: &Timer<'_, '_>, v: usize) -> (bool, u64) {
    pin_table_order(t.info.pin_id.get(&t.g.vertices[v].name).copied().unwrap_or(u64::MAX))
}

/// From `NetInfo::pin_id` (an instance pin's ITerm id × 2, a port's BTerm id × 2 + 1): the
/// instance-pin table first, then the port table, each by id.
fn pin_table_order(pin_id: u64) -> (bool, u64) {
    (pin_id & 1 == 1, pin_id >> 1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use buffered_net::{BNode, NULL_LAYER};

    fn node(kind: Kind, x: i32, y: i32, cap: f32) -> BNode {
        BNode { kind, x, y, layer: NULL_LAYER, cap, fanout: 1.0, max_load_slew: INF }
    }

    fn with_ctx<R>(f: impl FnOnce(&buffered_net::Ctx<'_, '_>) -> R) -> R {
        let netlist = vyges_sta::netlist::Netlist::default();
        let g = Graph::build(&[], &netlist).unwrap();
        let libs = crate::preamble::Libs::default();
        let sdc = vyges_sta::graph::SdcEnv::default();
        let limits = Limits::default();
        let rc = buffered_net::WireRc { h_res: 2.0e5, v_res: 1.0e5, h_cap: 1.0e-10, v_cap: 2.0e-10 };
        f(&buffered_net::Ctx { graph: &g, libs: &libs, sdc: &sdc, limits: &limits, dbu: 1000, rc })
    }

    // Rule (passesFanoutGuard): with a timer limit (0 < limit < INF) the fanout plus the buffer's
    // loads less its input must stay within it; with none the timer reports fanout 0 and only the
    // buffer's loads count, against 10.
    #[test]
    fn the_fanout_guard_reads_the_timer_limit_else_ten() {
        assert_eq!(fanout_rejects(3.0, 5.0, 3), None, "3 + 3 - 1 = 5 fits 5");
        assert_eq!(fanout_rejects(3.0, 5.0, 4), Some(FanoutLimit::Timer(5.0)));
        assert_eq!(fanout_rejects(0.0, INF, 11), None, "no limit: 10 loads after the buffer goes");
        assert_eq!(fanout_rejects(0.0, INF, 12), Some(FanoutLimit::Default));
        assert_eq!(fanout_rejects(0.0, 0.0, 12), Some(FanoutLimit::Default), "a zero limit is no limit");
    }

    // Rule (std::map<const Pin*, …> over odb objects, probed on the reference): instance pins in
    // id order, then ports in id order — not the interleaved pin ids.
    #[test]
    fn loads_are_visited_instance_pins_first() {
        let mut ids = vec![5u64, 40, 2, 9, 4];
        ids.sort_by_key(|&i| pin_table_order(i));
        assert_eq!(ids, vec![2, 4, 40, 5, 9]);
    }

    // Rule (stitchTrees): the stitching load becomes the inner tree — spliced in at the same
    // location, else reached by a new wire from the load's location; its ancestors are made anew
    // (their caps now carry the inner tree), an untouched branch is shared, and no match leaves
    // the outer root.
    #[test]
    fn stitching_replaces_the_buffer_input_load() {
        with_ctx(|ctx| {
            // outer: wire(0,0) -> junction(10,0) { load 7 at (10,0), load 8 at (10,0) }
            let mut bn = BufferedNet {
                nodes: vec![
                    node(Kind::Load { pin: 7 }, 10, 0, 1e-15),
                    node(Kind::Load { pin: 8 }, 10, 0, 2e-15),
                    node(Kind::Junction { r: 0, r2: 1 }, 10, 0, 3e-15),
                    node(Kind::Wire { r: 2 }, 0, 0, 4e-15),
                    // inner root: a load at (30, 0)
                    node(Kind::Load { pin: 9 }, 30, 0, 5e-15),
                ],
            };
            assert_eq!(stitch_trees(ctx, &mut bn, 3, 99, 4), 3, "no matching load: the outer root");
            let root = stitch_trees(ctx, &mut bn, 3, 8, 4);
            assert_ne!(root, 3);
            let Kind::Wire { r: j } = bn.nodes[root].kind else { panic!("a new wire at the root") };
            let Kind::Junction { r, r2 } = bn.nodes[j].kind else { panic!("a new junction") };
            assert_eq!(r, 0, "the untouched load is shared");
            let Kind::Wire { r: inner } = bn.nodes[r2].kind else { panic!("a wire to the inner tree") };
            assert_eq!((inner, bn.location(r2)), (4, (10, 0)));
            assert!(bn.nodes[j].cap > 5e-15, "the new junction carries the inner tree's cap");
            // At the inner root's own location the inner tree is spliced in directly.
            bn.nodes[4].x = 10;
            let n = bn.nodes.len();
            let root = stitch_trees(ctx, &mut bn, 3, 8, 4);
            let Kind::Wire { r: j } = bn.nodes[root].kind else { panic!() };
            assert!(j >= n);
            assert_eq!(bn.nodes[j].kind, Kind::Junction { r: 0, r2: 4 });
        });
    }

    // Rule (estimateSlewsInTree): down each wire the slew grows by
    // length · r · (ref cap + length · c / 2) · shape factor, in double; a junction passes it to
    // both sides; each load records it.
    #[test]
    fn slews_grow_down_each_wire() {
        with_ctx(|ctx| {
            let bn = BufferedNet {
                nodes: vec![
                    node(Kind::Load { pin: 1 }, 1000, 0, 1e-15),
                    node(Kind::Load { pin: 2 }, 0, 0, 2e-15),
                    node(Kind::Junction { r: 0, r2: 1 }, 0, 0, 3e-15),
                ],
            };
            let mut bn = bn;
            let w = bn.wire(ctx, (0, 0), 0);
            bn.nodes[2].kind = Kind::Junction { r: w, r2: 1 };
            let s = estimate_slews_in_tree(ctx, &bn, 2, 1e-11, 2.0);
            let len = 1e-6f64; // 1000 dbu at 1000 dbu/µm
            let want = f64::from(1e-11f32) + len * 2.0e5 * (f64::from(1e-15f32) + len * 1.0e-10 / 2.0) * f64::from(2.0f32);
            assert_eq!(s[&2], 1e-11f32, "the load at the junction sees the driver's slew");
            assert_eq!(s[&1], want as f32, "the load down the wire");
        });
    }

    // Rule (odb dbTable): a page's slots sit at its base plus slot × object size. A page holding
    // one live terminal gives no two-slot stride; the printed page base does (block port 1 at
    // base + 0x70 is a 112-byte terminal), and an address off that grid is refused.
    #[test]
    fn a_lone_terminal_takes_its_stride_from_the_page_base() {
        let a = PinAddr::parse("B 1 8060557fe5110000\nP B 0 0x11e57f556010\n").unwrap();
        assert_eq!(a.addr(2 * 3 + 1), Some(0x11e57f556010 + 3 * 0x70));
        assert!(PinAddr::parse("I 1 6840b07ee5110000\nI 2 c040b07ee5110000\nP I 0 0x11e57eb04011\n").is_err());
    }
}
