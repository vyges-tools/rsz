// SPDX-License-Identifier: Apache-2.0
//! What `RepairDesign::repairDesign` asks the timer before it repairs anything: the slew limits,
//! the forward pass that holds over-limit load slews at their limit, and the driver order.
//!
//! One corner (the reference's scene is named `default`), max slews only, a flat netlist.

use std::cmp::Ordering;

use vyges_sta::graph::Graph;
use vyges_sta::liberty::{Direction, Library, FALL, MAX, RISE};
use vyges_sta::netlist::Conn;

/// `sta::INF`.
pub const INF: f32 = 1e30;

/// The constraints the limits read.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Limits {
    /// `set_max_transition V [current_design]`, seconds.
    pub design_max_transition: Option<f32>,
    /// `set_max_fanout V [current_design]` (`Sdc::fanoutLimit(top cell)`), unitless.
    pub design_max_fanout: Option<f32>,
    /// `set_max_capacitance V [current_design]` (`Sdc::capacitanceLimit(top cell)`), farads.
    pub design_max_capacitance: Option<f32>,
    /// `set_max_capacitance V [get_ports …]` (`Sdc::capacitanceLimit(port)`), farads, per top-level
    /// port; a later command on the same port replaces the value.
    pub port_max_capacitance: std::collections::BTreeMap<String, f32>,
}

/// `set_max_transition V` in the reference: Tcl `time_ui_sta` is `Unit::userToSta(double)` —
/// the value times the time unit's FLOAT scale, in double — and the limit setter takes a float,
/// so it is narrowed once, at the end.
pub fn user_time_to_sta(value: f64, time_scale: f32) -> f32 {
    (value * f64::from(time_scale)) as f32
}

/// `set_max_capacitance V`: Tcl `capacitance_ui_sta` is the capacitance unit's `userToSta`, in
/// double, and the limit setter takes a float — the same rounding as [`user_time_to_sta`].
pub fn user_cap_to_sta(value: f64, cap_scale: f32) -> f32 {
    (value * f64::from(cap_scale)) as f32
}

/// Two max limits, the tighter kept: `if (exists1 && (!exists || min_max->compare(limit,
/// limit1)))` takes `limit1` — for max, when `limit > limit1`.
fn tighter_max(limit: Option<f32>, limit1: Option<f32>) -> Option<f32> {
    match (limit, limit1) {
        (Some(l), Some(l1)) if l > l1 => Some(l1),
        (Some(l), _) => Some(l),
        (None, l1) => l1,
    }
}

/// `CheckCapacitances::findLimit(pin, scene, max)`: "default to top (design) limit", then the
/// tighter of that and — on a top-level port — the port's own SDC limit, then its driving cell's
/// `-pin` port's; or — on an instance pin — its scene port's `max_capacitance`. An SDC limit on a
/// pin is not read here (only `Sdc::isConstrained` sees it); one on a cell is refused at read; the
/// library default is never set in the reference.
fn find_cap_limit(g: &Graph<'_>, v: usize, limits: &Limits) -> Option<f32> {
    let mut limit = limits.design_max_capacitance;
    if matches!(g.vertices[v].conn, Conn::Port(_)) {
        limit = tighter_max(limit, limits.port_max_capacitance.get(&g.vertices[v].name).copied());
    }
    tighter_max(limit, limit_port_of(g, v).and_then(|(_, p)| p.max_capacitance))
}

/// `Resizer::maxLoad(cell)`: over the output ports in order, the tighter of the design limit and
/// the port's own; the first port with either returns it, else 0. With a design limit that is
/// always the first output port.
pub fn max_load(cell: &vyges_sta::liberty::Cell, limits: &Limits) -> f32 {
    cell.ports.iter().filter(|p| p.direction == vyges_sta::liberty::Direction::Output).find_map(|p| tighter_max(limits.design_max_capacitance, p.max_capacitance)).unwrap_or(0.0)
}

/// A netlist the timer here cannot level the way the reference does, named — or `None`.
///
/// - an instance on a net whose master has no liberty cell: the reference's graph still has
///   its pins (a driver among them is in the driver order); ours has none;
/// - a cell with an arc set the timer here does not build an edge for (`Other`), which the
///   reference's levelization searches through;
/// - a tristate driver ON A NET: its enable/disable arcs carry no delays here, and its slew is read
///   (one with no net — an unconnected pad pin — is levelled through and never timed).
pub fn unlevelable(libs: &[Library], netlist: &vyges_sta::netlist::Netlist) -> Option<String> {
    let cell_of = |name: &str| libs.iter().find_map(|l| l.cells.get(name));
    for net in &netlist.nets {
        for c in &net.pins {
            if let Conn::Inst(i, pin) = c {
                let (inst, master) = &netlist.insts[*i];
                match cell_of(master) {
                    None => return Some(format!("{inst}/{pin} on net {}: master {master} has no liberty cell", net.name)),
                    Some(cell) => {
                        if let Some(set) = cell.arc_sets.iter().find(|s| s.role == vyges_sta::liberty::Role::Other && !is_check_timing_type(&s.timing_type)) {
                            return Some(format!("{inst} ({master}): a `{}` arc set is not timed here, and the reference levels through it", set.timing_type));
                        }
                        use vyges_sta::liberty::Role;
                        let on_net = |p: &str| netlist.nets.iter().any(|n| n.pins.iter().any(|c| matches!(c, Conn::Inst(j, q) if *j == *i && q == p)));
                        if let Some(set) = cell.arc_sets.iter().find(|s| matches!(s.role, Role::TristateEnable | Role::TristateDisable) && on_net(&s.to)) {
                            return Some(format!("{inst}/{} ({master}): a tristate driver on a net — its `{}` delays are not modelled", set.to, set.timing_type));
                        }
                    }
                }
            }
        }
    }
    None
}

/// Liberty `timing_type`s that are timing CHECKS (`TimingRole::isTimingCheck`) — no edge in the
/// reference's levelization either.
fn is_check_timing_type(t: &str) -> bool {
    t.starts_with("setup") || t.starts_with("hold") || t.starts_with("recovery") || t.starts_with("removal")
        || t.starts_with("skew") || t.starts_with("non_seq") || t.starts_with("nochange")
        || matches!(t, "min_pulse_width" | "minimum_period")
    // Not here: `min/max_clock_tree_path`. The reference lists them APART from isTimingCheck
    // (findTargetLoad excludes both), so they are not checks; a cell carrying one is refused.
}

/// `CheckSlews::findLimit(port, scene, max)`: the design's limit, then the port's own
/// `max_transition` — or, for an OUTPUT port without one, its library's `default_max_transition`
/// — when it is tighter. `(limit, exists)`.
pub fn find_slew_limit(lib: &Library, direction: Direction, port_max_transition: Option<f32>, limits: &Limits) -> (f32, bool) {
    let (mut limit, mut exists) = (INF, false);
    if let Some(l) = limits.design_max_transition {
        (limit, exists) = (l, true);
    }
    let port_limit = port_max_transition.or(if direction.drives() { lib.default_max_transition } else { None });
    if let Some(l1) = port_limit {
        if !exists || limit > l1 {
            (limit, exists) = (l1, true);
        }
    }
    (limit, exists)
}

/// `Resizer::maxInputSlew`: [`find_slew_limit`]; with none, or a limit of 0, the port's library
/// `default_max_transition` (applied to an input too); with none of that, [`INF`].
pub fn max_input_slew(lib: &Library, direction: Direction, port_max_transition: Option<f32>, limits: &Limits) -> f32 {
    max_input_slew_at(lib, lib, direction, port_max_transition, limits)
}

/// `Resizer::maxInputSlew(port, scene)`: the limit is the SCENE port's ([`find_slew_limit`] on
/// `scene_lib` and the scene port's direction and `max_transition`); the fallback default is the
/// LINK port's library's (`input->libertyLibrary()`), whatever the scene.
pub fn max_input_slew_at(link_lib: &Library, scene_lib: &Library, direction: Direction, port_max_transition: Option<f32>, limits: &Limits) -> f32 {
    let (limit, exists) = find_slew_limit(scene_lib, direction, port_max_transition, limits);
    if !exists || limit == 0.0 {
        return link_lib.default_max_transition.unwrap_or(INF);
    }
    limit
}

/// The timer over every scene: one graph per scene (its libraries' cells), over one netlist.
/// Vertices are named alike in every graph; [`Scenes::at`] maps scene 0's vertex to scene `k`'s
/// (port order may differ between a scene's libraries). Scene 0 is the LINK view.
pub struct Scenes<'s, 'a> {
    pub graphs: &'s [Graph<'a>],
    /// Per scene, its vertices by (name, is driver) — built only for a scene whose vertex order
    /// differs from scene 0's at a vertex asked for.
    index: Vec<std::cell::OnceCell<std::collections::HashMap<(&'s str, bool), usize>>>,
}

impl<'s, 'a> Scenes<'s, 'a> {
    pub fn new(graphs: &'s [Graph<'a>]) -> Scenes<'s, 'a> {
        Scenes { graphs, index: graphs.iter().map(|_| std::cell::OnceCell::new()).collect() }
    }
    /// Scene `k`'s vertex for scene 0's `v`: the vertex of the same name and direction — the same
    /// index when the scene lists it there, as it does unless its libraries order ports otherwise.
    pub fn at(&self, k: usize, v: usize) -> usize {
        let (g0, g) = (&self.graphs[0], &self.graphs[k]);
        let want = &g0.vertices[v];
        if g.vertices.get(v).is_some_and(|x| x.is_driver == want.is_driver && x.name == want.name) {
            return v;
        }
        let index = self.index[k].get_or_init(|| g.vertices.iter().enumerate().map(|(i, x)| ((x.name.as_str(), x.is_driver), i)).collect());
        index[&(want.name.as_str(), want.is_driver)]
    }
    pub fn link(&self) -> &Graph<'a> {
        &self.graphs[0]
    }
}

/// A vertex's instance name, or `None` for a top-level port.
fn instance_of<'a>(g: &'a Graph<'_>, v: usize) -> Option<&'a str> {
    match &g.vertices[v].conn {
        Conn::Inst(i, _) => Some(g.netlist.insts[*i].0.as_str()),
        Conn::Port(_) => None,
    }
}

/// `Network::pathNameCmp(pin, pin)` on a flat netlist: the instances' paths first — the top
/// instance's (a port's) is empty and sorts first, leaf instances by name — then the port names.
fn path_name_cmp(g: &Graph<'_>, a: usize, b: usize) -> Ordering {
    let port = |v: usize| -> &str {
        match &g.vertices[v].conn {
            Conn::Inst(_, p) => p.as_str(),
            Conn::Port(_) => g.vertices[v].name.as_str(),
        }
    };
    let inst = match (instance_of(g, a), instance_of(g, b)) {
        (None, None) => Ordering::Equal,
        (None, Some(_)) => Ordering::Less,
        (Some(_), None) => Ordering::Greater,
        (Some(x), Some(y)) => x.as_bytes().cmp(y.as_bytes()),
    };
    inst.then_with(|| port(a).as_bytes().cmp(port(b).as_bytes()))
}

/// `Resizer::orderedLoadPinVertices`: every vertex of an instance INPUT pin (`isAnyInput`, not
/// a top-level port), by level, ties by the full path name as a string (`VertexLevelLess`).
pub fn ordered_load_pin_vertices(g: &Graph<'_>, level: &[i32]) -> Vec<usize> {
    let mut loads: Vec<usize> = (0..g.vertices.len()).filter(|&v| instance_of(g, v).is_some() && !g.vertices[v].is_driver).collect();
    loads.sort_by(|&a, &b| level[a].cmp(&level[b]).then_with(|| g.vertices[a].name.as_bytes().cmp(g.vertices[b].name.as_bytes())));
    loads
}

/// `dbSta::levelizedDrvrVertices`: every driver vertex (instance outputs and top-level inputs),
/// by level, ties by [`path_name_cmp`].
pub fn levelized_drvr_vertices(g: &Graph<'_>, level: &[i32]) -> Vec<usize> {
    let mut drvrs: Vec<usize> = (0..g.vertices.len()).filter(|&v| g.vertices[v].is_driver).collect();
    drvrs.sort_by(|&a, &b| level[a].cmp(&level[b]).then_with(|| path_name_cmp(g, a, b)));
    drvrs
}

/// One forward-pass annotation: the load, and the limit its slew is held at.
#[derive(Debug, Clone, PartialEq)]
pub struct Annotation {
    pub load: usize,
    pub limit: f32,
}

/// The forward pass's limit for scene 0's load `v` in scene `k`, as `Resizer::maxInputSlew(port,
/// corner)` gives it: the LINK port must exist; the limit is the scene port's, the fallback default
/// the link library's ([`max_input_slew_at`]). `(scene k's vertex, limit)`.
pub fn load_limit(sc: &Scenes<'_, '_>, k: usize, v: usize, limits: &Limits) -> Option<(usize, f32)> {
    let (link_lib, _) = port_of(sc.link(), v)?;
    let vk = sc.at(k, v);
    let (scene_lib, p) = port_of(&sc.graphs[k], vk)?;
    Some((vk, max_input_slew_at(link_lib, scene_lib, p.direction, p.max_transition, limits)))
}

/// The forward pass of `RepairDesign::repairDesign` in ONE scene: each load (`(scene vertex,
/// scene 0 vertex, limit)`, in the reference's order) whose max slew in this scene exceeds its
/// limit on either transition is held at that limit on both, so the excess does not propagate.
/// The timer times every load after its fanin, so holding a load inside the delay calculation
/// gives the reference's level-ordered `findDelays` + annotate the same slews. Scenes do not
/// interact in delay calculation, so each is annotated on its own graph. Returns the annotated
/// loads (scene 0's vertices), in order.
pub fn annotate_load_slews(g: &mut Graph<'_>, items: &[(usize, usize, f32)], parasitics: &std::collections::HashMap<String, vyges_sta::graph::NetParasitics>) -> Result<Vec<Annotation>, String> {
    g.slew_limit = items.iter().map(|&(vk, _, l)| (vk, l)).collect();
    g.find_delays(parasitics, None)?;
    let clamped: std::collections::BTreeSet<usize> = g.clamped.iter().copied().collect();
    let out: Vec<Annotation> = items.iter().filter(|(vk, ..)| clamped.contains(vk)).map(|&(_, v, limit)| Annotation { load: v, limit }).collect();
    debug_assert!(items.iter().filter(|(vk, ..)| clamped.contains(vk)).all(|&(vk, _, l)| g.slew[vk][RISE][MAX] == l && g.slew[vk][FALL][MAX] == l));
    Ok(out)
}

/// The drivers of a load's net (`network_->drivers(pin)`), scene 0's vertices.
pub fn net_drivers(g: &Graph<'_>, load: usize) -> Vec<usize> {
    let Some(n) = g.vertex_net[load] else { return Vec::new() };
    g.netlist.nets[n].pins.iter().filter_map(|c| {
        let name = g.netlist.pin_name(c);
        g.vertices.iter().position(|x| x.name == name && x.is_driver)
    }).collect()
}

/// Each driver vertex by its pin name — the first, as [`net_drivers`]' search finds it.
pub fn driver_index<'g>(g: &'g Graph<'_>) -> std::collections::HashMap<&'g str, usize> {
    let mut index = std::collections::HashMap::new();
    for (i, v) in g.vertices.iter().enumerate().filter(|(_, v)| v.is_driver) {
        index.entry(v.name.as_str()).or_insert(i);
    }
    index
}

/// [`net_drivers`] through a [`driver_index`]: the same drivers in the same order, without a
/// search of every vertex for each pin of the net.
pub fn net_drivers_in(g: &Graph<'_>, load: usize, drivers: &std::collections::HashMap<&str, usize>) -> Vec<usize> {
    let Some(n) = g.vertex_net[load] else { return Vec::new() };
    g.netlist.nets[n].pins.iter().filter_map(|c| drivers.get(g.netlist.pin_name(c).as_str()).copied()).collect()
}

// ---- repairNet's checks ----------------------------------------------------------------------

/// What the checks read about the netlist besides the graph: each pin's id order and the nets
/// `repairDriver` passes over.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NetInfo {
    /// `dbNetwork::id(pin)` on a flat netlist: an instance pin's ITerm id × 2, a port's BTerm
    /// id × 2 + 1. A `PinSet` iterates in this order.
    pub pin_id: std::collections::HashMap<String, u64>,
    pub dont_touch: std::collections::BTreeSet<String>,
    /// Instances marked dont_touch (`Resizer::dontTouch(inst)`).
    pub dont_touch_insts: std::collections::BTreeSet<String>,
    /// Instances placed FIRM, LOCKED or COVER (`Resizer::isFixed(inst)` → `dbInst::isFixed`): they
    /// keep their footprint — no size move resizes them; a VT swap (same footprint) still may.
    pub fixed_insts: std::collections::BTreeSet<String>,
    pub abutment: std::collections::BTreeSet<String>,
}

/// `ClkNetwork::findClkPins` (no propagated clocks, so ideal = all): breadth first from each
/// clock's source pins over wire edges and COMBINATIONAL arcs only (`ClkTreeSearchPred`), never
/// into another clock's source pin (`ClkSearchPred::searchTo`). Membership only.
pub fn clock_pins(g: &Graph<'_>, sources: &[String]) -> std::collections::BTreeSet<usize> {
    use vyges_sta::graph::EdgeKind;
    let source_v: Vec<usize> = sources.iter().filter_map(|s| g.vertices.iter().position(|v| &v.name == s)).collect();
    let mut seen: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();
    let mut queue: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
    for &v in &source_v {
        if seen.insert(v) {
            queue.push_back(v);
        }
    }
    while let Some(v) = queue.pop_front() {
        for &e in &g.out_edges[v] {
            let thru = match g.edges[e].kind {
                EdgeKind::Wire => true,
                EdgeKind::Gate { set } => {
                    let vx = &g.vertices[g.edges[e].to];
                    let cell = &g.libs[vx.lib.expect("an instance pin")].cells[vx.cell.as_deref().expect("its cell")];
                    cell.arc_sets[set].role == vyges_sta::liberty::Role::Combinational
                }
            };
            let to = g.edges[e].to;
            if thru && !source_v.contains(&to) && seen.insert(to) {
                queue.push_back(to);
            }
        }
    }
    seen
}

/// `Network::connectedPinIterator(pin)` on a flat netlist: the pins of the pin's net, as a
/// `PinSet` — in [`NetInfo::pin_id`] order.
pub fn connected_pins(g: &Graph<'_>, info: &NetInfo, v: usize) -> Vec<usize> {
    let Some(n) = g.vertex_net[v] else { return vec![v] };
    let mut pins: Vec<usize> = g.netlist.nets[n]
        .pins
        .iter()
        .filter_map(|c| g.index().get(g.netlist.pin_name(c).as_str()).copied())
        .collect();
    if !pins.contains(&v) {
        pins.push(v);
    }
    pins.sort_by_key(|&p| info.pin_id.get(&g.vertices[p].name).copied().unwrap_or(u64::MAX));
    pins.dedup();
    pins
}

/// The liberty port of an instance pin.
fn port_of<'g>(g: &'g Graph<'_>, v: usize) -> Option<(&'g Library, &'g vyges_sta::liberty::Port)> {
    let vx = &g.vertices[v];
    let lib = &g.libs[vx.lib?];
    let port = lib.cells.get(vx.cell.as_deref()?)?.port(vx.port.as_deref()?)?;
    Some((lib, port))
}

/// The liberty port whose limits a pin's checks read (`CheckSlews` / `CheckCapacitances` /
/// `CheckFanouts::findLimit`): an instance pin's own; for a top-level port with a driving cell,
/// that cell's `-pin` port (in this scene's library).
fn limit_port_of<'g>(g: &'g Graph<'_>, v: usize) -> Option<(&'g Library, &'g vyges_sta::liberty::Port)> {
    port_of(g, v).or_else(|| drive_port(g, v))
}

/// A top-level port's driving cell output port (`InputDrive::driveCell`'s `to_port`).
pub fn drive_port<'g>(g: &'g Graph<'_>, v: usize) -> Option<(&'g Library, &'g vyges_sta::liberty::Port)> {
    if !matches!(g.vertices[v].conn, Conn::Port(_)) {
        return None;
    }
    let d = g.sdc.input_drive.get(&g.vertices[v].name)?;
    let lib = g.libs.iter().find(|l| l.cells.contains_key(&d.cell))?;
    Some((lib, lib.cells[&d.cell].port(&d.to_port)?))
}

/// `CheckSlews::findLimit(pin, …)` with no clock domains: an instance pin's is its port's
/// ([`find_slew_limit`]); a top-level port's is the design's alone (port limits and input
/// drives are refused upstream).
fn pin_slew_limit(g: &Graph<'_>, v: usize, limits: &Limits) -> (f32, bool) {
    match limit_port_of(g, v) {
        Some((lib, port)) => find_slew_limit(lib, port.direction, port.max_transition, limits),
        None => match limits.design_max_transition {
            Some(l) => (l, true),
            None => (INF, false),
        },
    }
}

/// One `CheckSlews::check` result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlewCheck {
    pub slew: f32,
    pub limit: f32,
    pub slack: f32,
    pub rf: usize,
    pub scene: usize,
}

/// `Sta::checkSlew(pin, scenes, max, check_clks = false)`: skipped on an ideal clock pin;
/// otherwise over the scenes in order, then rise then fall with a limit (the scene port's),
/// `slack = limit − slew` (max slew); a smaller slack wins, a fuzzy tie with a LOWER transition
/// index wins too (so a later scene's rise can replace an earlier scene's fall). `None`: no limit
/// (no scene).
pub fn check_slew(sc: &Scenes<'_, '_>, v: usize, limits: &Limits, clocks: &std::collections::BTreeSet<usize>) -> Option<SlewCheck> {
    if clocks.contains(&v) {
        return None;
    }
    let mut best: Option<SlewCheck> = None;
    for (k, g) in sc.graphs.iter().enumerate() {
        let vk = sc.at(k, v);
        for rf in [RISE, FALL] {
            let (limit, exists) = pin_slew_limit(g, vk, limits);
            if !exists {
                continue;
            }
            let slew = g.slew[vk][rf][MAX];
            let slack = limit - slew;
            let take = match &best {
                None => true,
                Some(b) => slack < b.slack || (vyges_sta::fuzzy::equal(slack, b.slack) && rf < b.rf),
            };
            if take {
                best = Some(SlewCheck { slew, limit, slack, rf, scene: k });
            }
        }
    }
    best
}

/// `limit · (1 − margin / 100)`: a float limit times a double factor, narrowed back.
fn margined(limit: f32, margin: f64) -> f32 {
    (f64::from(limit) * (1.0 - margin / 100.0)) as f32
}

/// `RepairDesign::checkSlew(drvr)`: the timer's check, its limit margined, `slack = limit −
/// slew`. With no limit: limit and slack INF and the slew NOT written — the caller's
/// uninitialized local, which this build reads as 0, so it is `None` here.
pub fn repair_check_slew(sc: &Scenes<'_, '_>, v: usize, limits: &Limits, clocks: &std::collections::BTreeSet<usize>, slew_margin: f64) -> (Option<f32>, f32, f32, Option<usize>) {
    let (mut slew, mut limit, mut slack, mut scene) = (None, INF, INF, None);
    if let Some(c) = check_slew(sc, v, limits, clocks) {
        let limit1 = margined(c.limit, slew_margin);
        let slack1 = limit1 - c.slew;
        if slack1 < slack {
            (slew, limit, slack, scene) = (Some(c.slew), limit1, slack1, Some(c.scene));
        }
    }
    (slew, limit, slack, scene)
}

/// The outputs of `Resizer::checkLoadSlews`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadSlews {
    /// Written only when a load sets the smallest slack; otherwise the CALLER'S value is kept
    /// (the reference passes in the variables of its driver check).
    pub slew: Option<f32>,
    pub limit: f32,
    pub slack: f32,
    /// The scene of the smallest slack, when a load set it.
    pub scene: Option<usize>,
}

/// `Resizer::checkLoadSlews(drvr, margin)`: every other connected pin, in pin-set order. With a
/// limit: margined, then the RUNNING minimum of the limits so far; `slack = that − slew`; the
/// smallest slack sets the slew. Without one (an ideal clock pin, or no limit): when the pin has
/// a liberty port whose library has a `default_max_transition`, that default becomes the
/// running limit — and nothing else (the reference computes a slew there and drops it).
pub fn check_load_slews(sc: &Scenes<'_, '_>, info: &NetInfo, drvr: usize, limits: &Limits, clocks: &std::collections::BTreeSet<usize>, slew_margin: f64) -> LoadSlews {
    let g = sc.link();
    let mut out = LoadSlews { slew: None, limit: INF, slack: INF, scene: None };
    for p in connected_pins(g, info, drvr) {
        if p == drvr {
            continue;
        }
        match check_slew(sc, p, limits, clocks) {
            None => {
                if let Some((lib, _)) = port_of(g, p) {
                    if let Some(default) = lib.default_max_transition {
                        out.limit = default;
                    }
                }
            }
            Some(c) => {
                let limit1 = margined(c.limit, slew_margin).min(out.limit);
                out.limit = limit1;
                let slack1 = limit1 - c.slew;
                if slack1 < out.slack {
                    out.slew = Some(c.slew);
                    out.slack = slack1;
                    out.scene = Some(c.scene);
                }
            }
        }
    }
    out
}

/// `Resizer::kDefaultMaxFanout`: the load-pin backstop when nothing constrains fanout.
pub const DEFAULT_MAX_FANOUT: f32 = 50.0;

/// `Sta::checkFanout(drvr, mode, max)` (`CheckFanouts::check`): the fanout load of the connected
/// non-port loads (`fanout_load`, else the library default, else nothing; a pin with no liberty
/// port counts 1) against the port's `max_fanout` or, for an output, its library's
/// `default_max_fanout`, the design's limit when tighter. No limit (or an ideal clock pin):
/// fanout 0, limit and slack INF.
pub fn sta_check_fanout(g: &Graph<'_>, info: &NetInfo, drvr: usize, limits: &Limits, clocks: &std::collections::BTreeSet<usize>) -> (f32, f32, f32) {
    let ideal_clock = clocks.contains(&drvr);
    let mut fanout = 0.0f32;
    for p in connected_pins(g, info, drvr) {
        let is_load = !g.vertices[p].is_driver;
        let top_port = matches!(g.vertices[p].conn, Conn::Port(_));
        if is_load && !top_port {
            match port_of(g, p) {
                Some((lib, port)) => {
                    if let Some(l) = port.fanout_load.or(lib.default_fanout_load) {
                        fanout += l;
                    }
                }
                None => fanout += 1.0,
            }
        }
    }
    let mut check = (0.0f32, INF, INF);
    if !ideal_clock {
        // `CheckFanouts::findLimit`: the design's limit, then the port's own (or, for an output,
        // its library's default) when tighter.
        let port_limit = match limit_port_of(g, drvr) {
            Some((lib, port)) => port.max_fanout.or(if port.direction.drives() { lib.default_max_fanout } else { None }),
            None => None,
        };
        let limit = match (limits.design_max_fanout, port_limit) {
            (Some(d), Some(p)) => Some(if d > p { p } else { d }),
            (d, p) => d.or(p),
        };
        if let Some(l) = limit {
            check = (fanout, l, l - fanout);
        }
    }
    check
}

/// `Resizer::checkFanout(drvr, max)`: the timer's check ([`sta_check_fanout`]); with no limit
/// the backstop applies: the number of load pins (`FindNetDrvrLoads`), against 50.
pub fn check_fanout(g: &Graph<'_>, info: &NetInfo, drvr: usize, limits: &Limits, clocks: &std::collections::BTreeSet<usize>) -> (f32, f32, f32) {
    let check = sta_check_fanout(g, info, drvr, limits, clocks);
    if check.1 < INF || clocks.contains(&drvr) {
        return check;
    }
    let loads = connected_pins(g, info, drvr).into_iter().filter(|&p| !g.vertices[p].is_driver).count();
    if loads == 0 {
        return check;
    }
    let count = loads as f32;
    (count, DEFAULT_MAX_FANOUT, DEFAULT_MAX_FANOUT - count)
}

/// `Sta::checkCapacitance(drvr, scenes, max)`: on a driver that is not an ideal clock pin, with
/// a limit ([`find_cap_limit`]: the design's and the pin's port's, the tighter) —
/// `slack = limit − loadCap` (rise taken; fall is the same cap). No limit: cap 0, limit −INF,
/// slack INF, no scene.
/// Over the scenes in order (each its scene port's limit and its own load cap), rise then fall:
/// a FUZZILY smaller slack wins, a fuzzy tie with a lower transition index too. Returns the scene.
pub fn check_capacitance<P: std::borrow::Borrow<std::collections::HashMap<String, vyges_sta::graph::NetParasitics>>>(sc: &Scenes<'_, '_>, drvr: usize, parasitics: &[P], clocks: &std::collections::BTreeSet<usize>, limits: &Limits) -> (f32, f32, f32, bool, Option<usize>) {
    let mut best: Option<(f32, f32, f32, usize, usize)> = None;
    if !clocks.contains(&drvr) {
        for (k, g) in sc.graphs.iter().enumerate() {
            let vk = sc.at(k, drvr);
            let Some(limit) = find_cap_limit(g, vk, limits) else { continue };
            for rf in [RISE, FALL] {
                let cap = g.load_cap(vk, std::borrow::Borrow::borrow(&parasitics[k]));
                let slack = limit - cap;
                let take = match &best {
                    None => true,
                    Some(b) => vyges_sta::fuzzy::less(slack, b.2) || (vyges_sta::fuzzy::equal(slack, b.2) && rf < b.3),
                };
                if take {
                    best = Some((cap, limit, slack, rf, k));
                }
            }
        }
    }
    match best {
        Some((cap, limit, slack, _, k)) => (cap, limit, slack, true, Some(k)),
        None => (0.0, -INF, INF, false, None),
    }
}

/// `PreChecks::checkCapLimit`'s threshold: the smallest input capacitance over every library's
/// buffers and inverters (liberty `dont_use` excluded, as `buffers()` / `inverters()` do).
pub fn min_cap_load(libs: &[Library]) -> f32 {
    let mut min = INF;
    for lib in libs {
        for cell in lib.cells.values().filter(|c| !c.dont_use && (c.is_buffer() || c.is_inverter())) {
            if let Some((input, _)) = cell.buffer_ports() {
                min = min.min(input.capacitance.iter().flatten().copied().fold(f32::MIN, f32::max));
            }
        }
    }
    min
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lib(default_max: Option<f32>) -> Library {
        Library { default_max_transition: default_max, ..Default::default() }
    }

    fn port(name: &str, direction: Direction, max_cap: Option<f32>) -> vyges_sta::liberty::Port {
        vyges_sta::liberty::Port { name: name.into(), direction, is_clock: false, capacitance: [[0.0; 2]; 2], function: None, max_transition: None, max_capacitance: max_cap, fanout_load: None, max_fanout: None, three_state: None, is_clock_gate_clock: false }
    }

    // A scene's graph lists an instance's pins in ITS library's port order, so scene k's vertex
    // for scene 0's is the one of the same name and direction — the same index when the orders
    // agree (the common case, answered without an index), found by name when they do not.
    #[test]
    fn scene_vertex_is_found_by_name_when_port_order_differs() {
        let cell = |ports: Vec<vyges_sta::liberty::Port>| vyges_sta::liberty::Cell { name: "c".into(), ports, ..Default::default() };
        let lib = |c: vyges_sta::liberty::Cell| Library { cells: [("c".to_string(), c)].into_iter().collect(), ..Default::default() };
        let ay = lib(cell(vec![port("A", Direction::Input, None), port("Y", Direction::Output, None)]));
        let ya = lib(cell(vec![port("Y", Direction::Output, None), port("A", Direction::Input, None)]));
        let nl = vyges_sta::netlist::Netlist { insts: vec![("u1".into(), "c".into())], ports: Vec::new(), nets: Vec::new() };
        let (l0, l1) = (vec![ay], vec![ya]);
        let same = [Graph::build(&l0, &nl).unwrap(), Graph::build(&l0, &nl).unwrap()];
        let sc = Scenes::new(&same);
        assert_eq!((sc.at(1, 0), sc.at(1, 1)), (0, 1));
        let swapped = [Graph::build(&l0, &nl).unwrap(), Graph::build(&l1, &nl).unwrap()];
        let sc = Scenes::new(&swapped);
        for v in 0..2 {
            let w = sc.at(1, v);
            assert_eq!((&swapped[1].vertices[w].name, swapped[1].vertices[w].is_driver), (&swapped[0].vertices[v].name, swapped[0].vertices[v].is_driver));
        }
        assert_ne!(sc.at(1, 0), 0);
    }

    // Rule (CheckCapacitances::findLimit, Resizer::maxLoad): "default to top (design) limit",
    // then `if (exists1 && (!exists || min_max->compare(limit, limit1)))` takes the other — for
    // max, the smaller; either alone stands.
    #[test]
    fn the_tighter_max_limit_wins_and_either_alone_stands() {
        assert_eq!(tighter_max(Some(5.0), Some(3.0)), Some(3.0));
        assert_eq!(tighter_max(Some(3.0), Some(5.0)), Some(3.0));
        assert_eq!(tighter_max(Some(3.0), None), Some(3.0));
        assert_eq!(tighter_max(None, Some(5.0)), Some(5.0));
        assert_eq!(tighter_max(None, None), None);
    }

    // Rule (Resizer::maxLoad): over the OUTPUT ports in order, the first with a limit — the
    // design's tightened by the port's — returns it, else 0. With a design limit the first output
    // port always answers, even when a later one has a tighter limit of its own.
    #[test]
    fn max_load_takes_the_first_output_port_with_any_limit() {
        let cell = vyges_sta::liberty::Cell { name: "c".into(), ports: vec![port("A", Direction::Input, Some(1.0)), port("Y", Direction::Output, None), port("Z", Direction::Output, Some(4.0))], ..Default::default() };
        let none = Limits::default();
        let design = |c: f32| Limits { design_max_capacitance: Some(c), ..Default::default() };
        assert_eq!(max_load(&cell, &none), 4.0);
        assert_eq!(max_load(&cell, &design(9.0)), 9.0);
        assert_eq!(max_load(&cell, &design(2.0)), 2.0);
        let bare = vyges_sta::liberty::Cell { name: "b".into(), ports: vec![port("Y", Direction::Output, None)], ..Default::default() };
        assert_eq!(max_load(&bare, &none), 0.0);
    }

    // Rule (CheckCapacitances::findLimit): on a top-level port, the design limit tightened by the
    // PORT's own SDC limit (`capacitanceLimit(port)`); on an instance pin, by its liberty port's —
    // a port limit never reaches an instance pin, and a looser one leaves the design's.
    #[test]
    fn a_port_cap_limit_tightens_only_its_own_port() {
        const LIB: &str = r#"library (t) { time_unit : "1ns"; capacitive_load_unit (1, pf);
          cell (buf) { pin (A) { direction : input; capacitance : 0.001; }
            pin (X) { direction : output; function : "A"; max_capacitance : 0.5; } } }"#;
        let libs = vec![Library::read(&vyges_sta::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let netlist = vyges_sta::netlist::Netlist {
            insts: vec![("b".into(), "buf".into())],
            ports: vec![("in".into(), vyges_sta::netlist::PortDir::Input), ("out".into(), vyges_sta::netlist::PortDir::Output)],
            nets: vec![
                vyges_sta::netlist::Net { name: "in".into(), pins: vec![Conn::Port(0), Conn::Inst(0, "A".into())] },
                vyges_sta::netlist::Net { name: "out".into(), pins: vec![Conn::Inst(0, "X".into()), Conn::Port(1)] },
            ],
        };
        let g = Graph::build(&libs, &netlist).unwrap();
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
        let limits = |design: Option<f32>, port: &[(&str, f32)]| Limits { design_max_capacitance: design, port_max_capacitance: port.iter().map(|&(p, c)| (p.to_string(), c)).collect(), ..Default::default() };
        assert_eq!(find_cap_limit(&g, v("in"), &limits(None, &[])), None);
        assert_eq!(find_cap_limit(&g, v("in"), &limits(None, &[("in", 2e-12)])), Some(2e-12));
        assert_eq!(find_cap_limit(&g, v("in"), &limits(Some(1e-12), &[("in", 2e-12)])), Some(1e-12), "a looser port limit loses");
        assert_eq!(find_cap_limit(&g, v("in"), &limits(Some(3e-12), &[("in", 2e-12)])), Some(2e-12));
        assert_eq!(find_cap_limit(&g, v("b/X"), &limits(None, &[("in", 1e-15), ("b/X", 1e-15)])), Some(0.5e-12), "an instance pin reads its liberty port only");
    }

    // Rule (set_max_capacitance, Tcl `capacitance_ui_sta`): the user value times the cap unit's
    // float scale in double, narrowed once. 1.1 fF lands on bits 269e86d6; narrowing 1.1 to float
    // first would give 269e86d7, one ulp up.
    #[test]
    fn a_design_cap_limit_rounds_where_the_command_does() {
        assert_eq!(user_cap_to_sta(1.1, 1e-15).to_bits(), 0x269e_86d6);
        assert_ne!(user_cap_to_sta(1.1, 1e-15).to_bits(), (1.1f32 * 1e-15f32).to_bits());
    }

    // Rule (Resizer::maxInputSlew(port, corner)): the limit is the CORNER port's; the fallback
    // default is the LINK port's library's, whatever the corner's says.
    #[test]
    fn a_corner_limit_falls_back_to_the_link_librarys_default() {
        let none = Limits::default();
        assert_eq!(max_input_slew_at(&lib(Some(5.0)), &lib(Some(9.0)), Direction::Input, None, &none), 5.0);
        assert_eq!(max_input_slew_at(&lib(Some(5.0)), &lib(Some(9.0)), Direction::Input, Some(3.0), &none), 3.0, "the corner port's own limit");
        assert_eq!(max_input_slew_at(&lib(Some(5.0)), &lib(Some(9.0)), Direction::Output, None, &none), 9.0, "an output's limit is the corner library's default (findLimit)");
    }

    // Rules (CheckSlews::findLimit, Resizer::maxInputSlew): the design limit, tightened by the
    // port's own; the library default counts for an OUTPUT in findLimit, and for any port once
    // there is no limit or it is 0; else INF.
    #[test]
    fn the_slew_limit_layers_design_port_and_library_default() {
        let none = Limits::default();
        let design = Limits { design_max_transition: Some(2.0), ..Default::default() };
        assert_eq!(max_input_slew(&lib(None), Direction::Input, None, &none), INF);
        assert_eq!(max_input_slew(&lib(Some(5.0)), Direction::Input, None, &none), 5.0, "input: the default, as maxInputSlew's fallback");
        assert_eq!(max_input_slew(&lib(Some(5.0)), Direction::Input, None, &design), 2.0, "the design limit exists, so no fallback");
        assert_eq!(max_input_slew(&lib(None), Direction::Input, Some(1.0), &design), 1.0, "the port's tighter limit");
        assert_eq!(max_input_slew(&lib(None), Direction::Input, Some(3.0), &design), 2.0, "a looser port limit loses");
        assert_eq!(find_slew_limit(&lib(Some(4.0)), Direction::Output, None, &design), (2.0, true), "an output's default, looser, loses");
        assert_eq!(find_slew_limit(&lib(Some(1.5)), Direction::Output, None, &design), (1.5, true));
        assert_eq!(max_input_slew(&lib(Some(7.0)), Direction::Input, Some(0.0), &none), 7.0, "a 0 limit falls back");
    }
}
