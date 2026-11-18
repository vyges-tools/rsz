// SPDX-License-Identifier: Apache-2.0
//! The design as the command-line engine edits it: an OpenDB database, the timer's netlist read
//! from it, the estimator's per-net cache with its invalid set, the incremental timer's log, the
//! journal levels, and — with global-route parasitics — the router alive across the repair. Shared
//! with the engines that repair through this crate (clock tree synthesis's `-repair_clock_nets`).

#![allow(clippy::too_many_arguments)]

use std::collections::{BTreeMap, BTreeSet, HashMap};

use vyges_opendb::Db;
use vyges_sta::graph::NetParasitics;

use crate::env::{est_stt, net_parasitic, PortCaps};
use crate::preamble::Libs;
use crate::timing::NetInfo;
use crate as vyges_rsz;
use crate::buffer_ports;

thread_local! {
    static LUT: vyges_stt::flute::lut::Lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

/// The design as `repair_design` edits it: the database, the timer's netlist read from it, and
/// the estimator's per-net cache with its invalid set (see [`vyges_rsz::design`]).
pub struct CliDesign<'a> {
    pub db: &'a mut Db,
    pub rc: &'a vyges_est::rc::Rc,
    pub liberty: &'a vyges_est::liberty::LibertyClocks,
    pub libs: &'a Libs,
    pub clock_sources: &'a [String],
    /// `Sim`'s constant pins (`inst/term` or port name), as the timer reads them.
    pub constants: HashMap<String, bool>,
    /// The SDC's constraint values (`set_case_analysis`, `set_logic_*`) the logic simulation seeds:
    /// the estimator's skips, when re-made after an edit, come from a simulation under them.
    pub constraints: vyges_est::sim::Constraints,
    /// The routing alpha in force while the repair runs.
    pub alpha: f32,
    /// Whether the estimator makes parasitics at all (not EST-0018).
    pub estimating: bool,
    /// Shared: a timing graph held across edits keeps the netlist it was built on (no copy).
    pub netlist: std::sync::Arc<vyges_sta::netlist::Netlist>,
    /// Moves on every refresh that changes [`CliDesign::netlist`], and only then
    /// ([`Design::netlist_generation`]).
    pub netlist_gen: u64,
    /// The database order of the netlist it was made for (and only that one, by pointer):
    /// [`CliDesign::refresh_touched`]'s edited path extends it rather than reading every name.
    pub inst_pos: Option<(std::sync::Weak<vyges_sta::netlist::Netlist>, DbOrder)>,
    /// Per scene.
    pub parasitics: Vec<HashMap<String, NetParasitics>>,
    pub invalid: BTreeSet<String>,
    pub info: NetInfo,
    /// `core_` and `core_exists_`.
    pub core: Option<(i32, i32, i32, i32)>,
    /// The port loads in force while this command runs: what an estimate made now is reduced against.
    pub port_caps: PortCaps,
    /// `set_propagated_clock` in force: a re-estimated clock net is a propagated one.
    pub propagated: bool,
    /// Per open journal level (odb's eco), the estimator's state as the level found it: undoing
    /// the level's edits restores the nets, and re-estimating a restored net gives back its old
    /// estimate, so the old state is put back whole.
    pub journal: Vec<JournalState>,
    /// The nets an SDC `set_load` names (`Sdc::hasNetWireCap`, keyed by the net object): such a
    /// net is constrained and is never merged away.
    pub sdc_nets: BTreeSet<String>,
    /// The instance pins an SDC `set_max_capacitance [get_pins …]` names (`Sdc::isConstrained(pin)`:
    /// a pin limit no check reads, but a constrained buffer pin keeps its buffer).
    pub sdc_pins: BTreeSet<String>,
    /// The edits as the incremental timer receives them.
    pub timer: TimerLog,
    /// Per instance, the cell its output-to-output edges carry after equivalent-arcs swaps.
    pub stale_out_arcs: BTreeMap<String, String>,
    /// The global router alive across the repair, when the parasitics are global-route ones.
    pub gr: Option<GrLive>,
    /// A router callback this engine refuses (raised at the next parasitics update).
    pub gr_error: Option<String>,
    /// Inside a journal undo on global-route parasitics: the estimator's callbacks put the nets
    /// the undo touched in its invalid set (`parasiticsInvalid`), the reference's only record of
    /// them — the restored routes are the guides', not the ones the estimate was made on.
    pub undoing: bool,
    /// The estimator's clock nets and constant pins, held across per-net estimates (see
    /// [`HeldSkips`]); `None` until the next estimate computes them again.
    pub skips: Option<HeldSkips>,
    /// `Resizer::design_area_`: the area `overMaxArea` reads. `initDesignArea` counts it; each
    /// edit moves it by its masters' areas, each narrowed to `float` (`designAreaIncr`).
    pub area: f64,
    /// The database's edit log is recording for [`HeldSkips`] (the timer's log, when on, is the
    /// same log).
    pub skip_log: bool,
}

/// What `estimate_net` skips — the clock network and the constant pins — computed once and held:
/// each is a whole-design walk, and a repair estimates one net per edit. An edit can change them
/// only through a net already in them — a buffer inserted on a non-constant, non-clock net is itself
/// neither, a resize keeps the function — so the held sets are dropped when an edit names one of
/// `nets` (the clock nets and every constant pin's net), and on any journal undo.
pub struct HeldSkips {
    pub skips: vyges_est::placement::Skips,
    pub nets: std::collections::HashSet<String>,
}

impl HeldSkips {
    /// An edit-log line naming a held net in any field (`|`, `,`, `;`, `=` separate them): an
    /// instance or pin name that equals a net name only costs a needless recompute.
    fn touched_by(&self, ev: &str) -> bool {
        edit_names_any(ev, &self.nets)
    }
}

/// A swap line that leaves the held skips standing: `Sta::replaceEquivCellBefore/After`
/// invalidate neither the clock network nor the simulator's constants, so a swap between
/// equivalent cells (`equiv(from, to)`) changes neither. `swap_before|inst|from|to|terms` carries
/// both masters and decides for the pair; its `swap_after` never drops them on its own.
fn swap_keeps_skips(ev: &str, equiv: impl Fn(&str, &str) -> bool) -> bool {
    if ev.starts_with("swap_after|") {
        return true;
    }
    let f: Vec<&str> = ev.splitn(5, '|').collect();
    matches!(f.as_slice(), ["swap_before", _, from, to, _] if equiv(from, to))
}

/// An edit-log line (`Db::edit_log_start`'s format) that names one of `nets` in any field.
///
/// 🔑 A connect's, a disconnect's and a net destroy's terminal list is not read: it is the pins
/// of the net the line names already (a port on it is the clock's own source net, held with it),
/// and reading it per event was O(fanout) — every load moved off a high-fanout net read the rest.
fn edit_names_any(ev: &str, nets: &std::collections::HashSet<String>) -> bool {
    let named = match ev.split_once('|').map(|(kind, _)| kind) {
        Some("iterm_connect" | "iterm_disconnect" | "bterm_connect" | "bterm_disconnect") => 3,
        Some("net_destroy") => 2,
        _ => usize::MAX,
    };
    ev.split('|').take(named).flat_map(|f| f.split([',', ';', '='])).any(|t| nets.contains(t))
}

/// `parasiticsInvalid(db_net)` for each net `IncrementalGRoute::updateRoutes` routed again
/// (upstream c5943b20d8f): each joins the estimator's invalid set and has its delays invalidated
/// with the rest — except `estimated_now`, the net `ensureWireParasitic` estimates and erases at
/// once, whose delays it does not invalidate.
fn absorb_rerouted(invalid: &mut BTreeSet<String>, delays: &mut BTreeSet<String>, routed: Vec<String>, estimated_now: Option<&str>) {
    for net in routed {
        if Some(net.as_str()) != estimated_now {
            delays.insert(net.clone());
        }
        invalid.insert(net);
    }
}

#[cfg(test)]
mod replay_tests {
    use super::*;
    use vyges_sta::netlist::Conn;

    fn c(k: usize, t: &str) -> Conn {
        Conn::Inst(k, t.into())
    }

    // Rule (odb dbITerm::connect / dbITerm::disconnectDbNet): a net's terminal list is a linked
    // list; connect puts the terminal at its head, disconnect unlinks it where it is. A buffer
    // inserted before two of three loads (its input connected to the net, then the loads moved
    // off) leaves [buf/A, the load not moved], and the new net lists its loads in reverse
    // connection order ahead of the buffer's output.
    #[test]
    fn a_net_list_replayed_from_the_callbacks_as_odb_keeps_it() {
        let pos = |i: &str| ["drv", "l1", "l2", "l3", "buf"].iter().position(|n| *n == i);
        let same = |k: usize| k;
        let old = [c(3, "A"), c(2, "A"), c(1, "A"), c(0, "Y")];
        let evs = [(true, "buf/A"), (false, "l1/A"), (false, "l3/A")];
        let (pins, joined) = replay_net_pins(&old, &same, &evs, &pos).expect("replayed");
        assert_eq!(pins, [c(4, "A"), c(2, "A"), c(0, "Y")]);
        assert_eq!(joined, [c(4, "A")]);
        // the new net, empty before: buf/Y, then l1 and l3 each at the head
        let (pins, joined) = replay_net_pins(&[], &same, &[(true, "buf/Y"), (true, "l1/A"), (true, "l3/A")], &pos).expect("replayed");
        assert_eq!(pins, [c(3, "A"), c(1, "A"), c(4, "Y")]);
        assert_eq!(joined, pins);
        // a terminal reconnected goes to the head, not back to its place
        let (pins, _) = replay_net_pins(&old, &same, &[(false, "l2/A"), (true, "l2/A")], &pos).expect("replayed");
        assert_eq!(pins, [c(2, "A"), c(3, "A"), c(1, "A"), c(0, "Y")]);
        // a pin leaving that was never on the list, an instance not placed, a port: read whole
        assert!(replay_net_pins(&old, &same, &[(false, "buf/A")], &pos).is_none());
        assert!(replay_net_pins(&old, &same, &[(true, "nowhere/A")], &pos).is_none());
        assert!(replay_net_pins(&[c(0, "Y"), Conn::Port(0)], &same, &[], &pos).is_none());
        // the kept indices move with the instances made before them
        let (pins, _) = replay_net_pins(&[c(1, "A")], &|k| k + 1, &[], &pos).expect("replayed");
        assert_eq!(pins, [c(2, "A")]);
    }

    #[test]
    fn only_instance_terminal_callbacks_replay() {
        let evs: Vec<String> = ["iterm_connect|b/A|n1|d/Y", "iterm_disconnect|l/A|n1|d/Y", "bterm_connect|p|n2|p", "net_create|n3", "swap_before|b|x|y|A=n1=d/Y"].iter().map(|s| s.to_string()).collect();
        let m = replayable_net_events(&evs);
        assert_eq!(m["n1"], Some(vec![(true, "b/A"), (false, "l/A")]));
        assert_eq!(m["n2"], None);
        assert_eq!(m["n3"], None);
        assert_eq!(m.len(), 3);
    }
}

#[cfg(test)]
mod rerouted_tests {
    use super::*;

    // Rule (EstimateParasitics::updateParasitics / ensureWireParasitic, upstream c5943b20d8f):
    // every net the incremental global route routes again is made invalid — re-estimated and its
    // delays invalidated — not only the edited ones; the net ensureWireParasitic is estimating
    // is erased from the invalid set right after, so its delays are left alone.
    #[test]
    fn every_rerouted_net_is_re_estimated() {
        let (mut invalid, mut delays) = (BTreeSet::from(["n1".to_string()]), BTreeSet::new());
        absorb_rerouted(&mut invalid, &mut delays, vec!["n1".into(), "n7".into()], None);
        assert_eq!(invalid, BTreeSet::from(["n1".to_string(), "n7".to_string()]));
        assert_eq!(delays, BTreeSet::from(["n1".to_string(), "n7".to_string()]));
        let (mut invalid, mut delays) = (BTreeSet::new(), BTreeSet::new());
        absorb_rerouted(&mut invalid, &mut delays, vec!["n1".into(), "n7".into()], Some("n1"));
        assert_eq!(delays, BTreeSet::from(["n7".to_string()]), "the net estimated now is not delay-invalidated");
        assert!(invalid.contains("n7"));
    }
}

#[cfg(test)]
mod held_skips_tests {
    use super::{edit_names_any, swap_keeps_skips};

    /// Rule (`Sta::replaceEquivCellBefore/After`: no `clkPinsInvalid`, no `constantsInvalid`): an
    /// equivalent swap keeps the held clock nets and constant pins even when its pins are on one
    /// (a register's clock pin); a swap to another function decides as any other line.
    #[test]
    fn an_equivalent_swap_keeps_the_held_skips() {
        let equiv = |a: &str, b: &str| a.starts_with("dff") == b.starts_with("dff");
        assert!(swap_keeps_skips("swap_before|r1|dff_1|dff_2|CK=clk=clk,r1/CK;Q=q=r1/Q", equiv));
        assert!(swap_keeps_skips("swap_after|r1|CK=clk=clk,r1/CK;Q=q=r1/Q", equiv));
        assert!(!swap_keeps_skips("swap_before|r1|dff_1|nand_2|CK=clk=clk,r1/CK", equiv));
        assert!(!swap_keeps_skips("iterm_connect|u1/A|clk|clk,u1/A", equiv));
    }

    /// The held skips are dropped by an edit naming a held net: as the connected net, a merged or
    /// destroyed one, or inside a swapped instance's `T=net=pins` terms. A connect's terminal
    /// list is the named net's own pins and is not read.
    #[test]
    fn an_edit_naming_a_held_net_in_any_field_drops_the_skips() {
        let nets = ["clk".to_string(), "const0".to_string()].into_iter().collect();
        assert!(edit_names_any("iterm_disconnect|u1/A|clk|u1/A,u2/A", &nets));
        assert!(edit_names_any("swap_after|u3|A=const0=u3/A,u9/Y;Y=n7=u3/Y", &nets));
        assert!(edit_names_any("net_merge|n5|clk", &nets));
        // A buffer on an ordinary net: the new net and its pins are neither clock nor constant.
        assert!(!edit_names_any("net_create|net12", &nets));
        assert!(!edit_names_any("iterm_connect|rebuffer1/A|n5|u4/Y,rebuffer1/A", &nets));
        // A name that only CONTAINS a held net is not one.
        assert!(!edit_names_any("iterm_connect|u1/A|clk_div|u1/A", &nets));
        assert!(edit_names_any("net_destroy|clk|u1/A|7", &nets));
        assert!(!edit_names_any("net_destroy|n9|clk|7", &nets));
        assert!(!edit_names_any("bterm_connect|p|n5|clk,u2/A", &nets));
    }
}

/// `IncrementalGRoute` over the repair: the router's state after `global_route` (its routes, its
/// graphs, the nets it holds), the options it ran with, and `dirty_nets_` by name — marked by
/// `GRouteDbCbk` from the database's callbacks, read in odb id order (a `PtrSet`).
pub struct GrLive {
    pub after: vyges_grt::global_route::AfterRoute,
    pub opts: vyges_grt::global_route::RouteOptions,
    pub dirty: BTreeSet<String>,
    /// The detailed placer's grid (`opendp_->initMacrosAndGrid()` in the repair's preamble), which
    /// `legalCellPos` snaps each new or resized instance to.
    pub grid: vyges_dpl::grid::Grid,
    /// The fixed nodes `initMacrosAndGrid` painted into that grid (`setFixedGridCells`), which
    /// `legalCellPos`' `pointOffMacro` reads to move a cell off a block.
    pub fixed: vyges_dpl::grid::FixedCells,
    /// What a buffered net on the route reads of the layers (`layerRC`, `viaResistance`).
    pub layers: std::sync::Arc<vyges_rsz::buffered_net::LayerRc>,
    /// The timer's net slacks for the next resistance-aware re-route (`set_route_slacks`).
    pub slacks: Option<std::collections::HashMap<String, f32>>,
}

/// The estimator's per-layer table and the cut layers' resistances, by routing level, at the first
/// corner — what `BufferedNet::wireRC` (on a layer) and `viaResistance` read.
pub fn gr_layer_rc(db: &Db, rc: &vyges_est::rc::Rc) -> vyges_rsz::buffered_net::LayerRc {
    let mut out = vyges_rsz::buffered_net::LayerRc::default();
    for name in db.tech_get_layers() {
        let level = db.layer_get_routing_level(&name);
        if level <= 0 {
            continue;
        }
        out.wire.insert(level, rc.layer_rc(db.layer_get_number(&name), 0));
        let cut = db.layer_get_upper_layer(&name);
        let res = (!cut.is_empty()).then(|| {
            let table = rc.layer_rc(db.layer_get_number(&cut), 0).0;
            if table == 0.0 { db.layer_get_resistance(&cut) } else { table }
        });
        out.cut_above.insert(level, res);
    }
    out
}

impl GrLive {
    /// `GlobalRouter::addDirtyNet`, as `GRouteDbCbk` calls it: a net set and not special, marked
    /// only when the router holds it.
    pub fn mark(&mut self, db: &Db, net: &str) {
        if !net.is_empty() && !db.net_is_special(net) && vyges_grt::global_route::holds(&self.after, net) {
            self.dirty.insert(net.to_string());
        }
    }

    /// `IncrementalGRoute::updateRoutes` → `updateDirtyRoutesFastRoute`: the dirty nets (odb id
    /// order) whose pins moved are released and routed again from the graphs as they stand.
    /// Returns every net it routed again (upstream c5943b20d8f: `updateRoutes` hands them back).
    pub fn update_routes(&mut self, db: &mut Db) -> Result<Vec<String>, String> {
        if self.dirty.is_empty() {
            return Ok(Vec::new());
        }
        let order: Vec<String> = db.net_names().into_iter().filter(|n| self.dirty.contains(n)).collect();
        self.dirty.clear();
        gr_trace(&format!("VYGR|upd|enter|{}", order.iter().map(|n| format!("{n};")).collect::<String>()));
        // A resistance-aware run's `updateSlacks` reads `sta_->slack(net)`: the timer's, handed
        // over before the update (`set_route_slacks`). Without them (a re-route outside
        // `updateParasitics`), a net a reroute marked is answered as constrained — which is all a
        // lone routed net's slack decides — and any other, or a second net, is NaN: refused.
        let slacks = self.slacks.take();
        let res_aware = self.after.res_aware_nets.clone();
        let asked = std::cell::Cell::new(0usize);
        let slack = move |net: &str| {
            let s = match &slacks {
                Some(m) => m.get(net).copied().unwrap_or(f32::NAN),
                None => {
                    asked.set(asked.get() + 1);
                    if asked.get() == 1 && res_aware.contains(net) { -1.0 } else { f32::NAN }
                }
            };
            rrt_trace(&format!("VYGU|net|{net}|{}", c_hex(s)));
            s
        };
        let routed = vyges_grt::global_route::update_dirty_routes_fast_route(db, &self.opts, &mut self.after, &order, &grt_stt, &grt_flutes, &mut |_, _| {}, true, Some(&slack)).map_err(|e| format!("incremental global route: {e}"))?;
        gr_trace(&format!("VYGR|upd|route|{}", routed.iter().map(|n| format!("{n};")).collect::<String>()));
        for net in &routed {
            gr_trace(&format!("VYGR|upd|seg|{net}|{}", gr_segs(&self.after, net)));
        }
        Ok(routed)
    }
}

/// A diagnostic: the net slacks a resistance-aware re-route read, appended to
/// `VYGES_RSZ_RRT_TRACE` in the shape of the reference's instrumented trace (`rsz-rrt-patch.py`).
pub fn rrt_trace(line: &str) {
    if let Ok(path) = std::env::var("VYGES_RSZ_RRT_TRACE") {
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = std::io::Write::write_all(&mut f, format!("{line}\n").as_bytes());
        }
    }
}

/// `%a` of a float as C prints it (the trace's raw values).
pub fn c_hex(v: f32) -> String {
    vyges_rsz::trace::c_hex(f64::from(v))
}

/// A route as the reference's trace prints it: `x0,y0,l0-x1,y1,l1;` per segment.
pub fn gr_segs(a: &vyges_grt::global_route::AfterRoute, net: &str) -> String {
    a.net_routes.iter().find(|r| r.name == net).map_or_else(String::new, |r| {
        r.segments.iter().map(|g| format!("{},{},{}-{},{},{};", g.init_x, g.init_y, g.init_layer, g.final_x, g.final_y, g.final_layer)).collect()
    })
}

/// A diagnostic: the router's and the estimator's calls inside the repair, appended to
/// `VYGES_RSZ_GR_TRACE` in the shape of the reference's instrumented trace (`VYGR|…`).
pub fn gr_trace(line: &str) {
    if let Ok(path) = std::env::var("VYGES_RSZ_GR_TRACE") {
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = std::io::Write::write_all(&mut f, format!("{line}\n").as_bytes());
        }
    }
}

/// What the incremental refresh knows of the database's order without reading every name.
///
/// The block iterates its instances and nets by id (`dbTable::next`), and a new object takes the
/// LAST freed id (`dbTable::create` pops the free list `dbTable::destroy` pushes). So after a
/// deletion — `remove_buffers`, or a database written after one — a new instance or net lands at
/// its id's rank, in the middle of the order, not at the end. A position is therefore an id's
/// rank: each listed instance's id by name and the listed instance ids ascending; the listed net
/// ids ascending and each listed net name's id.
pub struct DbOrder {
    inst_id: HashMap<String, u32>,
    inst_ids: Vec<u32>,
    net_ids: Vec<u32>,
    net_slot: HashMap<String, u32>,
    /// Held order ([`CliDesign::hold_order`]): the instances and nets made since, by id, at the
    /// positions they were appended to (after every listed one). The listed ranks stay put.
    appended_insts: HashMap<u32, usize>,
    appended_nets: HashMap<u32, usize>,
}

impl DbOrder {
    /// Read once per netlist made by a full read: `None` when the listed order is not the id order.
    fn read(db: &vyges_opendb::Db, nl: &vyges_sta::netlist::Netlist) -> Option<DbOrder> {
        let mut inst_id = HashMap::with_capacity(nl.insts.len());
        let mut inst_ids: Vec<u32> = Vec::with_capacity(nl.insts.len());
        for (n, _) in nl.insts.iter().map(|i| &**i) {
            let id = db.inst_id(n).ok()?;
            if inst_ids.last().is_some_and(|&last| id <= last) {
                return None;
            }
            inst_ids.push(id);
            inst_id.insert(n.clone(), id);
        }
        let mut net_ids: Vec<u32> = Vec::with_capacity(nl.nets.len());
        let mut net_slot = HashMap::with_capacity(nl.nets.len());
        for n in &nl.nets {
            let id = db.net_id(&n.name).ok()?;
            if net_ids.last().is_some_and(|&last| id <= last) {
                return None;
            }
            net_ids.push(id);
            net_slot.insert(n.name.clone(), id);
        }
        Some(DbOrder { inst_id, inst_ids, net_ids, net_slot, appended_insts: HashMap::new(), appended_nets: HashMap::new() })
    }

    /// A listed instance's position.
    fn inst_pos(&self, name: &str) -> Option<usize> {
        let id = self.inst_id.get(name)?;
        self.inst_ids.binary_search(id).ok().or_else(|| self.appended_insts.get(id).copied())
    }

    /// A listed net's position, by id.
    fn net_pos(&self, id: u32) -> Option<usize> {
        self.net_ids.binary_search(&id).ok().or_else(|| self.appended_nets.get(&id).copied())
    }

    /// Whether the netlist is in the database's order (nothing appended out of it).
    fn in_db_order(&self) -> bool {
        self.appended_insts.is_empty() && self.appended_nets.is_empty()
    }

    /// The database's order of a netlist this order describes: each position's id, ascending —
    /// `(instance positions, net positions)` in that order. The id ranks of the listed ones are
    /// their positions; an appended one's id is its key.
    fn db_permutation(&self) -> (Vec<usize>, Vec<usize>) {
        let by_id = |listed: &[u32], appended: &HashMap<u32, usize>| -> Vec<usize> {
            let mut all: Vec<(u32, usize)> = listed.iter().enumerate().map(|(k, &id)| (id, k)).chain(appended.iter().map(|(&id, &k)| (id, k))).collect();
            all.sort_unstable();
            all.into_iter().map(|(_, k)| k).collect()
        };
        (by_id(&self.inst_ids, &self.appended_insts), by_id(&self.net_ids, &self.appended_nets))
    }

    /// After [`DbOrder::db_permutation`] is applied: every id listed at its rank.
    fn settle(&mut self) {
        self.inst_ids.extend(self.appended_insts.drain().map(|(id, _)| id));
        self.inst_ids.sort_unstable();
        self.net_ids.extend(self.appended_nets.drain().map(|(id, _)| id));
        self.net_ids.sort_unstable();
    }

    /// A listed net's position, by name.
    fn net_slot_pos(&self, name: &str) -> Option<usize> {
        self.net_pos(*self.net_slot.get(name)?)
    }
}

/// [`CliDesign::edited_netlist`]'s answer: the next netlist, whether it differs from the one it
/// was made from, and (under `VYGES_RSZ_CHECK_REFRESH`) that one, to check against.
struct Edited {
    nl: vyges_sta::netlist::Netlist,
    changed: bool,
    old: Option<std::sync::Arc<vyges_sta::netlist::Netlist>>,
    /// Per net slot whose pins were replayed from the edit's callbacks, the pins that joined it:
    /// the only ones that can lack a pin id (every other kept its id when it came).
    joined: HashMap<usize, Vec<vyges_sta::netlist::Conn>>,
}

/// One touched net's pins after an edit: the kept list (`old`, its instance indices moved by
/// `shift`) with the edit's callbacks on that net applied in order as odb applies them to its
/// per-net terminal list — a connect puts the terminal at the head (`dbITerm::connect`), a
/// disconnect unlinks it (`dbITerm::disconnectDbNet`). `events`: (connect?, `inst/term`).
/// With the pins that joined (head first). `None` when a callback does not fit the list (a pin
/// left that was not on it, an instance not placed): the caller reads the net whole.
fn replay_net_pins(old: &[vyges_sta::netlist::Conn], shift: &dyn Fn(usize) -> usize, events: &[(bool, &str)], pos_of: &dyn Fn(&str) -> Option<usize>) -> Option<(Vec<vyges_sta::netlist::Conn>, Vec<vyges_sta::netlist::Conn>)> {
    use vyges_sta::netlist::Conn;
    // connected since, in connection order (the last is the head); left from the kept list
    let mut front: Vec<(usize, &str)> = Vec::new();
    let mut left: std::collections::HashSet<(usize, &str)> = std::collections::HashSet::new();
    for &(connect, pin) in events {
        let (inst, term) = pin.rsplit_once('/')?;
        let k = pos_of(inst)?;
        if connect {
            front.push((k, term));
        } else if let Some(j) = front.iter().rposition(|&(fk, ft)| fk == k && ft == term) {
            front.remove(j);
        } else if !left.insert((k, term)) {
            return None;
        }
    }
    let joined: Vec<Conn> = front.iter().rev().map(|&(k, t)| Conn::Inst(k, t.to_string())).collect();
    let mut pins = Vec::with_capacity(joined.len() + old.len());
    pins.extend(joined.iter().cloned());
    let mut gone = 0;
    for c in old {
        match c {
            Conn::Inst(i, p) => {
                let k = shift(*i);
                if !left.is_empty() && left.contains(&(k, p.as_str())) {
                    gone += 1;
                } else {
                    pins.push(Conn::Inst(k, p.clone()));
                }
            }
            Conn::Port(_) => return None,
        }
    }
    // every pin that left was on the kept list
    (gone == left.len()).then_some((pins, joined))
}

/// Where each new instance goes: `at[j]` is the number of listed instances before the `j`-th new
/// one (the new ones ascending by id); a listed instance at `k` moves up by the number of new ones
/// at or before it — its new position.
fn shifted(at: &[usize], k: usize) -> usize {
    k + at.partition_point(|&p| p <= k)
}

/// The netlist `nl` permuted (`inst_order[new] = old position`, the same for `net_order`), each
/// pin's instance index moved with its instance; a net whose indices all stay is shared.
fn reordered(nl: &vyges_sta::netlist::Netlist, inst_order: &[usize], net_order: &[usize]) -> vyges_sta::netlist::Netlist {
    use vyges_sta::netlist::{Conn, Net};
    let mut new_pos = vec![0usize; inst_order.len()];
    for (j, &k) in inst_order.iter().enumerate() {
        new_pos[k] = j;
    }
    let nets = net_order
        .iter()
        .map(|&k| {
            let n = &nl.nets[k];
            if n.pins.iter().all(|c| !matches!(c, Conn::Inst(i, _) if new_pos[*i] != *i)) {
                return n.clone();
            }
            let pins = n
                .pins
                .iter()
                .map(|c| match c {
                    Conn::Inst(i, p) => Conn::Inst(new_pos[*i], p.clone()),
                    Conn::Port(p) => Conn::Port(*p),
                })
                .collect();
            std::sync::Arc::new(Net { name: n.name.clone(), pins })
        })
        .collect();
    vyges_sta::netlist::Netlist { insts: inst_order.iter().map(|&k| nl.insts[k].clone()).collect(), ports: nl.ports.clone(), nets }
}

/// [`CliDesign::edited_netlist`] giving up, the reason traced (`VYGES_RSZ_ORDER_TRACE`).
/// The nets an edit-log line names (its connectivity on them may have changed).
fn event_nets(ev: &str) -> Vec<&str> {
    let f: Vec<&str> = ev.split('|').collect();
    match f.as_slice() {
        ["iterm_connect" | "iterm_disconnect" | "bterm_connect" | "bterm_disconnect" | "iterm_destroy", _, net, ..] => vec![*net],
        ["net_create" | "net_destroy", net, ..] => vec![*net],
        ["net_merge", a, b] => vec![*a, *b],
        _ => Vec::new(),
    }
}

/// Per net, the instance-terminal callbacks on it in order (connect?, `inst/term`), from
/// `events`; a net any other callback names (a port's, a destroy, a merge, a creation) maps to
/// `None`: its list is read whole.
fn replayable_net_events(events: &[String]) -> HashMap<&str, Option<Vec<(bool, &str)>>> {
    let mut out: HashMap<&str, Option<Vec<(bool, &str)>>> = HashMap::new();
    for ev in events {
        let f: Vec<&str> = ev.split('|').collect();
        match f.as_slice() {
            [kind @ ("iterm_connect" | "iterm_disconnect"), pin, net, _] => {
                if let Some(list) = out.entry(*net).or_insert_with(|| Some(Vec::new())) {
                    list.push((*kind == "iterm_connect", *pin));
                }
            }
            _ => {
                for net in event_nets(ev) {
                    out.insert(net, None);
                }
            }
        }
    }
    out
}

fn bail<T>(why: &str) -> Option<T> {
    if std::env::var_os("VYGES_RSZ_ORDER_TRACE").is_some() {
        eprintln!("VYGORDER|bail|{why}");
    }
    None
}

/// The incremental timer's side of the edits: the database's callbacks as read, and the
/// estimator's invalid set as ITS callbacks keep it (`parasitics_invalid_`) — apart from
/// [`CliDesign::invalid`], which a journal undo restores whole, where the reference's undo
/// invalidates every net it touches.
#[derive(Default)]
pub struct TimerLog {
    pub on: bool,
    /// The netlist's order held while a repair edits it ([`Design::hold_netlist_order`]): what an
    /// edit makes is appended, not put at its id's rank, so no listed index moves. After a
    /// deletion every new id is mid-order, and moving every index past it was O(design) per
    /// repeater here and in every timing graph — the reference's objects have no positions.
    pub hold_order: bool,
    /// The netlist slots the edits since the last take replaced or appended (held order), or
    /// `slots_lost`: a slot may have moved (a full read, an edit out of held order).
    pub slots: vyges_sta::graph::SlotDelta,
    pub slots_lost: bool,
    pub events: Vec<String>,
    pub est_invalid: BTreeSet<String>,
    pub updated: Vec<String>,
    /// Copies of the database's callbacks taken since the last netlist refresh: what
    /// [`CliDesign::refresh_touched`] replays on a touched net's kept pins (odb's own list
    /// operations: `dbITerm::connect` puts the terminal at the head of the net's list,
    /// `disconnectDbNet` unlinks it), so a repeater costs its moved pins, not its net.
    pub since_refresh: Vec<String>,
    /// Callbacks a refresh took from the database that [`CliDesign::pull_edits`] has not seen.
    pub raw_pending: Vec<String>,
    /// The log recorded every callback since the last refresh (on throughout, not restarted).
    pub replay_ok: bool,
    /// Nets a callback named that no refresh has read since: read again whole.
    pub unsure_nets: std::collections::HashSet<String>,
}

/// What a journal level puts back on undo besides the database (the netlist is read from the
/// database again, as the timer's network reads it live).
pub struct JournalState {
    pub parasitics: Vec<HashMap<String, NetParasitics>>,
    pub invalid: BTreeSet<String>,
    pub sdc_nets: BTreeSet<String>,
}

/// The nets an instance's deletion disconnects besides the ones the edit already accounts for
/// (`skip`): every connected iterm's net, in iterm order, an unconnected iterm (empty) left out.
fn nets_left_by_deletion(iterm_nets: impl Iterator<Item = String>, skip: &[&str]) -> Vec<String> {
    iterm_nets.filter(|n| !n.is_empty() && !skip.contains(&n.as_str())).collect()
}

impl<'a> CliDesign<'a> {
    /// A design over `db` from what every command must supply; every other field starts empty,
    /// off or zero, with the routing alpha at `set_routing_alpha`'s default (0.3) and the
    /// estimator on. A caller sets the rest by name with struct-update syntax
    /// (`CliDesign { constants, ..CliDesign::new(…) }`), so a field added here reaches every
    /// caller, in this crate or a consumer, at its default instead of breaking the literal.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        db: &'a mut Db,
        rc: &'a vyges_est::rc::Rc,
        liberty: &'a vyges_est::liberty::LibertyClocks,
        libs: &'a Libs,
        clock_sources: &'a [String],
        netlist: vyges_sta::netlist::Netlist,
        parasitics: Vec<HashMap<String, NetParasitics>>,
        info: NetInfo,
    ) -> Self {
        CliDesign {
            db,
            rc,
            liberty,
            libs,
            clock_sources,
            constants: HashMap::new(),
            constraints: vyges_est::sim::Constraints::default(),
            alpha: 0.3,
            estimating: true,
            netlist: std::sync::Arc::new(netlist),
            netlist_gen: 0,
            inst_pos: None,
            parasitics,
            invalid: BTreeSet::new(),
            info,
            core: None,
            port_caps: PortCaps::new(),
            propagated: false,
            journal: Vec::new(),
            sdc_nets: BTreeSet::new(),
            sdc_pins: BTreeSet::new(),
            timer: TimerLog::default(),
            stale_out_arcs: BTreeMap::new(),
            gr: None,
            gr_error: None,
            undoing: false,
            skips: None,
            area: 0.0,
            skip_log: false,
        }
    }
}

impl CliDesign<'_> {
    /// The nets on an instance's other iterms (its supply pins, when the design connects them):
    /// deleting the instance disconnects each, and `OdbCallBack::inDbITermPostDisconnect`
    /// invalidates the net whatever it is — VDD and VSS join the estimator's pending nets (the
    /// policy's `pendingNetCount`) until the next update finds nothing to estimate on them.
    fn other_iterm_nets(&self, inst: &str, skip: &[&str]) -> Result<Vec<String>, String> {
        let info = self.db.inst_iterm_info(inst).map_err(|e| e.to_string())?;
        Ok(nets_left_by_deletion(info.into_iter().map(|(_, _, n)| n), skip))
    }

    /// `Resizer::designAreaIncr(float delta)`: the area narrowed to `float`, added to the double.
    fn area_incr(&mut self, delta: f64) {
        self.area += f64::from(delta as f32);
    }

    /// After an edit: the timer's netlist and the pin order read again from the database.
    pub fn refresh(&mut self) -> Result<(), String> {
        if self.timer.on || self.skip_log {
            let raw = self.take_edit_log();
            self.timer.raw_pending.extend(raw);
        }
        self.timer.since_refresh.clear();
        self.timer.unsure_nets.clear();
        self.timer.replay_ok = self.timer.on || self.skip_log;
        let nl = vyges_grt::timer::netlist(self.db);
        if nl != *self.netlist {
            self.netlist_gen += 1;
        }
        self.netlist = std::sync::Arc::new(nl);
        self.timer.slots_lost = true;
        self.info = net_info(self.db, &self.netlist)?;
        Ok(())
    }

    /// The database order kept for the netlist as it is now (none after a full read until the next
    /// edit reads it): positions by name without a pass over the netlist. A name it does not
    /// answer for is looked for by the pass.
    fn kept_order(&self) -> Option<&DbOrder> {
        match self.inst_pos.as_ref() {
            Some((w, ord)) if std::ptr::eq(w.as_ptr(), std::sync::Arc::as_ptr(&self.netlist)) => Some(ord),
            _ => None,
        }
    }

    /// [`Design::hold_netlist_order`]: on, an edit appends what it makes; off, a netlist held out
    /// of the database's order is put back in it (one permutation, no read of the database).
    pub fn hold_order(&mut self, hold: bool) -> Result<(), String> {
        self.timer.hold_order = hold;
        if hold {
            return Ok(());
        }
        let Some((w, ord)) = self.inst_pos.as_mut() else { return Ok(()) };
        if ord.in_db_order() {
            return Ok(());
        }
        if !std::sync::Weak::ptr_eq(w, &std::sync::Arc::downgrade(&self.netlist)) {
            // the netlist was read again since (a full read is in the database's order)
            self.inst_pos = None;
            return Ok(());
        }
        let (io, no) = ord.db_permutation();
        ord.settle();
        self.netlist = std::sync::Arc::new(reordered(&self.netlist, &io, &no));
        self.timer.slots_lost = true;
        self.netlist_gen += 1;
        if let Some((w, _)) = self.inst_pos.as_mut() {
            *w = std::sync::Arc::downgrade(&self.netlist);
        }
        if std::env::var_os("VYGES_RSZ_CHECK_REFRESH").is_some() && vyges_grt::timer::netlist(self.db) != *self.netlist {
            return Err("hold_order: the settled netlist differs from the full read".into());
        }
        Ok(())
    }

    /// [`CliDesign::refresh`] after an edit that touched only `touched` nets and added `new_insts`:
    /// the same netlist and pin order, read again only where they can have changed.
    ///
    /// ⛔ The full read cost one terminal lookup BY NAME per pin of the design on every edit —
    /// O(design) per repeater, O(design²) over a repair (picorv32 `repair_design`: 13 min, 4 of 8
    /// samples in `dbBlock::findInst` under `net_info`). The order is still the database's own (one
    /// bulk read of the instance and net names), untouched nets keep their pins re-indexed, and the
    /// touched nets are read as the full read reads them. `VYGES_RSZ_CHECK_REFRESH=1` also runs the
    /// full read and fails on any difference; `VYGES_RSZ_FULL_REFRESH=1` uses it instead.
    pub fn refresh_touched(&mut self, new_insts: &[String], touched: &BTreeSet<String>) -> Result<(), String> {
        let _p = crate::prof::Span::new(0);
        crate::prof::items(0, touched.len() as u64);
        if std::env::var_os("VYGES_RSZ_FULL_REFRESH").is_some() {
            return self.refresh();
        }
        // the edit's callbacks, for the replay (the timer still takes them, in order)
        if self.timer.on || self.skip_log {
            let raw = self.take_edit_log();
            self.timer.raw_pending.extend(raw);
        }
        let r = match self.edited_netlist(new_insts, touched) {
            Some(fast) => self.refresh_finish(fast, new_insts, touched),
            None => self.refresh_read(new_insts, touched),
        };
        self.refreshed(touched);
        r
    }

    /// [`CliDesign::refresh_touched`]'s general path: every instance and net name read again.
    fn refresh_read(&mut self, new_insts: &[String], touched: &BTreeSet<String>) -> Result<(), String> {
        use vyges_sta::netlist::{Conn, Net, Netlist};
        // ⚠️ O(design): every instance and net name read again
        if std::env::var_os("VYGES_RSZ_ORDER_TRACE").is_some() {
            eprintln!("VYGORDER|general");
        }
        let old = self.netlist.clone();
        let old_master: HashMap<&str, &str> = old.insts.iter().map(|i| &**i).map(|(n, m)| (n.as_str(), m.as_str())).collect();
        let mut nl = Netlist::default();
        let mut inst_index: HashMap<String, usize> = HashMap::new();
        for inst in self.db.inst_names() {
            let master = match old_master.get(inst.as_str()) {
                Some(m) if !new_insts.contains(&inst) => m.to_string(),
                _ => self.db.inst_master(&inst),
            };
            inst_index.insert(inst.clone(), nl.insts.len());
            nl.insts.push(vyges_sta::netlist::inst(inst, master));
        }
        nl.ports = old.ports.clone();
        let port_index: HashMap<&str, usize> = nl.ports.iter().enumerate().map(|(i, (n, _))| (n.as_str(), i)).collect();
        let old_nets: HashMap<&str, &Net> = old.nets.iter().map(|n| (n.name.as_str(), &**n)).collect();
        let net_names = self.db.net_names();
        for net in &net_names {
            if touched.contains(net) {
                if self.db.net_is_special(net) {
                    continue;
                }
                let mut pins = Vec::new();
                for iterm in self.db.net_iterms(net) {
                    let (inst, term) = iterm.rsplit_once('/').unwrap_or((&iterm, ""));
                    if let Some(&i) = inst_index.get(inst) {
                        pins.push(Conn::Inst(i, term.to_string()));
                    }
                }
                for bterm in self.db.net_bterms(net) {
                    if let Some(&p) = port_index.get(bterm.as_str()) {
                        pins.push(Conn::Port(p));
                    }
                }
                if !pins.is_empty() {
                    nl.nets.push(Net { name: net.clone(), pins }.into());
                }
            } else if let Some(n) = old_nets.get(net.as_str()) {
                let pins = n
                    .pins
                    .iter()
                    .map(|c| match c {
                        Conn::Inst(i, p) => Conn::Inst(inst_index[&old.insts[*i].0], p.clone()),
                        Conn::Port(p) => Conn::Port(*p),
                    })
                    .collect();
                nl.nets.push(Net { name: net.clone(), pins }.into());
            }
        }
        // the pin order and the per-net / per-instance flags: only what the edit can have changed
        let live: BTreeSet<&str> = net_names.iter().map(String::as_str).collect();
        self.info.dont_touch.retain(|n| live.contains(n.as_str()));
        self.info.abutment.retain(|n| live.contains(n.as_str()));
        self.info.unplaced_sig_type.retain(|n| live.contains(n.as_str()));
        for net in nl.nets.iter().filter(|n| touched.contains(&n.name)) {
            for c in &net.pins {
                let name = nl.pin_name(c);
                let id = match c {
                    Conn::Inst(i, pin) => u64::from(self.db.iterm_id(&nl.insts[*i].0, pin).map_err(|e| e.to_string())?) << 1,
                    Conn::Port(_) => (u64::from(self.db.bterm_id(&name).map_err(|e| e.to_string())?) << 1) | 1,
                };
                self.info.pin_id.insert(name, id);
            }
            if self.db.net_is_do_not_touch(&net.name) {
                self.info.dont_touch.insert(net.name.clone());
            } else {
                self.info.dont_touch.remove(&net.name);
            }
            if self.db.net_is_connected_by_abutment(&net.name) {
                self.info.abutment.insert(net.name.clone());
            } else {
                self.info.abutment.remove(&net.name);
            }
            if crate::timing::is_valid_sig_type(&self.db.net_get_sig_type(&net.name)) {
                self.info.unplaced_sig_type.remove(&net.name);
            } else {
                self.info.unplaced_sig_type.insert(net.name.clone());
            }
        }
        for i in new_insts {
            if self.db.inst_is_do_not_touch(i) {
                self.info.dont_touch_insts.insert(i.clone());
            }
            if self.db.inst_is_fixed(i) {
                self.info.fixed_insts.insert(i.clone());
            }
        }
        // Changed, read from what this refresh could change: the instance and net counts, the
        // re-read instances' masters, and the touched nets' pins (an untouched net keeps its pins,
        // re-indexed — the same pins while the instances are the same).
        let changed = nl.insts.len() != old.insts.len()
            || nl.nets.len() != old.nets.len()
            || new_insts.iter().any(|i| old_master.get(i.as_str()).copied() != nl.insts.get(inst_index[i]).map(|e| e.1.as_str()))
            || {
                let new_nets: HashMap<&str, &Net> = nl.nets.iter().filter(|n| touched.contains(&n.name)).map(|n| (n.name.as_str(), &**n)).collect();
                touched.iter().any(|t| old_nets.get(t.as_str()).map(|n| &n.pins) != new_nets.get(t.as_str()).map(|n| &n.pins))
            };
        let check = std::env::var_os("VYGES_RSZ_CHECK_REFRESH").is_some();
        if check && changed != (nl != *old) {
            return Err(format!("refresh_touched: change test says {changed}, the netlists say {}", nl != *old));
        }
        drop(old_nets);
        drop(old_master);
        if changed {
            self.netlist_gen += 1;
        }
        self.netlist = std::sync::Arc::new(nl);
        self.timer.slots_lost = true;
        // ⚠️ O(design): every instance and net id, read once so the next edit is extended in place
        self.inst_pos = DbOrder::read(self.db, &self.netlist).map(|ord| (std::sync::Arc::downgrade(&self.netlist), ord));
        if self.inst_pos.is_none() && std::env::var_os("VYGES_RSZ_ORDER_TRACE").is_some() {
            eprintln!("VYGORDER|read|the listed order is not the id order");
        }
        if check {
            let full = vyges_grt::timer::netlist(self.db);
            let info = net_info(self.db, &full)?;
            if full != *self.netlist || info != self.info {
                return Err(format!("refresh_touched: differs from the full read (netlist {}, info {})", full == *self.netlist, info == self.info));
            }
        }
        Ok(())
    }

    /// [`CliDesign::refresh_touched`] when the edit only MADE instances and nets and reconnected
    /// pins: every listed instance and net keeps its id, so its place in the id order, and only
    /// the edit is read ([`DbOrder`]) — the full read cost every instance and net name per
    /// repeater, O(design²) over a repair.
    ///
    /// A new instance goes at its id's rank, which after a deletion is in the middle of the order
    /// (odb reuses the last freed id): the instances after it move up one and every pin naming one
    /// of them is re-indexed in place — an index pass over the pins, not a re-read by name. A
    /// touched net found at a listed id replaces that slot (a rename keeps the id); one at a new id
    /// goes at that id's rank. The netlist is edited in place when nothing else holds it.
    /// `None` (nothing changed) when the database's order does not bear that out: a new id already
    /// listed, a touched net gone, special or left pinless, a listed net renamed to a name not
    /// touched.
    fn edited_netlist(&mut self, new_insts: &[String], touched: &BTreeSet<String>) -> Option<Edited> {
        use vyges_sta::netlist::{Conn, Net};
        let mut ord = match self.inst_pos.take() {
            Some((w, ord)) if std::sync::Weak::ptr_eq(&w, &std::sync::Arc::downgrade(&self.netlist)) => ord,
            // ⛔ Not read here, from the netlist before the edit: the edit renamed nets (a buffer
            // at a port swaps two names), so an old name may now find another net's id. The
            // general path reads the order after its full read, and the next edit extends it.
            Some(_) => return bail("the order kept is another netlist's"),
            None => return bail("no order kept"),
        };
        // The plan: everything read and checked before anything is changed.
        let old = &*self.netlist;
        // an instance re-read (the general path's `new_insts`) takes its master from the database;
        // one not listed is new
        let mut remaster = Vec::new();
        let mut made = Vec::new();
        for i in new_insts {
            match ord.inst_pos(i) {
                Some(k) => remaster.push((k, self.db.inst_master(i))),
                None => made.push((self.db.inst_id(i).ok().or_else(|| bail("a new instance not found"))?, i.clone())),
            }
        }
        made.sort();
        let mut at = Vec::with_capacity(made.len());
        for (j, (id, _)) in made.iter().enumerate() {
            if j > 0 && made[j - 1].0 == *id {
                return bail("an instance named twice");
            }
            if ord.appended_insts.contains_key(id) {
                return bail("a new instance at a listed id");
            }
            match ord.inst_ids.binary_search(id) {
                Ok(_) => return bail("a new instance at a listed id"),
                // held order: after every listed instance, whatever its id
                Err(_) if self.timer.hold_order => at.push(old.insts.len()),
                Err(p) => at.push(p),
            }
        }
        let made_pos: HashMap<&str, usize> = made.iter().enumerate().map(|(j, (_, n))| (n.as_str(), at[j] + j)).collect();
        let pos_of = |inst: &str| made_pos.get(inst).copied().or_else(|| ord.inst_pos(inst).map(|k| shifted(&at, k)));
        let port_index: HashMap<&str, usize> = old.ports.iter().enumerate().map(|(i, (n, _))| (n.as_str(), i)).collect();
        let mut replaced: Vec<(usize, u32, Net)> = Vec::new();
        let mut added: Vec<(u32, Net)> = Vec::new();
        // a listed net's pins from the edit's own callbacks when the log saw them all (no journal
        // undo: its restores are not list operations), else read whole
        let replaying = self.timer.replay_ok && (self.timer.on || self.skip_log) && self.journal.is_empty() && !self.undoing && std::env::var_os("VYGES_RSZ_NO_REPLAY").is_none();
        let net_events = if replaying { replayable_net_events(&self.timer.since_refresh) } else { HashMap::new() };
        let shift = |i: usize| shifted(&at, i);
        let mut joined: HashMap<usize, Vec<Conn>> = HashMap::new();
        for net in touched {
            let Ok(id) = self.db.net_id(net) else { return bail("a touched net gone") };
            let slot = ord.net_pos(id);
            if self.db.net_is_special(net) {
                if slot.is_some() {
                    return bail("a listed net made special");
                }
                continue;
            }
            let fits = |k: usize| replaying && !self.timer.unsure_nets.contains(net) && old.nets.get(k).is_some_and(|n| &n.name == net);
            if let Some(k) = slot.filter(|&k| fits(k)) {
                let none = Vec::new();
                let evs = match net_events.get(net.as_str()) {
                    Some(Some(list)) => Some(list),
                    Some(None) => None,
                    None => Some(&none),
                };
                if let Some((pins, came)) = evs.and_then(|evs| replay_net_pins(&old.nets[k].pins, &shift, evs, &pos_of)) {
                    if pins.is_empty() {
                        return bail("a listed net left pinless");
                    }
                    joined.insert(k, came);
                    replaced.push((k, id, Net { name: net.clone(), pins }));
                    continue;
                }
            }
            let mut pins = Vec::new();
            let iterms = self.db.net_iterms(net);
            crate::prof::items(1, iterms.len() as u64);
            if iterms.len() >= 100 && std::env::var_os("VYGES_RSZ_REFRESH_TRACE").is_some() {
                eprintln!("VYGREFRESH|{net}|{}|{}", iterms.len(), new_insts.first().map_or("", |s| s.as_str()));
            }
            for iterm in iterms {
                let (inst, term) = iterm.rsplit_once('/').unwrap_or((&iterm, ""));
                let Some(k) = pos_of(inst) else { return bail("a pin on an instance not listed") };
                pins.push(Conn::Inst(k, term.to_string()));
            }
            for bterm in self.db.net_bterms(net) {
                if let Some(&p) = port_index.get(bterm.as_str()) {
                    pins.push(Conn::Port(p));
                }
            }
            match slot {
                Some(_) if pins.is_empty() => return bail("a listed net left pinless"),
                Some(k) => replaced.push((k, id, Net { name: net.clone(), pins })),
                None if pins.is_empty() => {}
                None => added.push((id, Net { name: net.clone(), pins })),
            }
        }
        // a listed net whose name was touched but is not at its slot any more was renamed to a
        // name the edit did not name — its slot would keep the stale name
        let replaced_slots: std::collections::HashSet<usize> = replaced.iter().map(|(k, ..)| *k).collect();
        if touched.iter().any(|t| ord.net_slot_pos(t).is_some_and(|k| !replaced_slots.contains(&k))) {
            return bail("a listed net renamed to a name not touched");
        }
        // Changed: a count, a re-read master, a touched net's pins (by name, as listed before and
        // after; a touched name listed before is at a replaced slot, checked above).
        let master_changed = remaster.iter().any(|(k, m)| *m != old.insts[*k].1);
        let changed = master_changed || !made.is_empty() || !added.is_empty() || {
            let before: HashMap<&str, &Vec<Conn>> = touched.iter().filter_map(|t| ord.net_slot_pos(t)).map(|k| (old.nets[k].name.as_str(), &old.nets[k].pins)).collect();
            let after: HashMap<&str, &Vec<Conn>> = replaced.iter().map(|(_, _, n)| (n.name.as_str(), &n.pins)).collect();
            before != after
        };
        drop(made_pos);
        // The edit, in place unless another holder (a timing graph) keeps the old generation.
        let kept = std::env::var_os("VYGES_RSZ_CHECK_REFRESH").is_some().then(|| self.netlist.clone());
        let mut nl = std::sync::Arc::try_unwrap(std::mem::take(&mut self.netlist)).unwrap_or_else(|shared| {
            // ⚠️ O(design) pointers copied (and every pin re-indexed below copies its net)
            if std::env::var_os("VYGES_RSZ_ORDER_TRACE").is_some() {
                eprintln!("VYGORDER|shared|{}", std::sync::Arc::strong_count(&shared));
            }
            (*shared).clone()
        });
        // the slots this edit replaces or appends (no slot moves in held order)
        let held = self.timer.hold_order;
        if held {
            let d = &mut self.timer.slots;
            d.insts.extend(remaster.iter().map(|(k, _)| *k));
            d.insts.extend((0..made.len()).map(|j| at[j] + j));
            d.nets.extend(replaced.iter().map(|(k, ..)| *k));
            d.nets.extend(nl.nets.len()..nl.nets.len() + added.len());
        } else {
            self.timer.slots_lost = true;
        }
        for (k, m) in remaster {
            std::sync::Arc::make_mut(&mut nl.insts[k]).1 = m;
        }
        if let Some(&first) = at.first().filter(|&&p| p < nl.insts.len()) {
            for net in &mut nl.nets {
                if net.pins.iter().any(|c| matches!(c, Conn::Inst(i, _) if *i >= first)) {
                    for c in &mut std::sync::Arc::make_mut(net).pins {
                        if let Conn::Inst(i, _) = c {
                            *i = shifted(&at, *i);
                        }
                    }
                }
            }
        }
        let trace = std::env::var_os("VYGES_RSZ_ORDER_TRACE").is_some();
        for (j, (id, inst)) in made.into_iter().enumerate() {
            let k = at[j] + j;
            if trace && k < nl.insts.len() {
                eprintln!("VYGORDER|mid|{inst}|{k}");
            }
            let master = self.db.inst_master(&inst);
            if self.timer.hold_order {
                ord.appended_insts.insert(id, k);
            } else {
                ord.inst_ids.insert(k, id);
            }
            ord.inst_id.insert(inst.clone(), id);
            nl.insts.insert(k, vyges_sta::netlist::inst(inst, master));
        }
        // every old name out before any new one in: two slots may trade names
        for (k, ..) in &replaced {
            ord.net_slot.remove(&nl.nets[*k].name);
        }
        for (k, id, r) in replaced {
            ord.net_slot.insert(r.name.clone(), id);
            nl.nets[k] = r.into();
        }
        added.sort_by_key(|(id, _)| *id);
        for (id, n) in added {
            let k = if self.timer.hold_order {
                ord.appended_nets.insert(id, nl.nets.len());
                nl.nets.len()
            } else {
                let k = ord.net_ids.partition_point(|&x| x < id);
                ord.net_ids.insert(k, id);
                k
            };
            ord.net_slot.insert(n.name.clone(), id);
            nl.nets.insert(k, n.into());
        }
        self.inst_pos = Some((std::sync::Weak::new(), ord));
        Some(Edited { nl, changed, old: kept, joined })
    }

    /// The rest of [`CliDesign::refresh_touched`] on an edited netlist: the per-net and
    /// per-instance flags, the generation.
    fn refresh_finish(&mut self, Edited { nl, changed, old, joined }: Edited, new_insts: &[String], touched: &BTreeSet<String>) -> Result<(), String> {
        use vyges_sta::netlist::Conn;
        // the touched nets by their slots (the edited path's own map), not by a pass over every net
        let slots: Vec<usize> = match self.inst_pos.as_ref() {
            Some((_, ord)) => touched.iter().filter_map(|t| ord.net_slot_pos(t)).collect(),
            None => nl.nets.iter().enumerate().filter(|(_, n)| touched.contains(&n.name)).map(|(k, _)| k).collect(),
        };
        for (k, net) in slots.iter().map(|&k| (k, &nl.nets[k])) {
            // a replayed net: only the pins that joined it can be new
            let pins = joined.get(&k).unwrap_or(&net.pins);
            for c in pins {
                let name = nl.pin_name(c);
                // a terminal keeps its id when it moves to another net: only a new one is read
                if self.info.pin_id.contains_key(&name) {
                    continue;
                }
                let id = match c {
                    Conn::Inst(i, pin) => u64::from(self.db.iterm_id(&nl.insts[*i].0, pin).map_err(|e| e.to_string())?) << 1,
                    Conn::Port(_) => (u64::from(self.db.bterm_id(&name).map_err(|e| e.to_string())?) << 1) | 1,
                };
                self.info.pin_id.insert(name, id);
            }
            if self.db.net_is_do_not_touch(&net.name) {
                self.info.dont_touch.insert(net.name.clone());
            } else {
                self.info.dont_touch.remove(&net.name);
            }
            if self.db.net_is_connected_by_abutment(&net.name) {
                self.info.abutment.insert(net.name.clone());
            } else {
                self.info.abutment.remove(&net.name);
            }
            if crate::timing::is_valid_sig_type(&self.db.net_get_sig_type(&net.name)) {
                self.info.unplaced_sig_type.remove(&net.name);
            } else {
                self.info.unplaced_sig_type.insert(net.name.clone());
            }
        }
        for i in new_insts {
            if self.db.inst_is_do_not_touch(i) {
                self.info.dont_touch_insts.insert(i.clone());
            }
            if self.db.inst_is_fixed(i) {
                self.info.fixed_insts.insert(i.clone());
            }
        }
        if let Some(old) = &old {
            if changed != (nl != **old) {
                return Err(format!("refresh_touched (edited): change test says {changed}, the netlists say {}", nl != **old));
            }
        }
        if changed {
            self.netlist_gen += 1;
        }
        self.netlist = std::sync::Arc::new(nl);
        if let Some((w, _)) = self.inst_pos.as_mut() {
            *w = std::sync::Arc::downgrade(&self.netlist);
        }
        if old.is_some() {
            let full = vyges_grt::timer::netlist(self.db);
            let info = net_info(self.db, &full)?;
            let ours = match self.inst_pos.as_ref() {
                Some((_, ord)) if !ord.in_db_order() => {
                    let (io, no) = ord.db_permutation();
                    std::sync::Arc::new(reordered(&self.netlist, &io, &no))
                }
                _ => self.netlist.clone(),
            };
            if full != *ours || info != self.info {
                return Err(format!("refresh_touched (edited): differs from the full read (netlist {}, info {})", full == *self.netlist, info == self.info));
            }
        }
        Ok(())
    }

    /// The estimator's clock nets and constant pins of the design as it is now: its own walk, the
    /// constants under the SDC's constraint values when there are any (`isSkipPin` asks
    /// `Sta::isConstant`).
    fn fresh_skips(&self, timing: &vyges_est::placement::Timing<'_>) -> Result<vyges_est::placement::Skips, String> {
        if self.constraints.case.is_empty() && self.constraints.logic.is_empty() {
            return vyges_est::placement::skips(self.db, timing);
        }
        let values = vyges_est::sim::constant_pins_constrained(self.db, self.liberty, &self.constraints).map_err(|e| format!("constant propagation: {e}"))?;
        vyges_est::placement::skips_with_constants(self.db, timing, &values)
    }

    /// Holds the estimator's clock nets and constant pins of the design as it is now, with the nets
    /// an edit must name to drop them, and starts the edit log they are checked against.
    pub fn hold_skips(&mut self, skips: vyges_est::placement::Skips) -> Result<(), String> {
        let mut nets: std::collections::HashSet<String> = skips.clock_nets().iter().cloned().collect();
        nets.extend(skips.constant_pins().iter().map(|p| self.net_of_load(p)).filter(|n| !n.is_empty()));
        self.skips = Some(HeldSkips { skips, nets });
        if !self.skip_log && !self.timer.on {
            self.db.edit_log_start().map_err(|e| e.to_string())?;
            self.timer.replay_ok = false;
        }
        self.skip_log = true;
        Ok(())
    }

    fn drop_skips_if_touched(&mut self, ev: &str) {
        // `Sta::replaceEquivCellBefore/After` invalidate neither the clock network nor the
        // simulator's constants: a swap between equivalent cells (same ports, functions and
        // sequentials) leaves both as they were, whatever nets its pins are on. Its `swap_before`
        // carries both masters and decides for the pair; a resize of a register (its clock pin on
        // the clock net) dropped the sets — a whole-design walk per resize.
        if swap_keeps_skips(ev, |from, to| self.equivalent_cells(from, to)) {
            return;
        }
        if self.skips.as_ref().is_some_and(|h| h.touched_by(ev)) {
            if std::env::var_os("VYGES_RSZ_SKIPS_TRACE").is_some() {
                eprintln!("VYGSKIPS|dropped|{ev}");
            }
            self.skips = None;
        }
    }

    /// Masters `sizing::equiv_cells` calls equivalent (undecided or unknown: not).
    fn equivalent_cells(&self, from: &str, to: &str) -> bool {
        match (self.libs.link_cell(from), self.libs.link_cell(to)) {
            (Some(a), Some(b)) => vyges_rsz::sizing::equiv_cells(a, b) == Some(true),
            _ => false,
        }
    }

    /// The database's callbacks since the last read, into the timer's log; the estimator's
    /// callbacks (`est::OdbCallBack`) on its invalid set: a net created, an instance terminal
    /// connected or disconnected, every net of a swapped instance → invalid; a net destroyed →
    /// erased.
    pub fn pull_edits(&mut self) {
        if !self.timer.on && !self.skip_log {
            return;
        }
        let mut evs = std::mem::take(&mut self.timer.raw_pending);
        evs.extend(self.take_edit_log());
        for ev in evs {
            self.drop_skips_if_touched(&ev);
            if !self.timer.on {
                continue;
            }
            let f: Vec<&str> = ev.split('|').collect();
            // GlobalRouter's callbacks (`GRouteDbCbk`), in the order the database made them.
            if let Some(gr) = self.gr.as_mut() {
                match f.as_slice() {
                    ["net_create", net] => {
                        vyges_grt::global_route::add_net(self.db, &mut gr.after, net);
                    }
                    ["net_destroy", net, ..] => {
                        gr.dirty.remove(*net);
                        if let Err(e) = vyges_grt::global_route::remove_net(&mut gr.after, net) {
                            self.gr_error.get_or_insert(e.to_string());
                        }
                    }
                    ["iterm_connect" | "iterm_disconnect" | "bterm_connect" | "bterm_disconnect", _, net, ..] => gr.mark(self.db, net),
                    // `inDbNetPostMerge(preserved, removed)` → `mergeNetsRouting`: before the removed
                    // net's destruction, which follows it.
                    ["net_merge", keep, gone] => match vyges_grt::global_route::merge_nets_routing(self.db, &gr.opts, &mut gr.after, keep, gone) {
                        Ok(true) => {}
                        Ok(false) => gr.mark(self.db, keep),
                        Err(e) => {
                            self.gr_error.get_or_insert(e.to_string());
                        }
                    },
                    ["swap_after", _, terms] => {
                        for net in terms.split(';').filter_map(|t| t.split('=').nth(1)) {
                            gr.mark(self.db, net);
                        }
                    }
                    _ => {}
                }
            }
            match f.as_slice() {
                ["net_create", net] | ["iterm_connect" | "iterm_disconnect", _, net, _] if !net.is_empty() => {
                    self.timer.est_invalid.insert(net.to_string());
                    if self.undoing {
                        self.invalid.insert(net.to_string());
                    }
                }
                ["net_destroy", net, ..] => {
                    self.timer.est_invalid.remove(*net);
                    if self.undoing {
                        // `inDbNetDestroy` → `eraseParasitics`: the network goes with the net.
                        self.invalid.remove(*net);
                        for map in self.parasitics.iter_mut() {
                            map.remove(*net);
                        }
                    }
                }
                ["swap_after", _, terms] => {
                    for net in terms.split(';').filter_map(|t| t.split('=').nth(1)).filter(|n| !n.is_empty()) {
                        self.timer.est_invalid.insert(net.to_string());
                        if self.undoing {
                            self.invalid.insert(net.to_string());
                        }
                    }
                }
                _ => {}
            }
            self.timer.events.push(ev);
        }
    }

    /// The database's callbacks since the last take, a copy kept for the next netlist refresh.
    fn take_edit_log(&mut self) -> Vec<String> {
        let raw = self.db.edit_log_take();
        self.timer.since_refresh.extend(raw.iter().cloned());
        raw
    }

    /// After a netlist refresh of `touched`: a callback naming a net it did not read leaves that
    /// net to be read whole next time; the replay holds while the log keeps recording.
    fn refreshed(&mut self, touched: &BTreeSet<String>) {
        let since = std::mem::take(&mut self.timer.since_refresh);
        for ev in &since {
            for net in event_nets(ev) {
                if !touched.contains(net) {
                    self.timer.unsure_nets.insert(net.to_string());
                }
            }
        }
        for net in touched {
            self.timer.unsure_nets.remove(net);
        }
        self.timer.replay_ok = self.timer.on || self.skip_log;
    }

    /// `GRouteDbCbk::inDbPostMoveInst` → `instItermsDirty`: every net on the moved instance's
    /// terminals marked dirty. Called after each `setLocation` this engine makes (the edit log does
    /// not carry moves).
    pub fn gr_inst_moved(&mut self, inst: &str) {
        if let Some(gr) = self.gr.as_mut() {
            for (term, _) in self.db.master_mterms(&self.db.inst_master(inst)).unwrap_or_default() {
                let net = self.db.net_of(inst, &term);
                gr.mark(self.db, &net);
            }
        }
    }

    /// `opendp_->legalCellPos(inst)` under global-route parasitics, "for accurate parasitic
    /// estimation": the instance moved to the nearest site and row when it is not on one (a MOVE:
    /// the router marks its nets dirty).
    pub fn gr_legal_cell_pos(&mut self, inst: &str) -> Result<(), String> {
        let Some(gr) = self.gr.as_ref() else { return Ok(()) };
        if self.db.inst_is_fixed(inst) {
            return Ok(());
        }
        let master = self.db.inst_master(inst);
        let (mut w, mut h) = (self.db.master_get_width(&master) as i32, self.db.master_get_height(&master) as i32);
        // `getBbox`: width and height swapped for a quarter-turn orientation (`swapWidthHeight`).
        if matches!(self.db.inst_get_orient(inst).as_str(), "R90" | "R270" | "MXR90" | "MYR90") {
            std::mem::swap(&mut w, &mut h);
        }
        let (x, y) = self.db.inst_location(inst);
        // A diagnostic: each instance `pointOffMacro` moves off a block (`<inst> <x> <y> -> <core-
        // relative point>`), appended to `VYGES_RSZ_POM_TRACE` — the witness that the rule fired.
        if let Ok(path) = std::env::var("VYGES_RSZ_POM_TRACE") {
            let init = (x - gr.grid.core.0, y - gr.grid.core.1);
            if let Some(pt) = gr.grid.point_off_macro(&gr.fixed, init, w, h).filter(|&pt| pt != init) {
                use std::io::Write;
                let mut f = std::fs::OpenOptions::new().create(true).append(true).open(&path).map_err(|e| format!("{path}: {e}"))?;
                writeln!(f, "{inst} {x} {y} -> {} {}", pt.0, pt.1).map_err(|e| format!("{path}: {e}"))?;
            }
        }
        if let Some((nx, ny)) = gr.grid.legal_cell_pos(&gr.fixed, x, y, w, h) {
            self.db.set_inst_location(inst, nx, ny).map_err(|e| e.to_string())?;
            self.gr_inst_moved(inst);
        }
        Ok(())
    }

    /// `estimateGlobalRouteRC(db_net)` into every scene: the net's network from its route as the
    /// router holds it now; a net with no route keeps what it had.
    pub fn gr_estimate(&mut self, net: &str) {
        let Some(gr) = self.gr.as_ref() else { return };
        let segs = gr_segs(&gr.after, net);
        gr_trace(&if segs.is_empty() { format!("VYGR|est|grc_none|{net}") } else { format!("VYGR|est|grc|{net}|{segs}") });
        if let Some(n) = vyges_grt::global_route::routed_network(&gr.after, net) {
            gr_trace(&format!("VYGR|est|cap|{net}|{:.9}", n.nodes.iter().map(|(_, c)| f64::from(*c)).sum::<f64>() * 1e12));
            let mut p = vyges_grt::timer::timer_network(net, &n);
            p.port_pin_caps = Some(crate::env::net_port_caps(&self.port_caps, &p.node_names));
            for map in self.parasitics.iter_mut() {
                map.insert(net.to_string(), p.clone());
            }
        }
    }

    /// The router's state as a refused callback left it, or the incremental re-route: the nets
    /// it routed again.
    pub fn gr_update_routes(&mut self) -> Result<Vec<String>, String> {
        if let Some(e) = self.gr_error.take() {
            return Err(e);
        }
        match self.gr.as_mut() {
            Some(gr) => gr.update_routes(self.db),
            None => Ok(Vec::new()),
        }
    }

    /// `journalRestore` → `undoEco` on global-route parasitics. The database's undo with its
    /// callbacks — the router's (dirty marks, nets made and destroyed) and the estimator's (each
    /// net it touched invalid; a destroyed net's network erased) — then each guide the level
    /// deleted created again (`inDbNetPostGuideRestore`: restore from guides, dirty). The estimate
    /// is NOT put back: the next update re-estimates the invalid nets on the routes as they come
    /// back (from the guides, not the routes the old estimate was made on).
    pub fn restore_journal_gr(&mut self) -> Result<bool, String> {
        self.pull_edits();
        // `journalRestore` → `init()` → `initDesignArea` BEFORE the undo: the count still holds
        // the undone edits (`recountDesignAreaAfterUndo` is the parallel policy's remedy).
        self.area = design_area(self.db);
        let had_changes = !self.db.eco_is_empty().map_err(|e| e.to_string())?;
        self.db.eco_undo().map_err(|e| e.to_string())?;
        // An undo can bring back what the held skips no longer describe: computed again.
        self.skips = None;
        let s = self.journal.pop().ok_or("undoEco without beginEco")?;
        self.sdc_nets = s.sdc_nets;
        self.refresh()?;
        self.undoing = true;
        self.pull_edits();
        self.undoing = false;
        if let Some(gr) = self.gr.as_mut() {
            gr_trace(&format!("VYGR|eco|undo|levels_left={}", gr.after.eco.len()));
            for net in vyges_grt::global_route::undo_eco(&mut gr.after) {
                gr_trace(&format!("VYGR|cb|restore|{net}"));
                gr.mark(self.db, &net);
            }
        }
        Ok(had_changes)
    }

    /// `EstimateParasitics::parasiticsInvalid(net)` (the odb callbacks call it).
    pub fn invalidate(&mut self, net: &str) {
        if self.estimating && !net.is_empty() {
            self.invalid.insert(net.to_string());
        }
    }

    /// The net a load pin (`inst/pin` or a port) is on.
    pub fn net_of_load(&self, load: &str) -> String {
        match load.rsplit_once('/') {
            Some((inst, pin)) if !self.netlist.ports.iter().any(|p| p.0 == load) => self.db.net_of(inst, pin),
            _ => self.db.bterm_net(load),
        }
    }

    pub fn clamp_loc_to_core(&self, loc: (i32, i32), master: &str) -> (i32, i32) {
        clamp_loc_to_core(self.core, (self.db.master_get_width(master) as i32, self.db.master_get_height(master) as i32), loc)
    }

    /// `insertBufferBeforeLoads(net, loads, …)`: on `net`, else the first load's.
    #[allow(clippy::too_many_arguments)]
    pub fn insert_before_loads(&mut self, net: Option<&str>, loads: &[String], cell: &str, loc: (i32, i32), reason: &str, loads_on_diff_nets: bool, uniquify: &str) -> Result<vyges_rsz::design::Repeater, String> {
        let original = match net {
            Some(n) => n.to_string(),
            None => self.net_of_load(loads.first().ok_or("insertBufferBeforeLoads: no loads specified")?),
        };
        // every net the insertion can change: the original, each load's (they may differ), then
        // the buffer's input and output nets
        let mut touched: BTreeSet<String> = loads.iter().map(|l| self.net_of_load(l)).filter(|n| !n.is_empty()).collect();
        touched.insert(original.clone());
        let mut iterms = Vec::new();
        let mut bterms = Vec::new();
        for l in loads {
            match l.rsplit_once('/') {
                Some((inst, pin)) if !self.netlist.ports.iter().any(|p| &p.0 == l) => iterms.push((inst.to_string(), pin.to_string())),
                _ => bterms.push(l.clone()),
            }
        }
        let inst = {
            let _p = crate::prof::Span::new(5);
            crate::prof::items(5, loads.len() as u64);
            self.db.insert_buffer_before_loads(net, &iterms, &bterms, cell, Some(loc), reason, None, uniquify, loads_on_diff_nets).map_err(|e| e.to_string())?
        };
        // `insertBufferPostProcess`: `designAreaIncr(area(master))`.
        self.area_incr(master_area(self.db, cell));
        let at = self.clamp_loc_to_core(self.db.inst_location(&inst), cell);
        self.db.set_inst_location(&inst, at.0, at.1).map_err(|e| e.to_string())?;
        self.gr_inst_moved(&inst);
        self.gr_legal_cell_pos(&inst)?;
        let c = self.libs.link_cell(cell).ok_or_else(|| format!("{cell}: no liberty cell"))?;
        let (input, output) = c.buffer_ports().ok_or_else(|| format!("{cell}: not a buffer"))?;
        let in_net = self.db.net_of(&inst, &input.name);
        let out_net = self.db.net_of(&inst, &output.name);
        // `ConcreteParasitics::disconnectPinBefore` on each load moved off the original net
        // deletes its drivers' reduced models, and the estimator keeps no network once it has
        // reduced one (`reduceParasitic`, then `deleteParasiticNetwork`): the buffer's input net
        // (the original, or its renamed self when a port load names the new net after the port)
        // is timed on its pin caps alone until it is estimated again, as is the new output net.
        // Witness: a resistance-aware route update's mid-update slacks read the driver of a net a
        // repeater was just inserted on (repair_design -reroute on global-route parasitics).
        for map in self.parasitics.iter_mut() {
            map.remove(&original);
            map.remove(&in_net);
            map.remove(&out_net);
        }
        self.invalidate(&in_net);
        self.invalidate(&out_net);
        touched.insert(in_net.clone());
        touched.insert(out_net.clone());
        self.refresh_touched(std::slice::from_ref(&inst), &touched)?;
        Ok(vyges_rsz::design::Repeater { input: format!("{inst}/{}", input.name), output: format!("{inst}/{}", output.name), inst, out_net })
    }
}

impl vyges_rsz::design::Design for CliDesign<'_> {
    fn netlist(&self) -> &vyges_sta::netlist::Netlist {
        &self.netlist
    }
    fn netlist_shared(&self) -> std::sync::Arc<vyges_sta::netlist::Netlist> {
        self.netlist.clone()
    }
    fn netlist_generation(&self) -> Option<u64> {
        Some(self.netlist_gen)
    }
    fn parasitics(&self, scene: usize) -> &HashMap<String, NetParasitics> {
        &self.parasitics[scene]
    }
    fn net_info(&self) -> &NetInfo {
        &self.info
    }

    /// `ensureWireParasitic(drvr_pin, net)` under placement parasitics: re-estimated (at the alpha
    /// in force now) only when invalid or with no pi model — no entry, here: every net with a
    /// network has its pi model once `findAllArrivals` has run, and an edited net is invalid.
    fn ensure_wire_parasitic(&mut self, net: &str) -> Result<(), String> {
        let _p = crate::prof::Span::new(4);
        // Estimated now and erased from the invalid set, with no delay invalidation.
        self.pull_edits();
        self.timer.est_invalid.remove(net);
        if self.gr.is_some() {
            // Under global-route parasitics: invalid or with no pi model (no network here — a net
            // with a route has one) → the dirty nets re-routed, then this net from its route.
            if self.invalid.contains(net) || !self.parasitics[0].contains_key(net) {
                // c5943b20d8f: the other nets the router moved stay invalid until the next
                // `updateParasitics` (re-estimated, their delays invalidated then); this one is
                // estimated now and erased.
                let routed = self.gr_update_routes()?;
                if self.estimating {
                    absorb_rerouted(&mut self.invalid, &mut self.timer.est_invalid, routed, Some(net));
                }
                self.gr_estimate(net);
                self.invalid.remove(net);
            }
            return Ok(());
        }
        if !self.estimating || !(self.invalid.contains(net) || !self.parasitics[0].contains_key(net)) {
            return Ok(());
        }
        let Some(drvr) = vyges_rsz::design::Design::net_slot(self, net).map(|k| &self.netlist.nets[k]).and_then(|n| {
            n.pins.iter().map(|c| self.netlist.pin_name(c)).find(|p| self.is_driver_pin(p))
        }) else {
            self.invalid.remove(net);
            return Ok(());
        };
        let timing = vyges_est::placement::Timing { liberty: Some(self.liberty), clock_sources: self.clock_sources.to_vec(), propagated: self.propagated };
        if self.skips.is_none() {
            if std::env::var_os("VYGES_RSZ_SKIPS_TRACE").is_some() {
                eprintln!("VYGSKIPS|computed|{net}");
            }
            let skips = self.fresh_skips(&timing)?;
            self.hold_skips(skips)?;
        }
        let skips = &self.skips.as_ref().expect("computed above").skips;
        // A check: the held sets against a fresh walk of the design as it is now.
        if std::env::var_os("VYGES_RSZ_CHECK_SKIPS").is_some() {
            let fresh = self.fresh_skips(&timing)?;
            if fresh.clock_nets() != skips.clock_nets() || fresh.constant_pins() != skips.constant_pins() {
                return Err(format!("held skips (estimating {net}): differ from a fresh walk (clock nets {}, constant pins {})", fresh.clock_nets() == skips.clock_nets(), fresh.constant_pins() == skips.constant_pins()));
            }
        }
        let e = vyges_est::placement::estimate_net_with_skips(self.db, &timing, skips, net, &drvr, self.alpha, &est_stt)?;
        for k in 0..self.parasitics.len() {
            match net_parasitic(self.db, self.rc, &e, k)? {
                Some(mut p) => {
                    p.port_pin_caps = Some(crate::env::net_port_caps(&self.port_caps, &p.node_names));
                    self.parasitics[k].insert(net.to_string(), p)
                }
                None => self.parasitics[k].remove(net),
            };
        }
        self.invalid.remove(net);
        Ok(())
    }

    /// `updateParasitics` under placement parasitics: each invalid net estimated again (the order
    /// is the reference's set's; each net's estimate is its own, so it is no value).
    fn update_parasitics(&mut self) -> Result<(), String> {
        // The estimator's invalid nets: each estimated, then `delaysInvalidFromFanin` each, the
        // set cleared.
        self.pull_edits();
        let mut updated = std::mem::take(&mut self.timer.est_invalid);
        if self.gr.is_some() {
            // Under global-route parasitics the dirty nets are re-routed FIRST, whatever is
            // invalid; every net routed again joins the invalid set (c5943b20d8f), then each
            // invalid net from its route (a net with none keeps its network), its delays
            // invalidated.
            gr_trace(&format!("VYGR|est|update|src=2|{}", self.invalid.iter().map(|n| format!("{n};")).collect::<String>()));
            let routed = self.gr_update_routes()?;
            if self.estimating {
                absorb_rerouted(&mut self.invalid, &mut updated, routed, None);
            }
            self.timer.updated.extend(updated);
            for net in std::mem::take(&mut self.invalid) {
                self.gr_estimate(&net);
            }
            return Ok(());
        }
        self.timer.updated.extend(updated);
        let invalid: Vec<String> = self.invalid.iter().cloned().collect();
        for net in invalid {
            self.ensure_wire_parasitic(&net)?;
        }
        Ok(())
    }

    fn steiner(&self, net: &str, drvr_pin: &str) -> Option<vyges_rsz::buffered_net::Tree> {
        steiner_tree(self.db, net, drvr_pin, self.alpha)
    }

    fn global_routing_parasitics(&self) -> bool {
        self.gr.is_some()
    }

    fn route_slacks_possible(&self) -> bool {
        self.gr.as_ref().is_some_and(|g| g.after.resistance_aware)
    }

    fn route_slacks_wanted(&mut self) -> bool {
        self.pull_edits();
        self.gr.as_ref().is_some_and(|g| g.after.resistance_aware && !g.dirty.is_empty())
    }

    fn set_route_slacks(&mut self, slacks: std::collections::HashMap<String, f32>) {
        if let Some(g) = self.gr.as_mut() {
            g.slacks = Some(slacks);
        }
    }

    fn ensure_routes(&mut self, net: &str) -> bool {
        self.pull_edits();
        self.gr.is_some() && (self.invalid.contains(net) || !self.parasitics[0].contains_key(net))
    }

    fn pin_net(&self, pin: &str) -> Option<(String, bool)> {
        let net = self.net_of_load(pin);
        (!net.is_empty()).then(|| (net.clone(), self.db.net_is_special(&net)))
    }

    fn net_res_aware(&self, net: &str) -> bool {
        self.gr.as_ref().is_some_and(|gr| vyges_grt::global_route::is_net_res_aware(&gr.after, net))
    }

    fn net_resistances(&self, net: &str) -> (f32, f32) {
        match self.gr.as_ref() {
            Some(gr) => (vyges_grt::global_route::net_resistance_on_layer(&gr.after, net, None), vyges_grt::global_route::net_resistance_on_min_resistance_layer(&gr.after, net)),
            None => (0.0, 0.0),
        }
    }

    fn reroute_net(&mut self, net: &str) -> Result<(), String> {
        let Some(gr) = self.gr.as_mut() else { return Err("rerouting needs global-route parasitics".into()) };
        vyges_grt::global_route::set_resistance_aware(&mut gr.after);
        gr.mark(self.db, net);
        vyges_grt::global_route::set_net_res_aware(&mut gr.after, net);
        // `parasiticsInvalid(db_net)`: re-estimated, and its delays invalidated, at the next update.
        self.invalidate(net);
        self.timer.est_invalid.insert(net.to_string());
        Ok(())
    }

    fn net_route_gcells(&self, net: &str) -> Option<(i32, i32)> {
        let a = &self.gr.as_ref()?.after;
        if a.tile_size <= 0 {
            return None;
        }
        let route = a.net_routes.iter().find(|r| r.name == net)?;
        let segs: Vec<(i32, i32, i32, i32)> = route.segments.iter().map(|g| (g.init_x, g.init_y, g.final_x, g.final_y)).collect();
        Some((vyges_rsz::repair_design::route_gcell_length(&segs, a.tile_size), a.tile_size))
    }

    /// Under global-route parasitics `makeBufferedNetGroute` reads the router: the net's pins
    /// (`getPinGridPositions`: as the router's last `updateNetPins` left them, each at its grid
    /// point on its connection layer) and its route (`routes_`, empty when it has none).
    fn net_shape(&self, net: &str, drvr_pin: &str) -> Option<vyges_rsz::buffered_net::NetShape> {
        let Some(gr) = self.gr.as_ref() else { return self.steiner(net, drvr_pin).map(vyges_rsz::buffered_net::NetShape::Steiner) };
        let a = &gr.after;
        let n = &a.router_nets[vyges_grt::global_route::live_id(a, net)?];
        let pins = n.net_pins.iter().map(|p| (p.name.clone(), p.on_grid.0, p.on_grid.1, p.connection_layer)).collect();
        let segments = a.net_routes.iter().find(|r| r.name == net).map(|r| r.segments.iter().map(|g| (g.init_x, g.init_y, g.init_layer, g.final_x, g.final_y, g.final_layer)).collect()).unwrap_or_default();
        Some(vyges_rsz::buffered_net::NetShape::Groute(vyges_rsz::buffered_net::GrouteNet { pins, drvr: drvr_pin.to_string(), segments, term_count: self.db.net_get_term_count(net) as usize, layers: gr.layers.clone() }))
    }

    /// `Resizer::insertBufferBeforeLoads(nullptr, loads, cell, &loc, reason)`: odb's own
    /// `dbNet::insertBufferBeforeLoads` on the first load's net (new names `<reason><n>` /
    /// `net<n>`, always uniquified), then `insertBufferPostProcess` (`setLocation`: clamped to
    /// the core, PLACED). The odb callbacks invalidate the new net and the original one.
    fn insert_repeater(&mut self, loads: &[String], cell: &str, loc: (i32, i32), reason: &str) -> Result<vyges_rsz::design::Repeater, String> {
        self.insert_before_loads(None, loads, cell, loc, reason, false, "ALWAYS")
    }

    /// `Resizer::replaceCell` → `dbInst::swapMaster`; the callback invalidates the net on every
    /// pin of the instance (not a tristate one).
    fn swap_master(&mut self, inst: &str, cell: &str) -> Result<(), String> {
        let _p = crate::prof::Span::new(6);
        // `Sta::replaceCell`: with equivalent arcs, `replaceEquivCellBefore` swaps the arc sets of
        // the edges leaving the INPUT pins only — an output-to-output edge keeps the cell it was
        // made with (the first one, across further equivalent swaps); otherwise the instance's
        // edges are made anew from the new cell.
        let from = vyges_rsz::design::Design::inst_slot(self, inst).map(|k| self.netlist.insts[k].1.clone());
        if let Some(from) = from {
            let equiv = matches!((self.libs.link_cell(&from), self.libs.link_cell(cell)), (Some(a), Some(b)) if vyges_sta::incr::equiv_cells_arcs(a, b));
            if equiv {
                self.stale_out_arcs.entry(inst.to_string()).or_insert(from);
            } else {
                self.stale_out_arcs.remove(inst);
            }
        }
        let old = self.db.inst_master(inst);
        if !self.db.swap_master(inst, cell).map_err(|e| e.to_string())? {
            return Err(format!("swapMaster {inst} -> {cell} failed"));
        }
        // `designAreaIncr(-area(master))`, the swap, `designAreaIncr(area(replacement))`.
        self.area_incr(-master_area(self.db, &old));
        self.area_incr(master_area(self.db, cell));
        // `Resizer::replaceCell`: "legalize the position of the instance in case it leaves the die".
        self.gr_legal_cell_pos(inst)?;
        // The instance re-read (its master) and the nets on its pins, not the whole design: a
        // full read per resize was O(design) — and dropped the order the next insertion extends.
        // The nets invalidated are the ones the netlist lists with a pin of it: a supply net
        // (special POWER / GROUND) is no timing net.
        let mut nets: Vec<String> = Vec::new();
        let iterms = self.db.inst_iterm_info(inst).map_err(|e| e.to_string())?;
        for (_, _, n) in &iterms {
            if !n.is_empty() && !nets.contains(n) {
                nets.push(n.clone());
            }
        }
        // Only the instance is read again (its master): upstream `dbInst::swapMaster` requires the
        // new master's terminals to match the old one's by name, remaps each terminal's master
        // index and re-sorts the instance's own terminal list — no terminal is connected or
        // disconnected, so every net keeps its pins, their order and their ids. Re-reading the
        // instance's nets was O(their fanout) per resize: a register's whole clock net each time
        // (jg8s: 21 M of the 28 M pins the driver loop's refreshes read).
        self.refresh_touched(&[inst.to_string()], &BTreeSet::new())?;
        nets.retain(|n| !(self.db.net_is_special(n) && matches!(self.db.net_get_sig_type(n).as_str(), "POWER" | "GROUND")));
        for n in nets {
            self.invalidate(&n);
        }
        // `Sta::replace(Equiv)CellAfter` → `loadPinCapacitanceChanged` on each input pin: the
        // reduced models of its net's drivers are deleted — a placement estimate IS one, so the
        // net has no parasitic until `updateParasitics` estimates it again (it is invalid).
        // Witness: SizeDownFanout's next round in the same batch reads the driver timed on its
        // pin caps alone.
        // the nets on its pins only (a net with a pin of it is one of them), not every net
        // read from the instance's own terminals (each listed net has its pin there), not from a
        // scan of every pin of each net: a register's clock net per resize
        let lib_cell = self.libs.link_cell(cell);
        let input_nets: BTreeSet<String> = iterms
            .iter()
            .filter(|(term, _, n)| !n.is_empty() && lib_cell.and_then(|lc| lc.port(term)).is_some_and(|p| matches!(p.direction, vyges_sta::liberty::Direction::Input | vyges_sta::liberty::Direction::Bidirect)))
            .filter(|(_, _, n)| vyges_rsz::design::Design::net_slot(self, n).is_some())
            .map(|(_, _, n)| n.clone())
            .collect();
        if self.estimating {
            for n in input_nets {
                for k in 0..self.parasitics.len() {
                    self.parasitics[k].remove(&n);
                }
            }
        }
        Ok(())
    }

    fn inst_location(&self, inst: &str) -> (i32, i32) {
        self.db.inst_location(inst)
    }

    fn stale_out_arcs(&self) -> Option<&BTreeMap<String, String>> {
        Some(&self.stale_out_arcs)
    }

    fn pin_location(&self, pin: &str) -> (i32, i32) {
        match pin.rsplit_once('/') {
            Some((inst, term)) if !self.netlist.ports.iter().any(|p| p.0 == pin) => {
                self.db.iterm_avg_xy(inst, term).unwrap_or_else(|| (self.db.inst_get_origin_x(inst), self.db.inst_get_origin_y(inst)))
            }
            _ => self.db.bterm_first_pin_location(pin).unwrap_or((0, 0)),
        }
    }

    fn start_timer_edits(&mut self) -> Result<(), String> {
        // Restarting clears the log: the held skips read what it holds first (the timer's own
        // events in it are discarded, as before).
        if self.skip_log {
            let mut evs = std::mem::take(&mut self.timer.raw_pending);
            evs.extend(self.db.edit_log_take());
            for ev in evs {
                self.drop_skips_if_touched(&ev);
            }
        }
        self.db.edit_log_start().map_err(|e| e.to_string())?;
        self.timer = TimerLog { on: true, hold_order: self.timer.hold_order, ..TimerLog::default() };
        Ok(())
    }

    fn net_slot(&self, net: &str) -> Option<usize> {
        let scan = || self.netlist.nets.iter().position(|n| n.name == net);
        if let Some(ord) = self.kept_order() {
            let hit = ord.net_slot_pos(net).filter(|&k| self.netlist.nets.get(k).is_some_and(|n| n.name == net));
            // 🔑 a name the kept map never held is no net of this netlist (the map is made from
            // its every net and takes each one an edit makes or renames): a supply net on a resized
            // instance's pins asked for by name was a pass over every net, per resize
            if hit.is_some() || !ord.net_slot.contains_key(net) {
                if std::env::var_os("VYGES_RSZ_CHECK_REFRESH").is_some() {
                    assert_eq!(hit, scan(), "net {net}: the kept name map and the netlist disagree");
                }
                return hit;
            }
        }
        scan()
    }

    fn inst_slot(&self, inst: &str) -> Option<usize> {
        self.kept_order().and_then(|ord| ord.inst_pos(inst)).filter(|&k| self.netlist.insts.get(k).is_some_and(|i| i.0 == inst)).or_else(|| self.netlist.insts.iter().position(|i| i.0 == inst))
    }

    fn hold_netlist_order(&mut self, hold: bool) -> Result<(), String> {
        CliDesign::hold_order(self, hold)
    }

    fn take_timer_edits(&mut self) -> vyges_rsz::design::TimerEdits {
        self.pull_edits();
        let slots = std::mem::take(&mut self.timer.slots);
        let slots = (!std::mem::take(&mut self.timer.slots_lost)).then_some(slots);
        vyges_rsz::design::TimerEdits { events: std::mem::take(&mut self.timer.events), updated_nets: std::mem::take(&mut self.timer.updated), slots }
    }

    fn visit_connected_pins(&self, pin: &str) -> Vec<String> {
        match pin.rsplit_once('/') {
            Some((inst, term)) if !self.netlist.ports.iter().any(|p| p.0 == pin) => self
                .db
                .visit_connected_pins(inst, term)
                .unwrap_or_default()
                .into_iter()
                .filter_map(|p| p.strip_prefix("I:").or_else(|| p.strip_prefix("B:")).map(String::from))
                .collect(),
            // A port driver: its flat net's pins (`net(term)`).
            _ => {
                let net = self.db.bterm_net(pin);
                let mut pins = self.db.net_iterms(&net);
                pins.extend(self.db.net_bterms(&net));
                pins
            }
        }
    }
}

impl vyges_rsz::repair_setup::SetupDesign for CliDesign<'_> {
    fn pending_nets(&self) -> BTreeSet<String> {
        self.invalid.clone()
    }

    fn steiner_points(&self, root: (i32, i32), sinks: &[(i32, i32)]) -> Option<vyges_rsz::buffered_net::Tree> {
        vyges_rsz::env::steiner_over_points(root, sinks, self.alpha)
    }

    fn net_is_plain_signal(&self, net: &str) -> bool {
        self.db.net_get_sig_type(net) == "SIGNAL" && !self.db.net_is_special(net)
    }

    fn net_is_special(&self, net: &str) -> bool {
        self.db.net_is_special(net)
    }

    fn global_routed(&self) -> bool {
        self.gr.is_some()
    }


    /// `Resizer::computeDesignArea`: over the block's instances in order, each master's
    /// `width × height` in m² (`dbuToMeters` each side) — 0 for a master that is not core
    /// autoplaceable (`isCoreAutoPlaceable`) — fillers (`CORE SPACER`) left out.
    fn design_area(&self) -> f64 {
        design_area(self.db)
    }

    fn tracked_design_area(&self) -> f64 {
        self.area
    }

    fn init_design_area(&mut self) {
        self.area = design_area(self.db);
    }

    fn master_area(&self, cell: &str) -> f64 {
        master_area(self.db, cell)
    }

    fn core_area(&self) -> f64 {
        let dbu = f64::from(self.db.tech_get_db_units_per_micron());
        self.core.map_or(0.0, |(x0, y0, x1, y1)| f64::from(x1 - x0) / (dbu * 1e6) * (f64::from(y1 - y0) / (dbu * 1e6)))
    }

    fn as_design(&self) -> &dyn vyges_rsz::design::Design {
        self
    }

    fn begin_journal(&mut self) -> Result<(), String> {
        self.db.eco_begin().map_err(|e| e.to_string())?;
        if let Some(gr) = self.gr.as_mut() {
            vyges_grt::global_route::begin_eco(&mut gr.after);
            gr_trace(&format!("VYGR|eco|begin|{}", gr.after.eco.len()));
        }
        self.journal.push(JournalState { parasitics: self.parasitics.clone(), invalid: self.invalid.clone(), sdc_nets: self.sdc_nets.clone() });
        Ok(())
    }

    fn commit_journal(&mut self) -> Result<(), String> {
        self.db.eco_commit().map_err(|e| e.to_string())?;
        if let Some(gr) = self.gr.as_mut() {
            vyges_grt::global_route::commit_eco(&mut gr.after);
            gr_trace(&format!("VYGR|eco|commit|{}", gr.after.eco.len()));
        }
        self.journal.pop();
        Ok(())
    }

    fn restore_journal(&mut self) -> Result<bool, String> {
        if self.gr.is_some() {
            return self.restore_journal_gr();
        }
        // `journalRestore` → `init()` → `initDesignArea` BEFORE the undo: the count still holds
        // the undone edits (`recountDesignAreaAfterUndo` is the parallel policy's remedy).
        self.area = design_area(self.db);
        let had_changes = !self.db.eco_is_empty().map_err(|e| e.to_string())?;
        self.db.eco_undo().map_err(|e| e.to_string())?;
        // An undo can bring back what the held skips no longer describe: computed again.
        self.skips = None;
        let s = self.journal.pop().ok_or("undoEco without beginEco")?;
        self.parasitics = s.parasitics;
        self.invalid = s.invalid;
        self.sdc_nets = s.sdc_nets;
        self.refresh()?;
        Ok(had_changes)
    }

    /// `canRemoveBuffer(buffer, true)`'s database checks: the buffer neither dont_touch nor FIXED,
    /// neither net dont_touch, an input net; then the net to go (the input net when the output net
    /// has a port and the input net none, else the output net): neither buffer pin a clock's source nor named by `set_max_capacitance`
    /// (`Sdc::isConstrained(pin)`: the pin constraints an accepted SDC can put on an instance pin), the net to go not named by
    /// `set_load` (`isConstrained(net)`), and that net mergeable into the other
    /// (`dbNet::canMergeNet`), or absent.
    fn can_remove_buffer(&self, inst: &str, in_pin: &str, out_pin: &str) -> bool {
        if self.db.inst_is_do_not_touch(inst) || self.db.inst_is_fixed(inst) {
            return false;
        }
        let in_net = self.db.net_of(inst, in_pin);
        let out_net = self.db.net_of(inst, out_pin);
        // No input net: nothing to merge into (upstream aa89cb9694).
        if in_net.is_empty() {
            return false;
        }
        if [&in_net, &out_net].iter().any(|n| !n.is_empty() && self.db.net_is_do_not_touch(n)) {
            return false;
        }
        // `bufferRemovalKeepsOutputNet` (flat: no module-net feedthrough): the output net
        // survives only when it has a port and the input net has none — the survivor removeBuffer
        // picks (upstream aa89cb9694, 9ffa16d003; before them a port on the output net alone).
        let has_port = |n: &str| !n.is_empty() && !self.db.net_bterms(n).is_empty();
        let keep_output = buffer_removal_keeps_output_net(has_port(&in_net), has_port(&out_net));
        let (survivor, removed) = if keep_output { (out_net, in_net) } else { (in_net, out_net) };
        let pins = [format!("{inst}/{in_pin}"), format!("{inst}/{out_pin}")];
        if pins.iter().any(|p| self.clock_sources.contains(p) || self.sdc_pins.contains(p)) || self.sdc_nets.contains(&removed) {
            return false;
        }
        if removed.is_empty() {
            return true;
        }
        !survivor.is_empty() && self.db.net_can_merge(&survivor, &removed).unwrap_or(false)
    }

    /// `swapPins`: the pins' nets, the dont_touch check on each (pin 1's first), then the database
    /// swap; the callbacks invalidate both nets.
    fn swap_pins(&mut self, inst: &str, pin1: &str, pin2: &str) -> Result<vyges_rsz::repair_setup::PinSwap, String> {
        use vyges_rsz::repair_setup::PinSwap;
        let net1 = self.db.net_of(inst, pin1);
        let net2 = self.db.net_of(inst, pin2);
        if net1.is_empty() || net2.is_empty() {
            return Ok(PinSwap::NoNet);
        }
        for n in [&net1, &net2] {
            if self.db.net_is_do_not_touch(n) {
                return Ok(PinSwap::DontTouch(n.clone()));
            }
        }
        self.db.swap_pins(inst, pin1, pin2).map_err(|e| e.to_string())?;
        self.invalidate(&net1);
        self.invalidate(&net2);
        self.refresh()
            .map(|_| PinSwap::Swapped)
    }

    fn ok_to_buffer_net(&self, drvr_pin: &str) -> bool {
        let (inst, term) = self.term(drvr_pin);
        let tristate = inst.is_some_and(|i| self.libs.link_cell(&self.db.inst_master(i)).and_then(|c| c.ports.iter().find(|p| p.name == term)).is_some_and(|p| p.is_any_tristate()));
        if tristate {
            return false;
        }
        let net = match inst {
            Some(i) => self.db.net_of(i, term),
            None => self.db.bterm_net(term),
        };
        !net.is_empty() && !self.db.net_is_do_not_touch(&net) && !self.db.net_is_connected_by_abutment(&net) && !self.db.net_is_special(&net)
    }

    /// `applyClone` on the database: the instance (odb's "clone" name, TIMING source, placed and
    /// clamped), each liberty INPUT pin of the driver in the master's terminal order connected to
    /// the same net, the first output on a new "net", the moved loads disconnected and connected
    /// to it. The callbacks invalidate every net touched.
    fn clone_instance(&mut self, drvr_inst: &str, cell: &str, loc: (i32, i32), moved_loads: &[String]) -> Result<String, String> {
        let e = |x: vyges_opendb::Error| x.to_string();
        // `makeInstance(cell, "clone", parent_)`: named and created in the driver's module.
        let name = self.db.make_new_inst_name_beside(drvr_inst, "clone", "ALWAYS").map_err(e)?;
        self.db.create_inst_beside(cell, &name, drvr_inst).map_err(e)?;
        // `Resizer::makeInstance`: `designAreaIncr(area(master))`.
        self.area_incr(master_area(self.db, cell));
        self.db.inst_set_source_type(&name, "TIMING").map_err(e)?;
        let at = self.clamp_loc_to_core(loc, cell);
        self.db.set_inst_location(&name, at.0, at.1).map_err(e)?;
        self.gr_inst_moved(&name);
        self.gr_legal_cell_pos(&name)?;
        let master = self.db.inst_master(drvr_inst);
        let lc = self.libs.link_cell(&master).ok_or_else(|| format!("{master}: no liberty cell"))?.clone();
        let terms: Vec<String> = self.db.master_mterms(&master).map_err(e)?.into_iter().filter(|(_, t)| t != "POWER" && t != "GROUND").map(|(n, _)| n).collect();
        let mut touched = Vec::new();
        for term in &terms {
            if !lc.port(term).is_some_and(|p| p.direction == vyges_sta::liberty::Direction::Input) {
                continue;
            }
            // `connectCloneInputs`: the flat net, then the hierarchical one (`iterm->connect(modnet)`).
            let net = self.db.net_of(drvr_inst, term);
            if !net.is_empty() {
                self.db.connect(&name, term, &net).map_err(e)?;
                touched.push(net);
            }
            self.db.connect_mod_net_of(&name, term, drvr_inst, term).map_err(e)?;
        }
        let clone_terms: Vec<String> = self.db.master_mterms(cell).map_err(e)?.into_iter().filter(|(_, t)| t != "POWER" && t != "GROUND").map(|(n, _)| n).collect();
        let cc = self.libs.link_cell(cell).ok_or_else(|| format!("{cell}: no liberty cell"))?.clone();
        let out = clone_terms.iter().find(|t| cc.port(t).is_some_and(|p| p.direction == vyges_sta::liberty::Direction::Output)).ok_or("Cannot find output pin of the clone instance")?;
        // `makeNet(parent_)`: named in the driver's module.
        let out_net = self.db.make_new_net_name_beside(drvr_inst, "net", "ALWAYS").map_err(e)?;
        self.db.create_net(&out_net).map_err(e)?;
        self.db.connect(&name, out, &out_net).map_err(e)?;
        touched.push(out_net.clone());
        // `moveLoads`: each load off its nets; in the clone's module onto the new net, in another
        // through the hierarchy (`hierarchicalConnect(clone output, load)`, connection "net").
        for load in moved_loads {
            let (inst, term) = load.rsplit_once('/').ok_or("a load pin")?;
            touched.push(self.db.net_of(inst, term));
            self.db.disconnect(inst, term).map_err(e)?;
            if self.db.same_owning_module(inst, drvr_inst).map_err(e)? {
                // The net itself, by the clone's output (a cross-module load may have renamed it).
                let cur = self.db.net_of(&name, out);
                self.db.connect(inst, term, &cur).map_err(e)?;
            } else {
                self.db.hierarchical_connect(&name, out, inst, term, "net").map_err(e)?;
            }
        }
        // A cross-module connection may rename the clone's flat net (after its highest modnet).
        let out_net = self.db.net_of(&name, out);
        touched.push(out_net);
        for n in touched {
            self.invalidate(&n);
        }
        self.refresh()?;
        Ok(name)
    }

    fn buffer_to_inverters(&mut self, buffer: &str, in_pin: &str, out_pin: &str, inv_cell: &str, inv_in: &str, inv_out: &str, locs: [(i32, i32); 2]) -> Result<Option<(String, String)>, String> {
        let e = |x: vyges_opendb::Error| x.to_string();
        let (in_net, out_net) = (self.db.net_of(buffer, in_pin), self.db.net_of(buffer, out_pin));
        let supplies = self.other_iterm_nets(buffer, &[&in_net, &out_net])?;
        // The database half, in the candidate's order (disconnect, deleteInstance, makeInstance ×2,
        // makeNet, connectPin ×4, the module nets): inv1 takes the freed slot.
        let Some((inv1, inv2, mid)) = self.db.buffer_to_inverters(buffer, in_pin, out_pin, inv_cell, inv_in, inv_out).map_err(e)? else { return Ok(None) };
        // ⚠️ The reference deletes the buffer through the network (`Sta::deleteInstance`), which
        // leaves its area in the tracked area; each `Resizer::makeInstance` adds an inverter's.
        for _ in 0..2 {
            self.area_incr(master_area(self.db, inv_cell));
        }
        // ⚠️ `makeInstance` places inv1 before inv2 exists; here both exist (inv2 unplaced) when
        // inv1 is placed. Placement reads placed instances only, so the order cannot show.
        for (name, loc) in [(&inv1, locs[0]), (&inv2, locs[1])] {
            self.db.inst_set_source_type(name, "TIMING").map_err(e)?;
            let at = self.clamp_loc_to_core(loc, inv_cell);
            self.db.set_inst_location(name, at.0, at.1).map_err(e)?;
            self.gr_inst_moved(name);
            self.gr_legal_cell_pos(name)?;
        }
        for n in [in_net, mid, out_net].into_iter().chain(supplies) {
            self.invalidate(&n);
        }
        self.refresh()?;
        Ok(Some((inv1, inv2)))
    }

    fn inst_id(&self, inst: &str) -> u32 {
        self.db.inst_id(inst).unwrap_or(u32::MAX)
    }

    fn pin_dont_touch(&self, pin: &str) -> bool {
        let (inst, term) = self.term(pin);
        let net = match inst {
            Some(i) => self.db.net_of(i, term),
            None => self.db.bterm_net(term),
        };
        inst.is_some_and(|i| self.db.inst_is_do_not_touch(i)) || (!net.is_empty() && self.db.net_is_do_not_touch(&net))
    }

    fn insert_buffer_before_loads(&mut self, net: Option<&str>, loads: &[String], cell: &str, loc: (i32, i32), reason: &str, loads_on_diff_nets: bool, uniquify: &str) -> Result<vyges_rsz::design::Repeater, String> {
        self.insert_before_loads(net, loads, cell, loc, reason, loads_on_diff_nets, uniquify)
    }

    /// `removeBuffer`: the database edit (`remove_buffer`), then the odb callbacks on the
    /// estimator — both nets were disconnected and the survivor took the other's pins (invalid),
    /// the merged-away net destroyed (its parasitics erased). The cache is keyed by name, so both
    /// old names go and the survivor is estimated afresh under its name now.
    fn remove_buffer(&mut self, inst: &str, in_pin: &str, out_pin: &str) -> Result<vyges_rsz::repair_setup::RemovedBuffer, String> {
        let in_net = self.db.net_of(inst, in_pin);
        let out_net = self.db.net_of(inst, out_pin);
        if in_net.is_empty() {
            return Err(format!("The input pin of buffer '{inst}' is undriven. Do not remove the buffer."));
        }
        // `designAreaIncr(-area(master))` once the input is known driven: both paths delete it.
        let master = self.db.inst_master(inst);
        let supplies = self.other_iterm_nets(inst, &[&in_net, &out_net])?;
        let survivor = self.db.remove_buffer(inst, in_pin, out_pin).map_err(|e| e.to_string())?;
        self.area_incr(-master_area(self.db, &master));
        for map in self.parasitics.iter_mut() {
            map.remove(&in_net);
            map.remove(&out_net);
        }
        self.invalid.remove(&in_net);
        self.invalid.remove(&out_net);
        self.invalidate(&survivor);
        for n in &supplies {
            self.invalidate(n);
        }
        // A constrained net only ever survives: its constraint follows it to its name now.
        if self.sdc_nets.remove(&in_net) | self.sdc_nets.remove(&out_net) {
            self.sdc_nets.insert(survivor.clone());
        }
        self.refresh()?;
        Ok(vyges_rsz::repair_setup::RemovedBuffer { in_net, out_net, survivor })
    }
}

impl CliDesign<'_> {
    /// A pin that drives its net: an instance output (or inout) pin, or an input port.
    pub fn is_driver_pin(&self, pin: &str) -> bool {
        match pin.rsplit_once('/') {
            Some((inst, term)) if !self.netlist.ports.iter().any(|p| p.0 == pin) => matches!(self.db.iterm_get_io_type(inst, term).as_str(), "OUTPUT" | "INOUT"),
            _ => matches!(self.db.bterm_get_io_type(pin).as_str(), "INPUT" | "INOUT"),
        }
    }
}

impl buffer_ports::PortDesign for CliDesign<'_> {
    fn top_ports(&self) -> Vec<String> {
        self.db.block_get_b_terms()
    }
    fn port_direction(&self, port: &str) -> String {
        self.db.bterm_get_io_type(port)
    }
    fn port_net(&self, port: &str) -> Option<String> {
        Some(self.db.bterm_net(port)).filter(|n| !n.is_empty())
    }
    fn net_dont_touch(&self, net: &str) -> bool {
        self.db.net_is_do_not_touch(net)
    }
    fn net_special(&self, net: &str) -> bool {
        self.db.net_is_special(net)
    }
    fn net_iterms(&self, net: &str) -> Vec<String> {
        self.db.net_iterms(net)
    }
    fn net_bterms(&self, net: &str) -> Vec<String> {
        self.db.net_bterms(net)
    }
    fn net_first_output(&self, net: &str) -> Option<String> {
        Some(self.db.net_get_first_output(net)).filter(|t| !t.is_empty())
    }
    fn inst_dont_touch(&self, inst: &str) -> bool {
        self.db.inst_is_do_not_touch(inst)
    }
    fn inst_is_buffer(&self, inst: &str) -> Option<bool> {
        self.libs.link_cell(&self.db.inst_master(inst)).map(|c| c.is_buffer())
    }
    /// `Network::drivers(net)`: its output (or inout) instance terminals and its input (or inout)
    /// ports; tristate when the liberty port is (`isAnyTristate`).
    fn net_drivers(&self, net: &str) -> Vec<(String, bool)> {
        let mut out = Vec::new();
        for it in self.db.net_iterms(net) {
            if !self.is_driver_pin(&it) {
                continue;
            }
            let (inst, term) = it.rsplit_once('/').expect("inst/pin");
            let tristate = self
                .libs
                .link_cell(&self.db.inst_master(inst))
                .and_then(|c| c.ports.iter().find(|p| p.name == term))
                .is_some_and(|p| p.is_any_tristate());
            out.push((it, tristate));
        }
        out.extend(self.db.net_bterms(net).into_iter().filter(|b| self.is_driver_pin(b)).map(|b| (b, false)));
        out
    }
    /// A top-level port is in the clock network when it is a clock's source.
    fn port_is_clock(&self, port: &str) -> bool {
        self.clock_sources.iter().any(|c| c == port)
    }
    fn port_is_constant(&self, port: &str) -> bool {
        self.constants.contains_key(port)
    }
    fn insert_buffer_after_driver(&mut self, drvr: &str, cell: &str, reason: &str) -> Result<String, String> {
        let term = self.term(drvr);
        let inst = self.db.insert_buffer_after_driver(term, cell, None, reason, None, "ALWAYS").map_err(|e| e.to_string())?;
        self.insert_buffer_post_process(&inst, cell)?;
        Ok(inst)
    }
    fn insert_buffer_before_load(&mut self, load: &str, cell: &str, reason: &str) -> Result<String, String> {
        let term = self.term(load);
        let inst = self.db.insert_buffer_before_load(term, cell, None, reason, None, "ALWAYS").map_err(|e| e.to_string())?;
        self.insert_buffer_post_process(&inst, cell)?;
        Ok(inst)
    }
}

impl CliDesign<'_> {
    /// A pin as the database addresses it: a port, or an instance's terminal.
    pub fn term<'p>(&self, pin: &'p str) -> (Option<&'p str>, &'p str) {
        match pin.rsplit_once('/') {
            Some((inst, t)) if !self.netlist.ports.iter().any(|p| p.0 == pin) => (Some(inst), t),
            _ => (None, pin),
        }
    }

    /// `insertBufferPostProcess` (`setLocation`: clamped to the core, placed), and the odb
    /// callbacks: the nets on the new buffer's input and output marked invalid.
    pub fn insert_buffer_post_process(&mut self, inst: &str, cell: &str) -> Result<(), String> {
        self.area_incr(master_area(self.db, cell));
        let at = self.clamp_loc_to_core(self.db.inst_location(inst), cell);
        self.db.set_inst_location(inst, at.0, at.1).map_err(|e| e.to_string())?;
        self.gr_inst_moved(inst);
        self.gr_legal_cell_pos(inst)?;
        let c = self.libs.link_cell(cell).ok_or_else(|| format!("{cell}: no liberty cell"))?;
        let (input, output) = c.buffer_ports().ok_or_else(|| format!("{cell}: not a buffer"))?;
        let in_net = self.db.net_of(inst, &input.name);
        let out_net = self.db.net_of(inst, &output.name);
        self.invalidate(&in_net);
        self.invalidate(&out_net);
        // the edit made the buffer and reconnected the pins of its two nets: only those are read
        // again (a full read per port buffer was O(design) each)
        let touched: BTreeSet<String> = [in_net, out_net].into_iter().filter(|n| !n.is_empty()).collect();
        self.refresh_touched(&[inst.to_string()], &touched)
    }
}

/// `EstimateParasitics::makeSteinerTree(drvr_pin)`, as the resizer's buffered net reads it.
pub fn steiner_tree(db: &Db, net: &str, drvr_pin: &str, alpha: f32) -> Option<vyges_rsz::buffered_net::Tree> {
    let tree = vyges_est::placement::make_steiner_tree_for_driver(db, net, drvr_pin, alpha, &est_stt).ok()??;
    Some(vyges_rsz::buffered_net::Tree {
        deg: tree.tree.deg,
        branch: tree.tree.branch.iter().map(|b| (b.x, b.y, b.n)).collect(),
        pinlocs: tree.pinlocs.iter().map(|p| (p.name.clone(), p.x, p.y)).collect(),
        drvr_pt: tree.drvr_pt,
    })
}

pub fn to_rsmt(t: &vyges_stt::Tree) -> vyges_grt::RsmtTree {
    vyges_grt::RsmtTree { deg: t.deg, length: t.length, branch: t.branch.iter().map(|b| vyges_grt::Branch { x: b.x, y: b.y, n: b.n }).collect() }
}

/// `SteinerTreeBuilder::makeSteinerTree(x, y, drvr, alpha)`, as the global router calls it.
pub fn grt_stt(x: &[i32], y: &[i32], drvr: usize, alpha: f32) -> vyges_grt::RsmtTree {
    LUT.with(|lut| to_rsmt(&vyges_stt::make_steiner_tree(lut, x, y, drvr, alpha).0.expect("a Steiner tree")))
}

/// FastRoute's pre-sorted FLUTE.
pub fn grt_flutes(xs: &[i32], ys: &[i32], s: &[usize], acc: i32) -> vyges_grt::RsmtTree {
    LUT.with(|lut| to_rsmt(&vyges_stt::flute::medium_degree::flutes_all_degree_acc(lut, xs.len(), xs, ys, s, acc).expect("flute")))
}

/// The pin order (`dbNetwork::id`, flat: ITerm id × 2, BTerm id × 2 + 1) of every net pin, and
/// the nets `repairDriver` passes over (dont_touch, connected by abutment).
pub fn net_info(db: &Db, nl: &vyges_sta::netlist::Netlist) -> Result<NetInfo, String> {
    let mut info = NetInfo::default();
    for net in &nl.nets {
        for c in &net.pins {
            let name = nl.pin_name(c);
            let id = match c {
                vyges_sta::netlist::Conn::Inst(i, pin) => u64::from(db.iterm_id(&nl.insts[*i].0, pin).map_err(|e| e.to_string())?) << 1,
                vyges_sta::netlist::Conn::Port(_) => (u64::from(db.bterm_id(&name).map_err(|e| e.to_string())?) << 1) | 1,
            };
            info.pin_id.insert(name, id);
        }
        if db.net_is_do_not_touch(&net.name) {
            info.dont_touch.insert(net.name.clone());
        }
        if db.net_is_connected_by_abutment(&net.name) {
            info.abutment.insert(net.name.clone());
        }
        if !crate::timing::is_valid_sig_type(&db.net_get_sig_type(&net.name)) {
            info.unplaced_sig_type.insert(net.name.clone());
        }
    }
    info.dont_touch_insts = nl.insts.iter().map(|i| &**i).filter(|(i, _)| db.inst_is_do_not_touch(i)).map(|(i, _)| i.clone()).collect();
    info.fixed_insts = nl.insts.iter().map(|i| &**i).filter(|(i, _)| db.inst_is_fixed(i)).map(|(i, _)| i.clone()).collect();
    Ok(info)
}

/// `Resizer::area(dbMaster)`: `dbuToMeters(width) × dbuToMeters(height)` (each side divided by
/// `dbu × 1e6` in `double`), 0 when the master is not core autoplaceable.
pub fn master_area(db: &Db, master: &str) -> f64 {
    let dbu = f64::from(db.tech_get_db_units_per_micron());
    let t = db.master_get_type(master).unwrap_or_default().replace(' ', "_");
    let placeable = (t.starts_with("CORE") || t.starts_with("BLOCK") || t.starts_with("ENDCAP")) && !matches!(t.as_str(), "ENDCAP_TOPLEFT" | "ENDCAP_TOPRIGHT" | "ENDCAP_BOTTOMLEFT" | "ENDCAP_BOTTOMRIGHT");
    if !placeable {
        return 0.0;
    }
    f64::from(db.master_get_width(master)) / (dbu * 1e6) * (f64::from(db.master_get_height(master)) / (dbu * 1e6))
}

/// `Resizer::computeDesignArea`: over the block's instances in order, each master's
/// `width × height` in m² (`dbuToMeters` each side) — 0 for a master that is not core
/// autoplaceable (`isCoreAutoPlaceable`) — fillers (`CORE SPACER`) left out.
pub fn design_area(db: &Db) -> f64 {
    let mut area = 0.0f64;
    for inst in db.inst_names() {
        let master = db.inst_master(&inst);
        let t = db.master_get_type(&master).unwrap_or_default().replace(' ', "_");
        if t == "CORE_SPACER" {
            continue;
        }
        area += master_area(db, &master);
    }
    area
}

/// `Resizer::clampLocToCore`: with a core, the origin held inside it so the master fits — the
/// upper bound is `max(core min, core max − master size)`, so a master wider than the core sits at
/// the core's lower edge. No core (`core_exists_` false): unchanged.
pub fn clamp_loc_to_core(core: Option<(i32, i32, i32, i32)>, (w, h): (i32, i32), (x, y): (i32, i32)) -> (i32, i32) {
    let Some((x0, y0, x1, y1)) = core else { return (x, y) };
    let x_max = x0.max(x1 - w);
    let y_max = y0.max(y1 - h);
    (x.clamp(x0, x_max), y.clamp(y0, y_max))
}

pub fn sizing_limits(db: &Db) -> Result<vyges_rsz::sizing::SizingLimits, String> {
    let d = vyges_rsz::sizing::SizingLimits::default();
    Ok(vyges_rsz::sizing::SizingLimits {
        area: db.block_double_property("limit_sizing_area").map_err(|e| e.to_string())?.or(d.area),
        leakage: db.block_double_property("limit_sizing_leakage").map_err(|e| e.to_string())?.or(d.leakage),
        keep_site: db.block_bool_property("keep_sizing_site").map_err(|e| e.to_string())?.unwrap_or(false),
        keep_vt: db.block_bool_property("keep_sizing_vt").map_err(|e| e.to_string())?.unwrap_or(false),
    })
}

/// The instance pins a `set_max_capacitance` names (`pin_cap_limit_map_`).
pub fn sdc_cap_pins(s: &vyges_loom::sdc::Sdc) -> BTreeSet<String> {
    s.max_capacitance_objects.iter().filter(|(_, o)| o.accessor == "get_pins").flat_map(|(_, o)| o.names.iter().cloned()).collect()
}

/// Each port's `set_max_capacitance` (`Sdc::setCapacitanceLimit(port)`, the last command's value),
/// in farads like the design's limit.
pub fn sdc_port_cap_limits(s: &vyges_loom::sdc::Sdc, cap_scale: f32) -> BTreeMap<String, f32> {
    let mut out = BTreeMap::new();
    for (v, o) in s.max_capacitance_objects.iter().filter(|(_, o)| o.accessor == "get_ports") {
        for port in &o.names {
            out.insert(port.clone(), vyges_rsz::timing::user_cap_to_sta(*v, cap_scale));
        }
    }
    out
}

/// `remove_buffers` on the database: the buffer test per master (the link corner's liberty cell,
/// a CORE master), the SDC's constrained instance pins ([`crate::env::remove_buffers_sdc_pins`];
/// an SDC naming an instance or a net is refused before this runs).
pub struct OdbRemoval<'a> {
    pub db: &'a mut Db,
    /// Per master: its buffer ports (input, output), or none when it is not a removable buffer.
    pub buffers: HashMap<String, Option<(String, String)>>,
    pub pins: BTreeSet<String>,
}

impl<'a> OdbRemoval<'a> {
    pub fn new(db: &'a mut Db, libs: &Libs, masters: &BTreeMap<String, crate::preamble::Master>, pins: BTreeSet<String>) -> Self {
        let mut buffers = HashMap::new();
        for (name, m) in masters {
            let ports = libs.link_cell(name).filter(|c| m.logic_std && c.is_buffer()).and_then(|c| c.buffer_ports()).map(|(i, o)| (i.name.clone(), o.name.clone()));
            buffers.insert(name.clone(), ports);
        }
        OdbRemoval { db, buffers, pins }
    }
    fn net(&self, inst: &str, pin: &str) -> Option<String> {
        Some(self.db.net_of(inst, pin)).filter(|n| !n.is_empty())
    }
}

impl crate::remove_buffers::RemovalDesign for OdbRemoval<'_> {
    fn insts(&self) -> Vec<String> {
        self.db.block_get_insts()
    }
    fn buffer_ports(&self, inst: &str) -> Option<(String, String)> {
        self.buffers.get(&self.db.inst_master(inst)).cloned().flatten()
    }
    fn net_of(&self, inst: &str, pin: &str) -> Option<String> {
        self.net(inst, pin)
    }
    fn inst_dont_touch(&self, inst: &str) -> bool {
        self.db.inst_is_do_not_touch(inst)
    }
    fn inst_fixed(&self, inst: &str) -> bool {
        self.db.inst_is_fixed(inst)
    }
    fn net_dont_touch(&self, net: &str) -> bool {
        self.db.net_is_do_not_touch(net)
    }
    fn net_has_port(&self, net: &str) -> bool {
        !self.db.net_bterms(net).is_empty()
    }
    fn can_merge_net(&self, survivor: &str, removed: &str) -> bool {
        self.db.net_can_merge(survivor, removed).unwrap_or(false)
    }
    fn net_insts(&self, net: &str) -> Vec<String> {
        self.db.net_iterms(net).into_iter().filter_map(|p| p.rsplit_once('/').map(|(i, _)| i.to_string())).collect()
    }
    fn pin_constrained(&self, pin: &str) -> bool {
        self.pins.contains(pin)
    }
    fn inst_constrained(&self, _inst: &str) -> bool {
        false
    }
    fn net_constrained(&self, _net: &str) -> bool {
        false
    }
    fn remove(&mut self, inst: &str, in_pin: &str, out_pin: &str, unprotect: bool) -> Result<(), String> {
        let e = |x: vyges_opendb::Error| x.to_string();
        let in_net = self.net(inst, in_pin).expect("removeBuffer checked the input net");
        let out_net = self.net(inst, out_pin);
        // `BufferRemovalProtection`: the buffer's and both nets' dont-touch cleared for the edit;
        // the survivor's restored after (the removed net is gone).
        let mut restore = false;
        if unprotect {
            let keep_output = out_net.as_deref().is_some_and(|o| buffer_removal_keeps_output_net(!self.db.net_bterms(&in_net).is_empty(), !self.db.net_bterms(o).is_empty()));
            let (survivor, removed) = if keep_output { (out_net.clone().expect("kept"), Some(in_net.clone())) } else { (in_net.clone(), out_net.clone()) };
            if self.db.inst_is_do_not_touch(inst) {
                self.db.inst_set_do_not_touch(inst, false).map_err(e)?;
            }
            restore = self.db.net_is_do_not_touch(&survivor);
            if restore {
                self.db.net_set_do_not_touch(&survivor, false).map_err(e)?;
            }
            if let Some(r) = removed.filter(|r| self.db.net_is_do_not_touch(r)) {
                self.db.net_set_do_not_touch(&r, false).map_err(e)?;
            }
        }
        let survivor = self.db.remove_buffer(inst, in_pin, out_pin).map_err(e)?;
        if restore {
            // With no output net the buffer is destroyed and the input net is the survivor.
            let name = if survivor.is_empty() { in_net } else { survivor };
            self.db.net_set_do_not_touch(&name, true).map_err(e)?;
        }
        Ok(())
    }
}

/// `bufferRemovalKeepsOutputNet` on a flat netlist (upstream aa89cb9694, 9ffa16d003): removing
/// a buffer keeps its output net only when that net has a port and the input net has none.
pub fn buffer_removal_keeps_output_net(in_net_has_port: bool, out_net_has_port: bool) -> bool {
    !in_net_has_port && out_net_has_port
}

#[cfg(test)]
mod swap_master_tests {
    use std::path::Path;

    /// Upstream rule (`dbInst::swapMaster` at 4155ac3): a swap between masters whose terminals
    /// match by name remaps each terminal's master index and re-sorts the instance's terminal
    /// list; it connects and disconnects nothing. Every net keeps its terminals, in the same
    /// order, with the same ids — what lets `CliDesign::swap_master` re-read the instance alone.
    #[test]
    fn a_swap_leaves_every_net_with_its_pins_in_order_and_their_ids() {
        let ex = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/long_wire");
        let dir = std::env::temp_dir().join(format!("vyges-rsz-swap-master-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        // b1 drives b2, b3 and b4 (the net a resized driver and a resized load share)
        let def = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN fan ;\nUNITS DISTANCE MICRONS 2000 ;\n\
DIEAREA ( 0 0 ) ( 200000 28000 ) ;\nROW ROW_0 core 0 0 N DO 500 BY 1 STEP 400 0 ;\nCOMPONENTS 4 ;\n\
    - b1 BUF_X1 + PLACED ( 20000 0 ) N ;\n    - b2 BUF_X1 + PLACED ( 40000 0 ) N ;\n\
    - b3 BUF_X2 + PLACED ( 60000 0 ) N ;\n    - b4 BUF_X1 + PLACED ( 80000 0 ) N ;\nEND COMPONENTS\n\
PINS 1 ;\n    - in1 + NET in1 + DIRECTION INPUT + USE SIGNAL\n      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 200 1400 ) N ;\nEND PINS\n\
NETS 2 ;\n    - in1 ( PIN in1 ) ( b1 A ) + USE SIGNAL ;\n    - n1 ( b1 Z ) ( b2 A ) ( b3 A ) ( b4 A ) + USE SIGNAL ;\nEND NETS\nEND DESIGN\n";
        std::fs::write(dir.join("d.def"), def).unwrap();
        let mut db = vyges_opendb::Db::new();
        db.read_lef(ex.join("cells.lef").to_str().unwrap()).unwrap();
        db.read_def(dir.join("d.def").to_str().unwrap(), "default").unwrap();
        let pins = |db: &vyges_opendb::Db| -> Vec<(String, Vec<(String, u32)>)> {
            db.net_names()
                .into_iter()
                .map(|n| {
                    let ts = db.net_iterms(&n).into_iter().map(|t| {
                        let (i, p) = t.rsplit_once('/').unwrap();
                        let id = db.iterm_id(i, p).unwrap();
                        (t.clone(), id)
                    });
                    (n.clone(), ts.collect())
                })
                .collect()
        };
        let before = pins(&db);
        assert_eq!(before.iter().find(|(n, _)| n == "n1").map(|(_, p)| p.len()), Some(4));
        assert!(db.swap_master("b3", "BUF_X4").unwrap());
        assert!(db.swap_master("b1", "BUF_X4").unwrap());
        assert_eq!(db.inst_master("b1"), "BUF_X4");
        assert_eq!(pins(&db), before);
    }
}

#[cfg(test)]
mod held_order_tests {
    use super::*;
    use vyges_sta::netlist::{Conn, Net, Netlist};

    /// Rule (odb `dbTable::create` takes the LAST freed id and `dbBlock` iterates by id): while
    /// the order is held, instances and nets made are appended; settling puts each at its id's
    /// rank, so a held netlist reordered by the permutation is the netlist a full read in id
    /// order gives, its pins' instance indices moved with their instances.
    #[test]
    fn a_held_netlist_settled_is_the_full_read_in_id_order() {
        let order = |appended_insts: &[(u32, usize)], appended_nets: &[(u32, usize)]| DbOrder {
            inst_id: HashMap::new(),
            inst_ids: vec![2, 5, 9],
            net_ids: vec![4, 8],
            net_slot: HashMap::new(),
            appended_insts: appended_insts.iter().copied().collect(),
            appended_nets: appended_nets.iter().copied().collect(),
        };
        assert!(order(&[], &[]).in_db_order());
        // ids 3 and 7 freed earlier and reused: appended at 3 and 4, their ranks are 1 and 3
        let mut o = order(&[(3, 3), (7, 4)], &[(1, 2)]);
        assert!(!o.in_db_order());
        let (insts, nets) = o.db_permutation();
        assert_eq!(insts, [0, 3, 1, 4, 2]);
        assert_eq!(nets, [2, 0, 1]);
        let pin = |i: usize| Conn::Inst(i, "A".into());
        let held = Netlist {
            insts: ["u2", "u5", "u9", "b3", "b7"].iter().map(|n| vyges_sta::netlist::inst(*n, "buf")).collect(),
            ports: vec![],
            nets: vec![
                Net { name: "n4".into(), pins: vec![pin(0), pin(3)] }.into(),
                Net { name: "n8".into(), pins: vec![pin(2)] }.into(),
                Net { name: "n1".into(), pins: vec![pin(4), pin(1)] }.into(),
            ],
        };
        let settled = reordered(&held, &insts, &nets);
        let names: Vec<&str> = settled.insts.iter().map(|i| i.0.as_str()).collect();
        assert_eq!(names, ["u2", "b3", "u5", "b7", "u9"]);
        let net = |k: usize| (settled.nets[k].name.clone(), settled.nets[k].pins.clone());
        assert_eq!(net(0), ("n1".into(), vec![pin(3), pin(2)]));
        assert_eq!(net(1), ("n4".into(), vec![pin(0), pin(1)]));
        assert_eq!(net(2), ("n8".into(), vec![pin(4)]));
        o.settle();
        assert!(o.in_db_order());
        assert_eq!((o.inst_ids, o.net_ids), (vec![2, 3, 5, 7, 9], vec![1, 4, 8]));
    }

    /// A net whose instances all keep their positions is the same allocation after the reorder.
    #[test]
    fn a_net_whose_instances_stay_is_shared() {
        let nl = Netlist {
            insts: vec![vyges_sta::netlist::inst("u1", "buf"), vyges_sta::netlist::inst("u2", "buf")],
            ports: vec![],
            nets: vec![Net { name: "a".into(), pins: vec![Conn::Inst(0, "A".into())] }.into()],
        };
        let same = reordered(&nl, &[0, 1], &[0]);
        assert!(std::sync::Arc::ptr_eq(&same.nets[0], &nl.nets[0]));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Upstream rule (aa89cb9694, 9ffa16d003): the output net survives a buffer removal only when
    /// it has a port and the input net has none; with ports on both sides the input net survives
    /// (before them a port on the output net alone decided it).
    /// Upstream rule (`OdbCallBack::inDbITermPostDisconnect` at 4155ac3): every iterm
    /// disconnect invalidates its net's parasitics, a supply net too, so deleting a buffer leaves
    /// VDD and VSS pending (`pendingNetCount`) until the next update. Witness: the instrumented
    /// reference on repair_setup_parallel_eval_path_group had VDD/VSS pending in exactly the 20
    /// of 268 counts that followed an UnbufferMove.
    #[test]
    fn deleting_an_instance_leaves_its_supply_nets_pending() {
        let nets = ["n1", "VDD", "", "VSS", "n2"].map(String::from);
        assert_eq!(nets_left_by_deletion(nets.into_iter(), &["n1", "n2"]), ["VDD", "VSS"]);
    }

    #[test]
    fn a_buffer_removal_keeps_the_output_net_only_when_the_input_net_has_no_port() {
        assert!(buffer_removal_keeps_output_net(false, true));
        assert!(!buffer_removal_keeps_output_net(true, true), "ports on both nets: the input net survives");
        assert!(!buffer_removal_keeps_output_net(true, false));
        assert!(!buffer_removal_keeps_output_net(false, false));
    }
}
