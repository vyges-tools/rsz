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
    /// [`CliDesign::refresh_touched`]'s appended path extends it rather than reading every name.
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

/// An edit-log line (`Db::edit_log_start`'s format) that names one of `nets` in any field.
fn edit_names_any(ev: &str, nets: &std::collections::HashSet<String>) -> bool {
    ev.split(['|', ',', ';', '=']).any(|t| nets.contains(t))
}

#[cfg(test)]
mod held_skips_tests {
    use super::edit_names_any;

    /// The held skips are dropped by an edit naming a held net ANYWHERE in the line: as the
    /// connected net, in a net's terminal list, or inside a swapped instance's `T=net=pins` terms.
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
    pub fn update_routes(&mut self, db: &mut Db) -> Result<(), String> {
        if self.dirty.is_empty() {
            return Ok(());
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
        Ok(())
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

/// What the appended refresh knows of the database's order without reading every name: each
/// listed instance's position and the highest instance id listed; each listed net's id, ascending
/// (the block iterates by id), its position, and each listed net name's position.
pub struct DbOrder {
    pos: HashMap<String, usize>,
    inst_max: u32,
    net_ids: Vec<u32>,
    net_pos: HashMap<u32, usize>,
    net_slot: HashMap<String, usize>,
}

impl DbOrder {
    /// Read once per netlist made by a full read: `None` when the listed order is not the id order.
    fn read(db: &vyges_opendb::Db, nl: &vyges_sta::netlist::Netlist) -> Option<DbOrder> {
        let pos: HashMap<String, usize> = nl.insts.iter().map(|i| &**i).enumerate().map(|(i, (n, _))| (n.clone(), i)).collect();
        let mut inst_max = 0;
        for (k, (n, _)) in nl.insts.iter().map(|i| &**i).enumerate() {
            let id = db.inst_id(n).ok()?;
            if k > 0 && id <= inst_max {
                return None;
            }
            inst_max = id;
        }
        let mut net_ids = Vec::with_capacity(nl.nets.len());
        for n in &nl.nets {
            let id = db.net_id(&n.name).ok()?;
            if net_ids.last().is_some_and(|&last| id <= last) {
                return None;
            }
            net_ids.push(id);
        }
        let net_pos = net_ids.iter().enumerate().map(|(k, &id)| (id, k)).collect();
        let net_slot = nl.nets.iter().enumerate().map(|(k, n)| (n.name.clone(), k)).collect();
        Some(DbOrder { pos, inst_max, net_ids, net_pos, net_slot })
    }
}

/// The incremental timer's side of the edits: the database's callbacks as read, and the
/// estimator's invalid set as ITS callbacks keep it (`parasitics_invalid_`) — apart from
/// [`CliDesign::invalid`], which a journal undo restores whole, where the reference's undo
/// invalidates every net it touches.
#[derive(Default)]
pub struct TimerLog {
    pub on: bool,
    pub events: Vec<String>,
    pub est_invalid: BTreeSet<String>,
    pub updated: Vec<String>,
}

/// What a journal level puts back on undo besides the database (the netlist is read from the
/// database again, as the timer's network reads it live).
pub struct JournalState {
    pub parasitics: Vec<HashMap<String, NetParasitics>>,
    pub invalid: BTreeSet<String>,
    pub sdc_nets: BTreeSet<String>,
}

impl CliDesign<'_> {
    /// After an edit: the timer's netlist and the pin order read again from the database.
    pub fn refresh(&mut self) -> Result<(), String> {
        let nl = vyges_grt::timer::netlist(self.db);
        if nl != *self.netlist {
            self.netlist_gen += 1;
        }
        self.netlist = std::sync::Arc::new(nl);
        self.info = net_info(self.db, &self.netlist)?;
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
        use vyges_sta::netlist::{Conn, Net, Netlist};
        if std::env::var_os("VYGES_RSZ_FULL_REFRESH").is_some() {
            return self.refresh();
        }
        let old = self.netlist.clone();
        if let Some(fast) = self.appended_netlist(&old, new_insts, touched) {
            return self.refresh_finish(old, fast, new_insts, touched);
        }
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
        if check {
            let full = vyges_grt::timer::netlist(self.db);
            let info = net_info(self.db, &full)?;
            if full != *self.netlist || info != self.info {
                return Err(format!("refresh_touched: differs from the full read (netlist {}, info {})", full == *self.netlist, info == self.info));
            }
        }
        Ok(())
    }

    /// [`CliDesign::refresh_touched`] when the edit only APPENDED: the database iterates by id and
    /// an insertion takes a fresh one, so every instance and every untouched net keeps its place.
    /// Read by id, not by every name ([`DbOrder`]): a new instance must take an id after every
    /// listed one; a touched net found at a listed id replaces that slot (a rename keeps the id),
    /// one at a new id after every listed one is appended in id order. Only the edit is read —
    /// the full read cost every instance and net name per repeater, O(design²) over a repair.
    /// `None` when the database's order does not bear that out: an id reused or before the last,
    /// a touched net gone, special or left pinless, a listed net renamed to a name not touched.
    fn appended_netlist(&mut self, old: &std::sync::Arc<vyges_sta::netlist::Netlist>, new_insts: &[String], touched: &BTreeSet<String>) -> Option<(vyges_sta::netlist::Netlist, bool, bool)> {
        use vyges_sta::netlist::{Conn, Net, Netlist};
        let mut ord = match self.inst_pos.take() {
            Some((w, ord)) if std::sync::Weak::ptr_eq(&w, &std::sync::Arc::downgrade(old)) => ord,
            cached => {
                // ⚠️ O(design): every instance and net id read again
                if std::env::var_os("VYGES_RSZ_ORDER_TRACE").is_some() {
                    eprintln!("VYGORDER|read|{}", if cached.is_some() { "another netlist" } else { "none kept" });
                }
                DbOrder::read(self.db, old)?
            }
        };
        let mut nl = Netlist { insts: old.insts.clone(), ports: old.ports.clone(), nets: Vec::with_capacity(old.nets.len() + 2) };
        // an instance re-read (the general path's `new_insts`) takes its master from the database;
        // one not listed is new, appended in id order
        let mut master_changed = false;
        let mut made = Vec::new();
        for i in new_insts {
            if let Some(&k) = ord.pos.get(i) {
                let m = self.db.inst_master(i);
                master_changed |= m != nl.insts[k].1;
                std::sync::Arc::make_mut(&mut nl.insts[k]).1 = m;
            } else {
                made.push((self.db.inst_id(i).ok()?, i));
            }
        }
        made.sort();
        for (id, inst) in made {
            if id <= ord.inst_max {
                return None;
            }
            ord.inst_max = id;
            ord.pos.insert(inst.clone(), nl.insts.len());
            nl.insts.push(vyges_sta::netlist::inst(inst.clone(), self.db.inst_master(inst)));
        }
        let port_index: HashMap<&str, usize> = nl.ports.iter().enumerate().map(|(i, (n, _))| (n.as_str(), i)).collect();
        let mut replaced: HashMap<usize, Net> = HashMap::new();
        let mut appended: Vec<(u32, Net)> = Vec::new();
        for net in touched {
            let id = self.db.net_id(net).ok()?;
            let slot = ord.net_pos.get(&id).copied();
            if self.db.net_is_special(net) {
                if slot.is_some() {
                    return None;
                }
                continue;
            }
            let mut pins = Vec::new();
            for iterm in self.db.net_iterms(net) {
                let (inst, term) = iterm.rsplit_once('/').unwrap_or((&iterm, ""));
                pins.push(Conn::Inst(*ord.pos.get(inst)?, term.to_string()));
            }
            for bterm in self.db.net_bterms(net) {
                if let Some(&p) = port_index.get(bterm.as_str()) {
                    pins.push(Conn::Port(p));
                }
            }
            match slot {
                Some(_) if pins.is_empty() => return None,
                Some(k) => {
                    replaced.insert(k, Net { name: net.clone(), pins });
                }
                None if pins.is_empty() => {}
                None if ord.net_ids.last().is_none_or(|&last| id > last) => appended.push((id, Net { name: net.clone(), pins })),
                None => return None,
            }
        }
        // a listed net whose name was touched but is not at its slot any more was renamed to a
        // name the edit did not name — its slot would keep the stale name
        if touched.iter().any(|t| ord.net_slot.get(t).is_some_and(|k| !replaced.contains_key(k))) {
            return None;
        }
        nl.nets.extend(old.nets.iter().cloned());
        // every old name out before any new one in: two slots may trade names
        for k in replaced.keys() {
            ord.net_slot.remove(&old.nets[*k].name);
        }
        for (k, r) in replaced {
            ord.net_slot.insert(r.name.clone(), k);
            nl.nets[k] = r.into();
        }
        appended.sort_by_key(|(id, _)| *id);
        for (id, n) in appended {
            ord.net_pos.insert(id, nl.nets.len());
            ord.net_slot.insert(n.name.clone(), nl.nets.len());
            ord.net_ids.push(id);
            nl.nets.push(n.into());
        }
        self.inst_pos = Some((std::sync::Weak::new(), ord));
        Some((nl, false, master_changed))
    }

    /// The rest of [`CliDesign::refresh_touched`] on an appended netlist: the per-net and
    /// per-instance flags, the change test, the generation.
    fn refresh_finish(&mut self, old: std::sync::Arc<vyges_sta::netlist::Netlist>, (nl, destroyed, master_changed): (vyges_sta::netlist::Netlist, bool, bool), new_insts: &[String], touched: &BTreeSet<String>) -> Result<(), String> {
        use vyges_sta::netlist::Conn;
        if destroyed {
            let live: BTreeSet<String> = self.db.net_names().into_iter().collect();
            self.info.dont_touch.retain(|n| live.contains(n));
            self.info.abutment.retain(|n| live.contains(n));
        }
        // the touched nets by their slots (the appended path's own map), not by a pass over every net
        let slots: Vec<usize> = match self.inst_pos.as_ref() {
            Some((_, ord)) => touched.iter().filter_map(|t| ord.net_slot.get(t).copied()).collect(),
            None => nl.nets.iter().enumerate().filter(|(_, n)| touched.contains(&n.name)).map(|(k, _)| k).collect(),
        };
        for net in slots.iter().map(|&k| &nl.nets[k]) {
            for c in &net.pins {
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
        }
        for i in new_insts {
            if self.db.inst_is_do_not_touch(i) {
                self.info.dont_touch_insts.insert(i.clone());
            }
            if self.db.inst_is_fixed(i) {
                self.info.fixed_insts.insert(i.clone());
            }
        }
        // Changed: a count, a re-read master (the old instances keep theirs), a touched net's pins.
        let changed = master_changed || nl.insts.len() != old.insts.len() || nl.nets.len() != old.nets.len() || {
            let pins = |l: &vyges_sta::netlist::Netlist| -> HashMap<String, Vec<Conn>> { l.nets.iter().filter(|n| touched.contains(&n.name)).map(|n| (n.name.clone(), n.pins.clone())).collect() };
            pins(&old) != pins(&nl)
        };
        let check = std::env::var_os("VYGES_RSZ_CHECK_REFRESH").is_some();
        if check && changed != (nl != *old) {
            return Err(format!("refresh_touched (appended): change test says {changed}, the netlists say {}", nl != *old));
        }
        if changed {
            self.netlist_gen += 1;
        }
        self.netlist = std::sync::Arc::new(nl);
        if let Some((w, _)) = self.inst_pos.as_mut() {
            *w = std::sync::Arc::downgrade(&self.netlist);
        }
        if check {
            let full = vyges_grt::timer::netlist(self.db);
            let info = net_info(self.db, &full)?;
            if full != *self.netlist || info != self.info {
                return Err(format!("refresh_touched (appended): differs from the full read (netlist {}, info {})", full == *self.netlist, info == self.info));
            }
        }
        Ok(())
    }

    /// The database's callbacks since the last read, into the timer's log; the estimator's
    /// callbacks (`est::OdbCallBack`) on its invalid set: a net created, an instance terminal
    /// connected or disconnected, every net of a swapped instance → invalid; a net destroyed →
    /// erased.
    pub fn pull_edits(&mut self) {
        if !self.timer.on && !self.skip_log {
            return;
        }
        for ev in self.db.edit_log_take() {
            if self.skips.as_ref().is_some_and(|h| h.touched_by(&ev)) {
                if std::env::var_os("VYGES_RSZ_SKIPS_TRACE").is_some() {
                    eprintln!("VYGSKIPS|dropped|{ev}");
                }
                self.skips = None;
            }
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
            p.port_pin_caps = Some(self.port_caps.clone());
            for map in self.parasitics.iter_mut() {
                map.insert(net.to_string(), p.clone());
            }
        }
    }

    /// The router's state as a refused callback left it, or the incremental re-route.
    pub fn gr_update_routes(&mut self) -> Result<(), String> {
        if let Some(e) = self.gr_error.take() {
            return Err(e);
        }
        match self.gr.as_mut() {
            Some(gr) => gr.update_routes(self.db),
            None => Ok(()),
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
        let inst = self.db.insert_buffer_before_loads(net, &iterms, &bterms, cell, Some(loc), reason, None, uniquify, loads_on_diff_nets).map_err(|e| e.to_string())?;
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
        // Estimated now and erased from the invalid set, with no delay invalidation.
        self.pull_edits();
        self.timer.est_invalid.remove(net);
        if self.gr.is_some() {
            // Under global-route parasitics: invalid or with no pi model (no network here — a net
            // with a route has one) → the dirty nets re-routed, then this net from its route.
            if self.invalid.contains(net) || !self.parasitics[0].contains_key(net) {
                self.gr_update_routes()?;
                self.gr_estimate(net);
                self.invalid.remove(net);
            }
            return Ok(());
        }
        if !self.estimating || !(self.invalid.contains(net) || !self.parasitics[0].contains_key(net)) {
            return Ok(());
        }
        let Some(drvr) = self.netlist.nets.iter().find(|n| n.name == net).and_then(|n| {
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
            let skips = vyges_est::placement::skips(self.db, &timing)?;
            let mut nets: std::collections::HashSet<String> = skips.clock_nets().iter().cloned().collect();
            nets.extend(skips.constant_pins().iter().map(|p| self.net_of_load(p)).filter(|n| !n.is_empty()));
            self.skips = Some(HeldSkips { skips, nets });
            if !self.skip_log && !self.timer.on {
                self.db.edit_log_start().map_err(|e| e.to_string())?;
            }
            self.skip_log = true;
        }
        let skips = &self.skips.as_ref().expect("computed above").skips;
        let e = vyges_est::placement::estimate_net_with_skips(self.db, &timing, skips, net, &drvr, self.alpha, &est_stt)?;
        for k in 0..self.parasitics.len() {
            match net_parasitic(self.db, self.rc, &e, k)? {
                Some(mut p) => {
                    p.port_pin_caps = Some(self.port_caps.clone());
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
        let updated = std::mem::take(&mut self.timer.est_invalid);
        self.timer.updated.extend(updated);
        if self.gr.is_some() {
            // Under global-route parasitics the dirty nets are re-routed FIRST, whatever is
            // invalid; then each invalid net from its route (a net with none keeps its network).
            gr_trace(&format!("VYGR|est|update|src=2|{}", self.invalid.iter().map(|n| format!("{n};")).collect::<String>()));
            self.gr_update_routes()?;
            for net in std::mem::take(&mut self.invalid) {
                self.gr_estimate(&net);
            }
            return Ok(());
        }
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
        // `Sta::replaceCell`: with equivalent arcs, `replaceEquivCellBefore` swaps the arc sets of
        // the edges leaving the INPUT pins only — an output-to-output edge keeps the cell it was
        // made with (the first one, across further equivalent swaps); otherwise the instance's
        // edges are made anew from the new cell.
        let from = self.netlist.insts.iter().map(|i| &**i).find(|(n, _)| n == inst).map(|(_, c)| c.clone());
        if let Some(from) = from {
            let equiv = matches!((self.libs.link_cell(&from), self.libs.link_cell(cell)), (Some(a), Some(b)) if vyges_sta::incr::equiv_cells_arcs(a, b));
            if equiv {
                self.stale_out_arcs.entry(inst.to_string()).or_insert(from);
            } else {
                self.stale_out_arcs.remove(inst);
            }
        }
        if !self.db.swap_master(inst, cell).map_err(|e| e.to_string())? {
            return Err(format!("swapMaster {inst} -> {cell} failed"));
        }
        // `Resizer::replaceCell`: "legalize the position of the instance in case it leaves the die".
        self.gr_legal_cell_pos(inst)?;
        self.refresh()?;
        let nets: Vec<String> = self.netlist.nets.iter().filter(|n| n.pins.iter().any(|c| matches!(c, vyges_sta::netlist::Conn::Inst(i, _) if self.netlist.insts[*i].0 == inst))).map(|n| n.name.clone()).collect();
        for n in nets {
            self.invalidate(&n);
        }
        // `Sta::replace(Equiv)CellAfter` → `loadPinCapacitanceChanged` on each input pin: the
        // reduced models of its net's drivers are deleted — a placement estimate IS one, so the
        // net has no parasitic until `updateParasitics` estimates it again (it is invalid).
        // Witness: SizeDownFanout's next round in the same batch reads the driver timed on its
        // pin caps alone.
        let lib_cell = self.libs.link_cell(cell);
        let input_nets: Vec<String> = self
            .netlist
            .nets
            .iter()
            .filter(|n| {
                n.pins.iter().any(|c| {
                    matches!(c, vyges_sta::netlist::Conn::Inst(i, port) if self.netlist.insts[*i].0 == inst
                        && lib_cell.and_then(|lc| lc.port(port)).is_some_and(|p| matches!(p.direction, vyges_sta::liberty::Direction::Input | vyges_sta::liberty::Direction::Bidirect)))
                })
            })
            .map(|n| n.name.clone())
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
        // Restarting clears the log: what the held skips have not yet read goes with it.
        self.skips = None;
        self.db.edit_log_start().map_err(|e| e.to_string())?;
        self.timer = TimerLog { on: true, ..TimerLog::default() };
        Ok(())
    }

    fn take_timer_edits(&mut self) -> vyges_rsz::design::TimerEdits {
        self.pull_edits();
        vyges_rsz::design::TimerEdits { events: std::mem::take(&mut self.timer.events), updated_nets: std::mem::take(&mut self.timer.updated) }
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
    /// neither net dont_touch; then the net to go (the input net when the output net has a port,
    /// else the output net): neither buffer pin a clock's source nor named by `set_max_capacitance`
    /// (`Sdc::isConstrained(pin)`: the pin constraints an accepted SDC can put on an instance pin), the net to go not named by
    /// `set_load` (`isConstrained(net)`), and that net mergeable into the other
    /// (`dbNet::canMergeNet`), or absent.
    fn can_remove_buffer(&self, inst: &str, in_pin: &str, out_pin: &str) -> bool {
        if self.db.inst_is_do_not_touch(inst) || self.db.inst_is_fixed(inst) {
            return false;
        }
        let in_net = self.db.net_of(inst, in_pin);
        let out_net = self.db.net_of(inst, out_pin);
        if [&in_net, &out_net].iter().any(|n| !n.is_empty() && self.db.net_is_do_not_touch(n)) {
            return false;
        }
        let out_net_ports = !out_net.is_empty() && !self.db.net_bterms(&out_net).is_empty();
        let (survivor, removed) = if out_net_ports { (out_net, in_net) } else { (in_net, out_net) };
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
        // The database half, in the candidate's order (disconnect, deleteInstance, makeInstance ×2,
        // makeNet, connectPin ×4, the module nets): inv1 takes the freed slot.
        let Some((inv1, inv2, mid)) = self.db.buffer_to_inverters(buffer, in_pin, out_pin, inv_cell, inv_in, inv_out).map_err(e)? else { return Ok(None) };
        // ⚠️ `makeInstance` places inv1 before inv2 exists; here both exist (inv2 unplaced) when
        // inv1 is placed. Placement reads placed instances only, so the order cannot show.
        for (name, loc) in [(&inv1, locs[0]), (&inv2, locs[1])] {
            self.db.inst_set_source_type(name, "TIMING").map_err(e)?;
            let at = self.clamp_loc_to_core(loc, inv_cell);
            self.db.set_inst_location(name, at.0, at.1).map_err(e)?;
            self.gr_inst_moved(name);
            self.gr_legal_cell_pos(name)?;
        }
        for n in [in_net, mid, out_net] {
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
        let survivor = self.db.remove_buffer(inst, in_pin, out_pin).map_err(|e| e.to_string())?;
        for map in self.parasitics.iter_mut() {
            map.remove(&in_net);
            map.remove(&out_net);
        }
        self.invalid.remove(&in_net);
        self.invalid.remove(&out_net);
        self.invalidate(&survivor);
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
        self.refresh()
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
