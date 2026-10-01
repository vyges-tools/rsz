// SPDX-License-Identifier: Apache-2.0
//! `vyges-rsz` — electrical repair from a JSON job.
//!
//! ```text
//! vyges-rsz repair_design <job.json> [-o FILE]
//! ```
//!
//! The job: `steps`, the case's commands in order as `{cmd, args}` with the arguments as Tcl
//! evaluated them (`read_lef`, `read_def`, `read_db`, `read_liberty`, `read_sdc`, `set_dont_use`,
//! `set_layer_rc`, `set_wire_rc`, `estimate_parasitics -placement`, `repair_design …`), and
//! `trace` (where to write one `tag|…` line per decision, in the order they are made).

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::process::ExitCode;

use serde_json::{json, Value};
use vyges_opendb::Db;
use vyges_rsz::preamble::{Libs, Master};
use vyges_rsz::repair_design::{self, Args, Inputs};
use vyges_rsz::trace::Trace;
use vyges_rsz::Stop;
use vyges_rsz::timing::{user_time_to_sta, Limits, NetInfo};
use vyges_sta::graph::NetParasitics;
use vyges_sta::liberty::Library;

thread_local! {
    static LUT: vyges_stt::flute::lut::Lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

/// `SteinerTreeBuilder::makeSteinerTree(x, y, drvr, alpha)`, as the estimator calls it.
fn est_stt(x: &[i32], y: &[i32], drvr: usize, alpha: f32) -> vyges_est::placement::SttTree {
    LUT.with(|lut| {
        let t = vyges_stt::make_steiner_tree(lut, x, y, drvr, alpha).0.expect("a Steiner tree");
        vyges_est::placement::SttTree { deg: t.deg, branch: t.branch.iter().map(|b| vyges_est::placement::Branch { x: b.x, y: b.y, n: b.n }).collect() }
    })
}

/// Whether the signal wire capacitance is zero or unset (EST-0018): the estimator then makes no
/// parasitics at all, and every net is timed lumped — as the router's timer does.
fn no_signal_cap(db: &Db, rc: &vyges_est::rc::Rc) -> bool {
    let tech = db.tech_get_name();
    let zero = (0..rc.scenes.len()).any(|k| {
        let v = rc.resolved(&tech, k);
        (v[2] + v[3]) / 2.0 == 0.0
    });
    zero || rc.resolve(&tech, |w| &w.signal_cap).is_empty()
}

/// One net's estimate in one scene (its corner's RC) as the timer reads it: its network, pin
/// nodes named by pin; `None` where the estimator makes none.
fn net_parasitic(db: &Db, rc: &vyges_est::rc::Rc, n: &vyges_est::placement::NetEstimate, scene: usize) -> Result<Option<NetParasitics>, String> {
    use vyges_est::placement::Decision;
    let tech = db.tech_get_name();
    let g = match &n.decision {
        Decision::Tree { tree, non_leaf_clock, .. } => {
            let cx = vyges_est::network::NetCtx { db, rc, tech: &tech, corner: scene, is_clk: *non_leaf_clock };
            vyges_est::network::make_steiner_parasitic(&cx, &n.net, tree).map_err(|e| e.to_string())?
        }
        Decision::Pad { pins } => vyges_est::network::make_pad_parasitic(db, pins).map_err(|e| e.to_string())?,
        _ => return Ok(None),
    };
    Ok(Some(vyges_grt::timer::placement_network(&n.net, &g)))
}

/// `estimate_parasitics -placement` as the timer reads it: every net's estimate, per scene ("make
/// separate parasitics for each corner"). Empty where the estimator makes none (EST-0018). `rc`'s
/// layers must be sorted already.
fn placement_parasitics_at(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], alpha: f32, scenes: usize) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    use vyges_est::placement::{estimate_wire_parasitics, Timing};
    if no_signal_cap(db, rc) {
        return Ok(vec![HashMap::new(); scenes]);
    }
    let timing = Timing { liberty: Some(liberty), clock_sources: clock_sources.to_vec(), propagated: false };
    let nets = estimate_wire_parasitics(db, &timing, alpha, &est_stt).map_err(|e| e.to_string())?;
    let mut out = vec![HashMap::new(); scenes];
    for (k, map) in out.iter_mut().enumerate() {
        for n in &nets {
            if let Some(p) = net_parasitic(db, rc, n, k)? {
                map.insert(n.net.clone(), p);
            }
        }
    }
    Ok(out)
}

/// `Resizer::clampLocToCore`: with a core, the origin held inside it so the master fits — the
/// upper bound is `max(core min, core max − master size)`, so a master wider than the core sits at
/// the core's lower edge. No core (`core_exists_` false): unchanged.
fn clamp_loc_to_core(core: Option<(i32, i32, i32, i32)>, (w, h): (i32, i32), (x, y): (i32, i32)) -> (i32, i32) {
    let Some((x0, y0, x1, y1)) = core else { return (x, y) };
    let x_max = x0.max(x1 - w);
    let y_max = y0.max(y1 - h);
    (x.clamp(x0, x_max), y.clamp(y0, y_max))
}

/// `PatternMatch` without regexp: `*` matches any run of characters, `?` any one; the rest exactly.
fn glob_match(pat: &str, s: &str) -> bool {
    let (p, t): (Vec<char>, Vec<char>) = (pat.chars().collect(), s.chars().collect());
    let (mut pi, mut ti, mut star, mut mark) = (0usize, 0usize, None::<usize>, 0usize);
    while ti < t.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == t[ti]) {
            pi += 1;
            ti += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = Some(pi);
            mark = ti;
            pi += 1;
        } else if let Some(sp) = star {
            pi = sp + 1;
            mark += 1;
            ti = mark;
        } else {
            return false;
        }
    }
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}

/// `parse_min_max_all_flags`: `-min` alone is min, `-max` alone max, neither or both all — as
/// `[min, max]` membership.
fn min_max_all(flags: &[String]) -> [bool; 2] {
    match (flags.iter().any(|f| f == "-min"), flags.iter().any(|f| f == "-max")) {
        (true, false) => [true, false],
        (false, true) => [false, true],
        _ => [true, true],
    }
}

/// `parse_rise_fall_flags`: `-rise` alone rise, `-fall` alone fall, neither or both both.
fn rise_fall(flags: &[String]) -> [bool; 2] {
    match (flags.iter().any(|f| f == "-rise"), flags.iter().any(|f| f == "-fall")) {
        (true, false) => [true, false],
        (false, true) => [false, true],
        _ => [true, true],
    }
}

/// The SDC's `set_load` and `set_input_transition`, as the timer keeps them: values through the
/// user unit (`capacitance_ui_sta` / `time_ui_sta`: the value times the default library's FLOAT
/// unit scale, in double, narrowed to the float setter), applied in file order (a later command
/// overwrites); a net's load keyed by the net's DRIVER pins when it is set (`setNetWireCap`); a
/// port's load is its pin load (the default). Refused: `-wire_load`, objects named by anything
/// but `get_nets` / `get_ports`, and an input transition for a clock.
fn sdc_env(s: &vyges_loom::sdc::Sdc, libs: &Libs, nl: &vyges_sta::netlist::Netlist) -> Result<vyges_sta::graph::SdcEnv, String> {
    let mut env = vyges_sta::graph::SdcEnv::default();
    // `set_driving_cell -lib_cell C -pin P [-from_pin F] [-input_transition_rise/-fall S]`: the
    // port is driven through C's arcs into P, from F (else the default from-port), at slews S
    // (user time unit; 0 when absent).
    for d in &s.driving_cells {
        if let Some(f) = d.flags.first() {
            return Err(format!("set_driving_cell {f}: not modelled"));
        }
        if d.library.is_some() {
            return Err("set_driving_cell -library: not modelled".into());
        }
        let (Some(cell), Some(pin)) = (&d.lib_cell, &d.pin) else {
            return Err("set_driving_cell without -lib_cell and -pin: not modelled".into());
        };
        let lib0 = libs.default_library().ok_or("set_driving_cell before any liberty library")?;
        if libs.link_cell(cell).and_then(|c| c.port(pin)).is_none() {
            return Err(format!("set_driving_cell -lib_cell {cell} -pin {pin}: no such cell port (not modelled)"));
        }
        let slews = [vyges_rsz::timing::user_time_to_sta(d.input_transition[0], lib0.time_scale), vyges_rsz::timing::user_time_to_sta(d.input_transition[1], lib0.time_scale)];
        for port in &d.objects {
            env.input_drive.insert(port.clone(), vyges_sta::graph::InputDrive { cell: cell.clone(), from_port: d.from_pin.clone(), to_port: pin.clone(), from_slews: slews });
        }
    }
    if s.env.is_empty() {
        return Ok(env);
    }
    let lib0 = libs.default_library().ok_or("set_load / set_input_transition before any liberty library")?;
    let graph = vyges_sta::graph::Graph::build(&libs.libs, nl).map_err(|e| format!("{e} (not modelled)"))?;
    for e in &s.env {
        let known = ["-min", "-max", "-rise", "-fall", "-pin_load", "-subtract_pin_load"];
        if let Some(f) = e.flags.iter().find(|f| !known.contains(&f.as_str())) {
            return Err(format!("{} {f}: not modelled", e.cmd));
        }
        let mm = min_max_all(&e.flags);
        let rf = rise_fall(&e.flags);
        match (e.cmd.as_str(), e.accessor.as_str()) {
            ("set_load", "get_nets") => {
                let cap = (e.value * f64::from(lib0.cap_scale)) as f32;
                let subtract = e.flags.iter().any(|f| f == "-subtract_pin_load");
                for net in &e.objects {
                    let n = nl.nets.iter().position(|x| &x.name == net).ok_or_else(|| format!("set_load: net {net} not found (not modelled)"))?;
                    for d in graph.vertices.iter().enumerate().filter(|(i, v)| v.is_driver && graph.vertex_net[*i] == Some(n)) {
                        let slot = env.net_wire_cap.entry(d.1.name.clone()).or_insert([None; 2]);
                        for k in 0..2 {
                            if mm[k] {
                                slot[k] = Some((cap, subtract));
                            }
                        }
                    }
                }
            }
            ("set_load", "get_ports") => {
                let cap = (e.value * f64::from(lib0.cap_scale)) as f32;
                for port in &e.objects {
                    let slot = env.port_pin_cap.entry(port.clone()).or_insert([[None; 2]; 2]);
                    for r in 0..2 {
                        for k in 0..2 {
                            if rf[r] && mm[k] {
                                slot[r][k] = Some(cap);
                            }
                        }
                    }
                }
            }
            ("set_input_transition", "get_ports") if e.clock.is_none() => {
                let slew = vyges_rsz::timing::user_time_to_sta(e.value, lib0.time_scale);
                for port in &e.objects {
                    let slot = env.input_slew.entry(port.clone()).or_insert([[None; 2]; 2]);
                    for r in 0..2 {
                        for k in 0..2 {
                            if rf[r] && mm[k] {
                                slot[r][k] = Some(slew);
                            }
                        }
                    }
                }
            }
            (cmd, acc) => return Err(format!("{cmd} on [{acc} …]{}: not modelled", if e.clock.is_some() { " -clock" } else { "" })),
        }
    }
    Ok(env)
}

/// `ConcreteParasiticNetwork::disconnectPin` for each pin: its node is renamed to a subnode of the
/// net (`<net>:<next id>`, no longer a pin) and its ground capacitance dropped.
fn disconnect_pins(p: &mut NetParasitics, net: &str, pins: &[String]) {
    for pin in pins {
        if let Some(i) = p.node_names.iter().position(|n| n == pin) {
            p.node_names[i] = format!("{net}:{}", p.node_names.len() + i);
            p.network.node_caps[i] = 0.0;
        }
    }
}

/// The design as `repair_design` edits it: the database, the timer's netlist read from it, and
/// the estimator's per-net cache with its invalid set (see [`vyges_rsz::design`]).
struct CliDesign<'a> {
    db: &'a mut Db,
    rc: &'a vyges_est::rc::Rc,
    liberty: &'a vyges_est::liberty::LibertyClocks,
    libs: &'a Libs,
    clock_sources: &'a [String],
    /// The routing alpha in force while the repair runs.
    alpha: f32,
    /// Whether the estimator makes parasitics at all (not EST-0018).
    estimating: bool,
    netlist: vyges_sta::netlist::Netlist,
    /// Per scene.
    parasitics: Vec<HashMap<String, NetParasitics>>,
    invalid: BTreeSet<String>,
    info: NetInfo,
    /// `core_` and `core_exists_`.
    core: Option<(i32, i32, i32, i32)>,
}

impl CliDesign<'_> {
    /// After an edit: the timer's netlist and the pin order read again from the database.
    fn refresh(&mut self) -> Result<(), String> {
        self.netlist = vyges_grt::timer::netlist(self.db);
        self.info = net_info(self.db, &self.netlist)?;
        Ok(())
    }

    /// `EstimateParasitics::parasiticsInvalid(net)` (the odb callbacks call it).
    fn invalidate(&mut self, net: &str) {
        if self.estimating && !net.is_empty() {
            self.invalid.insert(net.to_string());
        }
    }

    /// The net a load pin (`inst/pin` or a port) is on.
    fn net_of_load(&self, load: &str) -> String {
        match load.rsplit_once('/') {
            Some((inst, pin)) if !self.netlist.ports.iter().any(|p| p.0 == load) => self.db.net_of(inst, pin),
            _ => self.db.bterm_net(load),
        }
    }

    fn clamp_loc_to_core(&self, loc: (i32, i32), master: &str) -> (i32, i32) {
        clamp_loc_to_core(self.core, (self.db.master_get_width(master) as i32, self.db.master_get_height(master) as i32), loc)
    }
}

impl vyges_rsz::design::Design for CliDesign<'_> {
    fn netlist(&self) -> &vyges_sta::netlist::Netlist {
        &self.netlist
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
        if !self.estimating || !(self.invalid.contains(net) || !self.parasitics[0].contains_key(net)) {
            return Ok(());
        }
        let Some(drvr) = self.netlist.nets.iter().find(|n| n.name == net).and_then(|n| {
            n.pins.iter().map(|c| self.netlist.pin_name(c)).find(|p| self.is_driver_pin(p))
        }) else {
            self.invalid.remove(net);
            return Ok(());
        };
        let timing = vyges_est::placement::Timing { liberty: Some(self.liberty), clock_sources: self.clock_sources.to_vec(), propagated: false };
        let e = vyges_est::placement::estimate_net(self.db, &timing, net, &drvr, self.alpha, &est_stt)?;
        for k in 0..self.parasitics.len() {
            match net_parasitic(self.db, self.rc, &e, k)? {
                Some(p) => self.parasitics[k].insert(net.to_string(), p),
                None => self.parasitics[k].remove(net),
            };
        }
        self.invalid.remove(net);
        Ok(())
    }

    /// `updateParasitics` under placement parasitics: each invalid net estimated again (the order
    /// is the reference's set's; each net's estimate is its own, so it is no value).
    fn update_parasitics(&mut self) -> Result<(), String> {
        let invalid: Vec<String> = self.invalid.iter().cloned().collect();
        for net in invalid {
            self.ensure_wire_parasitic(&net)?;
        }
        Ok(())
    }

    fn steiner(&self, net: &str, drvr_pin: &str) -> Option<vyges_rsz::buffered_net::Tree> {
        steiner_tree(self.db, net, drvr_pin, self.alpha)
    }

    /// `Resizer::insertBufferBeforeLoads(nullptr, loads, cell, &loc, reason)`: odb's own
    /// `dbNet::insertBufferBeforeLoads` on the first load's net (new names `<reason><n>` /
    /// `net<n>`, always uniquified), then `insertBufferPostProcess` (`setLocation`: clamped to
    /// the core, PLACED). The odb callbacks invalidate the new net and the original one.
    fn insert_repeater(&mut self, loads: &[String], cell: &str, loc: (i32, i32), reason: &str) -> Result<vyges_rsz::design::Repeater, String> {
        let original = self.net_of_load(loads.first().ok_or("insertBufferBeforeLoads: no loads specified")?);
        let mut iterms = Vec::new();
        let mut bterms = Vec::new();
        for l in loads {
            match l.rsplit_once('/') {
                Some((inst, pin)) if !self.netlist.ports.iter().any(|p| &p.0 == l) => iterms.push((inst.to_string(), pin.to_string())),
                _ => bterms.push(l.clone()),
            }
        }
        let inst = self.db.insert_buffer_before_loads(None, &iterms, &bterms, cell, Some(loc), reason, None, "ALWAYS", false).map_err(|e| e.to_string())?;
        let at = self.clamp_loc_to_core(self.db.inst_location(&inst), cell);
        self.db.set_inst_location(&inst, at.0, at.1).map_err(|e| e.to_string())?;
        let c = self.libs.link_cell(cell).ok_or_else(|| format!("{cell}: no liberty cell"))?;
        let (input, output) = c.buffer_ports().ok_or_else(|| format!("{cell}: not a buffer"))?;
        let in_net = self.db.net_of(&inst, &input.name);
        let out_net = self.db.net_of(&inst, &output.name);
        // A port load names the new net after the port and renames the original: the cache
        // follows the net, not the name.
        for map in self.parasitics.iter_mut() {
            if in_net != original {
                if let Some(p) = map.remove(&original) {
                    map.insert(in_net.clone(), p);
                }
            }
            map.remove(&out_net);
            // `ConcreteParasitics::disconnectPinBefore` on each load moved off the original net:
            // its pin node in that net's (stale) network becomes a fresh internal subnode — the
            // resistors handed over, no ground capacitance, no pin. A connected pin adds nothing:
            // until the net is estimated again, its new input pin has no node.
            if let Some(p) = map.get_mut(&in_net) {
                disconnect_pins(p, &in_net, loads);
            }
        }
        self.invalidate(&in_net);
        self.invalidate(&out_net);
        self.refresh()?;
        Ok(vyges_rsz::design::Repeater { input: format!("{inst}/{}", input.name), output: format!("{inst}/{}", output.name), inst, out_net })
    }

    /// `Resizer::replaceCell` → `dbInst::swapMaster`; the callback invalidates the net on every
    /// pin of the instance (not a tristate one).
    fn swap_master(&mut self, inst: &str, cell: &str) -> Result<(), String> {
        if !self.db.swap_master(inst, cell).map_err(|e| e.to_string())? {
            return Err(format!("swapMaster {inst} -> {cell} failed"));
        }
        self.refresh()?;
        let nets: Vec<String> = self.netlist.nets.iter().filter(|n| n.pins.iter().any(|c| matches!(c, vyges_sta::netlist::Conn::Inst(i, _) if self.netlist.insts[*i].0 == inst))).map(|n| n.name.clone()).collect();
        for n in nets {
            self.invalidate(&n);
        }
        Ok(())
    }

    fn inst_location(&self, inst: &str) -> (i32, i32) {
        self.db.inst_location(inst)
    }

    fn pin_location(&self, pin: &str) -> (i32, i32) {
        match pin.rsplit_once('/') {
            Some((inst, term)) if !self.netlist.ports.iter().any(|p| p.0 == pin) => {
                self.db.iterm_avg_xy(inst, term).unwrap_or_else(|| (self.db.inst_get_origin_x(inst), self.db.inst_get_origin_y(inst)))
            }
            _ => self.db.bterm_first_pin_location(pin).unwrap_or((0, 0)),
        }
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

impl CliDesign<'_> {
    /// A pin that drives its net: an instance output (or inout) pin, or an input port.
    fn is_driver_pin(&self, pin: &str) -> bool {
        match pin.rsplit_once('/') {
            Some((inst, term)) if !self.netlist.ports.iter().any(|p| p.0 == pin) => matches!(self.db.iterm_get_io_type(inst, term).as_str(), "OUTPUT" | "INOUT"),
            _ => matches!(self.db.bterm_get_io_type(pin).as_str(), "INPUT" | "INOUT"),
        }
    }
}

/// `EstimateParasitics::makeSteinerTree(drvr_pin)`, as the resizer's buffered net reads it.
fn steiner_tree(db: &Db, net: &str, drvr_pin: &str, alpha: f32) -> Option<vyges_rsz::buffered_net::Tree> {
    let tree = vyges_est::placement::make_steiner_tree_for_driver(db, net, drvr_pin, alpha, &est_stt).ok()??;
    Some(vyges_rsz::buffered_net::Tree {
        deg: tree.tree.deg,
        branch: tree.tree.branch.iter().map(|b| (b.x, b.y, b.n)).collect(),
        pinlocs: tree.pinlocs.iter().map(|p| (p.name.clone(), p.x, p.y)).collect(),
        drvr_pt: tree.drvr_pt,
    })
}

fn read_text(path: &str) -> Result<String, String> {
    if path.ends_with(".gz") {
        use std::io::Read;
        let mut s = String::new();
        flate2::read::MultiGzDecoder::new(std::fs::File::open(path).map_err(|e| format!("{path}: {e}"))?)
            .read_to_string(&mut s)
            .map_err(|e| format!("{path}: {e}"))?;
        Ok(s)
    } else {
        std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))
    }
}

/// Every LEF master the database holds: its site and the IMPLANT layers among its obstructions.
fn masters(db: &Db) -> Result<BTreeMap<String, Master>, String> {
    let mut out = BTreeMap::new();
    let mut layer_type: BTreeMap<i64, (String, String)> = BTreeMap::new();
    for i in 0..db.num_masters().map_err(|e| e.to_string())? {
        let name = db.nth_master_name(i).map_err(|e| e.to_string())?;
        let mut implant_obs = Vec::new();
        for (layer, ..) in db.master_obstruction_boxes(&name).map_err(|e| e.to_string())? {
            let (lname, ltype) = layer_type.entry(layer).or_insert_with(|| {
                let lname = db.layer_name_by_number(layer);
                let ltype = db.layer_get_type(&lname).unwrap_or_default();
                (lname, ltype)
            });
            if ltype.eq_ignore_ascii_case("IMPLANT") && !implant_obs.contains(lname) {
                implant_obs.push(lname.clone());
            }
        }
        let area = i64::from(db.master_get_width(&name)) * i64::from(db.master_get_height(&name));
        out.insert(name.clone(), Master { site: db.master_get_site(&name), area, is_core: db.master_is_core(&name), logic_std: db.master_get_type(&name).map_err(|e| e.to_string())? == "CORE", implant_obs });
    }
    Ok(out)
}

/// Every master's signal terminals (not POWER / GROUND), the pins the timer's graph has.
fn master_pins(db: &Db) -> Result<HashMap<String, Vec<String>>, String> {
    let mut out = HashMap::new();
    for i in 0..db.num_masters().map_err(|e| e.to_string())? {
        let name = db.nth_master_name(i).map_err(|e| e.to_string())?;
        let terms = db.master_mterms(&name).map_err(|e| e.to_string())?;
        out.insert(name, terms.into_iter().filter(|(_, t)| t != "POWER" && t != "GROUND").map(|(n, _)| n).collect());
    }
    Ok(out)
}

/// The pin order (`dbNetwork::id`, flat: ITerm id × 2, BTerm id × 2 + 1) of every net pin, and
/// the nets `repairDriver` passes over (dont_touch, connected by abutment).
fn net_info(db: &Db, nl: &vyges_sta::netlist::Netlist) -> Result<NetInfo, String> {
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
    info.dont_touch_insts = nl.insts.iter().filter(|(i, _)| db.inst_is_do_not_touch(i)).map(|(i, _)| i.clone()).collect();
    Ok(info)
}

/// `repair_design`'s Tcl arguments. `-max_wire_length` is in microns (the default distance
/// unit) and reaches the repair in meters.
fn parse_args(args: &[String]) -> Result<Args, String> {
    let mut a = Args::default();
    let mut i = 0;
    let num = |v: Option<&String>, flag: &str| -> Result<f64, String> {
        v.ok_or(format!("{flag} needs a value"))?.parse::<f64>().map_err(|_| format!("{flag}: not a number"))
    };
    while i < args.len() {
        match args[i].as_str() {
            // `sta::distance_ui_sta`: microns times the distance unit's FLOAT scale, in double.
            "-max_wire_length" => {
                a.max_wire_length = num(args.get(i + 1), "-max_wire_length")? * f64::from(1e-6f32);
                i += 1;
            }
            "-slew_margin" => {
                a.slew_margin = num(args.get(i + 1), "-slew_margin")?;
                i += 1;
            }
            "-cap_margin" => {
                a.cap_margin = num(args.get(i + 1), "-cap_margin")?;
                i += 1;
            }
            // Deprecated: it only turns the early sizing round on.
            "-buffer_gain" => {
                a.pre_placement = true;
                i += 1;
            }
            "-pre_placement" => a.pre_placement = true,
            "-verbose" => {}
            f @ ("-match_cell_footprint" | "-reroute" | "-max_utilization") => return Err(format!("repair_design {f}: not modelled")),
            other => return Err(format!("repair_design argument {other}: not modelled")),
        }
        i += 1;
    }
    Ok(a)
}

fn run(job: &Value) -> Result<Value, String> {
    let mut db = Db::new();
    let mut libs = Libs::default();
    let mut set_dont_use: BTreeSet<String> = BTreeSet::new();
    let mut liberty = vyges_est::liberty::LibertyClocks::default();
    let mut rc = vyges_est::rc::Rc::new();
    let mut sdc: Option<vyges_loom::sdc::Sdc> = None;
    let mut estimated = false;
    // `set_routing_alpha` (the Steiner builder's; 0.3 by default): the estimate reads the value
    // in force when it ran, the repair's trees the value in force when repair_design does.
    let mut routing_alpha = 0.3f32;
    let mut estimate_alpha = routing_alpha;
    // The design the estimate saw, when it differs from the one repaired (a job's `db` on the
    // estimate step: the reference keeps an estimate across a later cell MOVE — est has no callback
    // for one — so it repairs on the earlier placement's parasitics).
    let mut estimate_db: Option<String> = None;
    // The RC in force when the estimate ran (later re-estimates read the RC in force then).
    let mut estimate_rc: Option<vyges_est::rc::Rc> = None;
    let units = |l: &vyges_est::liberty::LibertyClocks| -> Result<vyges_est::rc::Units, String> {
        let u = l.units.ok_or("RC before a liberty library: the timer's default units are not modelled")?;
        Ok(vyges_est::rc::Units { resistance: u.resistance, capacitance: u.capacitance, distance: u.distance })
    };
    let mut trace = Trace::default();
    let mut outcome: Option<Result<repair_design::Outcome, Stop>> = None;
    for step in job["steps"].as_array().ok_or("steps")? {
        let cmd = step["cmd"].as_str().ok_or("cmd")?;
        let args: Vec<String> = step["args"].as_array().map(|a| a.iter().filter_map(Value::as_str).map(String::from).collect()).unwrap_or_default();
        match cmd {
            "read_lef" => {
                let path = args.last().ok_or("read_lef path")?;
                db.read_lef(path).map_err(|e| format!("{path}: {e}"))?
            }
            "read_def" => {
                let path = args.last().ok_or("read_def path")?;
                db.read_def(path, "default").map_err(|e| format!("{path}: {e}"))?
            }
            "read_db" => {
                let path = args.last().ok_or("read_db path")?;
                db = Db::open(path).map_err(|e| format!("{path}: {e}"))?
            }
            "read_liberty" => {
                if args.iter().any(|a| a == "-min" || a == "-max") {
                    return Err("read_liberty -min / -max: min/max libraries are not modelled".into());
                }
                // `-corner c`: the library serves that scene only; without it, every scene.
                let scene = match args.iter().position(|a| a == "-corner") {
                    Some(i) => {
                        let c = args.get(i + 1).ok_or("read_liberty -corner needs a corner")?;
                        Some(libs.scenes.iter().position(|s| s == c).ok_or_else(|| format!("read_liberty -corner {c}: corner not defined"))?)
                    }
                    None => None,
                };
                let path = args.last().ok_or("read_liberty path")?;
                let text = read_text(path)?;
                liberty.read(&text).map_err(|e| format!("{path}: {e}"))?;
                let group = vyges_sta::liberty_parse::parse(&text).map_err(|e| format!("{path}: {e}"))?;
                // A model's value is scaled by the library's k_ factors at the operating
                // conditions; only an unscaled library is modelled.
                if group.simple.iter().any(|(n, _)| n.starts_with("k_")) || group.groups_of("scaling_factors").next().is_some() {
                    return Err(format!("{path}: liberty k_ scale factors are not modelled"));
                }
                let lib = Library::read(&group).map_err(|e| format!("{path}: {e} (not modelled)"))?;
                if !libs.scene_libs.is_empty() {
                    // Scene 0 is the timer's LINK view: the first library read must serve it.
                    if libs.libs.is_empty() && scene.is_some_and(|k| k != 0) {
                        return Err("the first liberty library read is not for the first corner: the link library's corner is not modelled".into());
                    }
                    for (k, sl) in libs.scene_libs.iter_mut().enumerate() {
                        if scene.is_none_or(|c| c == k) {
                            sl.push(lib.clone());
                        }
                    }
                } else if scene.is_some() {
                    return Err("read_liberty -corner without define_corners (not modelled)".into());
                }
                libs.libs.push(lib);
            }
            // `set_dont_use_cmd`: each element of the list through `get_lib_cells` — a cell-name
            // glob (`*`, `?`) over every library read so far; no match only warns.
            "set_dont_use" => {
                for a in &args {
                    for pat in a.split_whitespace() {
                        if pat.contains(['/', '[', ']', '\\']) {
                            return Err(format!("set_dont_use {pat}: library-qualified or bracketed patterns are not modelled"));
                        }
                        for lib in &libs.libs {
                            set_dont_use.extend(lib.cells.keys().filter(|c| glob_match(pat, c)).cloned());
                        }
                    }
                }
            }
            // The scenes, in order; before any library (a later one would remap them).
            "define_corners" => {
                if !libs.libs.is_empty() {
                    return Err("define_corners after read_liberty: not modelled".into());
                }
                rc.define_corners(&args);
                libs.scenes = args.clone();
                libs.scene_libs = vec![Vec::new(); args.len()];
            }
            "set_layer_rc" => rc.set_layer_rc(&mut db, units(&liberty)?, &args)?,
            // The argument checks first: the reference raises them before converting any value.
            "set_wire_rc" => {
                rc.check_set_wire_rc(&db, &args)?;
                rc.set_wire_rc(&db, units(&liberty)?, &args)?
            }
            "estimate_parasitics" => {
                if !args.iter().any(|a| a == "-placement") {
                    return Err("estimate_parasitics without -placement: not modelled".into());
                }
                estimated = true;
                estimate_alpha = routing_alpha;
                estimate_db = step["db"].as_str().map(String::from);
                estimate_rc = Some(rc.clone());
            }
            "set_routing_alpha" => {
                if args.iter().any(|a| a.starts_with('-')) {
                    return Err(format!("set_routing_alpha {}: per-net / min_fanout / min_hpwl alphas are not modelled", args.join(" ")));
                }
                let [v] = args.as_slice() else { return Err("set_routing_alpha needs one value".into()) };
                routing_alpha = v.parse::<f32>().map_err(|_| format!("set_routing_alpha {v}: not a number"))?;
            }
            // The design's constraints as the reference held them when repair_design ran.
            "read_sdc" => {
                let path = args.last().ok_or("read_sdc path")?;
                let s = vyges_loom::sdc::Sdc::load(path).map_err(|e| format!("{path}: {}", e.0))?;
                // set_max_transition on the design is modelled (the slew limit); on any other
                // object it is not.
                let unmodelled: Vec<&str> = s
                    .ignored_affecting_timing()
                    .into_iter()
                    .filter(|c| (*c != "set_max_transition" || s.max_transition_on_objects) && (*c != "set_max_fanout" || s.max_fanout_on_objects) && *c != "set_driving_cell")
                    .collect();
                if !unmodelled.is_empty() {
                    return Err(format!("{path}: {} not modelled", unmodelled.join(", ")));
                }
                sdc = Some(s);
            }
            "repair_design" => {
                if outcome.is_some() {
                    return Err("a second repair_design: not modelled".into());
                }
                let a = parse_args(&args)?;
                // dont_use_: every liberty dont_use cell (copyDontUseFromLiberty), then set_dont_use.
                let mut dont_use = set_dont_use.clone();
                for lib in &libs.libs {
                    dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
                }
                let m = masters(&db)?;
                let lib0 = libs.default_library().ok_or("repair_design before any liberty library")?;
                // SDC times are read in ns; the user time unit is the first library's.
                if lib0.time_scale != 1e-9f32 {
                    return Err("a library time unit other than 1ns: user-unit conversion is not modelled".into());
                }
                let limits = Limits {
                    design_max_transition: sdc.as_ref().and_then(|s| s.max_transition).map(|v| user_time_to_sta(v, lib0.time_scale)),
                    // `set_max_fanout` takes a float, unscaled.
                    design_max_fanout: sdc.as_ref().and_then(|s| s.max_fanout).map(|v| v as f32),
                };
                let clock_sources: Vec<String> = sdc.as_ref().map(|s| s.clocks.iter().filter(|c| !c.is_virtual()).map(|c| c.source.clone()).collect()).unwrap_or_default();
                let netlist = vyges_grt::timer::netlist(&db);
                if !estimated {
                    return Err("repair_design without estimate_parasitics -placement: not modelled".into());
                }
                // The estimate runs at the alpha in force when estimate_parasitics did.
                rc.sort_clk_and_signal_layers();
                let mut est_rc = estimate_rc.clone().expect("estimated");
                est_rc.sort_clk_and_signal_layers();
                let parasitics = match &estimate_db {
                    None => placement_parasitics_at(&db, &est_rc, &liberty, &clock_sources, estimate_alpha, libs.scene_count())?,
                    Some(path) => {
                        let seen = Db::open(path).map_err(|e| format!("{path}: {e}"))?;
                        // Only a move is modelled: a netlist edit since the estimate leaves the
                        // reference with an incremental state (its own updates) this does not replay.
                        if vyges_grt::timer::netlist(&seen) != netlist {
                            return Err("the netlist was edited between estimate_parasitics and repair_design (e.g. buffer_ports): the reference's incremental parasitics are not modelled".into());
                        }
                        placement_parasitics_at(&seen, &est_rc, &liberty, &clock_sources, estimate_alpha, libs.scene_count())?
                    }
                };
                let wire_rc: Vec<vyges_rsz::buffered_net::WireRc> = (0..libs.scene_count())
                    .map(|k| {
                        let v = rc.resolved(&db.tech_get_name(), k);
                        vyges_rsz::buffered_net::WireRc { h_res: v[0], v_res: v[1], h_cap: v[2], v_cap: v[3] }
                    })
                    .collect();
                if let Some(why) = vyges_rsz::timing::constant_cells(&libs.libs, &netlist) {
                    return Err(format!("{why} (not modelled)"));
                }
                let info = net_info(&db, &netlist)?;
                let netlist_for_env = netlist.clone();
                let env = match &sdc {
                    Some(s) => sdc_env(s, &libs, &netlist_for_env)?,
                    None => Default::default(),
                };
                let dbu = db.tech_get_db_units_per_micron();
                let core = (db.block_get_core_area_x_min(), db.block_get_core_area_y_min(), db.block_get_core_area_x_max(), db.block_get_core_area_y_max());
                let core = (core != (0, 0, 0, 0)).then_some(core);
                let estimating = !no_signal_cap(&db, &rc);
                // A diagnostic: the timer's delay calculation over the design as read, one line per
                // DMP gate call and per load (`Graph::find_delays`' trace), before any repair.
                let mpins = master_pins(&db)?;
                if let Some(path) = job["dcalc_trace"].as_str() {
                    let mut g = vyges_rsz::repair_design::timer_graph(&libs, 0, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
                    let mut lines = Vec::new();
                    g.find_delays(&parasitics[0], Some(&mut lines))?;
                    std::fs::write(path, lines.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                }
                let mut design = CliDesign { db: &mut db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, alpha: routing_alpha, estimating, netlist, parasitics, invalid: BTreeSet::new(), info, core };
                let inputs = Inputs { libs: &libs, masters: &m, dont_use: &dont_use, limits, clock_sources: &clock_sources, dbu, wire_rc, sdc: env, master_pins: mpins };
                outcome = Some(repair_design::repair_design(&inputs, &mut design, &a, &mut trace));
            }
            other => return Err(format!("step {other}: not modelled")),
        }
    }
    if let Some(path) = job["trace"].as_str() {
        std::fs::write(path, trace.text()).map_err(|e| format!("{path}: {e}"))?;
    }
    // The design as the repair left it.
    if let Some(path) = job["write_def"].as_str() {
        db.write_def(path).map_err(|e| format!("{path}: {e}"))?;
    }
    let tags: Vec<&str> = trace.tags.iter().copied().collect();
    let mut report = match outcome {
        None => json!({ "tool": "vyges-rsz", "status": "vacuous", "reason": "no repair_design step" }),
        // Every driver checked and none needed a repair: a CHECKED nothing-to-do, with its count.
        Some(Ok(o)) => {
            let summary: Vec<Value> = o.summary().into_iter().map(|(code, text)| json!({ "code": code, "message": text })).collect();
            json!({
                "tool": "vyges-rsz",
                "status": o.settle(),
                "nets_checked": o.nets_checked,
                "nets_repaired": o.nets_repaired,
                "inserted_buffers": o.inserted_buffers,
                "resized": o.resized,
                "drivers_skipped": o.drivers_skipped,
                "violations": { "slew": o.slew_violations, "capacitance": o.cap_violations, "fanout": o.fanout_violations, "length": o.length_violations },
                "summary": summary,
            })
        }
        Some(Err(Stop::Refused { code, msg })) => json!({ "tool": "vyges-rsz", "status": "refused", "code": code, "reason": msg }),
        Some(Err(Stop::Error { code, msg })) => json!({ "tool": "vyges-rsz", "status": "error", "code": code, "reason": msg }),
    };
    report["traced"] = json!(tags);
    Ok(report)
}

const USAGE: &str = "\
vyges loom rsz — electrical repair of a placed design: repeaters along each net's Steiner tree,
drivers resized, where a wire is too long or a capacitance, fanout or transition limit is broken

USAGE:
  vyges loom rsz repair_design <job.json> [-o FILE]
  vyges loom rsz --describe
  vyges loom rsz --help
  vyges loom rsz --version

JOB FIELDS:
  steps        required — the commands in order, each {\"cmd\": ..., \"args\": [...]}, as a flow
               script passes them:
                 read_lef, read_def, read_db, define_corners, read_liberty [-corner C],
                 read_sdc, set_dont_use, set_layer_rc, set_wire_rc, set_routing_alpha,
                 estimate_parasitics -placement, repair_design [options]
               an estimate_parasitics step may carry \"db\": the database as the estimate saw it,
               when cells were moved between it and the repair
  trace        write one line per decision, in the order the repair makes them, to this path
  write_def    write the design as the repair left it, as DEF, to this path
  dcalc_trace  (diagnostic) write the timer's delay-calculation trace of the design as read

REPAIR_DESIGN OPTIONS:
  -max_wire_length L    the longest wire, microns (0: none)
  -slew_margin P        percent taken off every slew limit
  -cap_margin P         percent taken off every capacitance limit
  -verbose              accepted
  refused: -pre_placement / -buffer_gain, -match_cell_footprint, -reroute, -max_utilization

CONSTRAINTS READ FROM SDC:
  create_clock, set_max_transition and set_max_fanout on the design, set_load on nets and ports,
  set_input_transition, set_driving_cell; any other timing-affecting command is refused

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

REPORT:
  status, nets_checked, nets_repaired, inserted_buffers, resized, drivers_skipped, violations
  {slew, capacitance, fanout, length}, and summary — the repair's closing lines, each with its code

EXIT STATUS:
  0  repaired     the design changed: buffers inserted or drivers resized
  0  up_to_date   drivers were checked and none needed a change (nets_checked)
  2  vacuous      no driver was checked. NOT a pass.
  2  error        usage, unreadable input, or an error the repair raises
  3  refused      an input or an option this engine does not model — see `reason`
";

const PIN_TOKEN: &str = "@OPENROAD_PIN@";

fn describe() -> String {
    DESCRIBE.replace(PIN_TOKEN, vyges_opendb::OPENROAD_PIN)
}

const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "openroad_pin": "@OPENROAD_PIN@",
  "name": "rsz",
  "summary": "electrical repair of a placed design: repeaters inserted along each net's Steiner tree for long wires, max capacitance and max slew",
  "maturity": "experimental",
  "provenance_limitations": [
    "input_hash covers the argument vector, not the content of the job file or of the design files it names.",
    "status is one of repaired, up_to_date, vacuous, refused or error. repaired means the design changed (buffers inserted or drivers resized); up_to_date means drivers were checked and none needed a change (nets_checked says how many); vacuous means nothing was checked and is NOT a pass. The declared assertion passes on repaired or up_to_date. Exit status is 0 for repaired and up_to_date, 2 for vacuous and for error, 3 for refused.",
    "Modelled: placement parasitics, one or more corners, flat and hierarchical netlists, the default buffer selection, the SDC constraints the usage lists. Refused rather than guessed: global-route parasitics, the early sizing round, footprint matching, rerouting, a netlist edited between the estimate and the repair, a tristate driver or a bidirect pin on a net, and any other timing-affecting SDC command."
  ],
  "invocation": {
    "args_template": ["repair_design", "{job}"],
    "optional": [ { "arg": "out", "flag": "-o" } ],
    "emits_json": true
  },
  "inputs": {
    "type": "object",
    "required": ["job"],
    "properties": {
      "job": { "type": "string", "description": "path to a JSON job: {steps: [{cmd, args}], trace, write_def}" },
      "out": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["job"],
  "artifacts": [],
  "assertion": {
    "id": "design-repaired",
    "field": "status",
    "pass_when": { "in": ["repaired", "up_to_date"] }
  }
}"#;

fn link(flag: &str) -> Option<(&'static str, &'static str)> {
    Some(match flag {
        "--bug-report" => ("Report a bug", "https://github.com/vyges/community/issues/new?template=bug_report_template.yaml"),
        "--feature-request" => ("Request a feature", "https://github.com/vyges/community/issues/new?labels=enhancement"),
        "--sponsor" => ("Sponsor Vyges", "https://github.com/sponsors/vyges-ip"),
        "--star" => ("Star this tool", "https://github.com/vyges-tools/rsz"),
        _ => return None,
    })
}

fn exit_for(status: &str) -> u8 {
    match status {
        "repaired" | "up_to_date" => 0,
        "refused" => 3,
        _ => 2,
    }
}

mod events {
    use vyges_events::{emit, Event, Severity};

    /// The run's closing events: the repair's own summary lines (one per non-zero count, under
    /// their message codes), then `RSZ-DONE` with the census — what was checked, what was
    /// changed, and what was passed over — or the refusal / error.
    pub fn outcome(report: &serde_json::Value) {
        let status = report["status"].as_str().unwrap_or("error");
        for line in report["summary"].as_array().into_iter().flatten() {
            if let (Some(code), Some(msg)) = (line["code"].as_str(), line["message"].as_str()) {
                emit(&Event::new("vyges-rsz", Severity::Info, msg.to_string()).with_code(code));
            }
        }
        let (code, severity) = match status {
            "refused" => ("RSZ-REFUSED", Severity::Error),
            "error" => ("RSZ-ERROR", Severity::Error),
            "repaired" | "up_to_date" => ("RSZ-DONE", Severity::Info),
            _ => ("RSZ-DONE", Severity::Warn),
        };
        let text = match report["reason"].as_str() {
            Some(r) => format!("{status}: {r}"),
            None if report["nets_checked"].is_u64() => format!(
                "{status}: {} driver(s) checked, {} passed over; {} net(s) repaired, {} buffer(s) inserted, {} driver(s) resized",
                report["nets_checked"], report["drivers_skipped"].as_u64().unwrap_or(0), report["nets_repaired"].as_u64().unwrap_or(0), report["inserted_buffers"].as_u64().unwrap_or(0), report["resized"].as_u64().unwrap_or(0)
            ),
            None => status.to_string(),
        };
        emit(&Event::new("vyges-rsz", severity, text).with_code(code));
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut out: Option<String> = None;
    let mut positional: Vec<&str> = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "-o" => match args.get(i + 1) {
                Some(v) => {
                    out = Some(v.clone());
                    i += 1;
                }
                None => {
                    eprintln!("vyges-rsz: -o needs a FILE");
                    return ExitCode::from(2);
                }
            },
            "--json" => {}
            a if a.starts_with('-') && !positional.is_empty() => {
                eprintln!("vyges-rsz: unknown option {a}\n\n{USAGE}");
                return ExitCode::from(2);
            }
            a => positional.push(a),
        }
        i += 1;
    }
    match positional.first().copied() {
        None | Some("-h") | Some("--help") => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Some("-V") | Some("--version") => {
            println!("vyges-rsz {} ({})\nCopyright (c) Vyges. Apache-2.0.", env!("CARGO_PKG_VERSION"), env!("VYGES_GIT_SHA"));
            return ExitCode::SUCCESS;
        }
        Some("--describe") => {
            println!("{}", describe());
            return ExitCode::SUCCESS;
        }
        Some(f) if link(f).is_some() => {
            let (label, url) = link(f).unwrap();
            println!("{label}:\n  {url}");
            return ExitCode::SUCCESS;
        }
        Some("repair_design") if positional.len() == 2 => {}
        _ => {
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    }
    vyges_opendb::init_events_logging();
    let path = positional[1];
    let report = match std::fs::read_to_string(path)
        .map_err(|e| format!("{path}: {e}"))
        .and_then(|t| serde_json::from_str::<Value>(&t).map_err(|e| format!("{path}: {e}")))
    {
        Err(e) => json!({ "tool": "vyges-rsz", "status": "error", "reason": e }),
        Ok(job) => match run(&job) {
            Ok(r) => r,
            Err(e) => json!({ "tool": "vyges-rsz", "status": if e.contains("not modelled") { "refused" } else { "error" }, "reason": e }),
        },
    };
    let status = report["status"].as_str().unwrap_or("error").to_string();
    events::outcome(&report);
    let text = format!("{report}\n");
    match &out {
        Some(f) => {
            if let Err(e) = std::fs::write(f, &text) {
                eprintln!("vyges-rsz: {f}: {e}");
                return ExitCode::from(2);
            }
        }
        None => print!("{text}"),
    }
    ExitCode::from(exit_for(&status))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_descriptor_is_valid_json_and_names_the_engine() {
        let d: Value = serde_json::from_str(&describe()).expect("--describe must be valid JSON");
        assert_eq!(d["name"], "rsz");
        assert_eq!(d["assertion"]["pass_when"]["in"], serde_json::json!(["repaired", "up_to_date"]));
    }

    // Rule: only `repaired` exits 0; a refusal is never a pass.
    #[test]
    fn only_repaired_exits_zero() {
        assert_eq!(exit_for("repaired"), 0);
        assert_eq!(exit_for("up_to_date"), 0);
        assert_eq!(exit_for("refused"), 3);
        assert_eq!(exit_for("vacuous"), 2);
        assert_eq!(exit_for("error"), 2);
    }

    // Rule (get_lib_cells' PatternMatch): `*` any run, `?` one character, the rest literal.
    #[test]
    fn dont_use_patterns_are_globs() {
        assert!(glob_match("sky130_fd_sc_hd__probe_p_*", "sky130_fd_sc_hd__probe_p_8"));
        assert!(!glob_match("sky130_fd_sc_hd__probe_p_*", "sky130_fd_sc_hd__probec_p_8"));
        assert!(glob_match("BUF_X?", "BUF_X4") && !glob_match("BUF_X?", "BUF_X16"));
        assert!(glob_match("*", "") && glob_match("BUF_X1", "BUF_X1") && !glob_match("BUF_X1", "BUF_X16"));
    }

    // Rule (Resizer::clampLocToCore): inside the core with room for the master; a master wider
    // than the core at the core's lower edge; no core, no clamp.
    #[test]
    fn a_repeater_is_clamped_into_the_core() {
        let core = Some((100, 200, 1100, 1200));
        assert_eq!(clamp_loc_to_core(core, (50, 50), (0, 5000)), (100, 1150));
        assert_eq!(clamp_loc_to_core(core, (50, 50), (500, 500)), (500, 500));
        assert_eq!(clamp_loc_to_core(core, (2000, 50), (900, 500)), (100, 500));
        assert_eq!(clamp_loc_to_core(None, (50, 50), (-7, 9)), (-7, 9));
    }

    // Rule (the command's argument parsing): -buffer_gain only sets pre_placement; -max_wire_length is microns.
    #[test]
    fn repair_design_arguments() {
        let a = parse_args(&["-max_wire_length".into(), "800".into(), "-buffer_gain".into(), "4".into()]).unwrap();
        assert_eq!((a.max_wire_length, a.pre_placement), (800.0 * f64::from(1e-6f32), true));
        assert!(parse_args(&["-reroute".into()]).unwrap_err().contains("not modelled"));
    }
}
