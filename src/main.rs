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
use vyges_rsz::buffer_ports;
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
/// The ports' `set_load -pin_load`, as an SDC holds them (`SdcEnv::port_pin_cap`).
type PortCaps = HashMap<String, [[Option<f32>; 2]; 2]>;

/// The port loads in force at a step: from the SDC the step carries (`"sdc"`: the constraints
/// as the reference held them when it ran), else from the job's SDC.
fn port_caps_at(path: Option<&str>, current: Option<&vyges_loom::sdc::Sdc>, libs: &Libs, nl: &vyges_sta::netlist::Netlist) -> Result<PortCaps, String> {
    let loaded;
    let s = match path {
        Some(p) => {
            loaded = vyges_loom::sdc::Sdc::load(p).map_err(|e| format!("{p}: {}", e.0))?;
            Some(&loaded)
        }
        None => current,
    };
    Ok(match s {
        Some(s) => sdc_env(s, libs, nl)?.port_pin_cap,
        None => PortCaps::new(),
    })
}

#[allow(clippy::too_many_arguments)]
fn placement_parasitics_at(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    use vyges_est::placement::{estimate_wire_parasitics, Timing};
    if no_signal_cap(db, rc) {
        return Ok(vec![HashMap::new(); scenes]);
    }
    // `isSkipPin`: an ideal clock's nets get no network; a propagated clock's do.
    let timing = Timing { liberty: Some(liberty), clock_sources: clock_sources.to_vec(), propagated };
    let nets = estimate_wire_parasitics(db, &timing, alpha, &est_stt).map_err(|e| e.to_string())?;
    let mut out = vec![HashMap::new(); scenes];
    for (k, map) in out.iter_mut().enumerate() {
        for n in &nets {
            if let Some(mut p) = net_parasitic(db, rc, n, k)? {
                // Reduced now, against the port loads in force now.
                p.port_pin_caps = Some(port_caps.clone());
                map.insert(n.net.clone(), p);
            }
        }
    }
    Ok(out)
}

/// The estimator's state when a command first reads it: every net estimated on the design the
/// estimate saw (`seen`, when cells moved after it; else the design now). Only a MOVE is modelled
/// between the two — a netlist edit is replayed by the command that made it, not read back.
#[allow(clippy::too_many_arguments)]
fn estimate_state(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    match seen {
        None => placement_parasitics_at(db, rc, liberty, clock_sources, propagated, alpha, scenes, port_caps),
        Some(path) => {
            let seen = Db::open(path).map_err(|e| format!("{path}: {e}"))?;
            if &vyges_grt::timer::netlist(&seen) != netlist {
                return Err("the netlist was edited between estimate_parasitics and this command by a step the job does not carry: not modelled".into());
            }
            placement_parasitics_at(&seen, rc, liberty, clock_sources, propagated, alpha, scenes, port_caps)
        }
    }
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

/// `Resizer::initBlock`'s sizing restrictions from the block's properties (`set_opt_config`
/// writes them before the repair): a limit absent is the default; `keep_sizing_site` absent is
/// off. `keep_sizing_vt` is moot: a VT library is refused.
fn sizing_limits(db: &Db) -> Result<vyges_rsz::sizing::SizingLimits, String> {
    let d = vyges_rsz::sizing::SizingLimits::default();
    Ok(vyges_rsz::sizing::SizingLimits {
        area: db.block_double_property("limit_sizing_area").map_err(|e| e.to_string())?.or(d.area),
        leakage: db.block_double_property("limit_sizing_leakage").map_err(|e| e.to_string())?.or(d.leakage),
        keep_site: db.block_bool_property("keep_sizing_site").map_err(|e| e.to_string())?.unwrap_or(false),
    })
}

/// The checks of repair_design / buffer_ports read the LINK library through scene 0: a first
/// library read for another corner is refused there.
fn link_corner_is_first(libs: &Libs) -> Result<(), String> {
    if libs.link_scene.is_some_and(|k| k != 0) {
        return Err("the first liberty library read is not for the first corner: the link library's corner is not modelled".into());
    }
    Ok(())
}

/// `Resizer::computeDesignArea`: over the block's instances in order, each master's
/// `width × height` in m² (`dbuToMeters` each side) — 0 for a master that is not core
/// autoplaceable (`isCoreAutoPlaceable`) — fillers (`CORE SPACER`) left out.
fn design_area(db: &Db) -> f64 {
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

/// `Resizer::area(dbMaster)`: `dbuToMeters(width) × dbuToMeters(height)` (each side divided by
/// `dbu × 1e6` in `double`), 0 when the master is not core autoplaceable.
fn master_area(db: &Db, master: &str) -> f64 {
    let dbu = f64::from(db.tech_get_db_units_per_micron());
    let t = db.master_get_type(master).unwrap_or_default().replace(' ', "_");
    let placeable = (t.starts_with("CORE") || t.starts_with("BLOCK") || t.starts_with("ENDCAP")) && !matches!(t.as_str(), "ENDCAP_TOPLEFT" | "ENDCAP_TOPRIGHT" | "ENDCAP_BOTTOMLEFT" | "ENDCAP_BOTTOMRIGHT");
    if !placeable {
        return 0.0;
    }
    f64::from(db.master_get_width(master)) / (dbu * 1e6) * (f64::from(db.master_get_height(master)) / (dbu * 1e6))
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
    /// The port loads in force while this command runs: what an estimate made now is reduced against.
    port_caps: PortCaps,
    /// `set_propagated_clock` in force: a re-estimated clock net is a propagated one.
    propagated: bool,
    /// Per open journal level (odb's eco), the estimator's state as the level found it: undoing
    /// the level's edits restores the nets, and re-estimating a restored net gives back its old
    /// estimate, so the old state is put back whole.
    journal: Vec<JournalState>,
    /// The nets an SDC `set_load` names (`Sdc::hasNetWireCap`, keyed by the net object): such a
    /// net is constrained and is never merged away.
    sdc_nets: BTreeSet<String>,
    /// The edits as the incremental timer receives them.
    timer: TimerLog,
}

/// The incremental timer's side of the edits: the database's callbacks as read, and the
/// estimator's invalid set as ITS callbacks keep it (`parasitics_invalid_`) — apart from
/// [`CliDesign::invalid`], which a journal undo restores whole, where the reference's undo
/// invalidates every net it touches.
#[derive(Default)]
struct TimerLog {
    on: bool,
    events: Vec<String>,
    est_invalid: BTreeSet<String>,
    updated: Vec<String>,
}

/// What a journal level puts back on undo besides the database (the netlist is read from the
/// database again, as the timer's network reads it live).
struct JournalState {
    parasitics: Vec<HashMap<String, NetParasitics>>,
    invalid: BTreeSet<String>,
    sdc_nets: BTreeSet<String>,
}

impl CliDesign<'_> {
    /// After an edit: the timer's netlist and the pin order read again from the database.
    fn refresh(&mut self) -> Result<(), String> {
        self.netlist = vyges_grt::timer::netlist(self.db);
        self.info = net_info(self.db, &self.netlist)?;
        Ok(())
    }

    /// The database's callbacks since the last read, into the timer's log; the estimator's
    /// callbacks (`est::OdbCallBack`) on its invalid set: a net created, an instance terminal
    /// connected or disconnected, every net of a swapped instance → invalid; a net destroyed →
    /// erased.
    fn pull_edits(&mut self) {
        if !self.timer.on {
            return;
        }
        for ev in self.db.edit_log_take() {
            let f: Vec<&str> = ev.split('|').collect();
            match f.as_slice() {
                ["net_create", net] | ["iterm_connect" | "iterm_disconnect", _, net, _] if !net.is_empty() => {
                    self.timer.est_invalid.insert(net.to_string());
                }
                ["net_destroy", net, ..] => {
                    self.timer.est_invalid.remove(*net);
                }
                ["swap_after", _, terms] => {
                    for net in terms.split(';').filter_map(|t| t.split('=').nth(1)).filter(|n| !n.is_empty()) {
                        self.timer.est_invalid.insert(net.to_string());
                    }
                }
                _ => {}
            }
            self.timer.events.push(ev);
        }
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

    /// `insertBufferBeforeLoads(net, loads, …)`: on `net`, else the first load's.
    #[allow(clippy::too_many_arguments)]
    fn insert_before_loads(&mut self, net: Option<&str>, loads: &[String], cell: &str, loc: (i32, i32), reason: &str, loads_on_diff_nets: bool, uniquify: &str) -> Result<vyges_rsz::design::Repeater, String> {
        let original = match net {
            Some(n) => n.to_string(),
            None => self.net_of_load(loads.first().ok_or("insertBufferBeforeLoads: no loads specified")?),
        };
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
        // Estimated now and erased from the invalid set, with no delay invalidation.
        self.pull_edits();
        self.timer.est_invalid.remove(net);
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
        let e = vyges_est::placement::estimate_net(self.db, &timing, net, &drvr, self.alpha, &est_stt)?;
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
        self.insert_before_loads(None, loads, cell, loc, reason, false, "ALWAYS")
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

    fn start_timer_edits(&mut self) -> Result<(), String> {
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
        self.journal.push(JournalState { parasitics: self.parasitics.clone(), invalid: self.invalid.clone(), sdc_nets: self.sdc_nets.clone() });
        Ok(())
    }

    fn commit_journal(&mut self) -> Result<(), String> {
        self.db.eco_commit().map_err(|e| e.to_string())?;
        self.journal.pop();
        Ok(())
    }

    fn restore_journal(&mut self) -> Result<bool, String> {
        let had_changes = !self.db.eco_is_empty().map_err(|e| e.to_string())?;
        self.db.eco_undo().map_err(|e| e.to_string())?;
        let s = self.journal.pop().ok_or("undoEco without beginEco")?;
        self.parasitics = s.parasitics;
        self.invalid = s.invalid;
        self.sdc_nets = s.sdc_nets;
        self.refresh()?;
        Ok(had_changes)
    }

    /// `canRemoveBuffer(buffer, true)`'s database checks: the buffer neither dont_touch nor FIXED,
    /// neither net dont_touch; then the net to go (the input net when the output net has a port,
    /// else the output net): neither buffer pin a clock's source (`Sdc::isConstrained(pin)`: the
    /// only pin constraint an accepted SDC can put on an instance pin), the net to go not named by
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
        if pins.iter().any(|p| self.clock_sources.contains(p)) || self.sdc_nets.contains(&removed) {
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
        let name = self.db.make_new_inst_name("clone", "ALWAYS").map_err(e)?;
        self.db.create_inst(cell, &name).map_err(e)?;
        self.db.inst_set_source_type(&name, "TIMING").map_err(e)?;
        let at = self.clamp_loc_to_core(loc, cell);
        self.db.set_inst_location(&name, at.0, at.1).map_err(e)?;
        let master = self.db.inst_master(drvr_inst);
        let lc = self.libs.link_cell(&master).ok_or_else(|| format!("{master}: no liberty cell"))?.clone();
        let terms: Vec<String> = self.db.master_mterms(&master).map_err(e)?.into_iter().filter(|(_, t)| t != "POWER" && t != "GROUND").map(|(n, _)| n).collect();
        let mut touched = Vec::new();
        for term in &terms {
            if !lc.port(term).is_some_and(|p| p.direction == vyges_sta::liberty::Direction::Input) {
                continue;
            }
            let net = self.db.net_of(drvr_inst, term);
            if !net.is_empty() {
                self.db.connect(&name, term, &net).map_err(e)?;
                touched.push(net);
            }
        }
        let clone_terms: Vec<String> = self.db.master_mterms(cell).map_err(e)?.into_iter().filter(|(_, t)| t != "POWER" && t != "GROUND").map(|(n, _)| n).collect();
        let cc = self.libs.link_cell(cell).ok_or_else(|| format!("{cell}: no liberty cell"))?.clone();
        let out = clone_terms.iter().find(|t| cc.port(t).is_some_and(|p| p.direction == vyges_sta::liberty::Direction::Output)).ok_or("Cannot find output pin of the clone instance")?;
        let out_net = self.db.make_new_net_name("net", "ALWAYS").map_err(e)?;
        self.db.create_net(&out_net).map_err(e)?;
        self.db.connect(&name, out, &out_net).map_err(e)?;
        touched.push(out_net.clone());
        for load in moved_loads {
            let (inst, term) = load.rsplit_once('/').ok_or("a load pin")?;
            touched.push(self.db.net_of(inst, term));
            self.db.disconnect(inst, term).map_err(e)?;
            self.db.connect(inst, term, &out_net).map_err(e)?;
        }
        for n in touched {
            self.invalidate(&n);
        }
        self.refresh()?;
        Ok(name)
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
    fn is_driver_pin(&self, pin: &str) -> bool {
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
    fn term<'p>(&self, pin: &'p str) -> (Option<&'p str>, &'p str) {
        match pin.rsplit_once('/') {
            Some((inst, t)) if !self.netlist.ports.iter().any(|p| p.0 == pin) => (Some(inst), t),
            _ => (None, pin),
        }
    }

    /// `insertBufferPostProcess` (`setLocation`: clamped to the core, placed), and the odb
    /// callbacks: the nets on the new buffer's input and output marked invalid.
    fn insert_buffer_post_process(&mut self, inst: &str, cell: &str) -> Result<(), String> {
        let at = self.clamp_loc_to_core(self.db.inst_location(inst), cell);
        self.db.set_inst_location(inst, at.0, at.1).map_err(|e| e.to_string())?;
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

/// The constraints the setup search reads (one clock and its port delays), from the SDC as the
/// reference wrote it. Everything else that moves a setup slack is refused here, never dropped.
///
/// ⚠️ The parsed SDC keeps an I/O delay's value and ports but not its flags, so the delays are read
/// from the text: `-max` / `-rise` / `-fall` place the value; a `-min`-only delay bounds hold paths
/// and is not a setup input; any other flag is refused.
fn search_sdc(s: &vyges_loom::sdc::Sdc, text: &str, time_scale: f32, propagated: bool) -> Result<vyges_sta::sdc::Sdc, String> {
    use vyges_sta::sdc::{Clock, PortDelay};
    let real: Vec<&vyges_loom::sdc::SdcClock> = s.clocks.iter().filter(|c| !c.is_virtual()).collect();
    let [clock] = real.as_slice() else {
        return Err(format!("repair_timing with {} clocks: one is modelled", real.len()));
    };
    let [source] = clock.sources.as_slice() else {
        return Err(format!("clock {} on {} sources: one is modelled", clock.name, clock.sources.len()));
    };
    if real.len() != s.clocks.len() {
        return Err("a virtual clock: not modelled".into());
    }
    if s.setup_uncertainty != 0.0 || s.hold_uncertainty != 0.0 {
        return Err("set_clock_uncertainty: not modelled".into());
    }
    if s.clock_latency != 0.0 || s.late_derate.is_some() || s.early_derate.is_some() || !s.exceptions.is_empty() || !s.async_groups.is_empty() {
        return Err("clock latency, derates, timing exceptions or clock groups: not modelled".into());
    }
    if text.contains("-waveform") {
        return Err("create_clock -waveform: not modelled".into());
    }
    // Per side (input, output): each port and its delay by `[rf][min/max]`, as set so far.
    type Delays = Vec<(String, [[Option<f32>; 2]; 2])>;
    let mut delays: [Delays; 2] = [Vec::new(), Vec::new()];
    for line in text.lines() {
        let line = line.trim();
        let side = if line.starts_with("set_input_delay ") { 0 } else if line.starts_with("set_output_delay ") { 1 } else { continue };
        let words: Vec<&str> = line.split_whitespace().collect();
        let value: f64 = words.get(1).and_then(|w| w.parse().ok()).ok_or_else(|| format!("{line}: the delay value is not modelled"))?;
        let flag = |f: &str| words.contains(&f);
        for f in ["-clock_fall", "-network_latency_included", "-source_latency_included", "-reference_pin"] {
            if flag(f) {
                return Err(format!("{} {f}: not modelled", words[0]));
            }
        }
        let clock_name = line.split("[get_clocks {").nth(1).and_then(|r| r.split('}').next()).ok_or_else(|| format!("{line}: a delay without -clock is not modelled"))?;
        if clock_name != clock.name {
            return Err(format!("{line}: a delay on another clock is not modelled"));
        }
        let ports = line.split("[get_ports {").nth(1).and_then(|r| r.split('}').next()).ok_or_else(|| format!("{line}: ports not read"))?;
        let mms: Vec<usize> = match (flag("-min"), flag("-max")) {
            (true, false) => vec![0],
            (false, true) => vec![1],
            _ => vec![0, 1],
        };
        let rfs: Vec<usize> = match (flag("-rise"), flag("-fall")) {
            (true, false) => vec![0],
            (false, true) => vec![1],
            _ => vec![0, 1],
        };
        let v = vyges_sta::sdc::user_to_sta(value, time_scale);
        for port in ports.split_whitespace() {
            let list = &mut delays[side];
            let k = match list.iter().position(|(p, _)| p == port) {
                Some(k) => k,
                None => {
                    list.push((port.to_string(), [[None; 2]; 2]));
                    list.len() - 1
                }
            };
            for &rf in &rfs {
                for &mm in &mms {
                    list[k].1[rf][mm] = Some(v);
                }
            }
        }
    }
    // An input delay: both maxima set (a min value missing takes the max's), every value present.
    // ⚠️ One set for min only is left out, as before hold was modelled — it would move only min
    // arrivals; no capture has one.
    let inputs = |list: &Delays| -> Result<Vec<PortDelay>, String> {
        list.iter()
            .filter(|(_, d)| d[0][1].is_some() || d[1][1].is_some())
            .map(|(p, d)| {
                let max = |rf: usize| d[rf][1].ok_or_else(|| format!("port {p}: an input delay without its max value for both transitions is not modelled"));
                let (r, f) = (max(0)?, max(1)?);
                Ok(PortDelay { port: p.clone(), delay: [[d[0][0].unwrap_or(r), r], [d[1][0].unwrap_or(f), f]], exists: [[true; 2]; 2] })
            })
            .collect()
    };
    // An output delay: each `[rf][min/max]` value as set (`RiseFallMinMax`) — a missing one makes no
    // path end. A max set for one transition only is refused (unwitnessed).
    let outputs = |list: &Delays| -> Result<Vec<PortDelay>, String> {
        list.iter()
            .map(|(p, d)| {
                if d[0][1].is_some() != d[1][1].is_some() {
                    return Err(format!("port {p}: an output delay set for one transition only is not modelled"));
                }
                let exists = [[d[0][0].is_some(), d[0][1].is_some()], [d[1][0].is_some(), d[1][1].is_some()]];
                let v = |rf: usize, mm: usize| d[rf][mm].unwrap_or(0.0);
                Ok(PortDelay { port: p.clone(), delay: [[v(0, 0), v(0, 1)], [v(1, 0), v(1, 1)]], exists })
            })
            .collect()
    };
    Ok(vyges_sta::sdc::Sdc {
        clock: Clock::new(&clock.name, vyges_sta::sdc::user_to_sta(clock.period, time_scale), source, propagated),
        input_delays: inputs(&delays[0])?,
        output_delays: outputs(&delays[1])?,
    })
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
    // The constraints as the estimate saw them (an estimate step's "sdc"), when a port load was
    // set after it: the estimate's pi models were reduced without it.
    let mut estimate_sdc: Option<String> = None;
    // The RC in force when the estimate ran (later re-estimates read the RC in force then).
    let mut estimate_rc: Option<vyges_est::rc::Rc> = None;
    let units = |l: &vyges_est::liberty::LibertyClocks| -> Result<vyges_est::rc::Units, String> {
        let u = l.units.ok_or("RC before a liberty library: the timer's default units are not modelled")?;
        Ok(vyges_est::rc::Units { resistance: u.resistance, capacitance: u.capacitance, distance: u.distance })
    };
    let mut trace = Trace::default();
    let mut outcome: Option<Result<repair_design::Outcome, Stop>> = None;
    // The estimator's per-net state after an edit made inside its incremental guard (buffer_ports):
    // a later repair starts from it rather than from a fresh estimate.
    let mut carried: Option<Vec<HashMap<String, NetParasitics>>> = None;
    let mut buffered: Vec<buffer_ports::Outcome> = Vec::new();
    // `set_propagated_clock` on the clock, and as the estimate saw it.
    let mut propagated = false;
    let mut estimate_propagated = false;
    // The SDC file last read (its text: the I/O delays' flags).
    let mut sdc_path: Option<String> = None;
    let mut sdc_propagated = false;
    // Each repair_timing's lines; a refusal after them stops the job.
    let mut timing_runs: Vec<Value> = Vec::new();
    // `set_debug_level`: each (tool, group) and its level.
    let mut debug_levels: BTreeMap<(String, String), i64> = BTreeMap::new();
    let mut timing_stop: Option<String> = None;
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
                    if libs.libs.is_empty() {
                        libs.link_scene = scene;
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
                // The buffer list is found once and kept (findBuffers returns early when it has one).
                if !buffered.is_empty() {
                    return Err("set_dont_use after buffer_ports: the kept buffer list is not modelled".into());
                }
                for a in &args {
                    for pat in a.split_whitespace() {
                        // A SWIG object handle (`_<hex>_p_LibertyCell`) is a cell object logged
                        // without its name: as a pattern it would silently match nothing.
                        if pat.starts_with('_') && pat.contains("_p_") {
                            return Err(format!("set_dont_use {pat}: an object handle, not a cell name (not modelled)"));
                        }
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
                carried = None;
                estimate_alpha = routing_alpha;
                estimate_db = step["db"].as_str().map(String::from);
                estimate_sdc = step["sdc"].as_str().map(String::from);
                estimate_rc = Some(rc.clone());
                estimate_propagated = propagated;
            }
            // One clock is modelled, so the clock it names is that one.
            "set_propagated_clock" => propagated = true,
            // `set_debug_level <tool> <group> <level>`: debug lines are not output this engine
            // scores, except a group that adds report lines (the move tracker's, RSZ-0211 on).
            "set_debug_level" => {
                let [tool, group, level] = args.as_slice() else { return Err(format!("set_debug_level {}: needs a tool, a group and a level", args.join(" "))) };
                let level: i64 = level.parse().map_err(|_| format!("set_debug_level {level}: not an integer"))?;
                debug_levels.insert((tool.clone(), group.clone()), level);
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
                    .filter(|c| (*c != "set_max_transition" || s.max_transition_on_objects) && (*c != "set_max_fanout" || s.max_fanout_on_objects) && *c != "set_driving_cell" && *c != "set_propagated_clock")
                    .collect();
                if !unmodelled.is_empty() {
                    return Err(format!("{path}: {} not modelled", unmodelled.join(", ")));
                }
                // `set_propagated_clock` in the file: read by repair_timing's search; the other
                // commands refuse it when they run.
                sdc_propagated = s.ignored.iter().any(|c| c == "set_propagated_clock");
                sdc = Some(s);
                sdc_path = Some(path.clone());
            }
            "buffer_ports" => {
                if sdc_propagated {
                    return Err("set_propagated_clock: not modelled for buffer_ports".into());
                }
                link_corner_is_first(&libs)?;
                let o = buffer_ports::Options::parse(&args)?;
                let mut dont_use = set_dont_use.clone();
                for lib in &libs.libs {
                    dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
                }
                let m = masters(&db)?;
                if !db.block_get_mod_insts().is_empty() {
                    return Err("buffer_ports on a hierarchical design: not modelled".into());
                }
                let netlist = vyges_grt::timer::netlist(&db);
                // `isConstant` holds nowhere: case analysis is refused at read_sdc, constant cells here.
                if let Some(why) = vyges_rsz::timing::constant_cells(&libs.libs, &netlist) {
                    return Err(format!("{why} (not modelled)"));
                }
                // selectBufferCell: the user's cell (a buffer), else findBuffers' weakest.
                let weakest = match &o.buffer_cell {
                    Some(c) => {
                        if !libs.link_cell(c).is_some_and(|c| c.is_buffer()) {
                            return Err(format!("buffer_ports -buffer_cell {c}: not a buffer"));
                        }
                        c.clone()
                    }
                    None => vyges_rsz::preamble::find_buffers(&libs, &m, &dont_use, true).map_err(|e| e.message().to_string())?.lowest,
                };
                let clock_sources: Vec<String> = sdc.as_ref().map(|s| s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect()).unwrap_or_default();
                rc.sort_clk_and_signal_layers();
                // Inside the guard every edit marks its nets invalid — once parasitics exist.
                let parasitics = match (carried.take(), estimated) {
                    (Some(p), _) => p,
                    (None, true) => {
                        let mut est_rc = estimate_rc.clone().expect("estimated");
                        est_rc.sort_clk_and_signal_layers();
                        let caps = port_caps_at(estimate_sdc.as_deref(), sdc.as_ref(), &libs, &netlist)?;
                        estimate_state(&db, estimate_db.as_deref(), &netlist, &est_rc, &liberty, &clock_sources, estimate_propagated, estimate_alpha, libs.scene_count(), &caps)?
                    }
                    (None, false) => vec![HashMap::new(); libs.scene_count()],
                };
                let info = net_info(&db, &netlist)?;
                let estimating = estimated && !no_signal_cap(&db, &rc);
                let core = (db.block_get_core_area_x_min(), db.block_get_core_area_y_min(), db.block_get_core_area_x_max(), db.block_get_core_area_y_max());
                let core = (core != (0, 0, 0, 0)).then_some(core);
                let port_caps = port_caps_at(step["sdc"].as_str(), sdc.as_ref(), &libs, &netlist)?;
                let mut design = CliDesign { db: &mut db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, alpha: routing_alpha, estimating, netlist, parasitics, invalid: BTreeSet::new(), info, core, port_caps, propagated: false, journal: Vec::new(), sdc_nets: BTreeSet::new(), timer: TimerLog::default() };
                let r = buffer_ports::buffer_ports(&mut design, &o, &weakest);
                let parasitics = std::mem::take(&mut design.parasitics);
                match r {
                    Ok(b) => buffered.push(b),
                    Err(Stop::Refused { msg, .. }) => return Err(format!("{msg} (not modelled)")),
                    Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                }
                if estimated {
                    carried = Some(parasitics);
                }
                // The design as buffer_ports left it.
                if let Some(path) = step["write_def"].as_str() {
                    db.write_def(path).map_err(|e| format!("{path}: {e}"))?;
                }
            }
            "repair_design" => {
                if outcome.is_some() {
                    return Err("a second repair_design: not modelled".into());
                }
                link_corner_is_first(&libs)?;
                if sdc_propagated {
                    return Err("set_propagated_clock: not modelled for repair_design".into());
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
                let clock_sources: Vec<String> = sdc.as_ref().map(|s| s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect()).unwrap_or_default();
                let netlist = vyges_grt::timer::netlist(&db);
                if !estimated {
                    return Err("repair_design without estimate_parasitics -placement: not modelled".into());
                }
                // The estimate runs at the alpha in force when estimate_parasitics did.
                rc.sort_clk_and_signal_layers();
                let mut est_rc = estimate_rc.clone().expect("estimated");
                est_rc.sort_clk_and_signal_layers();
                // The estimator's state as an earlier edit (buffer_ports) left it, else the estimate.
                let parasitics = match carried.take() {
                    Some(p) => p,
                    None => {
                        let caps = port_caps_at(estimate_sdc.as_deref(), sdc.as_ref(), &libs, &netlist)?;
                        estimate_state(&db, estimate_db.as_deref(), &netlist, &est_rc, &liberty, &clock_sources, estimate_propagated, estimate_alpha, libs.scene_count(), &caps)?
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
                let port_caps = env.port_pin_cap.clone();
                let limits_of_block = sizing_limits(&db)?;
                let mut design = CliDesign { db: &mut db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, alpha: routing_alpha, estimating, netlist, parasitics, invalid: BTreeSet::new(), info, core, port_caps, propagated: false, journal: Vec::new(), sdc_nets: BTreeSet::new(), timer: TimerLog::default() };
                let inputs = Inputs { libs: &libs, masters: &m, dont_use: &dont_use, limits, clock_sources: &clock_sources, dbu, wire_rc, sdc: env, master_pins: mpins, sizing_limits: limits_of_block };
                outcome = Some(repair_design::repair_design(&inputs, &mut design, &a, &mut trace));
            }
            "repair_timing" => {
                use vyges_rsz::repair_timing as rt;
                let a = rt::Args::parse(&args)?;
                if a.recover_power {
                    return Err("repair_timing -recover_power: not modelled".into());
                }
                // A phase list of LEGACY alone is the default pipeline without LAST_GASP.
                let legacy_only = a.phases.as_deref().is_some_and(|p| p.split_whitespace().eq(["LEGACY"]));
                if a.phases.is_some() && !legacy_only {
                    return Err("repair_timing -phases: not modelled".into());
                }
                if !estimated {
                    return Err("repair_timing without estimate_parasitics -placement: not modelled".into());
                }
                if libs.scene_count() != 1 && !a.setup {
                    return Err("repair_timing -hold over several corners: not modelled".into());
                }
                let m = masters(&db)?;
                if let Some((n, _)) = m.iter().find(|(_, mm)| !mm.implant_obs.is_empty()) {
                    return Err(format!("master {n} has IMPLANT obstructions: VT categories are not modelled"));
                }
                let lib0 = libs.default_library().ok_or("repair_timing before any liberty library")?;
                let time_scale = lib0.time_scale;
                let s = sdc.as_ref().ok_or("repair_timing without constraints: not modelled")?;
                let text = read_text(sdc_path.as_deref().ok_or("repair_timing: no SDC file")?)?;
                let netlist = vyges_grt::timer::netlist(&db);
                if let Some(why) = vyges_rsz::timing::constant_cells(&libs.libs, &netlist) {
                    return Err(format!("{why} (not modelled)"));
                }
                let clock_sources: Vec<String> = s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect();
                let parasitics = match carried.take() {
                    Some(p) => p,
                    None => {
                        let mut est_rc = estimate_rc.clone().expect("estimated");
                        est_rc.sort_clk_and_signal_layers();
                        let caps = port_caps_at(estimate_sdc.as_deref(), sdc.as_ref(), &libs, &netlist)?;
                        estimate_state(&db, estimate_db.as_deref(), &netlist, &est_rc, &liberty, &clock_sources, estimate_propagated, estimate_alpha, libs.scene_count(), &caps)?
                    }
                };
                let env = sdc_env(s, &libs, &netlist)?;
                let mpins = master_pins(&db)?;
                let mut g = repair_design::timer_graph(&libs, 0, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
                let clock_propagated = propagated || sdc_propagated;
                let clocks = vyges_rsz::timing::clock_pins(&g, &clock_sources);
                if text.contains("set_clock_transition") || text.contains("set_clock_latency") {
                    return Err("set_clock_transition / set_clock_latency: not modelled".into());
                }
                // An ideal clock's network (`ClkNetwork::isIdealClock`): its registers read the
                // ideal clock slew, 0.
                if !clock_propagated {
                    g.ideal_clock = clocks.iter().copied().collect();
                }
                g.find_delays(&parasitics[0], None)?;
                let ssdc = search_sdc(s, &text, time_scale, clock_propagated)?;
                let mut search = vyges_sta::search::Search::in_graph_order(&g, &ssdc);
                search.find_arrivals()?;
                search.find_requireds()?;
                // A diagnostic: every vertex's max-path arrivals, requireds and slews (seconds),
                // to set against the reference's `report_checks -fields {slew}` along a path.
                if let Ok(path) = std::env::var("VYGES_RSZ_TIMING_DUMP") {
                    let mut out = String::new();
                    for (v, vx) in g.vertices.iter().enumerate() {
                        for p in search.paths[v].iter().filter(|p| p.tag.mm == 1) {
                            out.push_str(&format!("{} rf={} clk={} arr={:e} req={:e} slew={:e}\n", vx.name, p.tag.rf, p.tag.is_clock, p.arrival, p.required, g.slew[v][p.tag.rf][1]));
                        }
                    }
                    std::fs::write(&path, out).map_err(|e| format!("{path}: {e}"))?;
                }
                let (ends, starts) = rt::timing_points(&g, &search, &ssdc, &libs, &clocks)?;
                // Several corners: every scene timed on its own libraries and parasitics, each
                // point's slack the least over the scenes (`Sta::slack` over every path).
                let (ends, starts) = if libs.scene_count() > 1 {
                    let mut per_scene = Vec::new();
                    for (k, par) in parasitics.iter().enumerate().take(libs.scene_count()) {
                        let mut gk = repair_design::timer_graph(&libs, k, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
                        let ck = vyges_rsz::timing::clock_pins(&gk, &clock_sources);
                        if !clock_propagated {
                            gk.ideal_clock = ck.iter().copied().collect();
                        }
                        gk.find_delays(par, None)?;
                        let mut sk = vyges_sta::search::Search::in_graph_order(&gk, &ssdc);
                        sk.find_arrivals()?;
                        sk.find_requireds()?;
                        per_scene.push(rt::timing_points(&gk, &sk, &ssdc, &libs, &ck)?);
                    }
                    (rt::least_over_scenes(per_scene.iter().map(|(e, _)| e.as_slice()).collect())?, rt::least_over_scenes(per_scene.iter().map(|(_, s)| s.as_slice()).collect())?)
                } else {
                    (ends, starts)
                };
                // `-hold` alone: `Resizer::repairHold` (its preamble with clock buffers allowed),
                // over the endpoints' min and max slacks.
                let hold_only = !a.setup;
                let margin = vyges_sta::sdc::user_to_sta(a.setup_margin, time_scale);
                let violating = rt::collect_violating(&ends, margin);
                let violating_starts = rt::collect_violating(&starts, margin);
                let seq = rt::move_sequence(&a, false);
                let mut lines = rt::preamble(&seq, violating.len(), a.repair_tns_end_percent, a.phases.as_deref());
                // The moves modelled: SizeUp and Unbuffer, in the LEGACY phase alone.
                let unmodelled = if let Some(m) = seq.iter().find(|m| !matches!(m, rt::Move::SizeUp | rt::Move::Unbuffer | rt::Move::SwapPins | rt::Move::Buffer | rt::Move::Clone | rt::Move::SplitLoad | rt::Move::SizeUpMatch | rt::Move::VtSwap)) {
                    Some(format!("repair_timing: {} is not modelled", m.name()))
                } else if a.match_cell_footprint {
                    Some("repair_timing -match_cell_footprint: not modelled".into())
                } else if libs.scene_count() > 1 && (seq.iter().any(|m| *m != rt::Move::SizeUp) || !(a.skip_last_gasp || a.phases.is_some())) {
                    // Several corners: SizeUpMove in the LEGACY phase is modelled (the path's
                    // scene for its delays and cells, every scene for slacks and max-cap checks).
                    Some("repair_timing over several corners: moves other than SizeUpMove, and LAST_GASP, are not modelled".into())
                } else if debug_levels.get(&("RSZ".to_string(), "move_tracker".to_string())).is_some_and(|&l| l > 0) {
                    Some("repair_timing: the move tracker's reports (set_debug_level RSZ move_tracker) are not modelled".into())
                } else {
                    None
                };
                if !hold_only && violating.is_empty() {
                    timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": 0 }));
                } else if let Some(why) = unmodelled.filter(|_| !hold_only) {
                    lines.extend(rt::row0(&ends, violating.len(), &violating_starts, time_scale));
                    timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": violating.len() }));
                    timing_stop = Some(why);
                    break;
                } else {
                    // Resizer::resizePreamble: the equivalent cells, the buffers, the target slews.
                    let mut dont_use = set_dont_use.clone();
                    for lib in &libs.libs {
                        dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
                    }
                    let equiv = vyges_rsz::sizing::make_equiv_cells(&libs);
                    let site_heights: BTreeMap<String, i32> = m.values().map(|mm| mm.site.clone()).filter(|s| !s.is_empty()).map(|s| (s.clone(), db.site_get_height(&s))).collect();
                    let db_dbu = db.tech_get_db_units_per_micron();
                    let buffers = vyges_rsz::preamble::find_buffers(&libs, &m, &dont_use, !hold_only).map_err(|e| format!("{}: {}", e.code(), e.message()))?;
                    let (tgt_slews, tgt_scene, target_loads) = vyges_rsz::preamble::find_target_loads(&libs, &buffers.cells, &dont_use);
                    let sizing = vyges_rsz::sizing::Sizing { libs: &libs, masters: &m, dont_use: &dont_use, equiv: &equiv, target_loads: &target_loads, tgt_slews, tgt_scene, limits: sizing_limits(&db)? };
                    let limits = Limits {
                        design_max_transition: s.max_transition.map(|v| user_time_to_sta(v, time_scale)),
                        design_max_fanout: s.max_fanout.map(|v| v as f32),
                    };
                    let wire_rc = {
                        rc.sort_clk_and_signal_layers();
                        let v = rc.resolved(&db.tech_get_name(), 0);
                        vyges_rsz::buffered_net::WireRc { h_res: v[0], v_res: v[1], h_cap: v[2], v_cap: v[3] }
                    };
                    let slew_shape_factor = vyges_rsz::preamble::compute_slew_shape_factor(lib0).map_err(|e| format!("{}: {}", e.code(), e.message()))?;
                    // The reference's pin addresses, when the gate supplies them (debug order only).
                    let pin_addr = match job["pin_address"].as_str() {
                        Some(path) => Some(vyges_rsz::unbuffer::PinAddr::parse(&read_text(path)?)?),
                        None => None,
                    };
                    // Rebuffer::init / initOnCorner, when the sequence has BufferMove.
                    let rb_sizes_store;
                    let rb_ctx_store;
                    let rb_ctx = if !hold_only && seq.contains(&rt::Move::Buffer) {
                        let base = vyges_rsz::rebuffer::Ctx { libs: &libs, sizes: &[], rc: wire_rc, dbu: db.tech_get_db_units_per_micron(), slew_shape_factor, tgt_slews, time_scale, cap_scale: lib0.cap_scale };
                        let lowest = libs.link_cell(&buffers.lowest).ok_or("no lowest-drive buffer")?;
                        let r_max = lowest.buffer_ports().map_or(0.0, |(_, o)| lowest.drive_resistance(&o.name));
                        let slew_limit = |c: &vyges_sta::liberty::Cell, port: &str| {
                            let lib = libs.link_library(&c.name).unwrap_or(lib0);
                            let p = c.port(port);
                            vyges_rsz::timing::find_slew_limit(lib, p.map_or(vyges_sta::liberty::Direction::Output, |p| p.direction), p.and_then(|p| p.max_transition), &limits)
                        };
                        let max_input_slew = |c: &vyges_sta::liberty::Cell, port: &str| {
                            let lib = libs.link_library(&c.name).unwrap_or(lib0);
                            let p = c.port(port);
                            vyges_rsz::timing::max_input_slew_at(lib, lib, p.map_or(vyges_sta::liberty::Direction::Input, |p| p.direction), p.and_then(|p| p.max_transition), &limits)
                        };
                        // maxLoad: the first output port with a capacitance limit.
                        let max_load = |c: &vyges_sta::liberty::Cell| c.ports.iter().filter(|p| p.direction == vyges_sta::liberty::Direction::Output).find_map(|p| p.max_capacitance).unwrap_or(0.0);
                        let ci = vyges_rsz::rebuffer::CharInputs { ctx: &base, r_max, slew_limit: &slew_limit, max_input_slew: &max_input_slew, max_load: &max_load };
                        rb_sizes_store = match vyges_rsz::rebuffer::characterize(&ci, &buffers.cells) {
                            Ok(s) => s,
                            Err(e) => {
                                lines.extend(rt::row0(&ends, violating.len(), &violating_starts, time_scale));
                                timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": violating.len() }));
                                timing_stop = Some(format!("{e} (not modelled)"));
                                break;
                            }
                        };
                        rb_ctx_store = vyges_rsz::rebuffer::Ctx { sizes: &rb_sizes_store, ..base };
                        Some(&rb_ctx_store)
                    } else {
                        None
                    };
                    let ctx = vyges_rsz::repair_setup::Ctx {
                        libs: &libs,
                        sizing: &sizing,
                        env: &env,
                        master_pins: &mpins,
                        ssdc: &ssdc,
                        clock_sources: &clock_sources,
                        ideal_clock: !clock_propagated,
                        margin,
                        time_scale,
                        sequence: &seq,
                        limits: &limits,
                        dbu: db.tech_get_db_units_per_micron(),
                        wire_rc,
                        slew_shape_factor,
                        debug: &debug_levels,
                        rebuffer: rb_ctx,
                        lowest_buffer: &buffers.lowest,
                        pin_addr: pin_addr.as_ref(),
                    };
                    drop(search);
                    drop(g);
                    let info = net_info(&db, &netlist)?;
                    let estimating = !no_signal_cap(&db, &rc);
                    rc.sort_clk_and_signal_layers();
                    let core = (db.block_get_core_area_x_min(), db.block_get_core_area_y_min(), db.block_get_core_area_x_max(), db.block_get_core_area_y_max());
                    let core = (core != (0, 0, 0, 0)).then_some(core);
                    let port_caps = env.port_pin_cap.clone();
                    let sdc_nets: BTreeSet<String> = s.env.iter().filter(|e| e.cmd == "set_load" && e.accessor == "get_nets").flat_map(|e| e.objects.iter().cloned()).collect();
                    let mut design = CliDesign { db: &mut db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, alpha: routing_alpha, estimating, netlist, parasitics, invalid: BTreeSet::new(), info, core, port_caps, propagated: clock_propagated, journal: Vec::new(), sdc_nets, timer: TimerLog::default() };
                    if hold_only {
                        let mut trace_head = Vec::new();
                        if debug_levels.get(&("RSZ".to_string(), "resizer".to_string())).is_some_and(|&l| l >= 1) {
                            // `findFastBuffers`' list (`pre-selected buffers`).
                            let base = vyges_rsz::rebuffer::Ctx { libs: &libs, sizes: &[], rc: wire_rc, dbu: db_dbu, slew_shape_factor, tgt_slews, time_scale, cap_scale: lib0.cap_scale };
                            let lowest = libs.link_cell(&buffers.lowest).ok_or("no lowest-drive buffer")?;
                            let r_max = lowest.buffer_ports().map_or(0.0, |(_, o)| lowest.drive_resistance(&o.name));
                            let slew_limit = |c: &vyges_sta::liberty::Cell, port: &str| {
                                let lib = libs.link_library(&c.name).unwrap_or(lib0);
                                let p = c.port(port);
                                vyges_rsz::timing::find_slew_limit(lib, p.map_or(vyges_sta::liberty::Direction::Output, |p| p.direction), p.and_then(|p| p.max_transition), &limits)
                            };
                            let max_input_slew = |c: &vyges_sta::liberty::Cell, port: &str| {
                                let lib = libs.link_library(&c.name).unwrap_or(lib0);
                                let p = c.port(port);
                                vyges_rsz::timing::max_input_slew_at(lib, lib, p.map_or(vyges_sta::liberty::Direction::Input, |p| p.direction), p.and_then(|p| p.max_transition), &limits)
                            };
                            let max_load = |c: &vyges_sta::liberty::Cell| c.ports.iter().filter(|p| p.direction == vyges_sta::liberty::Direction::Output).find_map(|p| p.max_capacitance).unwrap_or(0.0);
                            let ci = vyges_rsz::rebuffer::CharInputs { ctx: &base, r_max, slew_limit: &slew_limit, max_input_slew: &max_input_slew, max_load: &max_load };
                            trace_head.push("[DEBUG RSZ-resizer] pre-selected buffers:".to_string());
                            for b in vyges_rsz::rebuffer::find_fast_buffers(&ci, &buffers.cells) {
                                trace_head.push(format!("[DEBUG RSZ-resizer]  - {b}"));
                            }
                        }
                        let ha = vyges_rsz::repair_hold::HoldArgs {
                            setup_margin: a.setup_margin * f64::from(time_scale),
                            hold_margin: a.hold_margin * f64::from(time_scale),
                            allow_setup_violations: a.allow_setup_violations,
                            max_buffer_percent: (a.max_buffer_percent / 100.0) as f32,
                            max_passes: a.max_passes,
                            max_iterations: a.max_iterations,
                            verbose: a.verbose,
                            // `parse_max_util`: 0..100 percent, as a fraction.
                            max_utilization: match a.max_utilization.as_deref() {
                                None => None,
                                Some(v) => {
                                    let u: f64 = v.parse().map_err(|_| "repair_timing -max_utilization: not a number".to_string())?;
                                    if !(0.0..=100.0).contains(&u) {
                                        return Err("RSZ-0004: -max_utilization must be between 0 and 100%.".into());
                                    }
                                    Some(u / 100.0)
                                }
                            },
                        };
                        let hctx = vyges_rsz::repair_hold::HoldCtx { libs: &libs, masters: &m, site_heights: &site_heights, dont_use: &dont_use, tgt_slews, time_scale, debug: &debug_levels };
                        let r = vyges_rsz::repair_setup::repair_hold(&ctx, &hctx, &mut design, &ha);
                        carried = Some(std::mem::take(&mut design.parasitics));
                        let o = match r {
                            Ok(o) => o,
                            Err(Stop::Refused { msg, .. }) => {
                                timing_stop = Some(format!("{msg} (not modelled)"));
                                break;
                            }
                            Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                        };
                        if let Some(path) = job["timing_trace"].as_str() {
                            let all: Vec<String> = trace_head.into_iter().chain(o.trace.iter().cloned()).collect();
                            std::fs::write(path, all.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                        }
                        timing_runs.push(json!({ "hold": true, "lines": o.lines, "endpoints": ends.len(), "violating_endpoints": o.violating, "inserted": o.inserted, "resized": o.resized, "stopped": o.stopped, "error": o.error }));
                        if let Some(why) = o.stopped {
                            timing_stop = Some(why);
                            break;
                        }
                        // The reference's error ends the command (it throws): no later step runs.
                        if o.error.is_some() {
                            break;
                        }
                        continue;
                    }
                    let r = vyges_rsz::repair_setup::repair_setup(&ctx, &mut design, &a);
                    carried = Some(std::mem::take(&mut design.parasitics));
                    let o = match r {
                        Ok(o) => o,
                        Err(Stop::Refused { msg, .. }) => {
                            timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": violating.len() }));
                            timing_stop = Some(format!("{msg} (not modelled)"));
                            break;
                        }
                        Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                    };
                    if let Some(path) = job["timing_trace"].as_str() {
                        std::fs::write(path, o.trace.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                    }
                    lines.extend(o.lines);
                    timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": violating.len(), "resized": o.resized, "removed": o.removed, "inserted": o.inserted }));
                }
                if a.hold {
                    timing_stop = Some("repair_timing -hold: not modelled".into());
                    break;
                }
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
    // A diagnostic: the database itself (odb ids preserved), to set against the reference's.
    if let Some(path) = job["write_db"].as_str() {
        db.write(path).map_err(|e| format!("{path}: {e}"))?;
    }
    let tags: Vec<&str> = trace.tags.iter().copied().collect();
    let mut report = match outcome {
        // repair_timing alone: its preamble, then the repair it made, the moves it would make
        // (refused), or a CHECKED nothing-to-do with the endpoints it checked.
        None if buffered.is_empty() && !timing_runs.is_empty() => match &timing_stop {
            Some(reason) => json!({ "tool": "vyges-rsz", "status": "refused", "reason": reason }),
            None => {
                let checked: u64 = timing_runs.iter().filter_map(|r| r["endpoints"].as_u64()).sum();
                let sum = |k: &str| -> u64 { timing_runs.iter().filter_map(|r| r[k].as_u64()).sum() };
                let resized = sum("resized");
                let changed = resized + sum("removed") + sum("inserted");
                let violating = sum("violating_endpoints");
                let error = timing_runs.iter().find_map(|r| r["error"].as_str().map(String::from));
                // A repair that ran: what it changed. Nothing to repair: a CHECKED nothing-to-do.
                // The reference's error (RSZ-0050, RSZ-0060) ends the command: an error, with it.
                let status = if error.is_some() { "error" } else if changed > 0 { "repaired" } else if violating > 0 { "unrepaired" } else if checked > 0 { "up_to_date" } else { "vacuous" };
                let mut r = json!({ "tool": "vyges-rsz", "status": status, "endpoints_checked": checked, "resized": resized, "inserted": sum("inserted") });
                if let Some(e) = error {
                    r["reason"] = json!(e);
                }
                r
            }
        },
        None if buffered.is_empty() => json!({ "tool": "vyges-rsz", "status": "vacuous", "reason": "no repair_design or buffer_ports step" }),
        // buffer_ports alone: the ports it walked are what it checked.
        None => {
            let inserted: usize = buffered.iter().map(|b| b.inserted_inputs + b.inserted_outputs).sum();
            let checked: usize = buffered.iter().map(|b| b.ports_checked).sum();
            let status = if inserted > 0 { "repaired" } else if checked > 0 { "up_to_date" } else { "vacuous" };
            json!({ "tool": "vyges-rsz", "status": status, "ports_checked": checked, "inserted_buffers": inserted })
        }
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
                "warnings": o.warnings.iter().map(|(code, text)| json!({ "code": code, "message": text })).collect::<Vec<_>>(),
                "max_wire_lengths": o.max_wire_lengths.iter().map(|(k, b, m)| json!({ "scene": k, "buffer": b, "meters": m })).collect::<Vec<_>>(),
            })
        }
        Some(Err(Stop::Refused { code, msg })) => json!({ "tool": "vyges-rsz", "status": "refused", "code": code, "reason": msg }),
        Some(Err(Stop::Error { code, msg })) => json!({ "tool": "vyges-rsz", "status": "error", "code": code, "reason": msg }),
    };
    report["traced"] = json!(tags);
    if !timing_runs.is_empty() {
        report["repair_timing"] = json!(timing_runs);
    }
    if !buffered.is_empty() {
        report["buffer_ports"] = buffered
            .iter()
            .map(|b| {
                let lines: Vec<Value> = b.lines.iter().map(|l| json!({ "code": l.code, "severity": if l.warning { "warning" } else { "info" }, "message": l.text })).collect();
                json!({ "inserted_inputs": b.inserted_inputs, "inserted_outputs": b.inserted_outputs, "ports_checked": b.ports_checked, "lines": lines })
            })
            .collect();
    }
    Ok(report)
}

const USAGE: &str = "\
vyges loom rsz — electrical repair of a placed design: repeaters along each net's Steiner tree,
drivers resized, where a wire is too long or a capacitance, fanout or transition limit is broken

USAGE:
  vyges loom rsz repair_design <job.json> [-o FILE]
  vyges loom rsz buffer_ports <job.json> [-o FILE]     (the same job runner; a job may hold either)
  vyges loom rsz --describe
  vyges loom rsz --help
  vyges loom rsz --version

JOB FIELDS:
  steps        required — the commands in order, each {\"cmd\": ..., \"args\": [...]}, as a flow
               script passes them:
                 read_lef, read_def, read_db, define_corners, read_liberty [-corner C],
                 read_sdc, set_dont_use, set_layer_rc, set_wire_rc, set_routing_alpha,
                 estimate_parasitics -placement, set_propagated_clock, buffer_ports [options],
                 repair_design [options], repair_timing [options]
               an estimate_parasitics step may carry \"db\": the database as the estimate saw it,
               when cells were moved between it and the repair
               a buffer_ports step may carry \"write_def\": the design as it left it, as DEF
               an estimate_parasitics or buffer_ports step may carry \"sdc\": the constraints in
               force when it ran, when a port's set_load comes after it
  trace        write one line per decision, in the order the repair makes them, to this path
  write_def    write the design as the repair left it, as DEF, to this path
  dcalc_trace  (diagnostic) write the timer's delay-calculation trace of the design as read

REPAIR_DESIGN OPTIONS:
  -max_wire_length L    the longest wire, microns (0: none)
  -slew_margin P        percent taken off every slew limit
  -cap_margin P         percent taken off every capacitance limit
  -verbose              accepted
  refused: -pre_placement / -buffer_gain, -match_cell_footprint, -reroute, -max_utilization

BUFFER_PORTS OPTIONS:
  -inputs / -outputs    which side (neither: both) — a buffer after each input port, before
                        each output port, unless its net is dont-touch, special or pinless, an
                        input is a clock source, an input's loads are all buffers or one is
                        dont-touch, or an output's driver is tristate or dont-touch
  -buffer_cell C        the buffer to use (default: the weakest buffer the repair would pick)
  -verbose              each port's decision in the report's lines
  refused: -max_utilization, a hierarchical design

REPAIR_TIMING:
  -setup: every move of the default sequence (UnbufferMove, SizeUpMove, SwapPinsMove,
  BufferMove, CloneMove, SplitLoadMove; SizeUpMatchMove) in the LEGACY phase and LAST_GASP —
  the move sequence (RSZ-0100), RSZ-0094 / RSZ-0099 (and RSZ-0221) or RSZ-0098, every progress
  row, the summary (RSZ-0051, RSZ-0062) in the report's repair_timing[].lines, and the design it
  leaves (write_def). A job's timing_trace file gets the pass-by-pass decisions.
  -hold (alone): the hold buffer, RSZ-0046 or RSZ-0033, every progress row, RSZ-0064 / RSZ-0066,
  RSZ-0132, RSZ-0032 and the buffers it inserts; -max_utilization or -max_buffer_percent reached
  ends it with RSZ-0050 / RSZ-0060 (status error).
  status repaired (the design changed), unrepaired (violations, nothing kept), up_to_date or
  error. One clock, ideal or propagated, with its I/O delays. -setup and -hold together (or
  neither): the setup repair, then refused. Refused before the lines: -phases other than
  LEGACY, -recover_power, several corners, VT libraries, a latch, a virtual clock, clock
  uncertainty / latency / transition, derates, path exceptions; refused during the repair:
  -setup with -max_utilization, more than one repair per pass.

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
  buffer_ports (per step: inserted_inputs, inserted_outputs, ports_checked, lines — each with its
  code and severity), status, nets_checked, nets_repaired, inserted_buffers, resized, drivers_skipped, violations
  {slew, capacitance, fanout, length}, summary — the repair's closing lines, each with its code —
  warnings (RSZ-0065: -max_wire_length shorter than the length at which a buffer pays for itself),
  and max_wire_lengths (that length per buffer and scene, meters, as the check computed it)

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
  "summary": "electrical repair of a placed design: repeaters inserted along each net's Steiner tree for long wires, max capacitance and max slew; and port buffering",
  "maturity": "experimental",
  "provenance_limitations": [
    "input_hash covers the argument vector, not the content of the job file or of the design files it names.",
    "status is one of repaired, up_to_date, vacuous, refused or error. repaired means the design changed (buffers inserted or drivers resized); up_to_date means drivers were checked and none needed a change (nets_checked says how many; for a job with buffer_ports and no repair_design, ports_checked); vacuous means nothing was checked and is NOT a pass. The declared assertion passes on repaired or up_to_date. Exit status is 0 for repaired and up_to_date, 2 for vacuous and for error, 3 for refused.",
    "Modelled: placement parasitics, one or more corners, flat and hierarchical netlists, the default buffer selection, the SDC constraints the usage lists, buffer_ports before the repair (the estimate it leaves carried into it). Refused rather than guessed: global-route parasitics, the early sizing round, footprint matching, rerouting, any other netlist edit between the estimate and the repair, buffer_ports on a hierarchical design, a tristate driver or a bidirect pin on a net, and any other timing-affecting SDC command.",
    "repair_timing -setup is modelled for every move of the default sequence in the LEGACY phase and LAST_GASP, and repair_timing -hold alone in full (ending with RSZ-0050 / RSZ-0060 as the command does): every progress row, the summary and the design left, for one ideal or propagated clock; -setup with -hold runs the setup part and is refused after it; other -phases, several corners, VT libraries, latches, virtual clocks, clock uncertainty, latency or transition, derates and exceptions are refused before the lines."
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
        Some("repair_design") | Some("buffer_ports") if positional.len() == 2 => {}
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
