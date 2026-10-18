// SPDX-License-Identifier: Apache-2.0
//! The timing environment the command-line engine builds from a design: the placement parasitics
//! the estimator makes, the constraints as the timer and the search read them, the constant pins,
//! and the masters. Shared with the engines that time a design this crate repairs (clock tree
//! synthesis reads clock arrivals through it).

use std::collections::{BTreeMap, HashMap};

use vyges_opendb::Db;
use vyges_sta::graph::NetParasitics;

use crate::preamble::{Libs, Master};

thread_local! {
    static LUT: vyges_stt::flute::lut::Lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

/// `SteinerTreeBuilder::makeSteinerTree(x, y, drvr, alpha)`, as the estimator calls it.
pub fn est_stt(x: &[i32], y: &[i32], drvr: usize, alpha: f32) -> vyges_est::placement::SttTree {
    LUT.with(|lut| {
        let t = vyges_stt::make_steiner_tree(lut, x, y, drvr, alpha).0.expect("a Steiner tree");
        vyges_est::placement::SttTree { deg: t.deg, branch: t.branch.iter().map(|b| vyges_est::placement::Branch { x: b.x, y: b.y, n: b.n }).collect() }
    })
}

/// `EstimateParasitics::makeSteinerTree(drvr_location, sink_locations)`: the sinks sorted by
/// (x, y), the driver appended last, FLUTE at `alpha`; the driver point the first branch point at
/// the driver's location. None without a sink.
pub fn steiner_over_points(root: (i32, i32), sinks: &[(i32, i32)], alpha: f32) -> Option<crate::buffered_net::Tree> {
    if sinks.is_empty() {
        return None;
    }
    let mut pts = sinks.to_vec();
    pts.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
    let drvr_idx = pts.len();
    pts.push(root);
    let x: Vec<i32> = pts.iter().map(|p| p.0).collect();
    let y: Vec<i32> = pts.iter().map(|p| p.1).collect();
    let t = est_stt(&x, &y, drvr_idx, alpha);
    let branch: Vec<(i32, i32, usize)> = t.branch.iter().map(|b| (b.x, b.y, b.n)).collect();
    let drvr_pt = branch.iter().position(|b| (b.0, b.1) == root);
    Some(crate::buffered_net::Tree { deg: t.deg, branch, pinlocs: Vec::new(), drvr_pt })
}

/// Whether the signal wire capacitance is zero or unset (EST-0018): the estimator then makes no
/// parasitics at all, and every net is timed lumped — as the router's timer does.
pub fn no_signal_cap(db: &Db, rc: &vyges_est::rc::Rc) -> bool {
    let tech = db.tech_get_name();
    let zero = (0..rc.scenes.len()).any(|k| {
        let v = rc.resolved(&tech, k);
        (v[2] + v[3]) / 2.0 == 0.0
    });
    zero || rc.resolve(&tech, |w| &w.signal_cap).is_empty()
}

/// One net's estimate in one scene (its corner's RC) as the timer reads it: its network, pin
/// nodes named by pin; `None` where the estimator makes none.
pub fn net_parasitic(db: &Db, rc: &vyges_est::rc::Rc, n: &vyges_est::placement::NetEstimate, scene: usize) -> Result<Option<NetParasitics>, String> {
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
pub type PortCaps = HashMap<String, [[Option<f32>; 2]; 2]>;

/// The port loads in force at a step: from the SDC the step carries (`"sdc"`: the constraints
/// as the reference held them when it ran), else from the job's SDC.
pub fn port_caps_at(path: Option<&str>, current: Option<&vyges_loom::sdc::Sdc>, libs: &Libs, nl: &vyges_sta::netlist::Netlist) -> Result<PortCaps, String> {
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
pub fn placement_parasitics_at(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
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
pub fn estimate_state(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
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

/// `parse_min_max_all_flags`: `-min` alone is min, `-max` alone max, neither or both all — as
/// `[min, max]` membership.
pub fn min_max_all(flags: &[String]) -> [bool; 2] {
    match (flags.iter().any(|f| f == "-min"), flags.iter().any(|f| f == "-max")) {
        (true, false) => [true, false],
        (false, true) => [false, true],
        _ => [true, true],
    }
}

/// `parse_rise_fall_flags`: `-rise` alone rise, `-fall` alone fall, neither or both both.
pub fn rise_fall(flags: &[String]) -> [bool; 2] {
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
/// `Sim::isConstant` on the design as read (est's `constant_pins`: a signal pin on a power or
/// ground net, and what the cells' functions propagate from it), by pin name with its value — what
/// delay calculation and the search skip, and the search's conditional arcs and arc senses read.
pub fn sim_constants(db: &Db, liberty: &vyges_est::liberty::LibertyClocks) -> Result<HashMap<String, bool>, String> {
    use vyges_est::sim::Pin;
    let values = vyges_est::sim::constant_pins(db, liberty).map_err(|e| format!("constant propagation: {e}"))?;
    Ok(values
        .into_iter()
        .map(|(p, v)| match p {
            Pin::Port(n) => (n, v),
            Pin::ITerm(i, t) => (format!("{i}/{t}"), v),
        })
        .collect())
}

/// The ports `[all_outputs]` / `[all_inputs …]` name. The reference's `allOutputs` keeps every
/// top-level port whose direction is any output (output, bidirect, tristate); `allInputs(no_clks)`
/// every any-input port (input, bidirect), less the clock-source pins under `-no_clocks`. Bus
/// members are already single ports here; netlist order is kept. Each port then takes the setting
/// exactly as `[get_ports …]` would. Err names an accessor flag not modelled.
pub fn port_collection(accessor: &str, flags: &[String], ports: &[(String, vyges_sta::netlist::PortDir)], clock_sources: &std::collections::HashSet<&str>) -> Result<Vec<String>, String> {
    use vyges_sta::netlist::PortDir;
    let outputs = accessor == "all_outputs";
    if let Some(f) = flags.iter().find(|f| f.as_str() != "-no_clocks" || outputs) {
        return Err(f.clone());
    }
    let no_clocks = flags.iter().any(|f| f == "-no_clocks");
    Ok(ports
        .iter()
        .filter(|(name, dir)| {
            let any = if outputs { matches!(dir, PortDir::Output | PortDir::Inout) } else { matches!(dir, PortDir::Input | PortDir::Inout) };
            any && !(no_clocks && clock_sources.contains(name.as_str()))
        })
        .map(|(name, _)| name.clone())
        .collect())
}

pub fn sdc_env(s: &vyges_loom::sdc::Sdc, libs: &Libs, nl: &vyges_sta::netlist::Netlist) -> Result<vyges_sta::graph::SdcEnv, String> {
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
        let Some(cell) = &d.lib_cell else {
            return Err("set_driving_cell without -lib_cell: not modelled".into());
        };
        // Without `-pin` the reference takes the cell's ONE output port (output, tristate or
        // bidirect); a cell with several is its error 456, "-pin argument required".
        let inferred;
        let pin = match &d.pin {
            Some(p) => p,
            None => {
                use vyges_sta::liberty::Direction;
                let outs: Vec<&str> = libs
                    .link_cell(cell)
                    .map(|c| c.ports.iter().filter(|p| matches!(p.direction, Direction::Output | Direction::Tristate | Direction::Bidirect)).map(|p| p.name.as_str()).collect())
                    .unwrap_or_default();
                match outs.as_slice() {
                    [one] => {
                        inferred = one.to_string();
                        &inferred
                    }
                    [] => return Err(format!("set_driving_cell -lib_cell {cell}: no output port (not modelled)")),
                    _ => return Err(format!("set_driving_cell -lib_cell {cell}: -pin argument required for cells with multiple outputs")),
                }
            }
        };
        let lib0 = libs.default_library().ok_or("set_driving_cell before any liberty library")?;
        if libs.link_cell(cell).and_then(|c| c.port(pin)).is_none() {
            return Err(format!("set_driving_cell -lib_cell {cell} -pin {pin}: no such cell port (not modelled)"));
        }
        let slews = [crate::timing::user_time_to_sta(d.input_transition[0], lib0.time_scale), crate::timing::user_time_to_sta(d.input_transition[1], lib0.time_scale)];
        // ⛔ `[all_inputs]` arrives as the collection `*INPUTS*`, not a port: inserted as a name
        // it drove nothing. It is every input and bidirect port — the clocks included, as the
        // reference applies it.
        use vyges_sta::netlist::PortDir;
        let ports: Vec<String> = d
            .objects
            .iter()
            .flat_map(|o| -> Vec<String> {
                if o == "*INPUTS*" {
                    nl.ports.iter().filter(|(_, dir)| matches!(dir, PortDir::Input | PortDir::Inout)).map(|(n, _)| n.clone()).collect()
                } else {
                    vec![o.clone()]
                }
            })
            .collect();
        for port in ports {
            env.input_drive.insert(port, vyges_sta::graph::InputDrive { cell: cell.clone(), from_port: d.from_pin.clone(), to_port: pin.clone(), from_slews: slews });
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
        let expanded;
        let (accessor, objects) = match e.accessor.as_str() {
            "all_outputs" | "all_inputs" => {
                let clock_sources: std::collections::HashSet<&str> = s.clocks.iter().flat_map(|c| c.sources.iter().map(String::as_str)).collect();
                expanded = port_collection(&e.accessor, &e.accessor_flags, &nl.ports, &clock_sources).map_err(|f| format!("{} [{} {f}]: not modelled", e.cmd, e.accessor))?;
                ("get_ports", &expanded)
            }
            other => (other, &e.objects),
        };
        match (e.cmd.as_str(), accessor) {
            ("set_load", "get_nets") => {
                let cap = (e.value * f64::from(lib0.cap_scale)) as f32;
                let subtract = e.flags.iter().any(|f| f == "-subtract_pin_load");
                for net in objects {
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
                for port in objects {
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
                let slew = crate::timing::user_time_to_sta(e.value, lib0.time_scale);
                for port in objects {
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

/// Every LEF master the database holds: its site and the IMPLANT layers among its obstructions.
pub fn masters(db: &Db) -> Result<BTreeMap<String, Master>, String> {
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
        let (width, height) = (db.master_get_width(&name), db.master_get_height(&name));
        let area = i64::from(width) * i64::from(height);
        out.insert(name.clone(), Master { site: db.master_get_site(&name), area, width, height, is_core: db.master_is_core(&name), logic_std: db.master_get_type(&name).map_err(|e| e.to_string())? == "CORE", implant_obs });
    }
    Ok(out)
}

/// Every master's signal terminals (not POWER / GROUND), the pins the timer's graph has.
pub fn master_pins(db: &Db) -> Result<HashMap<String, Vec<String>>, String> {
    let mut out = HashMap::new();
    for i in 0..db.num_masters().map_err(|e| e.to_string())? {
        let name = db.nth_master_name(i).map_err(|e| e.to_string())?;
        let terms = db.master_mterms(&name).map_err(|e| e.to_string())?;
        out.insert(name, terms.into_iter().filter(|(_, t)| t != "POWER" && t != "GROUND").map(|(n, _)| n).collect());
    }
    Ok(out)
}

/// The constraints the setup search reads (one clock and its port delays), from the SDC as the
/// reference wrote it. Everything else that moves a setup slack is refused here, never dropped.
///
/// An I/O delay's flags place its value: `-max` / `-rise` / `-fall`; a `-min`-only delay bounds
/// hold paths and is not a setup input; `-add_delay` is read; any other flag is refused.
/// `[all_inputs]` / `[all_outputs]` in `set_input_delay` / `set_output_delay` as the ports they
/// name (`Sdc::allInputs`: every input or bidirect port; `allOutputs`: every output or bidirect —
/// the clock ports included, as without `-no_clocks` — less the delay's own clock's source ports,
/// which the reference skips), in the netlist's port order. Other delays
/// are kept as written. [`search_sdc`] reads only named ports; a caller with the netlist expands
/// first.
pub fn expand_io_delay_ports(s: &mut vyges_loom::sdc::Sdc, nl: &vyges_sta::netlist::Netlist) {
    use vyges_sta::netlist::PortDir;
    // the source ports of each clock, by name
    let sources: std::collections::HashMap<String, Vec<String>> = s.clocks.iter().map(|c| (c.name.clone(), c.sources.clone())).collect();
    for (list, want) in [(&mut s.input_delays, PortDir::Input), (&mut s.output_delays, PortDir::Output)] {
        let accessor = if want == PortDir::Input { "all_inputs" } else { "all_outputs" };
        for d in list.iter_mut().filter(|d| d.accessor == accessor) {
            // ⛔ A delay relative to a clock is not applied to that clock's own source port — the
            // reference skips it (STA-0441, "set_input_delay relative to a clock defined on the
            // same port/pin not allowed"); expanded with it, the clock port got a data arrival.
            let own: &[String] = d.clock.as_ref().and_then(|c| sources.get(c)).map(Vec::as_slice).unwrap_or(&[]);
            d.ports = nl
                .ports
                .iter()
                .filter(|(n, dir)| (*dir == want || *dir == PortDir::Inout) && !own.contains(n))
                .map(|(n, _)| n.clone())
                .collect();
            d.accessor = "get_ports".into();
            d.default = false;
        }
    }
}

pub fn search_sdc(s: &vyges_loom::sdc::Sdc, time_scale: f32, propagated: bool, hold_repair: bool) -> Result<vyges_sta::sdc::Sdc, String> {
    use vyges_sta::sdc::{Clock, PortDelay};
    // The clocks in the order they were made (`Clock::index`): each on one source port, or virtual
    // (no source — its edges launch input delays and close output delays). None: every path
    // unclocked. A clock is propagated when `set_propagated_clock` names it (the reference writes it
    // per clock; a virtual clock never is), or — named by none — when the run's command set it.
    let mut named_propagated: Vec<String> = Vec::new();
    for o in &s.propagated {
        if o.accessor != "get_clocks" {
            return Err(format!("set_propagated_clock on [{} {}]: not modelled (clocks are)", o.accessor, o.names.join(" ")));
        }
        named_propagated.extend(o.names.iter().cloned());
    }
    let mut clocks = Vec::new();
    for clock in &s.clocks {
        let source: &str = if clock.is_virtual() {
            ""
        } else {
            let [source] = clock.sources.as_slice() else {
                return Err(format!("clock {} on {} sources: not modelled (one is)", clock.name, clock.sources.len()));
            };
            source
        };
        let prop = !clock.is_virtual() && if named_propagated.is_empty() { propagated } else { named_propagated.contains(&clock.name) };
        let mut c = Clock::new(&clock.name, vyges_sta::sdc::user_to_sta(clock.period, time_scale), source, prop);
        // `-waveform {rise fall}`: each edge `time_ui_sta`, like the period. Two edges, the rise
        // first and within a period of the start, the fall after it and less than a period later.
        if let Some(w) = &clock.waveform {
            match w.as_slice() {
                &[r, f] if r >= 0.0 && r < clock.period && f > r && f - r < clock.period => {
                    c.waveform = [vyges_sta::sdc::user_to_sta(r, time_scale), vyges_sta::sdc::user_to_sta(f, time_scale)];
                }
                _ => return Err(format!("create_clock -waveform {{{}}} on period {}: not modelled (a rise then a fall within one period is)", w.iter().map(f64::to_string).collect::<Vec<_>>().join(" "), clock.period)),
            }
        }
        clocks.push(c);
    }
    let clock_index = |name: &str| s.clocks.iter().position(|c| c.name == name);
    // A hold uncertainty moves only hold checks: read by a hold repair alone.
    if s.setup_uncertainty != 0.0 || (s.hold_uncertainty != 0.0 && hold_repair) {
        return Err("set_clock_uncertainty: not modelled".into());
    }
    if s.late_derate.is_some() || s.early_derate.is_some() || !s.exceptions.is_empty() || !s.async_groups.is_empty() {
        return Err("derates, timing exceptions or clock groups: not modelled".into());
    }
    // `set_clock_latency` on pins (`Sdc::setClockLatency(clk | null, pin)`): an ideal clock's network
    // latency from that pin on. On clocks, `-source`, `-early` / `-late`, or on a propagated clock
    // (where the pin would turn it ideal) — refused.
    for l in &s.clock_latencies {
        if l.accessor != "get_pins" {
            return Err(format!("set_clock_latency on [{} …]: not modelled (pins are)", l.accessor));
        }
        if let Some(f) = l.flags.iter().find(|f| !matches!(f.as_str(), "-rise" | "-fall" | "-min" | "-max")) {
            return Err(format!("set_clock_latency {f}: not modelled"));
        }
        let rfs: Vec<usize> = match (l.flags.iter().any(|f| f == "-rise"), l.flags.iter().any(|f| f == "-fall")) {
            (true, false) => vec![0],
            (false, true) => vec![1],
            _ => vec![0, 1],
        };
        let mms: Vec<usize> = match (l.flags.iter().any(|f| f == "-min"), l.flags.iter().any(|f| f == "-max")) {
            (true, false) => vec![0],
            (false, true) => vec![1],
            _ => vec![0, 1],
        };
        let targets: Vec<usize> = match &l.clock {
            Some(name) => vec![s.clocks.iter().position(|c| &c.name == name).ok_or_else(|| format!("set_clock_latency -clock {name}: no such clock"))?],
            None => (0..clocks.len()).collect(),
        };
        let v = vyges_sta::sdc::user_to_sta(l.value, time_scale);
        for k in targets {
            let c: &mut Clock = &mut clocks[k];
            if c.propagated {
                return Err("set_clock_latency on a pin of a propagated clock: not modelled".into());
            }
            for pin in &l.objects {
                let specific = l.clock.is_some();
                let i = match c.pin_latencies.iter().position(|x| &x.pin == pin && x.specific == specific) {
                    Some(i) => i,
                    None => {
                        c.pin_latencies.push(vyges_sta::sdc::PinLatency { pin: pin.clone(), specific, delay: [[None; 2]; 2] });
                        c.pin_latencies.len() - 1
                    }
                };
                for &rf in &rfs {
                    for &mm in &mms {
                        c.pin_latencies[i].delay[rf][mm] = Some(v);
                    }
                }
            }
        }
    }
    // Per side (input, output): each delay by `[rf][min/max]` as set so far, keyed by port and
    // clock (`Sdc::findInputDelay` / `findOutputDelay`), in the order made. Without `-add_delay`,
    // `setInputDelay` deletes the port's delays on other clocks and overwrites the values it
    // names; with it, merges them — a max kept if larger, a min if smaller (`mergeValue`).
    type Delays = Vec<(String, usize, [[Option<f32>; 2]; 2])>;
    let mut delays: [Delays; 2] = [Vec::new(), Vec::new()];
    for (side, list) in [(0, &s.input_delays), (1, &s.output_delays)] {
        let cmd = ["set_input_delay", "set_output_delay"][side];
        for d in list {
            if !d.value_read {
                return Err(format!("{cmd} on {}: the delay value is not modelled", d.ports.join(" ")));
            }
            if let Some(f) = d.flags.iter().find(|f| !matches!(f.as_str(), "-max" | "-min" | "-rise" | "-fall" | "-add_delay")) {
                return Err(format!("{cmd} {f}: not modelled"));
            }
            if d.reference_pin.is_some() {
                return Err(format!("{cmd} -reference_pin: not modelled"));
            }
            let flag = |f: &str| d.flags.iter().any(|x| x == f);
            let Some(clock_name) = d.clock.as_deref() else {
                return Err(format!("{cmd} on {}: a delay without -clock is not modelled", d.ports.join(" ")));
            };
            let Some(ci) = clock_index(clock_name) else {
                return Err(format!("{cmd} -clock {clock_name}: a delay on an unknown clock is not modelled"));
            };
            if d.default || d.accessor != "get_ports" {
                return Err(format!("{cmd} -clock {clock_name} on [{} …]: ports not read (get_ports are)", d.accessor));
            }
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
            let add = flag("-add_delay");
            let v = vyges_sta::sdc::user_to_sta(d.value, time_scale);
            for port in &d.ports {
                let list = &mut delays[side];
                if !add {
                    list.retain(|(p, c, _)| p != port || *c == ci);
                }
                let k = match list.iter().position(|(p, c, _)| p == port && *c == ci) {
                    Some(k) => k,
                    None => {
                        list.push((port.clone(), ci, [[None; 2]; 2]));
                        list.len() - 1
                    }
                };
                for &rf in &rfs {
                    for &mm in &mms {
                        let slot = &mut list[k].2[rf][mm];
                        // `MinMax::compare`: max keeps the larger, min the smaller.
                        if !add || slot.is_none_or(|old| if mm == 1 { v > old } else { v < old }) {
                            *slot = Some(v);
                        }
                    }
                }
            }
        }
    }
    // An input delay: both maxima set (a min value missing takes the max's), every value present.
    // ⚠️ One set for min only is left out, as before hold was modelled — it would move only min
    // arrivals; no capture has one.
    let inputs = |list: &Delays| -> Result<Vec<PortDelay>, String> {
        list.iter()
            .filter(|(_, _, d)| d[0][1].is_some() || d[1][1].is_some())
            .map(|(p, clock, d)| {
                let max = |rf: usize| d[rf][1].ok_or_else(|| format!("port {p}: an input delay without its max value for both transitions is not modelled"));
                let (r, f) = (max(0)?, max(1)?);
                Ok(PortDelay { port: p.clone(), delay: [[d[0][0].unwrap_or(r), r], [d[1][0].unwrap_or(f), f]], exists: [[true; 2]; 2], clock: *clock })
            })
            .collect()
    };
    // An output delay: each `[rf][min/max]` value as set (`RiseFallMinMax`) — a missing one makes no
    // path end. A max set for one transition only is refused (unwitnessed).
    let outputs = |list: &Delays| -> Result<Vec<PortDelay>, String> {
        list.iter()
            .map(|(p, clock, d)| {
                if d[0][1].is_some() != d[1][1].is_some() {
                    return Err(format!("port {p}: an output delay set for one transition only is not modelled"));
                }
                let exists = [[d[0][0].is_some(), d[0][1].is_some()], [d[1][0].is_some(), d[1][1].is_some()]];
                let v = |rf: usize, mm: usize| d[rf][mm].unwrap_or(0.0);
                Ok(PortDelay { port: p.clone(), delay: [[v(0, 0), v(0, 1)], [v(1, 0), v(1, 1)]], exists, clock: *clock })
            })
            .collect()
    };
    Ok(vyges_sta::sdc::Sdc {
        clocks,
        input_delays: inputs(&delays[0])?,
        output_delays: outputs(&delays[1])?,
        path_delays: path_delays(s, time_scale)?,
    })
}

/// Global placement's virtual clock tree: each `(clock, pin, latency)` as
/// `setClockLatency(clk, pin, riseFall, all)` — a latency given with the clock, every transition
/// and min/max.
pub fn add_virtual_latencies(sdc: &mut vyges_sta::sdc::Sdc, latencies: &[(String, String, f32)]) -> Result<(), String> {
    for (clock, pin, d) in latencies {
        let c = sdc.clocks.iter_mut().find(|c| &c.name == clock).ok_or_else(|| format!("virtual clock latency on clock {clock}: no such clock"))?;
        if c.propagated {
            return Err("a virtual clock latency on a propagated clock: not modelled".into());
        }
        c.pin_latencies.retain(|l| !(l.specific && &l.pin == pin));
        c.pin_latencies.push(vyges_sta::sdc::PinLatency { pin: pin.clone(), specific: true, delay: [[Some(*d); 2]; 2] });
    }
    Ok(())
}

/// Every `set_max_delay` / `set_min_delay` as the reference's `write_sdc` writes it:
/// `[-ignore_clock_latency] [-probe] [-from <objs>] [-to <objs>] <delay>`, objects
/// `[get_pins {..}]` or `[get_clocks {..}]`. Any other flag is refused; so is more than one (an
/// exception's id is the order it was MADE in, which the file may not keep). Which forms the
/// timer models is the search's to say (`Search::path_delays_modelled`).
pub fn path_delays(s: &vyges_loom::sdc::Sdc, time_scale: f32) -> Result<Vec<vyges_sta::sdc::PathDelay>, String> {
    use vyges_sta::liberty::{MAX, MIN};
    let mut out = Vec::new();
    for d in &s.path_delays {
        let cmd = if d.max { "set_max_delay" } else { "set_min_delay" };
        if let Some(f) = d.flags.iter().find(|f| !matches!(f.as_str(), "-ignore_clock_latency" | "-probe")) {
            return Err(format!("{cmd} {f}: not modelled"));
        }
        if let Some((f, _)) = d.objects.iter().find(|(f, _)| f != "-from" && f != "-to") {
            return Err(format!("{cmd} {f}: not modelled"));
        }
        if d.value.is_nan() {
            return Err(format!("{cmd}: the delay value is not modelled"));
        }
        // The objects of one flag: (pins, names the clock).
        let objs = |flag: &str| -> Result<(Vec<String>, bool), String> {
            let Some((_, o)) = d.objects.iter().find(|(f, _)| f == flag) else { return Ok((Vec::new(), false)) };
            if o.accessor == "get_pins" {
                Ok((o.names.clone(), false))
            } else if o.accessor == "get_clocks" && o.names.len() == 1 && s.clocks.iter().any(|c| c.name == o.names[0]) {
                Ok((Vec::new(), true))
            } else {
                Err(format!("{cmd} {flag} [{} {}]: not modelled (pins or the clock are)", o.accessor, o.names.join(" ")))
            }
        };
        let (from_pins, from_clock) = objs("-from")?;
        let (to_pins, to_clock) = objs("-to")?;
        let flag = |f: &str| d.flags.iter().any(|x| x == f);
        out.push(vyges_sta::sdc::PathDelay {
            from_pins,
            from_clock,
            to_pins,
            to_clock,
            min_max: if d.max { MAX } else { MIN },
            ignore_clk_latency: flag("-ignore_clock_latency"),
            break_path: !flag("-probe"),
            delay: vyges_sta::sdc::user_to_sta(d.value, time_scale),
        });
    }
    if out.len() > 1 {
        return Err(format!("{} path delays: not modelled (one is; ids are the order they were made in)", out.len()));
    }
    Ok(out)
}

/// The design as an SDC object query sees it — the database's ports, instances and instance pins
/// (direction from the Liberty port, net from the database), for `Sdc::load_with`.
pub fn sdc_design(db: &vyges_opendb::Db, libs: &Libs) -> vyges_loom::sdc::SdcDesign {
    use vyges_sta::liberty::Direction;
    use vyges_sta::netlist::{Conn, PortDir};
    let nl = vyges_grt::timer::netlist(db);
    let dir_name = |d: &Direction| match d {
        Direction::Input => "input",
        Direction::Output => "output",
        Direction::Tristate => "tristate",
        Direction::Bidirect => "bidirect",
        Direction::Internal => "internal",
        Direction::Unknown => "unknown",
    };
    let ports = nl
        .ports
        .iter()
        .map(|(n, d)| (n.clone(), match d { PortDir::Input => "input", PortDir::Output => "output", PortDir::Inout => "bidirect" }.to_string()))
        .collect();
    let mut pins = Vec::new();
    for net in &nl.nets {
        for c in &net.pins {
            if let Conn::Inst(i, p) = c {
                let dir = libs.link_cell(&nl.insts[*i].1).and_then(|cell| cell.port(p)).map_or("unknown", |port| dir_name(&port.direction));
                pins.push((*i, p.clone(), dir.to_string(), Some(net.name.clone())));
            }
        }
    }
    vyges_loom::sdc::SdcDesign { ports, insts: nl.insts.clone(), pins }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rules (`Sdc::allOutputs` / `Sdc::allInputs`): any-output = output or bidirect; any-input =
    /// input or bidirect; `-no_clocks` drops clock-source ports only from `all_inputs`, and is not
    /// a flag of `all_outputs`. Netlist order is kept.
    #[test]
    fn port_collections_expand_as_the_reference_does() {
        use vyges_sta::netlist::PortDir::*;
        let ports = [("clk".to_string(), Input), ("a".into(), Input), ("io".into(), Inout), ("y".into(), Output)];
        let clk: std::collections::HashSet<&str> = ["clk"].into();
        let none: &[String] = &[];
        assert_eq!(port_collection("all_outputs", none, &ports, &clk).unwrap(), ["io", "y"]);
        assert_eq!(port_collection("all_inputs", none, &ports, &clk).unwrap(), ["clk", "a", "io"]);
        assert_eq!(port_collection("all_inputs", &["-no_clocks".into()], &ports, &clk).unwrap(), ["a", "io"]);
        assert_eq!(port_collection("all_outputs", &["-no_clocks".into()], &ports, &clk), Err("-no_clocks".into()));
        assert_eq!(port_collection("all_inputs", &["-level_sensitive".into()], &ports, &clk), Err("-level_sensitive".into()));
    }
}

