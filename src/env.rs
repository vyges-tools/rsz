// SPDX-License-Identifier: Apache-2.0
//! The timing environment the command-line engine builds from a design: the placement parasitics
//! the estimator makes, the constraints as the timer and the search read them, the constant pins,
//! and the masters. Shared with the engines that time a design this crate repairs (clock tree
//! synthesis reads clock arrivals through it).

use std::collections::{BTreeMap, HashMap};

use vyges_opendb::Db;
use vyges_sta::graph::NetParasitics;

use crate::preamble::{Libs, Master};

// The loom SDC as the timer reads it — moved to the timer (`vyges_sta::load`) so every engine
// that times a design from loom maps it one way; kept here under the names the engines call.
pub use vyges_sta::load::{expanded_io_delays, min_max_all, path_delays, port_collection, rise_fall, search_sdc, search_sdc_on};

/// No SDC constraint value: the estimator's skips are its own simulation's.
static NO_CONSTRAINTS: std::sync::LazyLock<vyges_est::sim::Constraints> = std::sync::LazyLock::new(vyges_est::sim::Constraints::default);

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

/// The entries of `caps` for the ports among one net's parasitic `nodes`: what that net's
/// [`NetParasitics::port_pin_caps`] keeps. The delay calculation asks it only for a port node of
/// the same net (`Graph::reduced`), and a port it does not hold reads 0, as in the whole map; it
/// stays `Some` even when empty (reduced against the loads in force THEN, not now).
/// ⛔ Not the whole map per net: on a large design (1.5 M nets, 3 scenes, ~2,000 ports) that clone
/// grew past 150 GB before repair_design began.
pub fn net_port_caps(caps: &PortCaps, nodes: &[String]) -> PortCaps {
    if caps.is_empty() {
        return PortCaps::new();
    }
    nodes.iter().filter_map(|n| caps.get_key_value(n)).map(|(k, v)| (k.clone(), *v)).collect()
}

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

/// Which clocks' nets the estimate skips as ideal: every clock alike (`All`, the run's
/// `set_propagated_clock` flag — the form the consumers call), or each clock on its own
/// (`PerClock`, est 8e440c9).
#[derive(Clone, Copy)]
enum Propagation<'c> {
    All(bool),
    PerClock(&'c vyges_est::placement::Clocks),
}

#[allow(clippy::too_many_arguments)]
pub fn placement_parasitics_at(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    parasitics_held(db, rc, liberty, clock_sources, Propagation::All(propagated), alpha, scenes, port_caps, None, &NO_CONSTRAINTS)
}

/// [`placement_parasitics_at`] with propagation read per clock (`clocks`, est 8e440c9).
#[allow(clippy::too_many_arguments)]
pub fn placement_parasitics_per_clock(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], clocks: &vyges_est::placement::Clocks, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    parasitics_held(db, rc, liberty, clock_sources, Propagation::PerClock(clocks), alpha, scenes, port_caps, None, &NO_CONSTRAINTS)
}

/// [`placement_parasitics_at`] with the clock nets and constant pins of `db` as it is now, when the
/// caller holds them (`held`); `None` walks the design for them.
#[allow(clippy::too_many_arguments)]
pub fn placement_parasitics_held(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    parasitics_held(db, rc, liberty, clock_sources, Propagation::All(propagated), alpha, scenes, port_caps, held, &NO_CONSTRAINTS)
}

#[allow(clippy::too_many_arguments)]
fn parasitics_held(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagation: Propagation<'_>, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>, constraints: &vyges_est::sim::Constraints) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    use vyges_est::placement::{estimate_wire_parasitics, estimate_wire_parasitics_with_skips, skips_with_clocks, skips_with_clocks_and_constants, skips_with_constants, Timing};
    if no_signal_cap(db, rc) {
        return Ok(vec![HashMap::new(); scenes]);
    }
    // `isSkipPin`: an ideal clock's nets get no network; a propagated clock's do — per clock
    // (est 8e440c9, `ClkNetwork::findClkPins` from the ideal clocks only).
    let propagated = match propagation {
        Propagation::All(p) => p,
        Propagation::PerClock(c) => c.all_propagated(),
    };
    let timing = Timing { liberty: Some(liberty), clock_sources: clock_sources.to_vec(), propagated };
    // `isSkipPin` asks `Sta::isConstant`, under the SDC's constraint values when it has any: the
    // skips then come from that simulation, not the estimator's own (tie cells alone).
    let constrained = if held.is_none() && !(constraints.case.is_empty() && constraints.logic.is_empty()) {
        let values = vyges_est::sim::constant_pins_constrained(db, liberty, constraints).map_err(|e| format!("constant propagation: {e}"))?;
        Some(match propagation {
            Propagation::All(_) => skips_with_constants(db, &timing, &values),
            Propagation::PerClock(c) => skips_with_clocks_and_constants(db, &timing, c, &values),
        }
        .map_err(|e| e.to_string())?)
    } else {
        None
    };
    let nets = match (held.or(constrained.as_ref()), propagation) {
        (Some(skips), _) => estimate_wire_parasitics_with_skips(db, &timing, skips, alpha, &est_stt),
        (None, Propagation::All(_)) => estimate_wire_parasitics(db, &timing, alpha, &est_stt),
        (None, Propagation::PerClock(c)) => estimate_wire_parasitics_with_skips(db, &timing, &skips_with_clocks(db, &timing, c).map_err(|e| e.to_string())?, alpha, &est_stt),
    }
    .map_err(|e| e.to_string())?;
    let mut out = vec![HashMap::new(); scenes];
    for (k, map) in out.iter_mut().enumerate() {
        for n in &nets {
            if let Some(mut p) = net_parasitic(db, rc, n, k)? {
                // Reduced now, against the port loads in force now.
                p.port_pin_caps = Some(net_port_caps(port_caps, &p.node_names));
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
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::All(propagated), alpha, scenes, port_caps, None, &NO_CONSTRAINTS)
}

/// [`estimate_state`] with propagation read per clock (`clocks`, est 8e440c9).
#[allow(clippy::too_many_arguments)]
pub fn estimate_state_per_clock(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], clocks: &vyges_est::placement::Clocks, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::PerClock(clocks), alpha, scenes, port_caps, None, &NO_CONSTRAINTS)
}

/// [`estimate_state_per_clock`] under the SDC's constraint values ([`sdc_constraints`]): the nets
/// whose driver they make constant get no network (`isSkipPin`).
#[allow(clippy::too_many_arguments)]
pub fn estimate_state_per_clock_sdc(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], clocks: &vyges_est::placement::Clocks, alpha: f32, scenes: usize, port_caps: &PortCaps, constraints: &vyges_est::sim::Constraints) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::PerClock(clocks), alpha, scenes, port_caps, None, constraints)
}

/// [`estimate_state`] with `held`: the clock nets and constant pins of `db` as it is now, when the
/// caller has them; a `seen` design is a different one, so they are not used for it.
#[allow(clippy::too_many_arguments)]
pub fn estimate_state_held(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::All(propagated), alpha, scenes, port_caps, held, &NO_CONSTRAINTS)
}

/// [`estimate_state_held`] with propagation read per clock (`clocks`, est 8e440c9).
#[allow(clippy::too_many_arguments)]
pub fn estimate_state_held_per_clock(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], clocks: &vyges_est::placement::Clocks, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::PerClock(clocks), alpha, scenes, port_caps, held, &NO_CONSTRAINTS)
}

/// [`estimate_state_held_per_clock`] under the SDC's constraint values: `held` (when used) was made
/// under them; a `seen` design's skips are made under them too.
#[allow(clippy::too_many_arguments)]
pub fn estimate_state_held_per_clock_sdc(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], clocks: &vyges_est::placement::Clocks, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>, constraints: &vyges_est::sim::Constraints) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::PerClock(clocks), alpha, scenes, port_caps, held, constraints)
}

#[allow(clippy::too_many_arguments)]
fn state_held(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagation: Propagation<'_>, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>, constraints: &vyges_est::sim::Constraints) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    match seen {
        None => parasitics_held(db, rc, liberty, clock_sources, propagation, alpha, scenes, port_caps, held, constraints),
        Some(path) => {
            let seen = Db::open(path).map_err(|e| format!("{path}: {e}"))?;
            if &vyges_grt::timer::netlist(&seen) != netlist {
                return Err("the netlist was edited between estimate_parasitics and this command by a step the job does not carry: not modelled".into());
            }
            parasitics_held(&seen, rc, liberty, clock_sources, propagation, alpha, scenes, port_caps, None, constraints)
        }
    }
}

/// but `get_nets` / `get_ports`, and an input transition for a clock.
/// `Sim::isConstant` on the design as read (est's `constant_pins`: a signal pin on a power or
/// ground net, and what the cells' functions propagate from it), by pin name with its value — what
/// delay calculation and the search skip, and the search's conditional arcs and arc senses read.
pub fn sim_constants(db: &Db, liberty: &vyges_est::liberty::LibertyClocks) -> Result<HashMap<String, bool>, String> {
    Ok(constants_by_name(constant_values(db, liberty)?))
}

/// The logic simulation's constant pins (`vyges_est::sim::constant_pins`), for a caller that also
/// hands them to the estimator.
pub fn constant_values(db: &Db, liberty: &vyges_est::liberty::LibertyClocks) -> Result<std::collections::BTreeMap<vyges_est::sim::Pin, bool>, String> {
    vyges_est::sim::constant_pins(db, liberty).map_err(|e| format!("constant propagation: {e}"))
}

/// [`constant_values`] keyed as the timer names pins: a port by name, an instance pin `inst/pin`.
pub fn constants_by_name(values: std::collections::BTreeMap<vyges_est::sim::Pin, bool>) -> HashMap<String, bool> {
    use vyges_est::sim::Pin;
    values
        .into_iter()
        .map(|(p, v)| match p {
            Pin::Port(n) => (n, v),
            Pin::ITerm(i, t) => (format!("{i}/{t}"), v),
        })
        .collect()
}

/// The constraint values `Sim::seedConstants` reads from the SDC (`setConstraintConstPins`): every
/// `set_logic_*` pin, then every `set_case_analysis` pin, a later command on a pin replacing an
/// earlier (`Sdc::setLogicValue` / `setCaseAnalysis` are maps). A value of 0 or 1 holds a constant;
/// `set_logic_dc` (X) holds none but blocks one. Refused: a `rise` / `fall` case value (it limits
/// the transitions the search takes, which this timer does not model), an object that is not a
/// port or an instance pin of the database, and a pattern.
pub fn sdc_constraints(s: &vyges_loom::sdc::Sdc, db: &Db) -> Result<vyges_est::sim::Constraints, String> {
    use vyges_est::sim::Pin;
    let mut out = vyges_est::sim::Constraints::default();
    if s.case_values.is_empty() && s.logic_values.is_empty() {
        return Ok(out);
    }
    let ports: std::collections::HashSet<String> = db.block_get_b_terms().into_iter().collect();
    let insts: std::collections::HashSet<String> = db.block_get_insts().into_iter().collect();
    for (cmds, map) in [(&s.logic_values, &mut out.logic), (&s.case_values, &mut out.case)] {
        for c in cmds {
            let value = match (c.constant(), c.value.as_str()) {
                (Some(v), _) => Some(v),
                (None, "X") => None,
                (None, v) => return Err(format!("{} {v}: a transition value is not modelled (0 and 1 are)", c.cmd)),
            };
            for o in &c.objects {
                if o.contains(['*', '?']) {
                    return Err(format!("{} {o}: a pattern is not modelled", c.cmd));
                }
                let pin = match c.accessor.as_str() {
                    "get_ports" if ports.contains(o) => Pin::Port(o.clone()),
                    "get_pins" => match o.rsplit_once('/') {
                        Some((i, t)) if insts.contains(i) && !db.net_of(i, t).is_empty() => Pin::ITerm(i.to_string(), t.to_string()),
                        _ => return Err(format!("{} {o}: not a connected instance pin (not modelled)", c.cmd)),
                    },
                    a => return Err(format!("{} [{a} {o}]: not modelled (ports and instance pins are)", c.cmd)),
                };
                map.insert(pin, value);
            }
        }
    }
    Ok(out)
}

/// `Sdc::isConstrained` as `remove_buffers` asks it (`canRemoveBuffer`: the buffer's two pins,
/// the net it would remove, the buffer itself): the instance pins the SDC constrains — a clock
/// source, an I/O delay's pin or reference pin, a case or logic value, a pin capacitance limit,
/// a clock latency, uncertainty or propagation on a pin. Anything that would constrain an
/// instance or a net (`pvt`, derates, exceptions on cells or nets, …) or an exception point on a
/// pin is refused: those maps are not modelled. Ports and clocks constrain no instance pin.
pub fn remove_buffers_sdc_pins(s: &vyges_loom::sdc::Sdc, db: &Db) -> Result<std::collections::BTreeSet<String>, String> {
    use vyges_loom::sdc::{KeptArg, Objects};
    let ports: std::collections::HashSet<String> = db.block_get_b_terms().into_iter().collect();
    let clocks: std::collections::HashSet<&str> = s.clocks.iter().map(|c| c.name.as_str()).collect();
    // Objects naming only ports, clocks or the design.
    fn port_or_clock(o: &Objects, ports: &std::collections::HashSet<String>, clocks: &std::collections::HashSet<&str>) -> bool {
        match o.accessor.as_str() {
            "get_ports" | "all_inputs" | "all_outputs" | "get_clocks" | "all_clocks" | "current_design" => true,
            _ if !o.items.is_empty() => o.items.iter().all(|i| port_or_clock(i, ports, clocks)),
            "" | "list" | "concat" => o.names.iter().all(|n| ports.contains(n) || clocks.contains(n.as_str())),
            _ => false,
        }
    }
    let refuse = |cmd: &str, o: &Objects| Err(format!("remove_buffers: {cmd} on [{} {}]: an SDC constraint on an instance, a net or an exception pin is not modelled", o.accessor, o.names.join(" ")));
    let mut pins = std::collections::BTreeSet::new();
    for c in &s.clocks {
        pins.extend(c.sources.iter().filter(|p| !ports.contains(*p)).cloned());
    }
    for d in s.input_delays.iter().chain(&s.output_delays) {
        if d.accessor == "get_pins" {
            pins.extend(d.ports.iter().cloned());
        }
        pins.extend(d.reference_pin.iter().cloned());
    }
    for c in s.case_values.iter().chain(&s.logic_values) {
        if c.accessor == "get_pins" {
            pins.extend(c.objects.iter().cloned());
        }
    }
    for (_, o) in &s.max_capacitance_objects {
        match o.accessor.as_str() {
            "get_pins" => pins.extend(o.names.iter().cloned()),
            _ if port_or_clock(o, &ports, &clocks) => {}
            _ => return refuse("set_max_capacitance", o),
        }
    }
    for e in s.clock_latencies.iter().chain(&s.clock_transitions).chain(&s.env) {
        match e.accessor.as_str() {
            "get_pins" => pins.extend(e.objects.iter().cloned()),
            "get_ports" | "all_inputs" | "all_outputs" | "get_clocks" | "all_clocks" | "current_design" => {}
            _ if e.objects.iter().all(|n| ports.contains(n) || clocks.contains(n.as_str())) => {}
            a => return refuse(&e.cmd, &Objects { accessor: a.to_string(), names: e.objects.clone(), ..Default::default() }),
        }
    }
    for u in &s.uncertainties {
        for o in &u.written {
            match o.accessor.as_str() {
                "get_pins" => pins.extend(o.names.iter().cloned()),
                _ if port_or_clock(o, &ports, &clocks) => {}
                _ => return refuse("set_clock_uncertainty", o),
            }
        }
    }
    for o in &s.propagated {
        match o.accessor.as_str() {
            "get_pins" => pins.extend(o.names.iter().cloned()),
            _ if port_or_clock(o, &ports, &clocks) => {}
            _ => return refuse("set_propagated_clock", o),
        }
    }
    for e in &s.derates {
        if !e.objects.is_empty() && !matches!(e.accessor.as_str(), "current_design") {
            return refuse("set_timing_derate", &Objects { accessor: e.accessor.clone(), names: e.objects.clone(), ..Default::default() });
        }
    }
    let exception_objects = s.exceptions.iter().flat_map(|e| e.args.iter()).chain(s.path_delays.iter().flat_map(|p| p.objects.iter())).chain(s.other_exceptions.iter().flat_map(|e| e.args.iter()));
    for (key, o) in exception_objects {
        if !port_or_clock(o, &ports, &clocks) {
            return refuse(key, o);
        }
    }
    for a in s.disabled_timing.iter().chain(&s.senses).chain(&s.disabled_gating_checks) {
        if !port_or_clock(&a.written, &ports, &clocks) {
            return refuse(&a.cmd, &a.written);
        }
    }
    for d in &s.driving_cells {
        if let Some(o) = d.written.iter().find(|o| !port_or_clock(o, &ports, &clocks)) {
            return refuse("set_driving_cell", o);
        }
    }
    for k in &s.kept {
        let objects = k.keys.iter().map(|(_, a)| a).chain(&k.args).filter_map(KeptArg::objects);
        for o in objects {
            if !port_or_clock(o, &ports, &clocks) {
                return refuse(&k.cmd, o);
            }
        }
    }
    Ok(pins)
}

/// The logic simulation's constant pins under the SDC's constraint values (`Sim::ensureConstantsPropagated`
/// with `set_case_analysis` / `set_logic_*` seeded — what `Sta::isConstant` answers): the design's
/// own constants (tie cells, supply nets) without an SDC or with no such command.
pub fn constant_values_sdc(db: &Db, liberty: &vyges_est::liberty::LibertyClocks, sdc: Option<&vyges_loom::sdc::Sdc>) -> Result<BTreeMap<vyges_est::sim::Pin, bool>, String> {
    let c = match sdc {
        Some(s) => sdc_constraints(s, db)?,
        None => vyges_est::sim::Constraints::default(),
    };
    if c.case.is_empty() && c.logic.is_empty() {
        return constant_values(db, liberty);
    }
    vyges_est::sim::constant_pins_constrained(db, liberty, &c).map_err(|e| format!("constant propagation: {e}"))
}

/// [`constant_values_sdc`] keyed as the timer names pins ([`constants_by_name`]).
pub fn sim_constants_sdc(db: &Db, liberty: &vyges_est::liberty::LibertyClocks, sdc: Option<&vyges_loom::sdc::Sdc>) -> Result<HashMap<String, bool>, String> {
    Ok(constants_by_name(constant_values_sdc(db, liberty, sdc)?))
}

/// The pins the SDC's constraint values make constant beyond the design's own (`with` against
/// `without`) must stay off every clock network: a constant there prunes the reference's clock
/// network (`ClkTreeSearchPred` does not search through a constant), which the clock-pin walks
/// here do not model.
pub fn constants_off_clock_networks(db: &Db, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], constraints: &vyges_est::sim::Constraints, with: &BTreeMap<vyges_est::sim::Pin, bool>) -> Result<(), String> {
    use vyges_est::sim::Pin;
    if clock_sources.is_empty() || constraints.case.is_empty() && constraints.logic.is_empty() {
        return Ok(());
    }
    let without = constant_values(db, liberty)?;
    // a constraint value of X holds no constant but its pin is still one the constraints name
    let named: Vec<Pin> = constraints.case.keys().chain(constraints.logic.keys()).cloned().collect();
    let clock_nets = vyges_est::clk_network::find_clk_nets(db, liberty, clock_sources).map_err(|e| e.to_string())?;
    for p in with.keys().filter(|p| !without.contains_key(*p)).chain(named.iter()) {
        let net = match p {
            Pin::Port(n) => db.bterm_net(n),
            Pin::ITerm(i, t) => db.net_of(i, t),
        };
        if clock_nets.contains(&net) {
            return Err(format!("set_case_analysis / set_logic_*: {} is constant on a clock network (the pruned clock network is not modelled)", p.path_name()));
        }
    }
    Ok(())
}

/// [`vyges_sta::load::sdc_env`] over the design's libraries: `set_driving_cell`, `set_load` and
/// `set_input_transition` as the timer keeps them.
pub fn sdc_env(s: &vyges_loom::sdc::Sdc, libs: &Libs, nl: &vyges_sta::netlist::Netlist) -> Result<vyges_sta::graph::SdcEnv, String> {
    vyges_sta::load::sdc_env(s, &libs.libs, nl)
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

/// [`expanded_io_delays`] written back into the constraints: `[all_inputs]` / `[all_outputs]` in
/// an I/O delay become the ports they name, for a caller that keeps the SDC.
pub fn expand_io_delay_ports(s: &mut vyges_loom::sdc::Sdc, nl: &vyges_sta::netlist::Netlist) {
    let [i, o] = expanded_io_delays(s, nl);
    (s.input_delays, s.output_delays) = (i, o);
}

/// The clock state the estimator reads (est's [`vyges_est::placement::Clocks`]), as the search
/// resolves it (`search_sdc_on`): each SDC clock made, then propagated when the run's
/// `propagated` holds and the SDC's `set_propagated_clock` names it — or names no clock at all, when
/// every clock is (the reference writes the propagation per clock; a virtual clock never is). A
/// latency on a propagated clock is refused by the search, so no latency reaches the order here.
pub fn est_clocks(sdc: Option<&vyges_loom::sdc::Sdc>, propagated: bool) -> vyges_est::placement::Clocks {
    use vyges_est::placement::{ClockObject, Clocks};
    let mut out = Clocks::default();
    let Some(s) = sdc else { return out };
    let named: Vec<&str> = s.propagated.iter().filter(|o| o.accessor == "get_clocks").flat_map(|o| o.names.iter().map(String::as_str)).collect();
    for c in &s.clocks {
        out.create_clock(&c.name, &c.sources);
    }
    if propagated {
        let objects: Vec<ClockObject> = s.clocks.iter().filter(|c| !c.is_virtual() && (named.is_empty() || named.contains(&c.name.as_str()))).map(|c| ClockObject::Clock(c.name.clone())).collect();
        out.set_propagated_clock(&objects).expect("each clock was made above");
    }
    out
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

/// The design as an SDC object query sees it — the database's ports, instances and instance pins
/// ([`vyges_sta::load::sdc_design`] over the database's netlist), for `Sdc::load_with`.
pub fn sdc_design(db: &vyges_opendb::Db, libs: &Libs) -> vyges_loom::sdc::SdcDesign {
    vyges_sta::load::sdc_design(&vyges_grt::timer::netlist(db), &libs.libs)
}

#[cfg(test)]
mod est_clocks_tests {
    use super::est_clocks;

    // Rule (est 8e440c9 over the search's resolution, the reference writing propagation per clock):
    // `set_propagated_clock [all_clocks]` then `set_clock_latency 0.6 [get_clocks clk2]` leaves clk1
    // propagated and clk2 ideal — the written SDC names clk1 alone — so the estimate skips only
    // clk2's nets. With no clock named, the run's flag decides for every clock; without it, none is.
    #[test]
    fn the_estimate_reads_propagation_per_clock() {
        let s = vyges_loom::sdc::Sdc::parse(
            "create_clock -name clk1 -period 2 [get_ports {clk1}]\nset_propagated_clock [get_clocks {clk1}]\ncreate_clock -name clk2 -period 2 [get_ports {clk2}]\nset_clock_latency 0.6 [get_clocks {clk2}]\n",
        )
        .unwrap();
        let c = est_clocks(Some(&s), true);
        assert_eq!(c.ideal_sources(), vec!["clk2".to_string()]);
        assert!(est_clocks(Some(&s), false).ideal_sources().len() == 2, "the run never propagated");
        let all = vyges_loom::sdc::Sdc::parse("create_clock -name a -period 2 [get_ports {a}]\ncreate_clock -name b -period 2 [get_ports {b}]\n").unwrap();
        assert!(est_clocks(Some(&all), true).all_propagated(), "no clock named: the flag covers all");
        assert!(est_clocks(None, true).ideal_sources().is_empty());
    }
}

#[cfg(test)]
mod net_port_caps_tests {
    use super::{net_port_caps, PortCaps};

    // Rule (`Graph::reduced` reads `port_pin_caps` for a PORT node of the same net, by its name, a
    // port not held reading 0): a net keeps only its own ports' loads — the same answer as the
    // whole map for every name it can be asked — and a net with none keeps an empty map, not
    // `None` (which would read the loads in force now).
    #[test]
    fn a_net_keeps_only_its_own_ports_loads() {
        let load = |c: f32| [[Some(c), Some(c)], [Some(c), Some(c)]];
        let caps: PortCaps = [("y".to_string(), load(0.5)), ("z".to_string(), load(0.7))].into_iter().collect();
        let nodes = vec!["u2/Y".to_string(), "y".to_string(), "n1:1".to_string()];
        let own = net_port_caps(&caps, &nodes);
        assert_eq!(own.len(), 1);
        for n in &nodes {
            assert_eq!(own.get(n), caps.get(n), "{n}: as the whole map answers");
        }
        assert!(net_port_caps(&caps, &["u1/Y".to_string()]).is_empty());
    }
}
