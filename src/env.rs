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
    parasitics_held(db, rc, liberty, clock_sources, Propagation::All(propagated), alpha, scenes, port_caps, None)
}

/// [`placement_parasitics_at`] with propagation read per clock (`clocks`, est 8e440c9).
#[allow(clippy::too_many_arguments)]
pub fn placement_parasitics_per_clock(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], clocks: &vyges_est::placement::Clocks, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    parasitics_held(db, rc, liberty, clock_sources, Propagation::PerClock(clocks), alpha, scenes, port_caps, None)
}

/// [`placement_parasitics_at`] with the clock nets and constant pins of `db` as it is now, when the
/// caller holds them (`held`); `None` walks the design for them.
#[allow(clippy::too_many_arguments)]
pub fn placement_parasitics_held(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    parasitics_held(db, rc, liberty, clock_sources, Propagation::All(propagated), alpha, scenes, port_caps, held)
}

#[allow(clippy::too_many_arguments)]
fn parasitics_held(db: &Db, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagation: Propagation<'_>, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    use vyges_est::placement::{estimate_wire_parasitics, estimate_wire_parasitics_with_skips, skips_with_clocks, Timing};
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
    let nets = match (held, propagation) {
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
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::All(propagated), alpha, scenes, port_caps, None)
}

/// [`estimate_state`] with propagation read per clock (`clocks`, est 8e440c9).
#[allow(clippy::too_many_arguments)]
pub fn estimate_state_per_clock(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], clocks: &vyges_est::placement::Clocks, alpha: f32, scenes: usize, port_caps: &PortCaps) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::PerClock(clocks), alpha, scenes, port_caps, None)
}

/// [`estimate_state`] with `held`: the clock nets and constant pins of `db` as it is now, when the
/// caller has them; a `seen` design is a different one, so they are not used for it.
#[allow(clippy::too_many_arguments)]
pub fn estimate_state_held(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagated: bool, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::All(propagated), alpha, scenes, port_caps, held)
}

/// [`estimate_state_held`] with propagation read per clock (`clocks`, est 8e440c9).
#[allow(clippy::too_many_arguments)]
pub fn estimate_state_held_per_clock(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], clocks: &vyges_est::placement::Clocks, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    state_held(db, seen, netlist, rc, liberty, clock_sources, Propagation::PerClock(clocks), alpha, scenes, port_caps, held)
}

#[allow(clippy::too_many_arguments)]
fn state_held(db: &Db, seen: Option<&str>, netlist: &vyges_sta::netlist::Netlist, rc: &vyges_est::rc::Rc, liberty: &vyges_est::liberty::LibertyClocks, clock_sources: &[String], propagation: Propagation<'_>, alpha: f32, scenes: usize, port_caps: &PortCaps, held: Option<&vyges_est::placement::Skips>) -> Result<Vec<HashMap<String, NetParasitics>>, String> {
    match seen {
        None => parasitics_held(db, rc, liberty, clock_sources, propagation, alpha, scenes, port_caps, held),
        Some(path) => {
            let seen = Db::open(path).map_err(|e| format!("{path}: {e}"))?;
            if &vyges_grt::timer::netlist(&seen) != netlist {
                return Err("the netlist was edited between estimate_parasitics and this command by a step the job does not carry: not modelled".into());
            }
            parasitics_held(&seen, rc, liberty, clock_sources, propagation, alpha, scenes, port_caps, None)
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
