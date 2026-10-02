// SPDX-License-Identifier: Apache-2.0
//! `repair_design`, as the reference sequences it: `Resizer::repairDesign` (its preamble), then
//! `RepairDesign::repairDesign`. This file is only the sequence — each stage is one call, in the
//! reference's order, and does its work elsewhere — so a trace that diverges points at one line.
//!
//! A stage that is not implemented yet stops the run as `refused` with the stage named: a run
//! that did not do the repair never reports one.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use vyges_sta::graph::{Graph, SdcEnv};

use crate::buffered_net::{self, WireRc};
use crate::design::Design;
use crate::driver_slew;
use crate::fanout;
use crate::preamble::{self, Libs, Master};
use crate::sizing::{self, Sizing};
use crate::timing::{self, Limits};
use crate::trace::{g9, Trace};
use crate::walk::{PreCheck, State, Walk};
use crate::Stop;

/// The `repair_design` arguments that reach the repair.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Args {
    /// `-max_wire_length`, meters; 0 for none.
    pub max_wire_length: f64,
    /// `-slew_margin`, `-cap_margin`, percent.
    pub slew_margin: f64,
    pub cap_margin: f64,
    /// `-pre_placement` (or the deprecated `-buffer_gain`): the early sizing round.
    pub pre_placement: bool,
}

/// What the repair reads besides the arguments and the design it edits.
pub struct Inputs<'a> {
    pub libs: &'a Libs,
    pub masters: &'a BTreeMap<String, Master>,
    /// `dont_use_`: liberty `dont_use` cells plus `set_dont_use`.
    pub dont_use: &'a BTreeSet<String>,
    /// The constraints the slew limits read.
    pub limits: Limits,
    /// Every `create_clock`'s source pins (the clock network's roots).
    pub clock_sources: &'a [String],
    /// Database units per micron.
    pub dbu: i32,
    /// The signal wire RC (`wireSignal{H,V}{Resistance,Capacitance}`), per scene.
    pub wire_rc: Vec<WireRc>,
    /// The SDC loads and input slews the timer reads (net loads keyed by their driver pins).
    pub sdc: SdcEnv,
    /// Each master's signal terminals (the database's): the timer's pins are these.
    pub master_pins: HashMap<String, Vec<String>>,
}

/// A run that reached its end.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Outcome {
    /// Drivers `repairNet` checked.
    pub nets_checked: usize,
    /// `repaired_net_count`: nets whose buffered net was walked or whose fanout was repaired.
    pub nets_repaired: usize,
    /// `inserted_buffer_count_`.
    pub inserted_buffers: usize,
    /// `reportViolationCounters`' counts: nets with a slew violation (driver, load, or one
    /// hidden by a forward-pass annotation), a capacitance violation, a fanout violation, a wire
    /// over the length limit; and drivers resized (`resize_count_`).
    pub slew_violations: usize,
    pub cap_violations: usize,
    pub fanout_violations: usize,
    pub length_violations: usize,
    pub resized: usize,
    /// Drivers `repairDriver` passed over: no net, dont_touch, connected by abutment, a clock.
    pub drivers_skipped: usize,
    /// Warnings the command writes before it repairs (`check_max_wire_length`: RSZ-0065).
    pub warnings: Vec<(&'static str, String)>,
    /// `check_max_wire_length`'s per-buffer lengths: (scene, buffer, meters), in computed order.
    pub max_wire_lengths: Vec<(usize, String, f64)>,
}

impl Outcome {
    /// The status word, settled in one place: `repaired` when the design changed (a buffer
    /// inserted or a driver resized); `up_to_date` when drivers were checked and none needed a
    /// change (the count is the evidence); `vacuous` when no driver was checked at all.
    pub fn settle(&self) -> &'static str {
        if self.inserted_buffers + self.resized > 0 {
            "repaired"
        } else if self.nets_checked > 0 {
            "up_to_date"
        } else {
            "vacuous"
        }
    }

    /// `reportViolationCounters`: one line per non-zero count, in its order — `(code, text)`.
    pub fn summary(&self) -> Vec<(&'static str, String)> {
        let mut out = Vec::new();
        if self.slew_violations > 0 {
            out.push(("RSZ-0034", format!("Found {} slew violations.", self.slew_violations)));
        }
        if self.fanout_violations > 0 {
            out.push(("RSZ-0035", format!("Found {} fanout violations.", self.fanout_violations)));
        }
        if self.cap_violations > 0 {
            out.push(("RSZ-0036", format!("Found {} capacitance violations.", self.cap_violations)));
        }
        if self.length_violations > 0 {
            out.push(("RSZ-0037", format!("Found {} long wires.", self.length_violations)));
        }
        if self.resized > 0 {
            out.push(("RSZ-0039", format!("Resized {} instances.", self.resized)));
        }
        if self.inserted_buffers > 0 {
            out.push(("RSZ-0038", format!("Inserted {} buffers in {} nets.", self.inserted_buffers, self.nets_repaired)));
        }
        out
    }
}

/// `RepairDesign::init`'s `r_strongest_buffer_`: the least drive resistance over the buffers.
fn r_strongest_buffer(inputs: &Inputs<'_>, buffers: &[String]) -> f32 {
    buffers.iter().filter_map(|b| inputs.libs.link_cell(b)).map(preamble::buffer_drive_resistance).fold(f32::MAX, f32::min)
}

/// `Resizer::repairDesign` → `RepairDesign::repairDesign`.
pub fn repair_design(inputs: &Inputs<'_>, design: &mut dyn Design, args: &Args, trace: &mut Trace) -> Result<Outcome, Stop> {
    // Resizer::resizePreamble
    let lib = inputs.libs.default_library().ok_or_else(|| Stop::error("RSZ-LIB", "no liberty library read".into()))?;
    let equiv = sizing::make_equiv_cells(inputs.libs);
    let buffers = preamble::find_buffers(inputs.libs, inputs.masters, inputs.dont_use)?;
    let (tgt_slews, tgt_scene, target_loads) = preamble::find_target_loads(inputs.libs, &buffers.cells, inputs.dont_use);
    // check_max_wire_length (the command's Tcl, before repair_design_cmd): the same buffer list and
    // target slews (findMaxWireLength runs findBuffers and findTargetLoads too).
    let mwl = crate::max_wire_length::Ctx { libs: inputs.libs, buffers: &buffers.cells, tgt_slews, wire_rc: &inputs.wire_rc, master_pins: &inputs.master_pins };
    let check = crate::max_wire_length::check_max_wire_length(&mwl, args.max_wire_length)?;
    let warnings: Vec<(&'static str, String)> = check.warning.into_iter().collect();
    let max_wire_lengths = check.lengths;
    let slew_shape_factor = preamble::compute_slew_shape_factor(lib)?;

    // RepairDesign::repairDesign
    let r_strongest = r_strongest_buffer(inputs, &buffers.cells);
    trace.push(format!(
        "pre|shape={}|r_strongest={}|slew_margin={}|cap_margin={}|initial_sizing={}",
        g9(f64::from(slew_shape_factor)),
        g9(f64::from(r_strongest)),
        g9(args.slew_margin),
        g9(args.cap_margin),
        args.pre_placement
    ));
    for (i, name) in buffers.cells.iter().enumerate() {
        let r = inputs.libs.link_cell(name).map_or(0.0, preamble::buffer_drive_resistance);
        let tl = target_loads.get(name).map_or(-1.0, |&l| f64::from(l));
        trace.push(format!("buf|{i}|{name}|{}|{}", g9(f64::from(r)), g9(tl)));
    }
    trace.push(format!("lowest|{}", buffers.lowest));
    trace.ran(&["pre", "buf", "lowest"]);
    // checkSlewsPreamble, checkCapacitancesPreamble, checkFanoutPreamble, searchPreamble,
    // findAllArrivals: the timing graphs (one per scene), levelized, their delays found.
    let netlist = design.netlist().clone();
    let (mut graphs, level) = timer_preambles(inputs, &netlist)?;
    if args.pre_placement {
        return Err(Stop::refused("RSZ-ABSENT", "the early sizing round (-pre_placement / -buffer_gain) is not modelled".into()));
    }
    // The forward pass: each load in order, each scene in order — an over-limit load slew
    // annotated at its limit in that scene, kept, by pin name, for the whole repair
    // (`annotations_to_clean_up` is cleared only after it); its drivers remember the scene.
    let loads = timing::ordered_load_pin_vertices(&graphs[0], &level);
    let items: Vec<Vec<(usize, usize, f32)>> = {
        let sc = timing::Scenes::new(&graphs);
        (0..graphs.len()).map(|k| loads.iter().filter_map(|&v| timing::load_limit(&sc, k, v, &inputs.limits).map(|(vk, l)| (vk, v, l))).collect()).collect()
    };
    let mut per_scene: Vec<Vec<timing::Annotation>> = Vec::new();
    for (k, g) in graphs.iter_mut().enumerate() {
        per_scene.push(timing::annotate_load_slews(g, &items[k], design.parasitics(k)).map_err(|e| Stop::refused("RSZ-TIMER", e))?);
    }
    let g0 = &graphs[0];
    let mut annotated: Vec<HashMap<String, f32>> = vec![HashMap::new(); graphs.len()];
    let mut drvr_with_load_slew_viol: HashMap<String, usize> = HashMap::new();
    for &v in &loads {
        for (k, anns) in per_scene.iter().enumerate() {
            if let Some(a) = anns.iter().find(|a| a.load == v) {
                trace.push(format!("annot|{}|{}|{}", g0.vertices[v].name, inputs.libs.scene_name(k), g9(f64::from(a.limit))));
                annotated[k].insert(g0.vertices[v].name.clone(), a.limit);
                for d in timing::net_drivers(g0, v) {
                    drvr_with_load_slew_viol.insert(g0.vertices[d].name.clone(), k);
                }
            }
        }
    }
    trace.ran(&["annot"]);
    // Fix violations from outputs to inputs: the drivers, levelized ONCE, from the LAST.
    let drivers: Vec<String> = timing::levelized_drvr_vertices(g0, &level).into_iter().map(|d| g0.vertices[d].name.clone()).collect();
    drop(graphs);
    let sizing = Sizing { libs: inputs.libs, masters: inputs.masters, dont_use: inputs.dont_use, equiv: &equiv, target_loads: &target_loads, tgt_slews, tgt_scene };
    let mut ctx = NetCtx {
        inputs,
        args,
        annotated: &annotated,
        drvr_with_load_slew_viol: &drvr_with_load_slew_viol,
        min_cap_load: timing::min_cap_load(&inputs.libs.libs),
        max_length: meters_to_dbu(args.max_wire_length, inputs.dbu),
        sizing: &sizing,
        slew_shape_factor,
        r_strongest_buffer: r_strongest,
        buffer_lowest_drive: &buffers.lowest,
        pre: PreCheck::default(),
    };
    trace.ran(&["drv"]);
    let mut outcome = Outcome { warnings, max_wire_lengths, ..Default::default() };
    for i in (0..drivers.len()).rev() {
        trace.push(format!("drv|{i}|{}", drivers[i]));
        repair_driver(design, &drivers[i], &mut ctx, &mut outcome, trace)?;
    }
    Ok(outcome)
}

/// `Resizer::metersToDbu`: rounded (`lround`), masked to a non-negative `int`.
fn meters_to_dbu(dist: f64, dbu: i32) -> i64 {
    ((dist * f64::from(dbu) * 1e6).round() as i64) & i64::from(i32::MAX)
}

/// What `repairNet` reads besides the design.
struct NetCtx<'a, 'b> {
    inputs: &'b Inputs<'a>,
    args: &'b Args,
    /// The forward pass's annotated load slews, per scene, by pin.
    annotated: &'b [HashMap<String, f32>],
    /// `drvr_with_load_slew_viol`: a driver of an annotated load, and the scene it was in.
    drvr_with_load_slew_viol: &'b HashMap<String, usize>,
    min_cap_load: f32,
    max_length: i64,
    sizing: &'b Sizing<'b>,
    slew_shape_factor: f32,
    r_strongest_buffer: f32,
    buffer_lowest_drive: &'b str,
    /// `PreChecks`, made once per `repairDesign` and kept across nets.
    pre: PreCheck,
}

/// The timer's graph over a netlist for scene `k` (its libraries' cells), with the SDC environment.
pub fn timer_graph<'n>(libs: &'n Libs, k: usize, netlist: &'n vyges_sta::netlist::Netlist, sdc: &SdcEnv, master_pins: &HashMap<String, Vec<String>>) -> Result<Graph<'n>, Stop> {
    let mut g = Graph::build_with_pins(libs.scene_slice(k), netlist, Some(master_pins)).map_err(|e| Stop::refused("RSZ-TIMER", e))?;
    g.sdc = sdc.clone();
    Ok(g)
}

/// The timing graphs (one per scene) over the design as it is NOW, each with its scene's
/// forward-pass annotations applied.
fn current_graphs<'n>(inputs: &'n Inputs<'_>, netlist: &'n vyges_sta::netlist::Netlist, annotated: &[HashMap<String, f32>]) -> Result<Vec<Graph<'n>>, Stop> {
    (0..inputs.libs.scene_count())
        .map(|k| {
            let mut g = timer_graph(inputs.libs, k, netlist, &inputs.sdc, &inputs.master_pins)?;
            g.slew_annotated = g.vertices.iter().enumerate().filter_map(|(i, v)| annotated[k].get(&v.name).map(|&l| (i, l))).collect();
            Ok(g)
        })
        .collect()
}

/// Delay calculation in every scene, each on its own parasitics. A diagnostic: with
/// `VYGES_RSZ_DCALC=<file>`, scene 0's delay-calculation trace is appended to it, headed by the
/// driver it was made for.
fn find_all_delays(graphs: &mut [Graph<'_>], design: &dyn Design, label: &str) -> Result<(), Stop> {
    let dump = std::env::var("VYGES_RSZ_DCALC").ok();
    for (k, g) in graphs.iter_mut().enumerate() {
        let mut lines = Vec::new();
        let trace = (k == 0 && dump.is_some()).then_some(&mut lines);
        g.find_delays(design.parasitics(k), trace).map_err(|e| Stop::refused("RSZ-TIMER", e))?;
        if let (0, Some(path)) = (k, &dump) {
            use std::io::Write;
            if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                let _ = writeln!(f, "find|{label}");
                for l in &lines {
                    let _ = writeln!(f, "{l}");
                }
            }
        }
    }
    Ok(())
}

/// `RepairDesign::repairDriver`: the driver's flat net, unless it is dont_touch, connected by
/// abutment, a clock pin or a constant (constants are refused upstream).
fn repair_driver(design: &mut dyn Design, drvr_name: &str, ctx: &mut NetCtx<'_, '_>, outcome: &mut Outcome, trace: &mut Trace) -> Result<(), Stop> {
    let netlist = design.netlist().clone();
    let graphs = current_graphs(ctx.inputs, &netlist, ctx.annotated)?;
    let g0 = &graphs[0];
    let Some(drvr) = g0.vertices.iter().position(|v| v.is_driver && v.name == drvr_name) else {
        outcome.drivers_skipped += 1;
        return Ok(());
    };
    // Special nets are not in the timer's netlist: a driver of one has no net here, and the
    // reference's repairNet passes it over.
    let Some(n) = g0.vertex_net[drvr] else {
        outcome.drivers_skipped += 1;
        return Ok(());
    };
    let net = netlist.nets[n].name.clone();
    let clocks = timing::clock_pins(g0, ctx.inputs.clock_sources);
    let info = design.net_info();
    if info.dont_touch.contains(&net) || info.abutment.contains(&net) || clocks.contains(&drvr) {
        outcome.drivers_skipped += 1;
        return Ok(());
    }
    outcome.nets_checked += 1;
    let viol = ctx.drvr_with_load_slew_viol.get(drvr_name).copied();
    repair_net(design, graphs, drvr, &net, &clocks, true, viol, ctx, outcome, trace)
}

/// `makeFanoutRepeater`'s `repairNet(out_net, repeater_out_pin, …)`: the new repeater's net, with
/// the fanout check off and no annotated violation — straight to `repairNet`, none of
/// `repairDriver`'s skips. Its violation and repaired-net counts are its own (discarded); its
/// buffers and resizes count.
///
/// `repairNet` times its driver with `GraphDelayCalc::findDelays(drvr)` — that ONE driver, from
/// the input slews the graph holds — and nothing has timed the new repeater's input pin yet: its
/// slew is the vertex's initial 0 (its driver is timed again only by the outer `repairNet`).
fn repair_repeater_net(design: &mut dyn Design, in_pin: &str, out_pin: &str, out_net: &str, ctx: &mut NetCtx<'_, '_>, outcome: &mut Outcome, trace: &mut Trace) -> Result<(), Stop> {
    let netlist = design.netlist().clone();
    let mut graphs = current_graphs(ctx.inputs, &netlist, ctx.annotated)?;
    for g in graphs.iter_mut() {
        if let Some(v) = g.vertices.iter().position(|v| !v.is_driver && v.name == in_pin) {
            g.slew_annotated.insert(v, 0.0);
        }
    }
    let drvr = graphs[0].vertices.iter().position(|v| v.is_driver && v.name == out_pin).ok_or_else(|| Stop::error("RSZ-INSERT", format!("{out_pin}: not in the timing graph")))?;
    let clocks = timing::clock_pins(&graphs[0], ctx.inputs.clock_sources);
    let mut own = Outcome::default();
    let r = repair_net(design, graphs, drvr, out_net, &clocks, false, None, ctx, &mut own, trace);
    // `inserted_buffer_count_` and `resize_count_` are members — they count across the nested
    // repair; its violation and repaired-net counts are its own locals.
    outcome.inserted_buffers += own.inserted_buffers;
    outcome.resized += own.resized;
    r
}

/// `RepairDesign::makeRegionRepeaters(region, max_fanout, level, drvr, …)`: a leaf region is left
/// for its parent; otherwise its sub-regions first (bottom up), then their pins and its own, each
/// taken from the BACK, into groups of `max_fanout` each buffered as it fills; a remainder of at
/// least `max_fanout / 2` (int) is buffered too, else it stays the region's pins; the repeaters'
/// inputs join them.
#[allow(clippy::too_many_arguments)]
fn make_region_repeaters(region: &mut fanout::LoadRegion, max_fanout: i32, drvr_pin: &str, link: &Graph<'_>, design: &mut dyn Design, ctx: &mut NetCtx<'_, '_>, outcome: &mut Outcome, trace: &mut Trace) -> Result<(), Stop> {
    if region.regions.is_empty() {
        return Ok(());
    }
    for k in 0..region.regions.len() {
        make_region_repeaters(&mut region.regions[k], max_fanout, drvr_pin, link, design, ctx, outcome, trace)?;
    }
    let mut repeater_inputs: Vec<String> = Vec::new();
    let mut repeater_loads: Vec<String> = Vec::new();
    let full = |n: usize| n as i64 == i64::from(max_fanout);
    for k in 0..region.regions.len() {
        while let Some(p) = region.regions[k].pins.pop() {
            repeater_loads.push(p);
            if full(repeater_loads.len()) {
                make_fanout_repeater(&mut repeater_loads, &mut repeater_inputs, drvr_pin, link, design, ctx, outcome, trace)?;
            }
        }
    }
    while let Some(p) = region.pins.pop() {
        repeater_loads.push(p);
        if full(repeater_loads.len()) {
            make_fanout_repeater(&mut repeater_loads, &mut repeater_inputs, drvr_pin, link, design, ctx, outcome, trace)?;
        }
    }
    if !repeater_loads.is_empty() && repeater_loads.len() as i64 >= i64::from(max_fanout / 2) {
        make_fanout_repeater(&mut repeater_loads, &mut repeater_inputs, drvr_pin, link, design, ctx, outcome, trace)?;
    } else {
        region.pins = repeater_loads;
    }
    region.pins.extend(repeater_inputs);
    Ok(())
}

/// `RepairDesign::makeFanoutRepeater`: the lowest-drive buffer, NOT resized, at the load nearest
/// the driver (`findClosedPinLoc`), level 1, reason `fanout`; then its net repaired; its input
/// joins the repeater inputs.
#[allow(clippy::too_many_arguments)]
fn make_fanout_repeater(repeater_loads: &mut Vec<String>, repeater_inputs: &mut Vec<String>, drvr_pin: &str, link: &Graph<'_>, design: &mut dyn Design, ctx: &mut NetCtx<'_, '_>, outcome: &mut Outcome, trace: &mut Trace) -> Result<(), Stop> {
    let inputs = ctx.inputs;
    let (x, y) = {
        let d: &dyn Design = &*design;
        fanout::find_closed_pin_loc(d.pin_location(drvr_pin), repeater_loads, &|p| d.pin_location(p))
    };
    // `corner_` is the command scene here (set just before makeRegionRepeaters).
    let bctx = buffered_net::Ctx { graph: link, libs: inputs.libs, sdc: &inputs.sdc, limits: &inputs.limits, dbu: inputs.dbu, rc: inputs.wire_rc[0] };
    let walk = Walk {
        ctx: &bctx,
        sizing: ctx.sizing,
        drvr: 0,
        max_cap: timing::INF,
        max_length: ctx.max_length,
        slew_margin: ctx.args.slew_margin,
        slew_shape_factor: ctx.slew_shape_factor,
        r_strongest_buffer: ctx.r_strongest_buffer,
        buffer_lowest_drive: ctx.buffer_lowest_drive,
        sdc: &inputs.sdc,
        master_pins: &inputs.master_pins,
        corner: 0,
    };
    let mut loads = repeater_loads.clone();
    let mut st = State { design, pre: std::mem::take(&mut ctx.pre), inserted: 0, last: None };
    let made = walk.make_repeater("fanout", x, y, ctx.buffer_lowest_drive, false, 1, &mut loads, &mut st, trace);
    ctx.pre = st.pre;
    outcome.inserted_buffers += st.inserted;
    made?;
    let r = st.last.expect("a repeater was made");
    repair_repeater_net(design, &r.input, &r.output, &r.out_net, ctx, outcome, trace)?;
    repeater_inputs.push(r.input);
    repeater_loads.clear();
    Ok(())
}

/// `RepairDesign::repairNet(net, drvr_pin, …)` — check_slew, check_cap and check_fanout on,
/// resizing the driver allowed, as `repairDesign` calls it. `drvr` is scene 0's vertex.
#[allow(clippy::too_many_arguments)]
fn repair_net(design: &mut dyn Design, graphs: Vec<Graph<'_>>, drvr: usize, net: &str, clocks: &BTreeSet<usize>, check_fanout: bool, corner_w_load_slew_viol: Option<usize>, ctx: &mut NetCtx<'_, '_>, outcome: &mut Outcome, trace: &mut Trace) -> Result<(), Stop> {
    let inputs = ctx.inputs;
    let name = graphs[0].vertices[drvr].name.clone();
    trace.push(format!("net|{name}|true|true|{check_fanout}|{}|true", ctx.max_length));
    trace.ran(&["net", "fo", "fanloads", "dslew", "lslew", "cap"]);
    // `corner`: the command scene, until a violation names one.
    let mut corner = 0usize;
    let mut repaired_net = false;
    // The graphs after an edit (the netlists they read are declared first so they outlive them).
    let fanout_netlist;
    let fanout_clocks;
    let resized_netlist;
    let resized_clocks;
    let mut graphs = graphs;
    let mut clocks = clocks;
    let mut drvr = drvr;
    let mut net_name = net.to_string();

    // Fanout is addressed by creating region repeaters.
    if check_fanout {
        let (fanout, max_fanout, fanout_slack) = timing::check_fanout(&graphs[0], design.net_info(), drvr, &inputs.limits, clocks);
        trace.push(format!("fo|{}|{}|{}", g9(f64::from(fanout)), g9(f64::from(max_fanout)), g9(f64::from(fanout_slack))));
        if max_fanout > 0.0 && fanout_slack < 0.0 {
            outcome.fanout_violations += 1;
            repaired_net = true;
            // findLoadRegions(net, drvr_pin, max_fanout): the limit as an int.
            let max_fanout = max_fanout as i32;
            let loads: Vec<String> = design.visit_connected_pins(&name).into_iter().filter(|p| graphs[0].vertices.iter().any(|v| &v.name == p && !v.is_driver)).collect();
            trace.push(format!("fanloads|{name}|{max_fanout}|{}", loads.iter().map(|p| format!("{p},")).collect::<String>()));
            let mut region = {
                let d: &dyn Design = &*design;
                fanout::find_load_regions(loads, max_fanout, inputs.dbu, &|p| d.pin_location(p))
            };
            make_region_repeaters(&mut region, max_fanout, &name, &graphs[0], design, ctx, outcome, trace)?;
            // The design changed under the driver: its graphs again.
            fanout_netlist = design.netlist().clone();
            graphs = current_graphs(inputs, &fanout_netlist, ctx.annotated)?;
            drvr = graphs[0].vertices.iter().position(|v| v.is_driver && v.name == name).expect("the driver");
            fanout_clocks = timing::clock_pins(&graphs[0], inputs.clock_sources);
            clocks = &fanout_clocks;
            // The reference holds the net OBJECT: a repeater before a port load renames it, so
            // its name is read again from the driver.
            if let Some(n) = graphs[0].vertex_net[drvr] {
                net_name = fanout_netlist.nets[n].name.clone();
            }
        }
    }

    // ensureWireParasitic(drvr_pin, net), then findDelays(drvr).
    design.ensure_wire_parasitic(&net_name).map_err(|e| Stop::error("RSZ-EST", e))?;
    find_all_delays(&mut graphs, &*design, &name)?;

    // First the driver's slew: resize the driver, and if that does not fix it, derive the load
    // cap that would, which then caps the buffered net.
    let mut max_cap = timing::INF;
    let mut repair_cap = false;
    let (mut slew1, mut max_slew1, mut slack1, mut corner1) = timing::repair_check_slew(&timing::Scenes::new(&graphs), drvr, &inputs.limits, clocks, ctx.args.slew_margin);
    trace.push(format!("dslew|{}|{}|{}", g9(f64::from(slew1.unwrap_or(0.0))), g9(f64::from(max_slew1)), g9(f64::from(slack1))));
    trace.ran(&["rds", "rdsc", "rdsel", "dresize", "dslew2", "slewcap", "slc0", "slc"]);
    let mut slew_violation = false;
    if slack1 < 0.0 {
        slew_violation = true;
        let c1 = corner1.expect("a violation has a scene");
        if let Some(cell) = repair_driver_slew(&*design, &graphs, c1, drvr, ctx, trace)? {
            design.swap_master(&inst_of(&name), &cell).map_err(|e| Stop::error("RSZ-RESIZE", e))?;
            outcome.resized += 1;
            trace.push(format!("dresize|{cell}"));
            design.update_parasitics().map_err(|e| Stop::error("RSZ-EST", e))?;
            resized_netlist = design.netlist().clone();
            graphs = current_graphs(inputs, &resized_netlist, ctx.annotated)?;
            drvr = graphs[0].vertices.iter().position(|v| v.is_driver && v.name == name).expect("the resized driver");
            resized_clocks = timing::clock_pins(&graphs[0], inputs.clock_sources);
            clocks = &resized_clocks;
            find_all_delays(&mut graphs, &*design, &name)?;
            (slew1, max_slew1, slack1, corner1) = timing::repair_check_slew(&timing::Scenes::new(&graphs), drvr, &inputs.limits, clocks, ctx.args.slew_margin);
            trace.push(format!("dslew2|{}|{}|{}", g9(f64::from(slew1.unwrap_or(0.0))), g9(f64::from(max_slew1)), g9(f64::from(slack1))));
        }
        // Still violating: the max cap that removes it, from the driver's (resized) LINK port, its
        // models at the violation's corner.
        if slack1 < 0.0 {
            let c1 = corner1.expect("a violation has a scene");
            if let (Some(cell), Some(port)) = (graphs[0].vertices[drvr].cell.as_deref(), graphs[0].vertices[drvr].port.as_deref()) {
                let link = inputs.libs.link_cell(cell).expect("a link cell");
                let scene_cell = inputs.libs.scene_cell(c1, cell).expect("a scene cell");
                max_cap = driver_slew::find_slew_load_cap(link, scene_cell, port, f64::from(max_slew1), ctx.sizing.tgt_slews, trace) as f32;
                trace.push(format!("slewcap|{}|{}", g9(f64::from(max_cap)), g9(f64::from(max_slew1))));
                corner = c1;
                repair_cap = true;
            }
        }
    }

    // Then the load slews (not a tristate driver: those are refused upstream). checkLoadSlews
    // writes the slew only when a load sets it; otherwise the driver check's stays.
    let sc = timing::Scenes::new(&graphs);
    let ls = timing::check_load_slews(&sc, design.net_info(), drvr, &inputs.limits, clocks, ctx.args.slew_margin);
    let viol_scene = corner_w_load_slew_viol;
    let annotated = viol_scene.is_some();
    let shown_slew = ls.slew.or(slew1).unwrap_or(0.0);
    trace.push(format!("lslew|{}|{}|{}|annot={annotated}", g9(f64::from(shown_slew)), g9(f64::from(ls.limit)), g9(f64::from(ls.slack))));
    let repair_load_slew = ls.slack < 0.0 || annotated;
    if repair_load_slew {
        slew_violation = true;
    }
    if slew_violation {
        outcome.slew_violations += 1;
    }
    if !repair_cap {
        if ls.slack < 0.0 {
            corner = ls.scene.expect("a violation has a scene");
        } else if let Some(k) = viol_scene {
            corner = k;
        }
    }

    // Then the capacitance: the pre-check first (an unreasonably small limit is an error).
    let parasitics: Vec<HashMap<String, vyges_sta::graph::NetParasitics>> = (0..graphs.len()).map(|k| design.parasitics(k).clone()).collect();
    let (cap1, max_cap1, _cap_slack1, has_corner, cap_scene) = timing::check_capacitance(&sc, drvr, &parasitics, clocks);
    if max_cap1 > 0.0 && max_cap1 < ctx.min_cap_load {
        return Err(Stop::error("RSZ-0169", format!("Max cap for driver {name} is unreasonably small {max_cap1:e}F. Min buffer or inverter input cap is {:e}F", ctx.min_cap_load)));
    }
    trace.push(format!("cap|{}|{}|{}|{has_corner}", g9(f64::from(cap1)), g9(f64::from(max_cap1)), g9(ctx.args.cap_margin)));
    // needRepairCap: the limit margined; a violation sets max_cap and the corner to its.
    if max_cap1 > 0.0 && has_corner {
        let margined = (f64::from(max_cap1) * (1.0 - ctx.args.cap_margin / 100.0)) as f32;
        if cap1 > margined {
            max_cap = margined;
            corner = cap_scene.expect("a check has a scene");
            outcome.cap_violations += 1;
            repair_cap = true;
        }
    }

    // makeBufferedNet(drvr_pin, corner) (placement parasitics: the Steiner tree); with none,
    // makeBufferedNetSteiner is called a second time. The rest runs at `corner`'s timer.
    trace.ran(&["stdrvr", "st", "flags", "bn"]);
    let g = &graphs[corner];
    let dk = sc.at(corner, drvr);
    let bnet = make_buffered_net(&*design, g, dk, &net_name, corner, ctx, trace).or_else(|| make_buffered_net(&*design, g, dk, &net_name, corner, ctx, trace));
    let Some((mut bn, root)) = bnet else {
        if repaired_net {
            outcome.nets_repaired += 1;
        }
        return Ok(());
    };
    let wire_length = bn.max_load_wire_length(root);
    let repair_wire = ctx.max_length > 0 && i64::from(wire_length) > ctx.max_length;
    if repair_wire {
        outcome.length_violations += 1;
    }
    trace.push(format!("flags|wl={wire_length}|max_length={}|repair_cap={repair_cap}|repair_load_slew={repair_load_slew}|repair_wire={repair_wire}|max_cap={}", ctx.max_length, g9(f64::from(max_cap))));
    buffered_net::trace_tree(&bn, root, 0, g, &mut trace.lines);
    if repair_cap || repair_load_slew || repair_wire {
        // RepairDesign::repairNet(bnet, drvr_pin, max_cap, max_length, corner).
        trace.ran(&["w", "wsplit", "wquad", "wbuf", "wexit", "j", "jexit", "l", "wpost", "rep", "rtsd", "rts", "repd"]);
        repaired_net = true;
        let bctx = buffered_net::Ctx { graph: g, libs: inputs.libs, sdc: &inputs.sdc, limits: &inputs.limits, dbu: inputs.dbu, rc: inputs.wire_rc[corner] };
        let walk = Walk {
            ctx: &bctx,
            sizing: ctx.sizing,
            drvr: dk,
            max_cap,
            max_length: ctx.max_length,
            slew_margin: ctx.args.slew_margin,
            slew_shape_factor: ctx.slew_shape_factor,
            r_strongest_buffer: ctx.r_strongest_buffer,
            buffer_lowest_drive: ctx.buffer_lowest_drive,
            sdc: &inputs.sdc,
            master_pins: &inputs.master_pins,
            corner,
        };
        let mut st = State { design, pre: std::mem::take(&mut ctx.pre), inserted: 0, last: None };
        let r = walk.repair_net(&mut bn, root, 0, &mut st, trace);
        ctx.pre = st.pre;
        outcome.inserted_buffers += st.inserted;
        r?;
    }
    if repaired_net {
        outcome.nets_repaired += 1;
    }
    Ok(())
}

/// The instance of an instance pin (`inst/port`).
fn inst_of(pin: &str) -> String {
    pin.rsplit_once('/').map_or(pin, |(i, _)| i).to_string()
}

/// `RepairDesign::repairDriverSlew(corner, drvr_pin)`: `ensureWireParasitic` (the driver's net is
/// ensured already) and `loadCap` at the corner, then — for an instance pin — the size to swap to.
fn repair_driver_slew(design: &dyn Design, graphs: &[Graph<'_>], corner: usize, drvr: usize, ctx: &NetCtx<'_, '_>, trace: &mut Trace) -> Result<Option<String>, Stop> {
    let sc = timing::Scenes::new(graphs);
    let g = &graphs[corner];
    let dk = sc.at(corner, drvr);
    let load_cap = g.load_cap(dk, design.parasitics(corner));
    let vx = &g.vertices[dk];
    trace.push(format!("rds|{}|{}|{}", vx.name, vx.cell.as_deref().unwrap_or("-"), g9(f64::from(load_cap))));
    let (Some(cell), Some(port), true) = (vx.cell.as_deref(), vx.port.as_deref(), vx.name.contains('/')) else { return Ok(None) };
    let inst = inst_of(&vx.name);
    let d = driver_slew::Driver {
        inst: &inst,
        cell,
        port,
        dont_touch: design.net_info().dont_touch_insts.contains(&inst),
        logic_std_cell: ctx.inputs.masters.get(cell).is_some_and(|m| m.logic_std),
    };
    let clocks = timing::clock_pins(g, ctx.inputs.clock_sources);
    driver_slew::repair_driver_slew(g, corner, &d, load_cap, ctx.sizing, &ctx.inputs.limits, ctx.args.slew_margin, &clocks, trace)
}

/// `Resizer::makeBufferedNetSteiner(drvr_pin, corner)`: the driver net's Steiner tree, printed,
/// then the buffered net built from it at the corner (`g` is its timer).
fn make_buffered_net(design: &dyn Design, g: &Graph<'_>, drvr: usize, net: &str, corner: usize, ctx: &NetCtx<'_, '_>, trace: &mut Trace) -> Option<(buffered_net::BufferedNet, usize)> {
    let tree = design.steiner(net, &g.vertices[drvr].name)?;
    buffered_net::trace_steiner(&tree, &g.vertices[drvr].name, &mut trace.lines);
    let bctx = buffered_net::Ctx { graph: g, libs: ctx.inputs.libs, sdc: &ctx.inputs.sdc, limits: &ctx.inputs.limits, dbu: ctx.inputs.dbu, rc: ctx.inputs.wire_rc[corner] };
    buffered_net::make_buffered_net_steiner(&bctx, &tree)
}

/// The timer's preambles: the graph over the netlist, its levels (`ensureLevelized`), refused
/// where this timer would level it differently from the reference.
fn timer_preambles<'n>(inputs: &'n Inputs<'_>, netlist: &'n vyges_sta::netlist::Netlist) -> Result<(Vec<Graph<'n>>, Vec<i32>), Stop> {
    if let Some(why) = timing::unlevelable(&inputs.libs.libs, netlist) {
        return Err(Stop::refused("RSZ-LEVELS", why));
    }
    let graphs = (0..inputs.libs.scene_count()).map(|k| timer_graph(inputs.libs, k, netlist, &inputs.sdc, &inputs.master_pins)).collect::<Result<Vec<_>, _>>()?;
    let level = graphs[0].levels().map_err(|e| Stop::refused("RSZ-TIMER", e))?;
    Ok((graphs, level))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rules (the status word): a changed design is `repaired`; checked drivers with nothing to
    // change are `up_to_date`; nothing checked is `vacuous` — never a pass word.
    #[test]
    fn the_status_word_is_settled_in_one_place() {
        let o = |checked, inserted, resized| Outcome { nets_checked: checked, inserted_buffers: inserted, resized, ..Default::default() };
        assert_eq!(o(0, 0, 0).settle(), "vacuous");
        assert_eq!(o(3, 0, 0).settle(), "up_to_date");
        assert_eq!(o(3, 2, 0).settle(), "repaired");
        assert_eq!(o(3, 0, 1).settle(), "repaired");
    }

    // Rule (reportViolationCounters): only non-zero counts, in the order slew, fanout, cap, long
    // wires, resized, inserted — the last naming the repaired-net count.
    #[test]
    fn the_summary_reports_non_zero_counts_in_order() {
        let o = Outcome { nets_repaired: 1, inserted_buffers: 3, slew_violations: 1, length_violations: 1, ..Default::default() };
        let codes: Vec<&str> = o.summary().iter().map(|(c, _)| *c).collect();
        assert_eq!(codes, ["RSZ-0034", "RSZ-0037", "RSZ-0038"]);
        assert_eq!(o.summary()[2].1, "Inserted 3 buffers in 1 nets.");
    }

    // Rule (Resizer::metersToDbu): meters × dbu × 1e6, ROUNDED (lround — RepairDesign's own
    // metersToDbu truncates instead), masked to a non-negative int.
    #[test]
    fn meters_to_dbu_rounds() {
        let m = 600.0 * f64::from(1e-6f32);
        assert_eq!(meters_to_dbu(m, 2000), 1_200_000);
        assert_eq!(meters_to_dbu(0.0, 2000), 0);
        assert_eq!(meters_to_dbu(2.5e-10, 2000), 1, "0.5 dbu rounds up");
    }
}
