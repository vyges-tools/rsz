// SPDX-License-Identifier: Apache-2.0
//! `RepairDesign::repairNet(bnet, …)`: the buffered net walked from its loads up — each visitor
//! as the reference writes it, every value in the reference's own type (the wire's Elmore slew in
//! double, the junction's in float, the slew quadratic's terms in float) — and `makeRepeater`.
//!
//! Load pins are carried by NAME: a repeater edits the design, and the pins it creates are not in
//! the timing graph the walk started from.

use vyges_sta::graph::Graph;
use vyges_sta::liberty::{FALL, MAX, RISE};

use crate::buffered_net::{BufferedNet, Ctx, Kind};
use crate::design::Design;
use crate::sizing::Sizing;
use crate::timing;
use crate::trace::{g9, Trace};
use crate::Stop;

/// The walk's fixed inputs (`RepairDesign`'s members while one net is repaired).
pub struct Walk<'a, 'g> {
    pub ctx: &'a Ctx<'a, 'g>,
    pub sizing: &'a Sizing<'a>,
    pub drvr: usize,
    /// `max_cap_`, `max_length_` (dbu), `slew_margin_` (percent).
    pub max_cap: f32,
    pub max_length: i64,
    pub slew_margin: f64,
    pub slew_shape_factor: f32,
    pub r_strongest_buffer: f32,
    pub buffer_lowest_drive: &'a str,
    /// The SDC environment every graph the walk builds reads.
    pub sdc: &'a vyges_sta::graph::SdcEnv,
    pub master_pins: &'a std::collections::HashMap<String, Vec<String>>,
    /// `corner_`: the scene the net is repaired at (`ctx.graph` is its timer).
    pub corner: usize,
}

/// `PreChecks`' best-case slew cache, kept across nets as the reference's member is.
#[derive(Debug, Clone, Default)]
pub struct PreCheck {
    computed: bool,
    load: f32,
    slew: f32,
}

/// What `repairNet(bnet, level, …)` returns: the wire length below this node and its load pins.
type Down = (i32, Vec<String>);

/// The walk's mutable state: the design it edits, the trace, the pre-check cache, the count of
/// repeaters inserted.
pub struct State<'d> {
    pub design: &'d mut dyn Design,
    pub pre: PreCheck,
    pub inserted: usize,
    /// The last repeater `makeRepeater` made (its out pin and net are the reference's outputs).
    pub last: Option<crate::design::Repeater>,
}

fn g(x: f64) -> String {
    g9(x)
}

impl Walk<'_, '_> {
    fn graph(&self) -> &Graph<'_> {
        self.ctx.graph
    }

    /// `maxSlewMargined`: a float limit times a double factor, returned as float.
    fn margined(&self, max_slew: f32) -> f32 {
        (f64::from(max_slew) * (1.0 - self.slew_margin / 100.0)) as f32
    }

    /// `RepairDesign::metersToDbu`: TRUNCATED to int (unlike `Resizer::metersToDbu`).
    fn meters_to_dbu(&self, dist: f64) -> i64 {
        (dist * f64::from(self.ctx.dbu) * 1e6) as i32 as i64
    }

    /// `Resizer::driveResistance(drvr_pin)`: an instance pin's port drive resistance; a top-level
    /// port's input drive (none is modelled: 0).
    fn drive_resistance(&self) -> f32 {
        let vx = &self.graph().vertices[self.drvr];
        match (vx.cell.as_deref(), vx.port.as_deref()) {
            // The LINK port's (`network_->libertyPort(pin)->driveResistance()`).
            (Some(c), Some(p)) => self.sizing.libs.link_cell(c).map_or(0.0, |cell| cell.drive_resistance(p)),
            // A top-level port: its driving cell's `-pin` port's (the larger over rise / fall and
            // min / max, all one value here); none ⇒ 0.
            _ => match self.graph().sdc.input_drive.get(&vx.name) {
                Some(d) => self.sizing.libs.link_cell(&d.cell).map_or(0.0, |cell| cell.drive_resistance(&d.to_port)),
                None => 0.0,
            },
        }
    }

    /// `repairNet(bnet, level, …)`: dispatch by node type.
    pub fn repair_net(&self, bn: &mut BufferedNet, n: usize, level: usize, st: &mut State<'_>, trace: &mut Trace) -> Result<Down, Stop> {
        match bn.nodes[n].kind.clone() {
            Kind::Wire { r } => self.repair_net_wire(bn, n, r, level, st, trace),
            Kind::Junction { r, r2 } => self.repair_net_junc(bn, n, r, r2, level, st, trace),
            Kind::Load { pin } => self.repair_net_load(bn, n, pin, level, st, trace),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn repair_net_wire(&self, bn: &mut BufferedNet, n: usize, r: usize, level: usize, st: &mut State<'_>, trace: &mut Trace) -> Result<Down, Stop> {
        let (mut wire_length_ref, mut load_pins) = self.repair_net(bn, r, level + 1, st, trace)?;
        let mut max_load_slew = bn.nodes[r].max_load_slew;
        let mut max_load_slew_margined = self.margined(max_load_slew);
        let (mut to_x, mut to_y) = bn.location(r);
        let (from_x, from_y) = bn.location(n);
        let mut length = (from_x - to_x).abs() + (from_y - to_y).abs();
        let mut wire_length = wire_length_ref + length;
        let mut length1 = self.ctx.dbu_to_meters(length);
        let (wire_res, wire_cap) = bn.wire_rc(n, self.ctx);
        let mut ref_cap = f64::from(bn.nodes[r].cap);
        let mut load_cap = length1 * wire_cap + ref_cap;
        let r_drvr = self.drive_resistance().max(self.r_strongest_buffer);
        let mut r_wire = length1 * wire_res;
        let mut c_wire = length1 * wire_cap;
        let shape = f64::from(self.slew_shape_factor);
        let mut load_slew = (f64::from(r_drvr) * (c_wire + ref_cap) + r_wire * ref_cap + r_wire * c_wire / 2.0) * shape;
        let mut buffer_cell = self.sizing.find_target_cell(self.buffer_lowest_drive, load_cap as f32, false)?;
        trace.push(format!(
            "w|{level}|{from_x}|{from_y}|{to_x}|{to_y}|len={length}|wl={wire_length}|wl_ref={wire_length_ref}|res={}|cap={}|ref_cap={}|load_cap={}|r_drvr={}|load_slew={}|mls={}|mlsm={}|cell={buffer_cell}",
            g(wire_res), g(wire_cap), g(ref_cap), g(load_cap), g(f64::from(r_drvr)), g(load_slew), g(f64::from(max_load_slew)), g(f64::from(max_load_slew_margined))
        ));
        bn.nodes[n].cap = load_cap as f32;
        bn.nodes[n].fanout = bn.nodes[r].fanout;
        bn.nodes[n].max_load_slew = (f64::from(bn.nodes[r].max_load_slew) - r_wire * (c_wire / 2.0 + ref_cap) * shape) as f32;

        // Back up from the load toward the driver, a repeater wherever a limit is broken.
        let max_cap = f64::from(self.max_cap);
        let mut zero_progress_iters = 0;
        while (self.max_length > 0 && i64::from(wire_length) > self.max_length) || (wire_cap > 0.0 && max_cap > 0.0 && load_cap > max_cap) || load_slew > f64::from(max_load_slew_margined) {
            const LENGTH_MARGIN: f64 = 0.05;
            let mut split_wire = false;
            let mut split_length: i64 = i64::from(i32::MAX);
            if self.max_length > 0 && i64::from(wire_length) > self.max_length {
                split_length = (self.max_length - i64::from(wire_length_ref)).max(0).min(i64::from(length / 2));
                split_wire = true;
            }
            if wire_cap > 0.0 && load_cap > max_cap {
                split_length = split_length.min(self.meters_to_dbu((max_cap - ref_cap) / wire_cap).max(0));
                split_wire = true;
            }
            if load_slew > f64::from(max_load_slew_margined) {
                // The quadratic's terms are declared float; `sqrt` of the float discriminant is the
                // FLOAT overload (both reference roots in the corpus reproduce only this way).
                let a = (wire_res * wire_cap * shape / 2.0) as f32;
                let b = (f64::from(r_drvr) * wire_cap + wire_res * ref_cap * shape) as f32;
                let c = (f64::from(r_drvr) * ref_cap - f64::from(max_load_slew_margined)) as f32;
                let mut l = 0.0f32;
                if a > 1e-12 {
                    let discriminant = b * b - 4.0 * a * c;
                    if discriminant >= 0.0 {
                        l = (-b + discriminant.sqrt()) / (2.0 * a);
                    }
                } else if b > 1e-12 {
                    l = -c / b;
                }
                trace.push(format!("wquad|{}|{}|{}|{}", g(f64::from(a)), g(f64::from(b)), g(f64::from(c)), g(f64::from(l))));
                if l >= 0.0 {
                    split_length = split_length.min(self.meters_to_dbu(f64::from(l)));
                } else {
                    split_length = 0;
                }
                split_wire = true;
            }
            trace.push(format!(
                "wsplit|{level}|lenv={}|capv={}|slewv={}|split={split_length}",
                self.max_length > 0 && i64::from(wire_length) > self.max_length,
                wire_cap > 0.0 && load_cap > max_cap,
                load_slew > f64::from(max_load_slew_margined)
            ));
            if !split_wire {
                break;
            }
            let buf_dist = if split_length >= i64::from(length) { f64::from(length) } else { split_length as f64 * (1.0 - LENGTH_MARGIN) };
            let prev_ref_cap = ref_cap;
            let zero_advance = buf_dist < 1.0;
            let dx = f64::from(from_x - to_x);
            let dy = f64::from(from_y - to_y);
            let d = if length == 0 { 0.0 } else { buf_dist / f64::from(length) };
            let buf_x = (f64::from(to_x) + d * dx) as i32;
            let buf_y = (f64::from(to_y) + d * dy) as i32;
            trace.push(format!("wbuf|{level}|buf_dist={}|{buf_x}|{buf_y}", g(buf_dist)));
            let (repeater_cap, repeater_fanout, repeater_max_slew) = self.make_repeater("wire", buf_x, buf_y, &buffer_cell, true, level, &mut load_pins, st, trace)?;
            max_load_slew = repeater_max_slew;
            // Update for the next round (`length -= buf_dist`: an int minus a double, truncated).
            length = shorten(length, buf_dist);
            wire_length = length;
            to_x = buf_x;
            to_y = buf_y;
            length1 = self.ctx.dbu_to_meters(length);
            wire_length_ref = 0;
            load_cap = f64::from(repeater_cap) + length1 * wire_cap;
            ref_cap = f64::from(repeater_cap);
            max_load_slew_margined = self.margined(max_load_slew);
            r_wire = length1 * wire_res;
            c_wire = length1 * wire_cap;
            load_slew = (f64::from(r_drvr) * (c_wire + ref_cap) + r_wire * ref_cap + r_wire * c_wire / 2.0) * shape;
            buffer_cell = self.sizing.find_target_cell(self.buffer_lowest_drive, load_cap as f32, false)?;
            bn.nodes[n].cap = load_cap as f32;
            bn.nodes[n].fanout = repeater_fanout;
            trace.push(format!(
                "wpost|{level}|len={length}|load_cap={}|ref_cap={}|load_slew={}|mlsm={}|cell={buffer_cell}|rfo={}|mls={}",
                g(load_cap), g(ref_cap), g(load_slew), g(f64::from(max_load_slew_margined)), g(f64::from(repeater_fanout)), g(f64::from(max_load_slew))
            ));
            bn.nodes[n].max_load_slew = (f64::from(max_load_slew) - r_wire * (c_wire / 2.0 + ref_cap) * shape) as f32;
            // No progress twice in a row (a repeater on the load, no smaller load seen): RSZ-170.
            if zero_advance && ref_cap >= prev_ref_cap {
                zero_progress_iters += 1;
                if zero_progress_iters >= 2 {
                    break;
                }
            } else {
                zero_progress_iters = 0;
            }
        }
        trace.push(format!("wexit|{level}|{}|{}|{}", g(f64::from(bn.nodes[n].cap)), g(f64::from(bn.nodes[n].fanout)), g(f64::from(bn.nodes[n].max_load_slew))));
        Ok((wire_length, load_pins))
    }

    #[allow(clippy::too_many_arguments)]
    fn repair_net_junc(&self, bn: &mut BufferedNet, n: usize, left: usize, right: usize, level: usize, st: &mut State<'_>, trace: &mut Trace) -> Result<Down, Stop> {
        let (x, y) = bn.location(n);
        let (mut wire_length_left, mut loads_left) = self.repair_net(bn, left, level + 1, st, trace)?;
        let (mut cap_left, mut fanout_left, mut max_load_slew_left) = (bn.nodes[left].cap, bn.nodes[left].fanout, bn.nodes[left].max_load_slew);
        let (mut wire_length_right, mut loads_right) = self.repair_net(bn, right, level + 1, st, trace)?;
        let (mut cap_right, mut fanout_right, mut max_load_slew_right) = (bn.nodes[right].cap, bn.nodes[right].fanout, bn.nodes[right].max_load_slew);
        let load_cap = cap_left + cap_right;
        let max_load_slew = max_load_slew_left.min(max_load_slew_right);
        let max_load_slew_margined = self.margined(max_load_slew);
        let buffer_cell = self.sizing.find_target_cell(self.buffer_lowest_drive, load_cap, false)?;
        let (mut repeater_left, mut repeater_right) = (false, false);
        // The junction does NOT clamp the drive resistance (the wire does).
        let r_drvr = self.drive_resistance();
        let load_slew = r_drvr * load_cap * self.slew_shape_factor;
        let mut reason: Option<&str> = None;
        if load_slew > max_load_slew_margined {
            let slew_left = f64::from(r_drvr * cap_left * self.slew_shape_factor);
            let slew_slack_left = f64::from(self.margined(max_load_slew_left)) - slew_left;
            let slew_right = f64::from(r_drvr * cap_right * self.slew_shape_factor);
            let slew_slack_right = f64::from(self.margined(max_load_slew_right)) - slew_right;
            if slew_slack_left < slew_slack_right {
                repeater_left = true;
            } else {
                repeater_right = true;
            }
            reason = Some("load_slew");
        }
        // No `max_cap > 0` guard here (the wire has one).
        if cap_left + cap_right > self.max_cap {
            if cap_left > cap_right {
                repeater_left = true;
            } else {
                repeater_right = true;
            }
            reason = Some("max_cap");
        }
        if self.max_length > 0 && i64::from(wire_length_left + wire_length_right) > self.max_length {
            if wire_length_left > wire_length_right {
                repeater_left = true;
            } else {
                repeater_right = true;
            }
            reason = Some("max_length");
        }
        trace.push(format!(
            "j|{level}|{x}|{y}|cl={}|cr={}|fl={}|fr={}|ml={}|mr={}|wl={wire_length_left}|wr={wire_length_right}|ls={}|mlsm={}|rl={repeater_left}|rr={repeater_right}|reason={}|cell={buffer_cell}",
            g(f64::from(cap_left)), g(f64::from(cap_right)), g(f64::from(fanout_left)), g(f64::from(fanout_right)), g(f64::from(max_load_slew_left)), g(f64::from(max_load_slew_right)),
            g(f64::from(load_slew)), g(f64::from(max_load_slew_margined)), reason.unwrap_or("-")
        ));
        // makeRepeater writes the branch's cap, fanout and slew limit (its by-reference outputs).
        let reason_s = reason.unwrap_or("-");
        if repeater_left {
            (cap_left, fanout_left, max_load_slew_left) = self.make_repeater(reason_s, x, y, &buffer_cell, true, level, &mut loads_left, st, trace)?;
            wire_length_left = 0;
        }
        if repeater_right {
            (cap_right, fanout_right, max_load_slew_right) = self.make_repeater(reason_s, x, y, &buffer_cell, true, level, &mut loads_right, st, trace)?;
            wire_length_right = 0;
        }
        let wire_length = wire_length_left.max(wire_length_right);
        bn.nodes[n].cap = cap_left + cap_right;
        bn.nodes[n].fanout = fanout_right + fanout_left;
        bn.nodes[n].max_load_slew = max_load_slew_left.min(max_load_slew_right);
        trace.push(format!("jexit|{level}|{}|{}|{}|wl={wire_length}", g(f64::from(bn.nodes[n].cap)), g(f64::from(bn.nodes[n].fanout)), g(f64::from(bn.nodes[n].max_load_slew))));
        let mut loads = loads_left;
        loads.append(&mut loads_right);
        Ok((wire_length, loads))
    }

    fn repair_net_load(&self, bn: &mut BufferedNet, n: usize, pin: usize, level: usize, st: &mut State<'_>, trace: &mut Trace) -> Result<Down, Stop> {
        self.check_slew_limit(bn.nodes[n].cap, bn.nodes[n].max_load_slew, &mut st.pre)?;
        let name = self.graph().vertices[pin].name.clone();
        trace.push(format!("l|{level}|{name}|{}|{}", g(f64::from(bn.nodes[n].cap)), g(f64::from(bn.nodes[n].max_load_slew))));
        Ok((0, vec![name]))
    }

    /// `RepairDesign::makeRepeater(reason, x, y, cell, resize, level, load_pins, …)`: the buffer
    /// inserted before the load pins (`insertBufferBeforeLoads`, then the location clamped to the
    /// core), resized to the load it now drives (`resizeToTargetSlew`) when `resize`; the load pins
    /// become its input; returns its input cap, fanout load and slew limit (and records the
    /// repeater in `st.last`).
    #[allow(clippy::too_many_arguments)]
    pub fn make_repeater(&self, reason: &str, x: i32, y: i32, buffer_cell: &str, resize: bool, level: usize, load_pins: &mut Vec<String>, st: &mut State<'_>, trace: &mut Trace) -> Result<(f32, f32, f32), Stop> {
        let loads: String = load_pins.iter().map(|p| format!("{p},")).collect();
        trace.push(format!("rep|{level}|{reason}|{x}|{y}|{buffer_cell}|{loads}"));
        let r = st.design.insert_repeater(load_pins, buffer_cell, (x, y), reason).map_err(|e| Stop::error("RSZ-INSERT", e))?;
        st.inserted += 1;
        let cell = if resize { self.resize_to_target_slew(&r, buffer_cell, st, trace)? } else { buffer_cell.to_string() };
        let (input, repeater_cap, repeater_fanout, repeater_max_slew) = repeater_values(self.sizing.libs, &cell, self.ctx.limits, self.corner).expect("a repeater is a liberty buffer");
        let (lx, ly) = st.design.inst_location(&r.inst);
        trace.push(format!("repd|{}|{}|{cell}|{lx}|{ly}|{}|{}|{}", r.inst, r.out_net, g(f64::from(repeater_cap)), g(f64::from(repeater_fanout)), g(f64::from(repeater_max_slew))));
        *load_pins = vec![format!("{}/{input}", r.inst)];
        st.last = Some(r);
        Ok((repeater_cap, repeater_fanout, repeater_max_slew))
    }

    /// `Resizer::resizeToTargetSlew(repeater output)` under placement parasitics: the new net's
    /// parasitic ensured, its load (`loadCap`) at the target-slew corner, `findTargetCell`, and a
    /// master swap when the target differs.
    fn resize_to_target_slew(&self, r: &crate::design::Repeater, cell: &str, st: &mut State<'_>, trace: &mut Trace) -> Result<String, Stop> {
        st.design.ensure_wire_parasitic(&r.out_net).map_err(|e| Stop::error("RSZ-EST", e))?;
        let nl = st.design.netlist().clone();
        // `loadCap(drvr_pin, tgt_slew_corner_, max)`: the target-slew corner's timer and parasitics.
        let k = self.sizing.tgt_scene;
        let par = st.design.parasitics(k).clone();
        let graph = crate::repair_design::timer_graph(self.sizing.libs, k, &nl, self.sdc, self.master_pins)?;
        let v = graph.vertices.iter().position(|x| x.name == r.output).ok_or_else(|| Stop::error("RSZ-INSERT", format!("{}: not in the timing graph", r.output)))?;
        let (pin_cap, wire_cap, has_pi) = graph.load_cap_parts(v, &par, RISE);
        trace.push(format!("rtsd|{}|pin={}|wire={}|pi={has_pi}", r.output, g(f64::from(pin_cap)), g(f64::from(wire_cap))));
        let load_cap = graph.load_cap(v, &par);
        let mut current = cell.to_string();
        if load_cap > 0.0 {
            let target = self.sizing.find_target_cell(cell, load_cap, false)?;
            trace.push(format!("rts|{}|{cell}|{}|{target}", r.output, g(f64::from(load_cap))));
            if target != cell {
                st.design.swap_master(&r.inst, &target).map_err(|e| Stop::error("RSZ-INSERT", e))?;
                current = target;
            }
        }
        Ok(current)
    }

    /// `PreChecks::checkSlewLimit(ref_cap, max_load_slew)`: the best slew any swappable size of
    /// the weakest buffer reaches at this load — cached, and recomputed only for a smaller load —
    /// above the limit is RSZ-0090.
    fn check_slew_limit(&self, ref_cap: f32, max_load_slew: f32, pre: &mut PreCheck) -> Result<(), Stop> {
        if !pre.computed || ref_cap < pre.load {
            let swappable = self.sizing.swappable_cells(self.buffer_lowest_drive)?;
            let slew_of = |name: &str| -> f32 {
                let cell = self.sizing.libs.link_cell(name).expect("a link cell");
                buffer_slew(self.sizing, cell, ref_cap)
            };
            let mut slew = slew_of(self.buffer_lowest_drive);
            for b in &swappable {
                slew = slew.min(slew_of(b));
            }
            *pre = PreCheck { computed: true, load: ref_cap, slew };
        }
        if max_load_slew < pre.slew {
            return Err(Stop::error("RSZ-0090", format!("Max transition time from SDC is {max_load_slew:e}s. Best achievable transition time is {:e}s with a load of {:e}F", pre.slew, pre.load)));
        }
        Ok(())
    }
}

/// `length -= buf_dist`: an `int` less a `double`, converted back to `int` — truncated toward zero.
fn shorten(length: i32, buf_dist: f64) -> i32 {
    (f64::from(length) - buf_dist) as i32
}

/// What `makeRepeater` returns for a repeater of `cell` (after its resize), at the net's corner:
/// the input pin's name, `portCapacitance(input, corner_)` (the CORNER port's
/// `LibertyPort::capacitance()`: the MAX-side value, the larger of rise and fall — never a min-side
/// one), `portFanoutLoad` (the LINK port's `fanout_load`, else its library's `default_fanout_load`,
/// else 0) and `bufferInputMaxSlew(cell, corner_)` (`maxInputSlew`: the corner port's limit, the
/// link library's default).
pub fn repeater_values(libs: &crate::preamble::Libs, cell: &str, limits: &timing::Limits, corner: usize) -> Option<(String, f32, f32, f32)> {
    let c = libs.link_cell(cell)?;
    let library = libs.link_library(cell)?;
    let (input, _) = c.buffer_ports()?;
    let scene_lib = libs.scene_library(corner, cell)?;
    let scene_input = libs.scene_cell(corner, cell)?.port(&input.name)?;
    let cap = scene_input.capacitance[RISE][MAX].max(scene_input.capacitance[FALL][MAX]);
    let fanout = input.fanout_load.or(library.default_fanout_load).unwrap_or(0.0);
    let max_slew = timing::max_input_slew_at(library, scene_lib, scene_input.direction, scene_input.max_transition, limits);
    Some((input.name.clone(), cap, fanout, max_slew))
}

/// `Resizer::bufferSlew(cell, load_cap, tgt_slew_corner, max)`: over the arcs into the output,
/// each at its input transition's target slew and the lumped load, the larger slew per output
/// transition (from −INF); then the larger of rise and fall.
pub fn buffer_slew(sizing: &Sizing<'_>, cell: &vyges_sta::liberty::Cell, load_cap: f32) -> f32 {
    use vyges_sta::liberty::Model;
    // At the target-slew corner (`tgt_slew_corner_`).
    let cell = sizing.libs.scene_cell(sizing.tgt_scene, &cell.name).unwrap_or(cell);
    let Some((_, output)) = cell.buffer_ports() else { return -crate::timing::INF };
    let mut slews = [-crate::timing::INF; 2];
    for set in cell.arc_sets.iter().filter(|s| s.to == output.name && !s.role.is_timing_check()) {
        for arc in &set.arcs {
            if let Model::Gate(m) = &arc.model {
                let (_, slew) = m.gate_delay(sizing.tgt_slews[arc.from_rf], load_cap);
                slews[arc.to_rf] = slews[arc.to_rf].max(slew);
            }
        }
    }
    slews[0].max(slews[1])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preamble::Libs;
    use vyges_sta::liberty::Library;
    use vyges_sta::liberty_parse::parse as lparse;

    fn libs(lib_attrs: &str, pin_a: &str) -> Libs {
        let text = format!(
            r#"library (t) {{ time_unit : "1ns" ; capacitive_load_unit (1, ff) ; {lib_attrs}
              cell (BUF) {{ pin (A) {{ direction : input ; {pin_a} }} pin (Z) {{ direction : output ; function : "A" ; }} }} }}"#
        );
        Libs { libs: vec![Library::read(&lparse(&text).unwrap()).unwrap()], ..Default::default() }
    }

    // Rule (RepairDesign::makeRepeater → Resizer::portCapacitance → LibertyPort::capacitance()):
    // the MAX-side capacitance, the larger of rise and fall — a larger MIN-side value is not it.
    #[test]
    fn repeater_cap_is_the_max_side_value() {
        let l = libs("", "rise_capacitance_range (5, 1) ; fall_capacitance_range (4, 2) ;");
        let (pin, cap, ..) = repeater_values(&l, "BUF", &timing::Limits::default(), 0).unwrap();
        assert_eq!((pin.as_str(), cap), ("A", 2e-15));
    }

    // Rule (Resizer::portFanoutLoad): the port's fanout_load, else the library's
    // default_fanout_load, else 0.
    #[test]
    fn repeater_fanout_falls_back_to_the_library_default_then_zero() {
        let f = |lib: &str, pin: &str| repeater_values(&libs(lib, pin), "BUF", &timing::Limits::default(), 0).unwrap().2;
        assert_eq!(f("default_fanout_load : 2 ;", "fanout_load : 3 ;"), 3.0);
        assert_eq!(f("default_fanout_load : 2 ;", ""), 2.0);
        assert_eq!(f("", ""), 0.0);
    }

    // Rule (repairNetWire: `length -= buf_dist`): an int less a double, back to int — truncated,
    // so a repeater 95% along a 1001-dbu wire leaves 50, not 51.
    #[test]
    fn the_remaining_length_is_truncated() {
        assert_eq!(shorten(1001, 950.95), 50);
        assert_eq!(shorten(100, 100.0), 0);
        assert_eq!(shorten(7, 0.5), 6);
    }
}
