// SPDX-License-Identifier: Apache-2.0
//! `RepairDesign::repairNet(bnet, …)`: the buffered net walked from its loads up — each visitor
//! as the reference writes it, every value in the reference's own type (the wire's Elmore slew in
//! double, the junction's in float, the slew quadratic's terms in float) — and `makeRepeater`.
//!
//! Load pins are carried by NAME: a repeater edits the design, and the pins it creates are not in
//! the timing graph the walk started from.

use std::collections::HashMap;

use vyges_sta::graph::{Graph, NetParasitics};
use vyges_sta::netlist::{Conn, Net, Netlist};
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
    /// `buffer_cells_`: the pruned buffer list.
    pub buffer_cells: &'a [String],
    /// The SDC environment every graph the walk builds reads.
    pub sdc: &'a vyges_sta::graph::SdcEnv,
    pub master_pins: &'a std::collections::HashMap<String, Vec<String>>,
    /// `corner_`: the scene the net is repaired at (`ctx.graph` is its timer).
    pub corner: usize,
}

/// `PreChecks`' best-case slew cache, kept across nets as the reference's member is: one per
/// corner (`slew_check_by_corner_`, upstream 77b7867bfd6), so one corner's best slew never judges
/// another's limit.
#[derive(Debug, Clone, Default)]
pub struct PreCheck {
    by_corner: std::collections::BTreeMap<usize, SlewCheckCache>,
}

/// `PreChecks::SlewCheckCache`.
#[derive(Debug, Clone, Copy, Default)]
struct SlewCheckCache {
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

/// `repairNetVia`'s max load slew: the ref's, less the via's RC slew — `float r_via`, so the
/// double resistance is narrowed first and the product is float throughout.
fn via_max_load_slew(ref_max_load_slew: f32, r_via: f64, ref_cap: f32, slew_shape_factor: f32) -> f32 {
    ref_max_load_slew - (r_via as f32) * ref_cap * slew_shape_factor
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
            Kind::Via { r, .. } => self.repair_net_via(bn, n, r, level, st, trace),
        }
    }

    /// `repairNetVia` (a global route's buffered net): its ref repaired, the ref's cap and fanout
    /// taken, its max load slew less the via's own RC slew — float throughout (`float r_via`).
    fn repair_net_via(&self, bn: &mut BufferedNet, n: usize, r: usize, level: usize, st: &mut State<'_>, trace: &mut Trace) -> Result<Down, Stop> {
        let down = self.repair_net(bn, r, level + 1, st, trace)?;
        bn.nodes[n].cap = bn.nodes[r].cap;
        bn.nodes[n].fanout = bn.nodes[r].fanout;
        let r_via = bn.via_resistance(n).map_err(|e| Stop::error("RSZ-0093", e))?;
        bn.nodes[n].max_load_slew = via_max_load_slew(bn.nodes[r].max_load_slew, r_via, bn.nodes[r].cap, self.slew_shape_factor);
        Ok(down)
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
        crate::tpush!(trace, 
            "w|{level}|{from_x}|{from_y}|{to_x}|{to_y}|len={length}|wl={wire_length}|wl_ref={wire_length_ref}|res={}|cap={}|ref_cap={}|load_cap={}|r_drvr={}|load_slew={}|mls={}|mlsm={}|cell={buffer_cell}",
            g(wire_res), g(wire_cap), g(ref_cap), g(load_cap), g(f64::from(r_drvr)), g(load_slew), g(f64::from(max_load_slew)), g(f64::from(max_load_slew_margined))
        );
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
                crate::tpush!(trace, "wquad|{}|{}|{}|{}", g(f64::from(a)), g(f64::from(b)), g(f64::from(c)), g(f64::from(l)));
                if l >= 0.0 {
                    split_length = split_length.min(self.meters_to_dbu(f64::from(l)));
                } else {
                    split_length = 0;
                }
                split_wire = true;
            }
            crate::tpush!(trace, 
                "wsplit|{level}|lenv={}|capv={}|slewv={}|split={split_length}",
                self.max_length > 0 && i64::from(wire_length) > self.max_length,
                wire_cap > 0.0 && load_cap > max_cap,
                load_slew > f64::from(max_load_slew_margined)
            );
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
            crate::tpush!(trace, "wbuf|{level}|buf_dist={}|{buf_x}|{buf_y}", g(buf_dist));
            let hint = wire_load_cap_hint(buf_dist, self.ctx.dbu, wire_cap, ref_cap);
            let (repeater_cap, repeater_fanout, repeater_max_slew) = self.make_repeater("wire", buf_x, buf_y, &buffer_cell, true, level, &mut load_pins, Some(hint), st, trace)?;
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
            crate::tpush!(trace, 
                "wpost|{level}|len={length}|load_cap={}|ref_cap={}|load_slew={}|mlsm={}|cell={buffer_cell}|rfo={}|mls={}",
                g(load_cap), g(ref_cap), g(load_slew), g(f64::from(max_load_slew_margined)), g(f64::from(repeater_fanout)), g(f64::from(max_load_slew))
            );
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
        crate::tpush!(trace, "wexit|{level}|{}|{}|{}", g(f64::from(bn.nodes[n].cap)), g(f64::from(bn.nodes[n].fanout)), g(f64::from(bn.nodes[n].max_load_slew)));
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
        crate::tpush!(trace, 
            "j|{level}|{x}|{y}|cl={}|cr={}|fl={}|fr={}|ml={}|mr={}|wl={wire_length_left}|wr={wire_length_right}|ls={}|mlsm={}|rl={repeater_left}|rr={repeater_right}|reason={}|cell={buffer_cell}",
            g(f64::from(cap_left)), g(f64::from(cap_right)), g(f64::from(fanout_left)), g(f64::from(fanout_right)), g(f64::from(max_load_slew_left)), g(f64::from(max_load_slew_right)),
            g(f64::from(load_slew)), g(f64::from(max_load_slew_margined)), reason.unwrap_or("-")
        );
        // makeRepeater writes the branch's cap, fanout and slew limit (its by-reference outputs).
        let reason_s = reason.unwrap_or("-");
        if repeater_left {
            (cap_left, fanout_left, max_load_slew_left) = self.make_repeater(reason_s, x, y, &buffer_cell, true, level, &mut loads_left, Some(cap_left), st, trace)?;
            wire_length_left = 0;
        }
        if repeater_right {
            (cap_right, fanout_right, max_load_slew_right) = self.make_repeater(reason_s, x, y, &buffer_cell, true, level, &mut loads_right, Some(cap_right), st, trace)?;
            wire_length_right = 0;
        }
        let wire_length = wire_length_left.max(wire_length_right);
        bn.nodes[n].cap = cap_left + cap_right;
        bn.nodes[n].fanout = fanout_right + fanout_left;
        bn.nodes[n].max_load_slew = max_load_slew_left.min(max_load_slew_right);
        crate::tpush!(trace, "jexit|{level}|{}|{}|{}|wl={wire_length}", g(f64::from(bn.nodes[n].cap)), g(f64::from(bn.nodes[n].fanout)), g(f64::from(bn.nodes[n].max_load_slew)));
        let mut loads = loads_left;
        loads.append(&mut loads_right);
        Ok((wire_length, loads))
    }

    fn repair_net_load(&self, bn: &mut BufferedNet, n: usize, pin: usize, level: usize, st: &mut State<'_>, trace: &mut Trace) -> Result<Down, Stop> {
        let name = self.graph().vertices[pin].name.clone();
        self.check_slew_limit(&name, bn.nodes[n].cap, bn.nodes[n].max_load_slew, self.corner, &mut st.pre)?;
        crate::tpush!(trace, "l|{level}|{name}|{}|{}", g(f64::from(bn.nodes[n].cap)), g(f64::from(bn.nodes[n].max_load_slew)));
        Ok((0, vec![name]))
    }

    /// `RepairDesign::makeRepeater(reason, x, y, cell, resize, level, load_pins, …)`: the buffer
    /// inserted before the load pins (`insertBufferBeforeLoads`, then the location clamped to the
    /// core), resized to the load it now drives (`resizeToTargetSlew`) when `resize`; the load pins
    /// become its input; returns its input cap, fanout load and slew limit (and records the
    /// repeater in `st.last`).
    #[allow(clippy::too_many_arguments)]
    pub fn make_repeater(&self, reason: &str, x: i32, y: i32, buffer_cell: &str, resize: bool, level: usize, load_pins: &mut Vec<String>, load_cap_hint: Option<f32>, st: &mut State<'_>, trace: &mut Trace) -> Result<(f32, f32, f32), Stop> {
        let loads: String = load_pins.iter().map(|p| format!("{p},")).collect();
        crate::tpush!(trace, "rep|{level}|{reason}|{x}|{y}|{buffer_cell}|{loads}");
        let r = st.design.insert_repeater(load_pins, buffer_cell, (x, y), reason).map_err(|e| Stop::error("RSZ-INSERT", e))?;
        st.inserted += 1;
        let cell = if resize { self.resize_to_target_slew(&r, buffer_cell, load_cap_hint, st, trace)? } else { buffer_cell.to_string() };
        let (input, repeater_cap, repeater_fanout, repeater_max_slew) = repeater_values(self.sizing.libs, &cell, self.ctx.limits, self.corner).expect("a repeater is a liberty buffer");
        let (lx, ly) = st.design.inst_location(&r.inst);
        crate::tpush!(trace, "repd|{}|{}|{cell}|{lx}|{ly}|{}|{}|{}", r.inst, r.out_net, g(f64::from(repeater_cap)), g(f64::from(repeater_fanout)), g(f64::from(repeater_max_slew)));
        *load_pins = vec![format!("{}/{input}", r.inst)];
        st.last = Some(r);
        Ok((repeater_cap, repeater_fanout, repeater_max_slew))
    }

    /// `Resizer::resizeToTargetSlew(repeater output, load_cap_hint)`: its load the caller's hint
    /// under global-route parasitics, else the new net's parasitic ensured and its load (`loadCap`)
    /// at the target-slew corner; `findTargetCell`, and a master swap when the target differs.
    fn resize_to_target_slew(&self, r: &crate::design::Repeater, cell: &str, load_cap_hint: Option<f32>, st: &mut State<'_>, trace: &mut Trace) -> Result<String, Stop> {
        let load_cap = match hinted_load(load_cap_hint, st.design.global_routing_parasitics()) {
            Some(hint) => hint,
            None => self.estimated_load_cap(r, st, trace)?,
        };
        let mut current = cell.to_string();
        if load_cap > 0.0 {
            let target = self.sizing.find_target_cell(cell, load_cap, false)?;
            crate::tpush!(trace, "rts|{}|{cell}|{}|{target}", r.output, g(f64::from(load_cap)));
            if target != cell {
                st.design.swap_master(&r.inst, &target).map_err(|e| Stop::error("RSZ-INSERT", e))?;
                current = target;
            }
        }
        Ok(current)
    }

    /// `ensureWireParasitic(drvr_pin)` then `loadCap(drvr_pin, tgt_slew_corner_, max_)`.
    fn estimated_load_cap(&self, r: &crate::design::Repeater, st: &mut State<'_>, trace: &mut Trace) -> Result<f32, Stop> {
        st.design.ensure_wire_parasitic(&r.out_net).map_err(|e| Stop::error("RSZ-EST", e))?;
        // `loadCap(drvr_pin, tgt_slew_corner_, max)`: the target-slew corner's timer and parasitics.
        // 🔑 It reads the driver's net only (its pins' caps, the SDC on its pins, its parasitic), so
        // the graph is made over that net alone and its own parasitic — not the whole design, which
        // made every repeater an O(design) graph build.
        let k = self.sizing.tgt_scene;
        let nl = net_alone(st.design.netlist(), &r.out_net).ok_or_else(|| Stop::error("RSZ-INSERT", format!("{}: no such net", r.out_net)))?;
        let par: HashMap<String, NetParasitics> = st.design.parasitics(k).get(&r.out_net).map(|p| (r.out_net.clone(), p.clone())).into_iter().collect();
        let graph = crate::repair_design::timer_graph(self.sizing.libs, k, &nl, self.sdc, self.master_pins)?;
        let v = graph.vertices.iter().position(|x| x.name == r.output).ok_or_else(|| Stop::error("RSZ-INSERT", format!("{}: not in the timing graph", r.output)))?;
        let (pin_cap, wire_cap, has_pi) = graph.load_cap_parts(v, &par, RISE);
        crate::tpush!(trace, "rtsd|{}|pin={}|wire={}|pi={has_pi}", r.output, g(f64::from(pin_cap)), g(f64::from(wire_cap)));
        Ok(graph.load_cap(v, &par))
    }

    /// `PreChecks::checkSlewLimit(load_pin, ref_cap, max_load_slew, corner)`: see
    /// [`check_slew_limit`].
    fn check_slew_limit(&self, load_pin: &str, ref_cap: f32, max_load_slew: f32, corner: usize, pre: &mut PreCheck) -> Result<(), Stop> {
        let swappable = || self.sizing.swappable_cells(self.buffer_lowest_drive);
        check_slew_limit(self.sizing.libs, self.buffer_lowest_drive, &swappable, self.buffer_cells, load_pin, ref_cap, max_load_slew, corner, pre)
    }
}

/// `PreChecks::checkSlewLimit(load_pin, ref_cap, max_load_slew, corner)` (upstream 77b7867bfd6):
/// the best slew the weakest buffer or any swappable size of it reaches at this load AT THE LOAD'S
/// CORNER — the input slews that corner's own buffer target slews over `buffer_cells` (not the
/// slowest corner's), cached per corner and recomputed only for a smaller load — above the limit
/// is RSZ-0090, naming the pin and the corner. A corner whose buffers time no arc of one output
/// transition cannot be judged: RSZ-3247 (f0eafeec0ca), rise checked first.
#[allow(clippy::too_many_arguments)]
fn check_slew_limit(libs: &crate::preamble::Libs, lowest: &str, swappable: &dyn Fn() -> Result<Vec<String>, Stop>, buffer_cells: &[String], load_pin: &str, ref_cap: f32, max_load_slew: f32, corner: usize, pre: &mut PreCheck) -> Result<(), Stop> {
    let cache = match pre.by_corner.get(&corner).copied() {
        Some(c) if ref_cap >= c.load => c,
        _ => {
            let (sums, counts) = crate::preamble::buffer_target_slew_sums(libs, buffer_cells, corner);
            let mut input_slews = [0.0f32; 2];
            for rf in [RISE, FALL] {
                if counts[rf] == 0 {
                    let rf_name = if rf == RISE { "rise" } else { "fall" };
                    return Err(Stop::error("RSZ-3247", format!("Cannot check the slew limit for pin {load_pin} at corner {}: no buffer timing models for {rf_name} transitions.", libs.scene_name(corner))));
                }
                input_slews[rf] = sums[rf] / counts[rf] as f32;
            }
            let slew_of = |name: &str| -> f32 {
                let cell = libs.link_cell(name).expect("the buffer and its swaps are link cells");
                buffer_slew_at(libs, cell, ref_cap, input_slews, corner)
            };
            let mut slew = slew_of(lowest);
            for b in &swappable()? {
                slew = slew.min(slew_of(b));
            }
            let c = SlewCheckCache { load: ref_cap, slew };
            pre.by_corner.insert(corner, c);
            c
        }
    };
    if max_load_slew < cache.slew {
        return Err(Stop::error("RSZ-0090", format!("Max transition time for pin {load_pin} at corner {} is {max_load_slew:e}s. Best achievable transition time is {:e}s with a load of {:e}F", libs.scene_name(corner), cache.slew, cache.load)));
    }
    Ok(())
}

/// `resizeToTargetSlew`: the caller's hint is the load only under global-route parasitics (it
/// skips `ensureWireParasitic`, an incremental re-route per buffer); otherwise none.
fn hinted_load(load_cap_hint: Option<f32>, global_routing: bool) -> Option<f32> {
    load_cap_hint.filter(|_| global_routing)
}

/// repairNetWire's hint for the repeater it inserts: the downstream wire (`buf_dist` long) and
/// `ref_cap`, in double, the sum converted to float.
fn wire_load_cap_hint(buf_dist: f64, dbu: i32, wire_cap: f64, ref_cap: f64) -> f32 {
    (buf_dist / (f64::from(dbu) * 1e6) * wire_cap + ref_cap) as f32
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
    buffer_slew_at(sizing.libs, cell, load_cap, sizing.tgt_slews, sizing.tgt_scene)
}

/// `gateDelays(output, load_cap, in_slews, corner, max)` then the larger of the rise and fall
/// slews: the corner's cell, each arc into the output at its input transition's slew.
pub fn buffer_slew_at(libs: &crate::preamble::Libs, cell: &vyges_sta::liberty::Cell, load_cap: f32, in_slews: [f32; 2], corner: usize) -> f32 {
    use vyges_sta::liberty::Model;
    let cell = libs.scene_cell(corner, &cell.name).unwrap_or(cell);
    let Some((_, output)) = cell.buffer_ports() else { return -crate::timing::INF };
    let mut slews = [-crate::timing::INF; 2];
    for set in cell.arc_sets.iter().filter(|s| s.to == output.name && !s.role.is_timing_check()) {
        for arc in &set.arcs {
            if let Model::Gate(m) = &arc.model {
                let (_, slew) = m.gate_delay(in_slews[arc.from_rf], load_cap);
                slews[arc.to_rf] = slews[arc.to_rf].max(slew);
            }
        }
    }
    slews[0].max(slews[1])
}

/// The netlist of net `net` alone: the instances and ports on its pins (in the design's order) and
/// the net, its pins in their order — every pin of those instances still a vertex, unconnected. A
/// driver's load ([`Graph::load_cap`]) reads nothing past its own net, so it is the same on this
/// netlist as on the design's. `None`: no such net.
fn net_alone(nl: &Netlist, net: &str) -> Option<Netlist> {
    let n = nl.nets.iter().find(|n| n.name == net)?;
    let mut insts: Vec<usize> = n.pins.iter().filter_map(|c| if let Conn::Inst(i, _) = c { Some(*i) } else { None }).collect();
    let mut ports: Vec<usize> = n.pins.iter().filter_map(|c| if let Conn::Port(i) = c { Some(*i) } else { None }).collect();
    insts.sort_unstable();
    insts.dedup();
    ports.sort_unstable();
    ports.dedup();
    let at = |v: &[usize], i: usize| v.binary_search(&i).expect("a pin of the net");
    let pins = n
        .pins
        .iter()
        .map(|c| match c {
            Conn::Inst(i, p) => Conn::Inst(at(&insts, *i), p.clone()),
            Conn::Port(i) => Conn::Port(at(&ports, *i)),
        })
        .collect();
    Some(Netlist {
        insts: insts.iter().map(|&i| nl.insts[i].clone()).collect(),
        ports: ports.iter().map(|&i| nl.ports[i].clone()).collect(),
        nets: vec![std::sync::Arc::new(Net { name: n.name.clone(), pins })],
    })
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

    /// A two-corner session ("fast", "slow") of buffer B whose output slew is `r` ns per pF of load
    /// at each corner; `arcs` the transition tables B's arc carries.
    fn two_corners(fast: f32, slow: f32, arcs: &dyn Fn(f32) -> String) -> Libs {
        let lib = |r: f32| {
            let text = format!(
                r#"library (l) {{ time_unit : "1ns" ; capacitive_load_unit (1, pf) ;
                  lu_table_template (t) {{ variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ; index_1 ("0, 1") ; index_2 ("0, 1") ; }}
                  cell (B) {{ pin (A) {{ direction : input ; capacitance : 0.01 ; }} pin (Z) {{ direction : output ; function : "A" ;
                    timing () {{ related_pin : "A" ; timing_sense : positive_unate ; {} }} }} }} }}"#,
                arcs(r)
            );
            Library::read(&lparse(&text).unwrap()).unwrap()
        };
        Libs { libs: vec![lib(fast)], scenes: vec!["fast".into(), "slow".into()], scene_libs: vec![vec![lib(fast)], vec![lib(slow)]], link_scene: Some(0) }
    }

    fn both(r: f32) -> String {
        format!(r#"rise_transition (t) {{ values ("0, {r}", "0, {r}") ; }} fall_transition (t) {{ values ("0, {r}", "0, {r}") ; }}"#)
    }

    fn check(l: &Libs, cap: f32, limit: f32, corner: usize, pre: &mut PreCheck) -> Result<(), Stop> {
        check_slew_limit(l, "B", &|| Ok(Vec::new()), &["B".to_string()], "load/A", cap, limit, corner, pre)
    }

    // Rules (PreChecks::checkSlewLimit, upstream 77b7867bfd6; its gtest TestPreChecks): the best
    // slew is computed at the LOAD'S corner and cached per corner, so the slow corner's best slew
    // neither rejects a fast-corner limit it would fail nor is reused for it; an impossible limit is
    // RSZ-0090 naming the pin and the corner, and no longer says the limit came from SDC.
    // Rule (GraphDelayCalc::loadCap via Resizer::resizeToTargetSlew): a driver's load is its net's
    // pins' caps (in the net's order) and its net's parasitic — nothing past its net — so the graph
    // over the net alone gives the same load, bit for bit, as the design's.
    #[test]
    fn a_drivers_load_over_its_net_alone_is_the_designs() {
        use vyges_sta::netlist::{inst, PortDir};
        let text = r#"library (t) { time_unit : "1ns" ; capacitive_load_unit (1, ff) ;
              cell (BUF) { pin (A) { direction : input ; capacitance : 1 ; } pin (Z) { direction : output ; function : "A" ; } }
              cell (BIG) { pin (A) { direction : input ; capacitance : 7 ; } pin (Z) { direction : output ; function : "A" ; } } }"#;
        let libs = vec![Library::read(&lparse(text).unwrap()).unwrap()];
        let net = |name: &str, pins: Vec<Conn>| std::sync::Arc::new(Net { name: name.into(), pins });
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let nl = Netlist {
            insts: vec![inst("u0", "BIG"), inst("u1", "BUF"), inst("u2", "BIG"), inst("u3", "BUF"), inst("u4", "BIG")],
            ports: vec![("a".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![net("na", vec![pin(0, "A"), Conn::Port(0)]), net("n1", vec![pin(1, "Z"), pin(4, "A"), pin(3, "A"), Conn::Port(1)]), net("n2", vec![pin(0, "Z"), pin(2, "A"), pin(1, "A")])],
        };
        let alone = net_alone(&nl, "n1").unwrap();
        let names: Vec<&str> = alone.insts.iter().map(|i| i.0.as_str()).collect();
        assert_eq!(names, ["u1", "u3", "u4"]);
        assert_eq!(alone.ports, [("y".to_string(), PortDir::Output)]);
        assert!(net_alone(&nl, "nx").is_none());
        let load = |n: &Netlist| {
            let g = Graph::build(&libs, n).unwrap();
            let v = g.vertices.iter().position(|x| x.name == "u1/Z").unwrap();
            g.load_cap(v, &HashMap::new())
        };
        assert_eq!(load(&alone).to_bits(), load(&nl).to_bits());
        assert!((load(&alone) - 8e-15).abs() < 1e-20, "{}", load(&alone));
    }

    #[test]
    fn the_slew_pre_check_is_per_corner() {
        let l = two_corners(1.0, 2.0, &both);
        const CAP: f32 = 1e-15;
        let (fast, slow) = (1e-12_f32, 2e-12_f32);
        let limit = (fast + slow) / 2.0;
        let mut pre = PreCheck::default();
        assert!(check(&l, CAP, limit, 0, &mut pre).is_ok(), "the fast corner reaches it");
        let e = check(&l, CAP, limit, 1, &mut pre).unwrap_err();
        assert_eq!(e.code(), "RSZ-0090");
        assert!(e.message().contains("pin load/A at corner slow ") && !e.message().contains("from SDC"), "{}", e.message());
        assert!(check(&l, CAP, limit, 0, &mut pre).is_ok(), "the slow corner's result is not reused for the fast one");
        let mut pre = PreCheck::default();
        assert!(check(&l, CAP, 1.0, 1, &mut pre).is_ok());
        assert!(check(&l, CAP, limit, 0, &mut pre).is_ok(), "a slow-corner check first does not reject the fast corner");
    }

    // Rule (PreChecks::checkSlewLimit): within a corner the cached best slew is recomputed for a
    // smaller load (and kept for a larger one).
    #[test]
    fn the_slew_pre_check_recomputes_for_a_smaller_load() {
        let l = two_corners(1.0, 2.0, &both);
        let (small, large) = (1e-15_f32, 1e-13_f32);
        let limit = (1e-12 + 1e-10) / 2.0;
        let mut pre = PreCheck::default();
        assert!(check(&l, large, 1.0, 0, &mut pre).is_ok());
        assert!(check(&l, small, limit, 0, &mut pre).is_ok(), "the smaller load's best slew is under the limit");
        assert!(check(&l, large, limit, 0, &mut pre).is_ok(), "a larger load keeps the smaller load's cache");
    }

    // Rule (PreChecks::checkSlewLimit, upstream f0eafeec0ca): a corner whose buffers time no arc
    // to one output transition cannot be judged — RSZ-3247, rise checked before fall.
    #[test]
    fn the_slew_pre_check_refuses_a_corner_without_buffer_timing() {
        let fall_only = |r: f32| format!(r#"fall_transition (t) {{ values ("0, {r}", "0, {r}") ; }}"#);
        let e = check(&two_corners(1.0, 2.0, &fall_only), 1e-15, 1.0, 1, &mut PreCheck::default()).unwrap_err();
        assert_eq!(e.code(), "RSZ-3247");
        assert_eq!(e.message(), "Cannot check the slew limit for pin load/A at corner slow: no buffer timing models for rise transitions.");
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

    // Rule (repairNetVia: `float r_via = bnet->viaResistance(...)`): the via resistance is
    // narrowed to float, and `maxLoadSlew - r_via * cap * slew_shape_factor_` is float arithmetic —
    // in double, these inputs land one ulp lower.
    #[test]
    fn a_vias_slew_is_taken_off_in_float() {
        let (mls, r_via, cap) = (1.6378709499775823e-10_f64 as f32, 30.875086493095175, 7.075775783436239e-14_f64 as f32);
        let shape = 0.932027698_f64 as f32;
        assert_eq!(via_max_load_slew(mls, r_via, cap, shape).to_bits(), 0x2f31_d8d8);
        let in_double = (f64::from(mls) - r_via * f64::from(cap) * f64::from(shape)) as f32;
        assert_eq!(in_double.to_bits(), 0x2f31_d8d7);
    }

    // Rule (resizeToTargetSlew: `load_cap_hint.has_value() && getParasiticsSrc() ==
    // kGlobalRouting`): the hint replaces the estimate under global-route parasitics only; under
    // placement the repeater is sized from its net's own estimate, hint or not.
    #[test]
    fn a_load_hint_counts_only_on_global_routes() {
        assert_eq!(hinted_load(Some(5.8e-14), true), Some(5.8e-14));
        assert_eq!(hinted_load(Some(5.8e-14), false), None);
        assert_eq!(hinted_load(None, true), None);
    }

    // Rule (repairNetWire: `buffer_load_cap = static_cast<float>(buf_dist / (dbu_ * 1e+6) *
    // wire_cap + ref_cap)`): the hint is summed in double and narrowed once; in float these
    // inputs land one ulp lower.
    #[test]
    fn a_wire_repeaters_hint_is_summed_in_double() {
        let (buf_dist, wire_cap, ref_cap) = (34086.0, 1.5101380514788433e-10, 5.004807362210216e-14);
        assert_eq!(wire_load_cap_hint(buf_dist, 2000, wire_cap, ref_cap).to_bits(), 0x296c_fccf);
        let in_float = (buf_dist as f32 / (2000.0_f64 * 1e6) as f32) * wire_cap as f32 + ref_cap as f32;
        assert_eq!(in_float.to_bits(), 0x296c_fcce);
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
