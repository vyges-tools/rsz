// SPDX-License-Identifier: Apache-2.0
//! The check the repair command runs before it repairs (`check_max_wire_length`): the wire length
//! past which splitting a wire with a buffer is faster, and the advisory warning RSZ-0065 when
//! `-max_wire_length` asks for less.
//!
//! One function per stage, in call order: [`check_max_wire_length`] → [`find_max_wire_length`]
//! (every scene with a wire resistance, every buffer of `findBuffers`, the least length) →
//! [`find_max_wire_length_for`] (a binary search on [`split_wire_delay_diff`]) →
//! [`buffer_wire_delay`] → [`cell_wire_delay`] (a buffer driving a buffer through one wire, timed
//! at the target slews, the largest gate-plus-wire delay over every scene and arc).
//!
//! The values are the repair's own: the same buffer list and target slews, computed by the same
//! preamble functions.

use std::collections::HashMap;

use vyges_sta::graph::{EdgeKind, NetParasitics, SdcEnv};
use vyges_sta::netlist::{Conn, Net, Netlist, PortDir};
use vyges_sta::parasitics::Network;

use crate::buffered_net::WireRc;
use crate::preamble::Libs;
use crate::Stop;

const MAX: usize = 1;

/// What the check reads.
pub struct Ctx<'a> {
    pub libs: &'a Libs,
    /// `buffer_cells_`, in order.
    pub buffers: &'a [String],
    /// `tgt_slews_[rise, fall]`.
    pub tgt_slews: [f32; 2],
    /// The signal wire RC per scene, SI units per meter.
    pub wire_rc: &'a [WireRc],
    pub master_pins: &'a HashMap<String, Vec<String>>,
}

/// `est::wire_signal_resistance(scene)`: the mean of the horizontal and vertical resistance.
fn wire_signal_resistance(rc: &WireRc) -> f64 {
    (rc.h_res + rc.v_res) / 2.0
}

fn wire_signal_capacitance(rc: &WireRc) -> f64 {
    (rc.h_cap + rc.v_cap) / 2.0
}

/// `check_max_wire_length max_wire_length false` (repair_design's): with a wire resistance at the
/// command scene, the length past which buffering pays; RSZ-0065 when `-max_wire_length` (meters,
/// 0 for none) is shorter. Returns the warning line, if any.
/// What the check found: the warning, if any, and each buffer's length (meters) per scene, in the
/// order computed (what the reference's `max_wire_length` debug prints).
#[derive(Debug, Clone, Default)]
pub struct Check {
    pub warning: Option<(&'static str, String)>,
    pub lengths: Vec<(usize, String, f64)>,
}

pub fn check_max_wire_length(ctx: &Ctx<'_>, max_wire_length: f64) -> Result<Check, Stop> {
    let mut out = Check::default();
    let Some(rc0) = ctx.wire_rc.first() else { return Ok(out) };
    if wire_signal_resistance(rc0) <= 0.0 {
        return Ok(out);
    }
    let min_delay_max_wire_length = find_max_wire_length(ctx, &mut out.lengths)?;
    out.warning = warning(max_wire_length, min_delay_max_wire_length);
    Ok(out)
}

/// RSZ-0065 when a `-max_wire_length` was given (> 0) and is shorter than the length past which
/// buffering pays; the length as `%.0f` microns (`sta::distance_sta_ui`).
fn warning(max_wire_length: f64, min_delay_max_wire_length: f64) -> Option<(&'static str, String)> {
    if max_wire_length > 0.0 && max_wire_length < min_delay_max_wire_length {
        let um = min_delay_max_wire_length / 1e-6;
        return Some(("RSZ-0065", format!("max wire length less than {um:.0}u increases wire delays.")));
    }
    None
}


/// `Resizer::findMaxWireLength1(false)`: over the scenes with a wire resistance, over the buffers,
/// the least length (−INF when no scene has one).
pub fn find_max_wire_length(ctx: &Ctx<'_>, lengths: &mut Vec<(usize, String, f64)>) -> Result<f64, Stop> {
    let mut max_length: Option<f64> = None;
    for (k, rc) in ctx.wire_rc.iter().enumerate() {
        if wire_signal_resistance(rc) <= 0.0 {
            continue;
        }
        for buffer in ctx.buffers {
            let length = find_max_wire_length_for(ctx, buffer, rc)?;
            lengths.push((k, buffer.clone(), length));
            max_length = Some(max_length.unwrap_or(f64::INFINITY).min(length));
        }
    }
    Ok(max_length.unwrap_or(f64::NEG_INFINITY))
}

/// `Resizer::findMaxWireLength(drvr_port, corner)`: from the length whose wire resistance equals
/// the driver's, double while one wire is still faster than two halves and a buffer, then halve the
/// bracket to 1%; the lower bound.
pub fn find_max_wire_length_for(ctx: &Ctx<'_>, buffer: &str, rc: &WireRc) -> Result<f64, Stop> {
    let cell = ctx.libs.link_cell(buffer).ok_or_else(|| Stop::error("RSZ-0070", format!("no liberty cell for {buffer}.")))?;
    let (_, drvr_port) = cell.buffer_ports().ok_or_else(|| Stop::error("RSZ-MWL", format!("{buffer}: not a buffer")))?;
    let drvr_r = f64::from(cell.drive_resistance(&drvr_port.name));
    let mut low = 0.0f64;
    let mut high = drvr_r / wire_signal_resistance(rc);
    let tol = 0.01;
    let mut diff_ub = split_wire_delay_diff(ctx, high, buffer)?;
    while (low - high).abs() > low.max(high) * tol {
        if diff_ub < 0.0 {
            low = high;
            high *= 2.0;
            diff_ub = split_wire_delay_diff(ctx, high, buffer)?;
        } else {
            let mid = (low + high) / 2.0;
            let diff_mid = split_wire_delay_diff(ctx, mid, buffer)?;
            if diff_mid < 0.0 {
                low = mid;
            } else {
                high = mid;
                diff_ub = diff_mid;
            }
        }
    }
    Ok(low)
}

/// `Resizer::splitWireDelayDiff`: one wire's delay less twice a half wire's — `Delay` arithmetic,
/// in `float`.
pub fn split_wire_delay_diff(ctx: &Ctx<'_>, wire_length: f64, buffer: &str) -> Result<f64, Stop> {
    let (delay1, _) = buffer_wire_delay(ctx, buffer, wire_length)?;
    let (delay2, _) = buffer_wire_delay(ctx, buffer, wire_length / 2.0)?;
    Ok(f64::from(delay1 - delay2 * 2.0))
}

/// `Resizer::bufferWireDelay`: the buffer driving its own input pin through the wire.
pub fn buffer_wire_delay(ctx: &Ctx<'_>, buffer: &str, wire_length: f64) -> Result<(f32, f32), Stop> {
    let cell = ctx.libs.link_cell(buffer).ok_or_else(|| Stop::error("RSZ-MWL", format!("{buffer}: no liberty cell")))?;
    let (load_port, drvr_port) = cell.buffer_ports().ok_or_else(|| Stop::error("RSZ-MWL", format!("{buffer}: not a buffer")))?;
    cell_wire_delay(ctx, buffer, &drvr_port.name, &load_port.name, wire_length)
}

/// `Resizer::cellWireDelay`: a scratch net `wire` from `drvr/<drvr_port>` to `load/<load_port>`,
/// per scene its wire parasitic (`makeWireParasitic`: half the wire's capacitance at each end, its
/// resistance between), every arc into the driver port at the target slew of its input transition;
/// the largest gate-plus-wire delay and load slew over every scene and arc.
pub fn cell_wire_delay(ctx: &Ctx<'_>, cell: &str, drvr_port: &str, load_port: &str, wire_length: f64) -> Result<(f32, f32), Stop> {
    // The driver's input comes from a port at the target slews.
    let (input, _) = ctx.libs.link_cell(cell).and_then(|c| c.buffer_ports()).ok_or_else(|| Stop::error("RSZ-MWL", format!("{cell}: not a buffer")))?;
    let netlist = Netlist {
        insts: vec![("drvr".into(), cell.into()), ("load".into(), cell.into())],
        ports: vec![("in".into(), PortDir::Input)],
        nets: vec![
            Net { name: "in".into(), pins: vec![Conn::Inst(0, input.name.clone()), Conn::Port(0)] },
            Net { name: "wire".into(), pins: vec![Conn::Inst(0, drvr_port.into()), Conn::Inst(1, load_port.into())] },
        ],
    };
    let mut sdc = SdcEnv::default();
    sdc.input_slew.insert("in".into(), [[Some(ctx.tgt_slews[0]); 2], [Some(ctx.tgt_slews[1]); 2]]);
    let (drvr_name, load_name) = (format!("drvr/{drvr_port}"), format!("load/{load_port}"));
    let mut delay = f32::NEG_INFINITY;
    let mut slew = f32::NEG_INFINITY;
    for (k, rc) in ctx.wire_rc.iter().enumerate() {
        let wire_cap = wire_length * wire_signal_capacitance(rc);
        let wire_res = wire_length * wire_signal_resistance(rc);
        let half = (wire_cap / 2.0) as f32;
        let par: HashMap<String, NetParasitics> = [(
            "wire".to_string(),
            NetParasitics { node_names: vec![drvr_name.clone(), load_name.clone()], network: Network { node_caps: vec![half, half], resistors: vec![(0, 1, wire_res as f32)] }, port_pin_caps: None },
        )]
        .into();
        let mut g = crate::repair_design::timer_graph(ctx.libs, k, &netlist, &sdc, ctx.master_pins)?;
        g.find_delays(&par, None).map_err(|e| Stop::error("RSZ-MWL", e))?;
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n);
        let (Some(d), Some(l)) = (v(&drvr_name), v(&load_name)) else { return Err(Stop::error("RSZ-MWL", "scratch pins not in the graph".into())) };
        let wire = g.edges.iter().position(|e| e.from == d && e.to == l && matches!(e.kind, EdgeKind::Wire)).ok_or_else(|| Stop::error("RSZ-MWL", "no wire edge".into()))?;
        let scene_cell = ctx.libs.scene_cell(k, cell).ok_or_else(|| Stop::error("RSZ-MWL", format!("{cell}: not in scene {k}")))?;
        for (e, edge) in g.edges.iter().enumerate() {
            let EdgeKind::Gate { set } = edge.kind else { continue };
            if edge.to != d || g.is_check(e) {
                continue;
            }
            for (i, arc) in scene_cell.arc_sets[set].arcs.iter().enumerate() {
                let gate_wire = g.delay[e][i][MAX] + g.delay[wire][arc.to_rf][MAX];
                if gate_wire > delay {
                    delay = gate_wire;
                }
                slew = slew.max(g.slew[l][arc.to_rf][MAX]);
            }
        }
    }
    Ok((delay, slew))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (check_max_wire_length): no -max_wire_length (0) never warns; a longer one does not;
    /// a shorter one names the length, rounded to whole microns.
    #[test]
    fn rsz_0065_only_for_a_shorter_max_wire_length() {
        let min = 692.8e-6;
        assert_eq!(warning(0.0, min), None);
        assert_eq!(warning(700e-6, min), None);
        assert_eq!(warning(600e-6, min).unwrap().1, "max wire length less than 693u increases wire delays.");
    }
}
