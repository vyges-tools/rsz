// SPDX-License-Identifier: Apache-2.0
//! CloneMove: a high-fanout combinational driver duplicated, the clone taking the less critical
//! half of its loads.
//!
//! One function per stage, named after the reference's and in its call order:
//! - [`collect_fanout_slacks`] — `collectFanoutSlacks`: each fanout's slack less the driver's (at
//!   the path's transition), largest first, a tie by path name;
//! - [`select_moved_loads`] — `selectMovedLoads`: the first half, top-level ports and dont_touch
//!   instances kept on the original;
//! - [`choose_clone_cell`] — `halfDrivingPowerCell` (`closestDriver(cell, swappable, 0.5)`);
//! - [`compute_clone_location`] — the centroid of the driver pin and the first half's pins.
//!
//! The sequencer ([`crate::repair_setup`]) checks the target (`resolveDriverTarget`: fanout above
//! 8, a net it may buffer, a single-output combinational cell) and the pending Buffer / SplitLoad
//! moves, then makes the clone (`CloneCandidate::applyClone`).

use vyges_sta::liberty::{Cell, Direction};

/// `kCloneMinFanout`.
pub const CLONE_MIN_FANOUT: usize = 8;

/// One fanout: its pin and its slack less the driver's.
#[derive(Debug, Clone, PartialEq)]
pub struct FanoutSlack {
    pub pin: String,
    pub slack: f32,
    pub top_port: bool,
}

/// `collectFanoutSlacks`' order: larger slack delta first; equal deltas by path name.
pub fn collect_fanout_slacks(mut fanouts: Vec<FanoutSlack>) -> Vec<FanoutSlack> {
    fanouts.sort_by(|a, b| b.slack.partial_cmp(&a.slack).unwrap_or(std::cmp::Ordering::Equal).then_with(|| a.pin.cmp(&b.pin)));
    fanouts
}

/// `selectMovedLoads`: of the first `len / 2`, every pin that is not a top-level port and not on a
/// dont_touch instance.
pub fn select_moved_loads(fanouts: &[FanoutSlack], inst_dont_touch: &dyn Fn(&str) -> bool) -> Vec<String> {
    fanouts[..fanouts.len() / 2]
        .iter()
        .filter(|f| !f.top_port && !inst_dont_touch(f.pin.rsplit_once('/').map_or("", |(i, _)| i)))
        .map(|f| f.pin.clone())
        .collect()
}

/// `computeCloneLocation`: the driver pin's location and the first half's pins (all of them,
/// moved or not), averaged with integer division.
pub fn compute_clone_location(drvr: (i32, i32), fanouts: &[FanoutSlack], loc: &dyn Fn(&str) -> (i32, i32)) -> (i32, i32) {
    let (mut x, mut y, mut count) = (drvr.0, drvr.1, 1);
    for f in &fanouts[..fanouts.len() / 2] {
        let (lx, ly) = loc(&f.pin);
        x += lx;
        y += ly;
        count += 1;
    }
    (x / count, y / count)
}

/// `libraryOutputPins` counts any output (tristate and bidirect included).
fn output_ports(cell: &Cell) -> Vec<&vyges_sta::liberty::Port> {
    cell.ports.iter().filter(|p| matches!(p.direction, Direction::Output | Direction::Tristate | Direction::Bidirect)).collect()
}

/// `isSingleOutputCombinational(cell)`: one output, and combinational — not a clock gate, pad or
/// macro, not sequential. `Err` for a cell with a `statetable` (a clock gate the reader here does
/// not mark).
pub fn is_single_output_combinational(cell: &Cell) -> Result<bool, String> {
    if cell.has_statetable {
        return Err(format!("{}: a statetable cell (clock gate?) is not modelled", cell.name));
    }
    let combinational = !cell.is_pad && cell.seqs.is_empty() && !cell.has_seq_bank;
    Ok(output_ports(cell).len() == 1 && combinational)
}

/// `Resizer::maxLoad(cell)`: the first output port with a capacitance limit (the library default
/// is never set), else 0.
pub fn max_load(cell: &Cell) -> f32 {
    cell.ports.iter().filter(|p| p.direction == Direction::Output).find_map(|p| p.max_capacitance).unwrap_or(0.0)
}

/// `closestDriver(cell, candidates, scale)`: none unless the cell is single-output combinational;
/// the first candidate (not dont_use) whose max load equals `scale` × the cell's, else the closest.
pub fn closest_driver(cell: &Cell, candidates: &[&Cell], dont_use: &dyn Fn(&str) -> bool, scale: f32) -> Result<Option<String>, String> {
    if candidates.is_empty() || !is_single_output_combinational(cell)? {
        return Ok(None);
    }
    let current = scale * max_load(cell);
    let mut diff = 1e30f32;
    let mut closest = None;
    for c in candidates {
        if dont_use(&c.name) {
            continue;
        }
        let limit = max_load(c);
        if limit == current {
            return Ok(Some(c.name.clone()));
        }
        let d = (limit - current).abs();
        if d < diff {
            diff = d;
            closest = Some(c.name.clone());
        }
    }
    Ok(closest)
}

/// `chooseCloneCell`: the half-driving cell, else the original.
pub fn choose_clone_cell(cell: &Cell, candidates: &[&Cell], dont_use: &dyn Fn(&str) -> bool) -> Result<String, String> {
    Ok(closest_driver(cell, candidates, dont_use, 0.5)?.unwrap_or_else(|| cell.name.clone()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(pin: &str, slack: f32) -> FanoutSlack {
        FanoutSlack { pin: pin.into(), slack, top_port: pin.starts_with("out") }
    }

    // Rule (collectFanoutSlacks / selectMovedLoads): largest slack delta first, equal deltas by
    // name; the first half moves, minus top-level ports.
    #[test]
    fn the_less_critical_half_moves() {
        let s = collect_fanout_slacks(vec![f("u3/A", 0.1), f("u1/A", 0.3), f("out1", 0.3), f("u2/A", 0.2)]);
        let names: Vec<&str> = s.iter().map(|x| x.pin.as_str()).collect();
        assert_eq!(names, ["out1", "u1/A", "u2/A", "u3/A"]);
        assert_eq!(select_moved_loads(&s, &|_| false), vec!["u1/A"], "the port stays");
        assert!(select_moved_loads(&s, &|i| i == "u1").is_empty(), "a dont_touch load stays");
    }

    // Rule (computeCloneLocation): the driver and the first half's pins averaged, integer division.
    #[test]
    fn the_clone_sits_at_the_centroid() {
        let s = vec![f("a/A", 2.0), f("b/A", 1.0), f("c/A", 0.0)];
        let loc = |p: &str| if p == "a/A" { (10, 1) } else { (100, 100) };
        assert_eq!(compute_clone_location((0, 0), &s, &loc), (5, 0));
    }
}
