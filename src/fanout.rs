// SPDX-License-Identifier: Apache-2.0
//! Fanout repair's geometry: `RepairDesign::findLoadRegions` (the loads' bounding box, cut in two
//! along its longer side until each region holds at most `max_fanout` pins) and
//! `findClosedPinLoc`. The repeaters themselves are `makeRegionRepeaters`, in the sequencer.

/// `LoadRegion`: a region's own pins (left over after its sub-regions took theirs), its box, and
/// its sub-regions (none, or two).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LoadRegion {
    pub pins: Vec<String>,
    /// `(x_min, y_min, x_max, y_max)`.
    pub bbox: (i32, i32, i32, i32),
    pub regions: Vec<LoadRegion>,
}

/// `RepairDesign::findBbox`: the box of the pins' locations (`mergeInit`, then each point).
pub fn find_bbox(pins: &[String], loc: &dyn Fn(&str) -> (i32, i32)) -> (i32, i32, i32, i32) {
    let mut b = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
    for p in pins {
        let (x, y) = loc(p);
        b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
    }
    b
}

/// `RepairDesign::findLoadRegions(net, drvr, max_fanout)`: one region of every load, subdivided.
pub fn find_load_regions(loads: Vec<String>, max_fanout: i32, dbu: i32, loc: &dyn Fn(&str) -> (i32, i32)) -> LoadRegion {
    let bbox = find_bbox(&loads, loc);
    let mut region = LoadRegion { pins: loads, bbox, regions: Vec::new() };
    subdivide_region(&mut region, max_fanout, dbu, loc);
    region
}

/// `RepairDesign::subdivideRegion`: with more than `max_fanout` pins and a box wider AND taller than
/// one micron (`dbu`), cut it at the midpoint (int division) of its longer side — `dx > dy` cuts
/// vertically, else horizontally — a pin at or below the cut going to the first half; the region
/// keeps no pins, and each non-empty half is subdivided in turn.
pub fn subdivide_region(region: &mut LoadRegion, max_fanout: i32, dbu: i32, loc: &dyn Fn(&str) -> (i32, i32)) {
    let (x_min, y_min, x_max, y_max) = region.bbox;
    let (dx, dy) = (x_max - x_min, y_max - y_min);
    if region.pins.len() as i64 > i64::from(max_fanout) && dx > dbu && dy > dbu {
        let x_mid = (x_min + x_max) / 2;
        let y_mid = (y_min + y_max) / 2;
        let horz_partition = dx > dy;
        let boxes = if horz_partition { [(x_min, y_min, x_mid, y_max), (x_mid, y_min, x_max, y_max)] } else { [(x_min, y_min, x_max, y_mid), (x_min, y_mid, x_max, y_max)] };
        let mut subs = [LoadRegion { bbox: boxes[0], ..Default::default() }, LoadRegion { bbox: boxes[1], ..Default::default() }];
        for p in region.pins.drain(..) {
            let (x, y) = loc(&p);
            let first = if horz_partition { x <= x_mid } else { y <= y_mid };
            subs[usize::from(!first)].pins.push(p);
        }
        for sub in subs.iter_mut() {
            if !sub.pins.is_empty() {
                subdivide_region(sub, max_fanout, dbu, loc);
            }
        }
        region.regions = subs.into();
    }
}

/// `RepairDesign::findClosedPinLoc(drvr, pins)`: the location of the pin nearest the driver
/// (Manhattan, the first of equals), else the driver's own.
pub fn find_closed_pin_loc(drvr_loc: (i32, i32), pins: &[String], loc: &dyn Fn(&str) -> (i32, i32)) -> (i32, i32) {
    let mut closest = drvr_loc;
    let mut closest_dist = i64::MAX;
    for p in pins {
        let l = loc(p);
        let dist = i64::from((l.0 - drvr_loc.0).abs()) + i64::from((l.1 - drvr_loc.1).abs());
        if dist < closest_dist {
            closest = l;
            closest_dist = dist;
        }
    }
    closest
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rules (subdivideRegion): the LONGER side is cut at its int midpoint; a pin ON the cut goes to
    // the first half; a region with no more than max_fanout pins, or no wider than a micron, is
    // left whole; the parent keeps no pins.
    #[test]
    fn regions_are_cut_on_the_longer_side_at_the_midpoint() {
        let at = |p: &str| -> (i32, i32) {
            let n: i32 = p[1..].parse().unwrap();
            (n * 1000, (n % 2) * 3000)
        };
        let pins: Vec<String> = (0..5).map(|i| format!("p{i}")).collect();
        let r = find_load_regions(pins.clone(), 2, 1000, &at);
        assert!(r.pins.is_empty());
        assert_eq!(r.regions.len(), 2);
        assert_eq!(r.regions[0].bbox, (0, 0, 2000, 3000), "dx 4000 > dy 3000: a vertical cut at x 2000");
        assert!(r.regions[0].pins.is_empty() && !r.regions[0].regions.is_empty(), "p0, p1, p2 (p2 on the cut) — more than 2, cut again");
        assert_eq!(r.regions[1].pins, vec!["p3".to_string(), "p4".to_string()]);
        assert_eq!(find_load_regions(pins.clone(), 5, 1000, &at).regions.len(), 0);
        assert_eq!(find_load_regions(pins, 2, 5000, &at).regions.len(), 0, "a box no wider than a micron is not cut");
    }

    // Rule (findClosedPinLoc): the nearest pin by Manhattan distance, the first of equals; none
    // nearer than "infinitely far" means the first pin.
    #[test]
    fn the_repeater_sits_at_the_nearest_load() {
        let at = |p: &str| -> (i32, i32) { [("a", (10, 0)), ("b", (0, 10)), ("c", (3, 3))].iter().find(|x| x.0 == p).unwrap().1 };
        assert_eq!(find_closed_pin_loc((0, 0), &["a".into(), "b".into(), "c".into()], &at), (3, 3));
        assert_eq!(find_closed_pin_loc((0, 0), &["a".into(), "b".into()], &at), (10, 0));
        assert_eq!(find_closed_pin_loc((7, 7), &[], &at), (7, 7));
    }
}
