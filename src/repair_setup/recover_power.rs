//! `RecoverPower`: `repair_timing -recover_power percent` — along the worst path of each endpoint
//! with positive setup slack (greatest slack first), the first driver, by load-dependent delay,
//! that a weaker cell of the same height and no greater width can replace within the path's
//! slack; the resize kept unless the worst slack falls by half or more.

use std::collections::HashSet;

use super::{gate_delay, port_capacitance, port_intrinsic_delay, snapshot, timer_stop, Ctx, PathView, SetupDesign, Snapshot, Timer};
use crate::repair_timing::delay_as_string;
use crate::Stop;

/// `setup_slack_margin_`, `setup_slack_max_margin_` (`float`): an endpoint is a candidate with a
/// slack strictly between them.
const SETUP_SLACK_MARGIN: f32 = 1e-11;
const SETUP_SLACK_MAX_MARGIN: f32 = 1e-4;
/// `failed_move_threshold_limit_`.
const FAILED_MOVE_THRESHOLD_LIMIT: i64 = 500;
/// `min_print_interval_`, `max_print_interval_`.
const MIN_PRINT_INTERVAL: i64 = 10;
const MAX_PRINT_INTERVAL: i64 = 100;
/// `downsizeCell`'s `delay_margin` ("prevent overly aggressive downsizing").
const DELAY_MARGIN: f64 = 1.5;

/// The command's arguments as `Resizer::recoverPower` receives them.
pub struct PowerArgs {
    /// `recover_power_percent`: the Tcl's double over 100, passed as `float`.
    pub percent: f32,
    pub verbose: bool,
    /// `max_area_` (`setMaxUtilization`): 0 for no limit.
    pub max_area: f64,
}

#[derive(Debug, Default)]
pub struct PowerOutcome {
    /// The command's report lines; `trace`, those with the debug lines among them.
    pub lines: Vec<String>,
    pub trace: Vec<String>,
    /// `resize_count_`.
    pub resized: i64,
    /// `RSZ-0125`, which ends the command.
    pub error: Option<String>,
}

/// `Resizer::recoverPower` after `resizePreamble`: `RecoverPower::recoverPower(percent, verbose)`.
pub fn recover_power(ctx: &Ctx<'_>, design: &mut dyn SetupDesign, pa: &PowerArgs) -> Result<PowerOutcome, Stop> {
    // Every scene timed (`Sta::slack`, `worstSlack`, `vertexWorstSlackPath` read them all).
    let mut timer = Timer::new(design, ctx.libs.scene_count())?;
    let edits = design.take_timer_edits();
    let timing = snapshot(ctx, design.as_design(), &[], &mut timer, edits)?;
    // `init`: `initial_design_area_`; `Resizer::init` (the preamble) set `design_area_` too.
    let initial_area = design.design_area();
    let mut r = Recovery { ctx, pa, design, timer, timing, out: PowerOutcome::default(), initial_area, design_area: initial_area, print_interval: 0, bad_vertices: HashSet::new() };
    r.run()?;
    Ok(r.out)
}

/// `RecoverPower`'s state over one call.
struct Recovery<'c, 'd> {
    ctx: &'c Ctx<'c>,
    pa: &'c PowerArgs,
    design: &'d mut dyn SetupDesign,
    timer: Timer,
    /// The timer over the design as it is now.
    timing: Snapshot,
    out: PowerOutcome,
    /// `initial_design_area_`.
    initial_area: f64,
    /// `Resizer::design_area_`: moved by `designAreaIncr(float)` in `replaceCell`, recomputed by
    /// `journalRestore` (`Resizer::init`).
    design_area: f64,
    print_interval: i64,
    /// `bad_vertices_`: drivers whose downsize was undone, by pin (a resize keeps the vertex).
    bad_vertices: HashSet<String>,
}

impl Recovery<'_, '_> {
    fn debug(&mut self, level: i64, line: String) {
        if self.ctx.debug.get(&("RSZ".to_string(), "recover_power".to_string())).is_some_and(|&l| l >= level) {
            self.out.trace.push(format!("[DEBUG RSZ-recover_power] {line}"));
        }
    }

    fn report(&mut self, line: String) {
        self.out.trace.push(line.clone());
        self.out.lines.push(line);
    }

    fn ds(&self, v: f32) -> String {
        delay_as_string(v, 3, self.ctx.time_scale)
    }

    /// The timer again (`updateParasitics`, `findRequireds`), with the worst path of `want` ready.
    fn retime(&mut self, want: &[String]) -> Result<(), Stop> {
        self.design.update_parasitics().map_err(timer_stop)?;
        let edits = self.design.take_timer_edits();
        self.timing = snapshot(self.ctx, self.design.as_design(), want, &mut self.timer, edits)?;
        Ok(())
    }

    /// `Resizer::overMaxArea`: a limit, and `fuzzyGreaterEqual` of the two (`float` arguments).
    fn over_max_area(&self) -> bool {
        self.pa.max_area != 0.0 && {
            let (a, m) = (self.design_area as f32, self.pa.max_area as f32);
            a > m || vyges_sta::fuzzy::equal(a, m)
        }
    }

    /// `recoverPower(recover_power_percent, verbose)`.
    fn run(&mut self) -> Result<(), Stop> {
        // The endpoints (`sta_->endpoints()`, vertex order) with a slack strictly inside the
        // margins, sorted by slack, greatest first (libc++ `std::ranges::sort`).
        let endpoint_count = self.timing.ends.len();
        let mut ends_with_slack: Vec<(String, f32)> = self.timing.ends.iter().filter(|p| p.slack > SETUP_SLACK_MARGIN && p.slack < SETUP_SLACK_MAX_MARGIN).map(|p| (p.pin.clone(), p.slack)).collect();
        crate::order::libcxx_sort_by(&mut ends_with_slack, |a, b| a.1 > b.1).map_err(|h| Stop::refused("RSZ-ORDER", format!("{} candidate endpoints reach the sort's heap fallback, which is not modelled", h.len)))?;
        let n = ends_with_slack.len();
        self.debug(1, format!("Candidate paths {}/{} {}%", n, endpoint_count, (n as f64 / endpoint_count as f64 * 100.0) as i32));
        // `size() * recover_power_percent` in `float`, truncated; at least one path.
        let max_end_count = ((n as f32 * self.pa.percent) as i32).max(1);
        let (worst_slack_before, _) = self.timing.worst();
        self.print_interval = if i64::from(max_end_count) > 5 * MAX_PRINT_INTERVAL { MAX_PRINT_INTERVAL } else { MIN_PRINT_INTERVAL };
        self.print_progress(0, false, false);
        let mut end_index: i64 = 0;
        let mut failed_move_threshold: i64 = 0;
        for (end, _) in &ends_with_slack {
            self.begin_journal()?;
            // `slack(end)` and `vertexWorstSlackPath(end)` on the timer as the last move left it.
            self.retime(std::slice::from_ref(end))?;
            let end_slack_before = self.timing.slack(end);
            end_index += 1;
            self.debug(2, format!("Doing {} / {}", end_index, max_end_count));
            if self.pa.verbose || end_index == 1 {
                self.print_progress(end_index, false, false);
            }
            if end_index > i64::from(max_end_count) {
                self.commit_journal()?;
                break;
            }
            let Some(view) = self.timing.paths.get(end).cloned() else {
                return Err(timer_stop(format!("{end}: no worst path at a candidate endpoint")));
            };
            let changed = self.recover_power_path(&view, end_slack_before)?;
            if let Some(changed) = changed {
                self.retime(&[])?;
                let end_slack_after = self.timing.slack(end);
                let (worst_slack_after, _) = self.timing.worst();
                // In `float`, then compared with the `double` constants.
                let worst_slack_percent = ((worst_slack_before - worst_slack_after) / worst_slack_before * 100.0).abs();
                let better = f64::from(worst_slack_percent) < 0.0001 || (worst_slack_before > 0.0 && f64::from(worst_slack_after / worst_slack_before) > 0.5);
                self.debug(2, format!("slack = {} worst_slack = {} better = {}", self.ds(end_slack_after), self.ds(worst_slack_after), if better { "save" } else { "" }));
                if better {
                    failed_move_threshold = 0;
                    self.commit_journal()?;
                    self.out.resized += 1;
                    self.debug(2, format!("{}/{} Resize for power Slack change {} -> {}", end_index, n, self.ds(worst_slack_before), self.ds(worst_slack_after)));
                    if self.over_max_area() {
                        break;
                    }
                } else {
                    // The vertex saved so it is not tried again; the change undone.
                    self.bad_vertices.insert(changed);
                    failed_move_threshold += 1;
                    if failed_move_threshold > FAILED_MOVE_THRESHOLD_LIMIT {
                        self.report(format!("[INFO RSZ-0142] {FAILED_MOVE_THRESHOLD_LIMIT} successive tries yielded negative slack. Ending power recovery"));
                        self.commit_journal()?;
                        self.out.resized += 1;
                        break;
                    }
                    self.restore_journal()?;
                    self.debug(2, format!("{}/{} Undo resize for power Slack change {} -> {}", end_index, n, self.ds(worst_slack_before), self.ds(worst_slack_after)));
                }
            } else {
                self.commit_journal()?;
            }
        }
        self.print_progress(end_index, true, true);
        self.bad_vertices.clear();
        if self.out.resized > 0 {
            self.report(format!("[INFO RSZ-0141] Resized {} instances.", self.out.resized));
        }
        if self.over_max_area() {
            // `logger_->error`: the line, then the command ends (it throws).
            self.report("[ERROR RSZ-0125] max utilization reached.".into());
            self.out.error = Some("RSZ-0125: max utilization reached.".into());
        }
        Ok(())
    }

    /// `recoverPower(path, path_slack)`: the path's drivers (past its start, not top-level ports)
    /// by load-dependent delay — the arc delay less the scene arc's intrinsic delay — greatest
    /// first, the earlier on a tie; the first not already undone that `downsizeDrvr` resizes.
    /// Returns that driver's pin.
    fn recover_power_path(&mut self, view: &PathView, path_slack: f32) -> Result<Option<String>, Stop> {
        if view.stages.len() <= 1 {
            return Ok(None);
        }
        let mut load_delays: Vec<(usize, f32)> = Vec::new();
        for (i, st) in view.stages.iter().enumerate().skip(view.start) {
            if i > 0 && st.is_driver && !st.top_port {
                let load_delay = st.load_delay.unwrap_or(0.0);
                load_delays.push((i, load_delay));
                self.debug(3, format!("{} load_delay = {}", st.pin, self.ds(load_delay)));
            }
        }
        sort_load_delays(&mut load_delays);
        for (drvr_index, _) in load_delays {
            let st = &view.stages[drvr_index];
            if self.bad_vertices.contains(&st.pin) {
                continue;
            }
            let line = format!("{} {} fanout = {}", st.pin, st.cell.as_deref().unwrap_or("none"), st.fanout);
            self.debug(3, line);
            if self.downsize_drvr(view, drvr_index, path_slack)? {
                return Ok(Some(view.stages[drvr_index].pin.clone()));
            }
        }
        Ok(None)
    }

    /// `downsizeDrvr(drvr_path, drvr_index, expanded, only_same_size_swap = true, path_slack)`:
    /// the driver's load, the input port the path entered by (none: nothing), the previous
    /// driver's drive resistance (its link port); unless the instance is dont_touch, the cell
    /// `downsizeCell` picks, replaced (`replaceCell`).
    fn downsize_drvr(&mut self, view: &PathView, drvr_index: usize, path_slack: f32) -> Result<bool, Stop> {
        let st = &view.stages[drvr_index];
        let Some(in_port) = st.in_port.as_deref() else { return Ok(false) };
        let (Some(inst), Some(cell), Some(port)) = (st.inst.as_deref(), st.cell.as_deref(), st.port.as_deref()) else { return Ok(false) };
        if self.design.net_info().dont_touch_insts.contains(inst) {
            return Ok(false);
        }
        let prev_drive = match drvr_index.checked_sub(2).map(|k| &view.stages[k]) {
            Some(super::Stage { cell: Some(c), port: Some(p), .. }) => self.ctx.libs.link_cell(c).map_or(0.0, |c| c.drive_resistance(p)),
            _ => 0.0,
        };
        let Some(downsize) = self.downsize_cell(in_port, cell, port, st.load_cap, prev_drive, view.scene, path_slack)? else { return Ok(false) };
        self.debug(3, format!("resize {} {} -> {}", st.pin, cell, downsize));
        // `replaceCell(drvr, downsize, journal)`: `designAreaIncr(-area(old))`, the swap,
        // `designAreaIncr(area(new))`, each area narrowed to `float`.
        let (old_area, new_area) = (self.design.master_area(cell), self.design.master_area(&downsize));
        self.design_area += f64::from(-(old_area as f32));
        self.design.swap_master(inst, &downsize).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        self.design_area += f64::from(new_area as f32);
        Ok(true)
    }

    /// `meetsSizeCriteria(cell, candidate, match_size)`: no wider, the same height.
    fn meets_size_criteria(&self, cell: &str, candidate: &str, match_size: bool) -> bool {
        if !match_size {
            return true;
        }
        let masters = self.ctx.sizing.masters;
        match (masters.get(candidate), masters.get(cell)) {
            (Some(c), Some(cur)) => c.width <= cur.width && c.height == cur.height,
            _ => false,
        }
    }

    /// `downsizeCell(in_port, drvr_port, load_cap, prev_drive, scene, max, match_size, path_slack)`:
    /// the swappable cells sorted (`std::stable_sort`) by the driver port's drive resistance at the
    /// scene, ascending, then its intrinsic delay, descending; the LAST that drives weaker, is
    /// slower — `gateDelay` at the load plus the previous driver's resistance times the input
    /// capacitance — by less than the path slack with a 1.5 margin, is not dont_use, and fits.
    #[allow(clippy::too_many_arguments)]
    fn downsize_cell(&self, in_port: &str, cell: &str, drvr_port: &str, load_cap: f32, prev_drive: f32, scene: usize, path_slack: f32) -> Result<Option<String>, Stop> {
        let libs = self.ctx.libs;
        let sizing = self.ctx.sizing;
        let mut swappable = sizing.swappable_cells(cell)?;
        if swappable.is_empty() {
            return Ok(None);
        }
        let key = |name: &str| {
            let c = libs.scene_cell(scene, name).expect("a swappable cell at the scene");
            (c.drive_resistance(drvr_port), port_intrinsic_delay(c, drvr_port))
        };
        // `std::tie(drive1, intrinsic2) < std::tie(drive2, intrinsic1)`.
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        let less = |a: &str, b: &str| {
            let ((d1, i1), (d2, i2)) = (key(a), key(b));
            d1 < d2 || (!(d2 < d1) && i2 < i1)
        };
        swappable.sort_by(|a, b| {
            if less(a, b) {
                std::cmp::Ordering::Less
            } else if less(b, a) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });
        let Some(c) = libs.scene_cell(scene, cell) else { return Ok(None) };
        let drive = c.drive_resistance(drvr_port);
        let delay = gate_delay(sizing, c, drvr_port, load_cap) + prev_drive * port_capacitance(c, in_port).unwrap_or(0.0);
        let mut best_cell = None;
        for name in &swappable {
            let Some(s) = libs.scene_cell(scene, name) else { continue };
            let current_drive = s.drive_resistance(drvr_port);
            let current_delay = gate_delay(sizing, s, drvr_port, load_cap) + prev_drive * port_capacitance(s, in_port).unwrap_or(0.0);
            if !sizing.dont_use.contains(name) && current_drive > drive && current_delay > delay && f64::from(current_delay - delay) * DELAY_MARGIN < f64::from(path_slack) && self.meets_size_criteria(cell, name, true) {
                best_cell = Some(name.clone());
            }
        }
        Ok(best_cell)
    }

    /// `printProgress(iteration, force, end)`: the header at iteration 0; a row every
    /// `print_interval_` iterations, when forced, or at the end ("final"), then the rule.
    fn print_progress(&mut self, iteration: i64, force: bool, end: bool) {
        let start = iteration == 0;
        if start && !end {
            self.report("Iteration |   Area    |  Resized |   WNS    | Endpt".into());
            self.report("---------------------------------------------------".into());
        }
        if iteration % self.print_interval == 0 || force || end {
            let (wns, worst) = self.timing.worst();
            let field = if end { "final".to_string() } else { iteration.to_string() };
            let design_area = self.design.design_area();
            let area_growth_percent = if self.initial_area.abs() > 0.0 { (design_area - self.initial_area) / self.initial_area * 100.0 } else { f64::INFINITY };
            let row = format!("{:>9} | {:>+8.1}% | {:>8} | {:>8} | {}", field, area_growth_percent, self.out.resized, self.ds(wns), worst.unwrap_or_default());
            self.report(row);
        }
        if end {
            self.report("---------------------------------------------------".into());
        }
    }

    /// `Resizer::journalBegin` / `journalEnd` / `journalRestore` (which re-inits the resizer:
    /// `design_area_` recomputed).
    fn begin_journal(&mut self) -> Result<(), Stop> {
        self.design.begin_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))
    }

    fn commit_journal(&mut self) -> Result<(), Stop> {
        self.design.commit_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))
    }

    fn restore_journal(&mut self) -> Result<(), Stop> {
        self.design.restore_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        self.design_area = self.design.design_area();
        self.retime(&[])
    }
}

/// `recoverPower(path)`'s ranking, `pair1.second > pair2.second || (== && pair1.first <
/// pair2.first)`: load delay descending, then the EARLIER path index — the reverse of the setup
/// repair's tie-break. A total order on distinct indices, so any sort gives the reference's.
fn sort_load_delays(v: &mut [(usize, f32)]) {
    v.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal).then(a.0.cmp(&b.0)));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reference sorts load delays descending, ties to the EARLIER path index.
    #[test]
    fn drivers_rank_by_load_delay_then_earlier_index() {
        let mut v = vec![(5usize, 0.1f32), (3, 0.2), (2, 0.1), (7, 0.3)];
        sort_load_delays(&mut v);
        assert_eq!(v, vec![(7, 0.3), (3, 0.2), (2, 0.1), (5, 0.1)]);
    }
}
