//! The move tracker at level 2 (`set_debug_level RSZ move_tracker 2`), as `MoveCommitter` feeds
//! it: each hook here is one of the committer's, called where the reference's policy calls it, and
//! a no-op below level 2 (`moveTrackerEnabled(2)`; the level-1 hooks find nothing to finalize).
//! The state and the reports are [`move_tracker::Moves`].

use std::collections::{BTreeSet, HashMap};

use vyges_sta::fuzzy;
use vyges_sta::graph::Graph;
use vyges_sta::liberty::MAX;
use vyges_sta::search::Search;

use super::{timed_all, Repair, Stop, INF};
use crate::move_tracker;
use crate::repair_timing::Move;

/// `moveName`.
pub(super) fn move_name(m: Move) -> &'static str {
    match m {
        Move::Buffer => "BufferMove",
        Move::Clone => "CloneMove",
        Move::SizeUp => "SizeUpMove",
        Move::SizeUpMatch => "SizeUpMatchMove",
        Move::SizeDownFanout => "SizeDownFanoutMove",
        Move::SwapPins => "SwapPinsMove",
        Move::VtSwap => "VtSwapMove",
        Move::Unbuffer => "UnbufferMove",
        Move::SplitLoad => "SplitLoadMove",
        Move::Reroute => "RerouteMove",
    }
}

/// `Sta::slack(pin, max)` over every scene and the pin's vertices, both transitions: the fuzzily
/// least (the level-1 view's endpoint slack).
fn pin_slack(gs: &[Graph<'_>], ss: &[Search<'_, '_>], name: &str) -> Option<f32> {
    let mut s = INF;
    let mut any = false;
    for (k, g) in gs.iter().enumerate() {
        for (u, x) in g.vertices.iter().enumerate() {
            if x.name == name {
                any = true;
                let v = ss[k].slack_of(u, MAX, None);
                if fuzzy::less(v, s) {
                    s = v;
                }
            }
        }
    }
    any.then_some(s)
}

impl Repair<'_, '_> {
    /// The pin's odb terminal id (the tracker's key).
    fn trk_id(&self, pin: &str) -> Result<u64, Stop> {
        self.design.net_info().pin_id.get(pin).copied().ok_or_else(|| Stop::refused("RSZ-ABSENT", format!("the move tracker at level 2 keys pin {pin}, which is on no net: not modelled")))
    }

    /// The tracker's slack reads (`Sta::slack` brings the timer up to date on the edits so far; no
    /// parasitics are estimated), by terminal id.
    fn trk_slacks(&mut self, ids: &[u64]) -> Result<HashMap<u64, f32>, Stop> {
        let names: HashMap<u64, String> = self.design.net_info().pin_id.iter().filter(|(_, id)| ids.contains(id)).map(|(n, &id)| (id, n.clone())).collect();
        let edits = self.design.take_timer_edits();
        let ctx = self.ctx;
        let design = self.design.as_design();
        timed_all(ctx, design, &mut self.timer, edits, |gs, ss, _: &[BTreeSet<usize>]| {
            let mut out = HashMap::new();
            for (&id, name) in &names {
                if let Some(s) = pin_slack(gs, ss, name) {
                    out.insert(id, s);
                }
            }
            Ok(out)
        })
    }

    /// `MoveCommitter::setCurrentEndpoint`.
    pub(super) fn trk_set_current_endpoint(&mut self, pin: &str) -> Result<(), Stop> {
        if self.moves.is_none() {
            return Ok(());
        }
        let id = self.trk_id(pin)?;
        let slack = if self.moves.as_ref().is_some_and(|m| !m.knows_endpoint(id)) { self.trk_slacks(&[id])?.get(&id).copied() } else { None };
        if let Some(m) = self.moves.as_mut() {
            m.set_current_endpoint(id, slack);
        }
        Ok(())
    }

    /// `MoveCommitter::trackViolatorWithTimingInfo`: `pin_slack` the driver's slack as the pass
    /// read it (only Category 1's -1 ms test reads it), `endpoint_slack` the focus or path slack.
    pub(super) fn trk_violator(&mut self, pin: &str, pin_slack: f32, endpoint_slack: f32) -> Result<(), Stop> {
        if self.moves.is_none() {
            return Ok(());
        }
        let id = self.trk_id(pin)?;
        if let Some(m) = self.moves.as_mut() {
            m.track_violator_with_info(id, pin, pin_slack, endpoint_slack);
        }
        Ok(())
    }

    /// `MoveCommitter::trackMoveAttempt`: a candidate whose estimate is legal, before its apply.
    pub(super) fn trk_attempt(&mut self, pin: &str, m: Move) -> Result<(), Stop> {
        if self.moves.is_none() {
            return Ok(());
        }
        let id = self.trk_id(pin)?;
        if let Some(t) = self.moves.as_mut() {
            t.track_move(id, move_name(m));
        }
        Ok(())
    }

    /// `MoveCommitter::capturePrePhaseSlack`.
    pub(super) fn trk_capture_pre_phase_slack(&mut self) -> Result<(), Stop> {
        let Some(ids) = self.moves.as_ref().map(|m| m.endpoint_slack_pins()) else { return Ok(()) };
        let slacks = self.trk_slacks(&ids)?;
        if let Some(m) = self.moves.as_mut() {
            m.capture_pre_phase_slack(&|id| slacks.get(&id).copied());
        }
        Ok(())
    }

    /// The reference's `const Pin*` of a terminal (its address from the capture, with dbNetwork's
    /// tag).
    pub(super) fn trk_pointer(&self) -> impl Fn(u64) -> Option<u64> + '_ {
        let pa = self.ctx.pin_addr;
        move |id| pa.and_then(|pa| move_tracker::pin_pointer(|i| pa.addr(i), id))
    }

    /// `MoveCommitter::printTrackerPhaseSummary(summary, profiler, include)`: the pass's moves,
    /// the endpoint profile (level 1: its empty line), then the pass cleared.
    pub(super) fn trk_phase_summary(&mut self, summary: &str, profiler: Option<&str>) -> Result<(), Stop> {
        if self.tracker.is_none() {
            return Ok(());
        }
        let Some(mut m) = self.moves.take() else {
            if let Some(p) = profiler {
                self.tracker_line(move_tracker::endpoint_summary(p));
            }
            return Ok(());
        };
        let mut lines = m.print_move_summary(summary);
        if let Some(p) = profiler {
            let slacks = if m.has_endpoint_profile() { self.trk_slacks(&m.endpoint_slack_pins())? } else { HashMap::new() };
            let names: HashMap<u64, String> = self.design.net_info().pin_id.iter().map(|(n, &id)| (id, n.clone())).collect();
            let ptr = self.trk_pointer();
            let profile = m.print_endpoint_summary(p, &|id| slacks.get(&id).copied(), &ptr, &|id| names.get(&id).cloned().unwrap_or_default());
            lines.extend(profile.map_err(|e| Stop::refused("RSZ-ABSENT", e))?);
        }
        m.clear();
        self.moves = Some(m);
        for l in lines {
            self.tracker_line(l);
        }
        Ok(())
    }
}
