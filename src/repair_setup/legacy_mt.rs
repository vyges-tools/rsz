//! `SetupLegacyMtPolicy` (`LEGACY_MT`, experimental): the LEGACY phase — its endpoints, ranked path
//! drivers and move sequence unchanged — but every target is PREPARED (the delay estimator's
//! context, `OptimizationPolicy::prepareTarget`) and VtSwap / SizeUp are the MT generators: their
//! candidates scored by the delay estimator, the strictly best legal one committed through the
//! candidate's own `apply` (`MoveCommitter::commit`), and no other tried when it is rejected. The
//! other moves run as LEGACY's do, on the caller thread.
//!
//! The reference scores the MT candidates on a thread pool; the estimates read only the prepared
//! context, so ours scores them in order. Each function is one of the reference's.

use super::mt1::{Candidate, Estimate, Mt1Target};
use super::{MoveResult, PathView, Repair, Stop};
use crate::delay_estimator::{ArcDelayState, PathPin, Reader};
use crate::repair_timing::Move;

/// `MoveType` as the reference's enum numbers it (`VYGL|commit` prints it).
fn move_type_index(m: Move) -> u8 {
    match m {
        Move::Buffer => 0,
        Move::Clone => 1,
        Move::SizeUp => 2,
        Move::SizeUpMatch => 3,
        Move::SizeDownFanout => 4,
        Move::SwapPins => 5,
        Move::VtSwap => 6,
        Move::Unbuffer => 7,
        Move::SplitLoad => 8,
        Move::Reroute => 9,
    }
}

impl Repair<'_, '_> {
    /// One `VYGL|` decision line, as the instrumented reference prints it (`rsz-lmt-patch.py`), to
    /// `VYGES_RSZ_LMT_TRACE=<file>` (a diagnostic: no decision reads it).
    pub(super) fn lmt_trace(&self, line: String) {
        let Some(path) = std::env::var_os("VYGES_RSZ_LMT_TRACE") else { return };
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = writeln!(f, "VYGL|{line}");
        }
    }

    /// `OptimizationPolicy::prepareTarget` → `prepareArcDelayState` → `DelayEstimator::buildContext`
    /// on the path driver `index` of `view`: in one timer read, the endpoint's worst path in the
    /// view's scene expanded again (the stage graph indices are that read's) and checked against
    /// the view — the timer is the one the view was read from (one repair per pass) — then the
    /// context. `None`: `buildContext` failed.
    pub(super) fn lmt_prepare_target(&mut self, view: &PathView, index: usize) -> Result<Option<ArcDelayState>, Stop> {
        let Some(end) = view.stages.last().map(|s| s.pin.clone()) else { return Ok(None) };
        let pins: Vec<&str> = view.stages.iter().map(|s| s.pin.as_str()).collect();
        let (ctx, k) = (self.ctx, view.scene);
        let (levels, bias) = (ctx.policy.delay_estimation_levels, ctx.policy.sta_slew_bias);
        let edits = self.design.take_timer_edits();
        self.timer.trace_at = self.out.trace.len();
        let design = self.design.as_design();
        super::timed_all(ctx, design, &mut self.timer, edits, |gs, ss, cs| {
            let Some(v) = gs[k].vertices.iter().position(|x| x.name == end) else {
                return Err(Stop::refused("RSZ-ABSENT", format!("LEGACY_MT: endpoint {end} not in the timer read that prepares its target")));
            };
            let ideal = if ctx.ideal_clock { cs[k].clone() } else { std::collections::BTreeSet::new() };
            let Some(again) = super::expand(ctx, gs, ss, cs, design, k, v, &ideal)? else {
                return Err(Stop::refused("RSZ-ABSENT", format!("LEGACY_MT: no path to {end} in the timer read that prepares its target")));
            };
            if again.start != view.start || again.stages.iter().map(|s| s.pin.as_str()).ne(pins.iter().copied()) {
                return Err(Stop::refused("RSZ-ABSENT", format!("LEGACY_MT: the worst path to {end} differs between the pass's read and the target's prepare: not modelled")));
            }
            let path: Vec<PathPin> = again.stages.iter().map(|s| PathPin { vertex: s.vertex, in_edge: s.in_edge }).collect();
            let reader = Reader { libs: ctx.libs, g: &gs[k], parasitics: design.parasitics(k), scene: k, equiv: ctx.sizing.equiv, dont_use: ctx.sizing.dont_use };
            Ok(reader.build_context(&path, again.start, index, levels, bias))
        })
    }

    /// The prepared target, as `tryRepairTarget` holds it; traced as the reference prints it.
    pub(super) fn lmt_target(&mut self, view: &PathView, index: usize) -> Result<Mt1Target, Stop> {
        let arc_delay = self.lmt_prepare_target(view, index)?;
        let st = &view.stages[index];
        let t = Mt1Target {
            pin: st.pin.clone(),
            inst: st.inst.clone().unwrap_or_default(),
            cell: st.cell.clone().unwrap_or_default(),
            endpoint: view.stages.last().map(|s| s.pin.clone()).unwrap_or_default(),
            slack: self.target_slack,
            path_index: index,
            fanin_caps: st.fanin_caps.clone(),
            arc_delay,
        };
        if std::env::var_os("VYGES_RSZ_LMT_TRACE").is_some() {
            self.lmt_trace(format!("tgt|{}|{index}|{}|{}", t.pin, st.fanout, u8::from(t.arc_delay.is_some())));
            if let Some(ad) = &t.arc_delay {
                // The LEGACY_MT instrumenter prints no merge arcs.
                for (tag, rest) in self.arc_delay_lines(ad).into_iter().filter(|(tag, _)| *tag != "merge") {
                    self.lmt_trace(format!("{tag}|{rest}"));
                }
            }
        }
        Ok(t)
    }

    /// `canTryGenerator` for the MT generators: `VtSwapMtGenerator::isApplicable` (prepared; an
    /// instance not dont_touch, a logic standard cell with a library cell, two VT categories or
    /// more, more than one VT-equivalent cell) and `SizeUpMtGenerator::isApplicable` (prepared, an
    /// instance not dont_touch).
    pub(super) fn lmt_is_applicable(&self, m: Move, t: &Mt1Target) -> bool {
        if t.arc_delay.is_none() || t.inst.is_empty() || self.design.net_info().dont_touch_insts.contains(&t.inst) {
            return false;
        }
        match m {
            Move::VtSwap => {
                self.ctx.sizing.masters.get(&t.cell).is_some_and(|c| c.logic_std)
                    && self.ctx.vt_category_count >= 2
                    && self.ctx.libs.link_cell(&t.cell).is_some()
                    && self.ctx.sizing.vt_equiv_cells(&t.cell, self.ctx.vt_category_count).len() > 1
            }
            _ => true,
        }
    }

    /// `estimateAndCommitMtCandidates`: the generator's candidates, each estimated
    /// (`estimateCandidatesMt`: the strictly highest legal score, the first on a tie), the best
    /// committed (`commitCandidate` → the candidate's `apply`); nothing else tried when there is
    /// none or it is rejected.
    pub(super) fn lmt_estimate_and_commit(&mut self, m: Move, t: &Mt1Target) -> Result<Option<MoveResult>, Stop> {
        let candidates: Vec<Candidate> = match m {
            Move::VtSwap => self.mt1_vt_swap_generate(t),
            _ => self.mt1_size_up_generate(t),
        };
        for c in &candidates {
            self.lmt_trace(format!("gen|{}|{}", if c.kind == Move::VtSwap { "vt" } else { "up" }, c.cell));
        }
        let estimates: Vec<Estimate> = candidates.iter().map(|c| self.mt1_estimate(t, c)).collect();
        let mut best: Option<usize> = None;
        for (k, e) in estimates.iter().enumerate() {
            self.lmt_trace(format!("est|{k}|{}|{}", u8::from(e.legal), super::measured_vt_swap::c_hex(f64::from(e.score))));
            if e.legal && best.is_none_or(|b| e.score > estimates[b].score) {
                best = Some(k);
            }
        }
        let Some(b) = best else {
            if !candidates.is_empty() {
                self.lmt_trace("best|-|0x0p+0".into());
            }
            return Ok(None);
        };
        self.lmt_trace(format!("best|{b}|{}", super::measured_vt_swap::c_hex(f64::from(estimates[b].score))));
        let c = &candidates[b];
        let result = self.lmt_apply(t, c)?;
        self.lmt_trace(format!("commit|{}|{}", u8::from(result.is_some()), move_type_index(c.kind)));
        Ok(result)
    }

    /// `SizeUpMtCandidate::apply` (max capacitance re-checked, then `replaceCell`) and
    /// `VtSwapMtCandidate::apply` (`replaceCell`).
    fn lmt_apply(&mut self, t: &Mt1Target, c: &Candidate) -> Result<Option<MoveResult>, Stop> {
        if c.kind == Move::SizeUp && !super::replacement_preserves_max_cap(self.ctx.libs, &t.cell, &c.cell, &t.fanin_caps) {
            self.debug("opt_moves", 1, format!("REJECT size_up_mt1 {}: {} -> {} max-cap re-check failed", t.pin, t.cell, c.cell));
            return Ok(None);
        }
        self.design.swap_master(&t.inst, &c.cell).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        let tag = if c.kind == Move::VtSwap { "vt_swap_mt1" } else { "size_up_mt1" };
        self.debug("opt_moves", 1, format!("ACCEPT {tag} {}: {} -> {}", t.pin, t.cell, c.cell));
        Ok(Some(MoveResult { kind: c.kind, count: 1, insts: vec![t.inst.clone()] }))
    }
}

#[cfg(test)]
mod tests {
    use super::move_type_index;
    use crate::repair_timing::Move;

    /// `MoveType`'s declaration order (Buffer, Clone, SizeUp, SizeUpMatch, SizeDownFanout,
    /// SwapPins, VtSwap, Unbuffer, SplitLoad, Reroute): the number a `VYGL|commit` line prints.
    #[test]
    fn move_type_numbers_follow_the_declaration_order() {
        let order = [Move::Buffer, Move::Clone, Move::SizeUp, Move::SizeUpMatch, Move::SizeDownFanout, Move::SwapPins, Move::VtSwap, Move::Unbuffer, Move::SplitLoad, Move::Reroute];
        for (k, m) in order.into_iter().enumerate() {
            assert_eq!(usize::from(move_type_index(m)), k);
        }
    }
}
