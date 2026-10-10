//! `MeasuredVtSwapPolicy` (`-phases MEASURED_VT_SWAP`, experimental): one VT swap at a time on the
//! worst violating endpoint's worst path, the swappable driver with the largest stage delay; each
//! VT-equivalent cell is MEASURED — applied in a journal, the timer updated, the driver's arrival
//! read, the journal undone and the timer updated again — and the best improvement committed.
//!
//! Each function is one of the policy's (or its generator's, or its candidate's), in the reference's
//! call order; the sequencers do no work of their own.

use std::collections::BTreeSet;

use vyges_sta::fuzzy;

use super::{replacement_preserves_max_cap, MoveResult, PathView, Repair, Stop};
use crate::repair_timing::{collect_violating, Move};

/// The policy's run state (`MeasuredVtSwapPolicy`'s members).
#[derive(Default)]
pub(super) struct MeasuredVtSwap {
    committed_moves: i32,
    iteration_index: i32,
    attempt_index: i32,
    /// `exhausted_endpoints_` (by endpoint pin) and `exhausted_instances_` (by odb id: the
    /// reference keys by `Instance*`). Only looked up, never iterated: their order is no value.
    exhausted_endpoints: BTreeSet<String>,
    exhausted_instances: BTreeSet<u32>,
    converged: bool,
}

/// The chosen stage (`Target` of the path driver view): its path index, pin, instance, cell, and
/// the path's scene.
struct StageTarget {
    index: usize,
    pin: String,
    inst: String,
    cell: String,
    scene: usize,
}

/// `Estimate`: `legal` and the score (the driver's arrival improvement).
#[derive(Clone, Copy)]
struct Estimate {
    legal: bool,
    score: f32,
}

/// C's `%a` of a double, as glibc prints it: `0x1.<hex, trailing zeros dropped>p<exp>`, `0x0p+0`
/// for zero, `inf` / `-inf` — the instrumented reference prints its floats so (`VYGV` lines).
pub(super) fn c_hex(v: f64) -> String {
    if v.is_infinite() {
        return if v < 0.0 { "-inf".into() } else { "inf".into() };
    }
    if v.is_nan() {
        return "nan".into();
    }
    let sign = if v.is_sign_negative() { "-" } else { "" };
    if v == 0.0 {
        return format!("{sign}0x0p+0");
    }
    let bits = v.to_bits();
    let mut exp = ((bits >> 52) & 0x7ff) as i64;
    let mant = bits & ((1u64 << 52) - 1);
    let lead = if exp == 0 {
        // Subnormal: glibc prints 0x0.<mant>p-1022.
        exp = -1022;
        0
    } else {
        exp -= 1023;
        1
    };
    let mut digits = format!("{mant:013x}");
    while digits.ends_with('0') {
        digits.pop();
    }
    let frac = if digits.is_empty() { String::new() } else { format!(".{digits}") };
    let esign = if exp < 0 { "-" } else { "+" };
    format!("{sign}0x{lead}{frac}p{esign}{}", exp.abs())
}

impl Repair<'_, '_> {
    /// One `VYGV|` decision line, as the instrumented reference prints it (`rsz-mvt-patch.py`), to
    /// `VYGES_RSZ_MVT_TRACE=<file>` (a diagnostic: no decision reads it).
    fn mvt_trace(&self, line: String) {
        let Some(path) = std::env::var_os("VYGES_RSZ_MVT_TRACE") else { return };
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = writeln!(f, "VYGV|{line}");
        }
    }

    /// `Optimizer::run` for the phase: `start()`, then `iterate()` until converged.
    pub(super) fn measured_vt_swap_policy(&mut self) -> Result<(), Stop> {
        let mut p = MeasuredVtSwap::default();
        self.mvt_start(&mut p)?;
        while !p.converged {
            self.mvt_iterate(&mut p)?;
        }
        // Back to the timer the other phases read.
        self.timer.driver_arrivals = false;
        Ok(())
    }

    /// `MeasuredVtSwapPolicy::start`: the base's start (RSZ-2024, the policy tunables), then the
    /// whole policy skipped when the library has no VT alternative.
    fn mvt_start(&mut self, p: &mut MeasuredVtSwap) -> Result<(), Stop> {
        self.report("[WARNING RSZ-2024] Experimental repair setup policy 'MeasuredVtSwapPolicy' selected. Do not use this for production.".to_string());
        // Every measure reads a driver's arrival: each snapshot keeps them from here, this one
        // too (no edit since the last: the same timing, read again).
        self.timer.driver_arrivals = true;
        self.retime_without_estimate()?;
        if self.args.skip_vt_swap || self.ctx.vt_category_count < 2 {
            let clean = !self.has_setup_violations();
            self.mvt_finish_run(p, clean);
        }
        Ok(())
    }

    /// `OptimizationPolicy::hasSetupViolations`: `fuzzyLess(worstSlack(max), setup_slack_margin)`.
    fn has_setup_violations(&self) -> bool {
        fuzzy::less(self.timing.worst().0, self.ctx.margin)
    }

    /// `MeasuredVtSwapPolicy::iterate`: one pass — the worst endpoint not exhausted, one VT-swap
    /// stage on it, until the design is clean, the move cap is reached or no endpoint is left; then
    /// converged unless the pass improved TNS.
    fn mvt_iterate(&mut self, p: &mut MeasuredVtSwap) -> Result<(), Stop> {
        if p.converged {
            return Ok(());
        }
        if !self.has_setup_violations() {
            self.mvt_finish_run(p, true);
            return Ok(());
        }
        let tns_before = self.timing.tns();
        self.mvt_trace(format!("iter|{}|{}", p.iteration_index, c_hex(f64::from(tns_before))));
        p.attempt_index = 0;
        p.exhausted_endpoints.clear();
        p.exhausted_instances.clear();
        let max_moves = self.ctx.policy.max_committed_moves;
        while self.has_setup_violations() && (max_moves <= 0 || p.committed_moves < max_moves) {
            let Some((endpoint, slack)) = self.mvt_find_worst_violating_endpoint(p) else {
                self.mvt_trace(format!("ep|{}|-", p.attempt_index));
                let violating = self.mvt_count_violating_endpoints();
                let line = format!("MeasuredVtSwapPolicy pass exhausted all viable endpoints after {} committed moves (violating_endpoints={violating}, exhausted_endpoints={}, exhausted_instances={})", p.committed_moves, p.exhausted_endpoints.len(), p.exhausted_instances.len());
                self.debug("repair_setup", 1, line);
                break;
            };
            self.mvt_trace(format!("ep|{}|{endpoint}|{}", p.attempt_index, c_hex(f64::from(slack))));
            let Some(path) = self.mvt_find_worst_slack_path(&endpoint)? else {
                self.mvt_trace(format!("nopath|{}", p.attempt_index));
                p.exhausted_endpoints.insert(endpoint);
                p.attempt_index += 1;
                continue;
            };
            let Some(target) = self.mvt_select_largest_stage_delay_target(p, &path)? else {
                p.exhausted_endpoints.insert(endpoint);
                p.attempt_index += 1;
                continue;
            };
            if !self.mvt_estimate_and_commit_best_candidate(p, &path, &target)? {
                self.mvt_trace(format!("exhaust_inst|{}", target.inst));
                let id = self.design.inst_id(&target.inst);
                p.exhausted_instances.insert(id);
                p.attempt_index += 1;
                continue;
            }
            p.attempt_index += 1;
        }
        self.mvt_trace(format!("iterend|{}|{}", p.committed_moves, c_hex(f64::from(self.timing.tns()))));
        if !self.has_setup_violations() {
            self.mvt_finish_run(p, true);
            return Ok(());
        }
        if max_moves > 0 && p.committed_moves >= max_moves {
            self.mvt_finish_run(p, false);
            return Ok(());
        }
        if !fuzzy::less(tns_before, self.timing.tns()) {
            self.mvt_finish_run(p, false);
            return Ok(());
        }
        p.iteration_index += 1;
        Ok(())
    }

    /// `MeasuredVtSwapPolicy::countViolatingEndpoints` (its debug line only).
    fn mvt_count_violating_endpoints(&self) -> usize {
        collect_violating(&self.timing.ends, self.ctx.margin).iter().filter(|e| fuzzy::less(e.slack, self.ctx.margin)).count()
    }

    /// `MeasuredVtSwapPolicy::findWorstViolatingEndpoint`: over the collector's violating
    /// endpoints (by slack, STA order on a tie), the first not exhausted with the least slack —
    /// a strictly smaller slack replaces it (`endpoint_slack < worst_slack`).
    fn mvt_find_worst_violating_endpoint(&self, p: &MeasuredVtSwap) -> Option<(String, f32)> {
        let mut worst: Option<(String, f32)> = None;
        for e in collect_violating(&self.timing.ends, self.ctx.margin) {
            if p.exhausted_endpoints.contains(&e.pin) {
                continue;
            }
            if fuzzy::less(e.slack, self.ctx.margin) && worst.as_ref().is_none_or(|(_, w)| e.slack < *w) {
                worst = Some((e.pin.clone(), e.slack));
            }
        }
        worst
    }

    /// `RepairTargetCollector::findWorstSlackPath` → `Sta::vertexWorstSlackPath(endpoint, max)`.
    fn mvt_find_worst_slack_path(&mut self, endpoint: &str) -> Result<Option<PathView>, Stop> {
        if !self.timing.paths.contains_key(endpoint) {
            let edits = self.design.take_timer_edits();
            self.timer.trace_at = self.out.trace.len();
            self.timing = super::snapshot(self.ctx, self.design.as_design(), &[endpoint.to_string()], &mut self.timer, edits)?;
        }
        Ok(self.timing.paths.get(endpoint).cloned())
    }

    /// `MeasuredVtSwapPolicy::selectLargestStageDelayTarget`: over `collectPathDriverTargets` (the
    /// drivers from the start index that are not top-level ports), each with a previous arc, a
    /// liberty cell, an instance not exhausted and not VT-swapped, and VT-equivalent cells; its
    /// stage delay is the arc's delay plus the wire into the next pin. The largest (strictly; the
    /// first on a tie) is the target.
    fn mvt_select_largest_stage_delay_target(&mut self, p: &MeasuredVtSwap, path: &PathView) -> Result<Option<StageTarget>, Stop> {
        // A latch D fanin path's targets index into the main path's expansion in the reference
        // (`expanded.path(index)` of the latch segment's index): not modelled.
        if !path.latch_segments.is_empty() {
            return Err(Stop::refused("RSZ-ABSENT", "MEASURED_VT_SWAP on a path through a latch D -> Q: not modelled".into()));
        }
        let mut largest: Option<(usize, f32)> = None;
        for index in path.start..path.stages.len() {
            let st = &path.stages[index];
            // `collectExpandedPathDriverTargets`.
            if !st.is_driver || st.top_port {
                continue;
            }
            let Some(cell_delay) = st.arc_delay else {
                self.mvt_trace(format!("stage|{index}|{}|noarc", st.pin));
                continue;
            };
            let why = match (&st.cell, &st.inst) {
                (None, _) => Some("nocell"),
                (_, None) => Some("noinst"),
                (Some(cell), Some(inst)) => {
                    let id = self.design.inst_id(inst);
                    if p.exhausted_instances.contains(&id) {
                        Some("exhausted")
                    } else if self.committer.has_moves(Move::VtSwap, id) {
                        Some("moved")
                    } else if self.ctx.sizing.vt_equiv_cells(cell, self.ctx.vt_category_count).len() <= 1 {
                        Some("noequiv")
                    } else {
                        None
                    }
                }
            };
            if let Some(why) = why {
                self.mvt_trace(format!("stage|{index}|{}|{why}", st.pin));
                continue;
            }
            let mut stage_delay = cell_delay;
            // `delayIncr(stage_delay, delayDiff(next arrival, driver arrival))` when the next pin
            // is reached by a wire.
            if let Some(next) = path.stages.get(index + 1) {
                if next.in_wire_delay.is_some() {
                    stage_delay += next.arrival - st.arrival;
                }
            }
            self.mvt_trace(format!("stage|{index}|{}|=|{}|{}", st.pin, c_hex(f64::from(cell_delay)), c_hex(f64::from(stage_delay))));
            if largest.is_none_or(|(_, d)| stage_delay > d) {
                largest = Some((index, stage_delay));
            }
        }
        let Some((index, _)) = largest else { return Ok(None) };
        let st = &path.stages[index];
        // `target.canBePathDriver() && target.vertex()`: a driver stage of a path always is.
        self.mvt_trace(format!("target|{index}|{}|1", st.pin));
        let (Some(inst), Some(cell)) = (st.inst.clone(), st.cell.clone()) else { return Ok(None) };
        Ok(Some(StageTarget { index, pin: st.pin.clone(), inst, cell, scene: path.scene }))
    }

    /// `MeasuredVtSwapPolicy::estimateAndCommitBestCandidate`: the generator's candidates, each
    /// measured; the highest legal score (strictly; the first on a tie) committed, the timer
    /// updated, the pending moves accepted.
    fn mvt_estimate_and_commit_best_candidate(&mut self, p: &mut MeasuredVtSwap, path: &PathView, target: &StageTarget) -> Result<bool, Stop> {
        let candidates = self.mvt_generate(target);
        if candidates.is_empty() {
            return Ok(false);
        }
        let mut best: Option<(String, Estimate)> = None;
        for to in candidates {
            let estimate = self.mvt_candidate_estimate(path, target, &to)?;
            if !estimate.legal {
                continue;
            }
            if best.as_ref().is_none_or(|(_, b)| estimate.score > b.score) {
                best = Some((to, estimate));
            }
        }
        let Some((to, _)) = best else { return Ok(false) };
        let result = self.mvt_candidate_apply(target, &to)?;
        self.commit(result);
        self.mvt_trace(format!("commit|{}|{to}|1", target.inst));
        self.retime(&[])?;
        self.committer.accept_pending();
        p.committed_moves += 1;
        self.report(format!("[INFO RSZ-2023] MeasuredVtSwapPolicy committed {} / {} moves.", p.committed_moves, self.ctx.policy.max_committed_moves));
        Ok(true)
    }

    /// `MeasuredVtSwapGenerator::generate`: `resolveTargetContext` (not dont_touch, a logic
    /// standard cell, two VT categories or more, not VT-swapped), then `selectCandidateCells` —
    /// the VT-equivalent cells in `getVTEquivCells` order but the cell itself, capped at
    /// `RSZ_VTSWAP_CANDIDATES` when positive.
    fn mvt_generate(&self, target: &StageTarget) -> Vec<String> {
        let id = self.design.inst_id(&target.inst);
        if self.design.net_info().dont_touch_insts.contains(&target.inst)
            || !self.ctx.sizing.masters.get(&target.cell).is_some_and(|m| m.logic_std)
            || self.ctx.vt_category_count < 2
            || self.committer.has_moves(Move::VtSwap, id)
        {
            return Vec::new();
        }
        let cap = self.ctx.policy.max_candidate_generation;
        let mut out = Vec::new();
        for cell in self.ctx.sizing.vt_equiv_cells(&target.cell, self.ctx.vt_category_count) {
            if cell != target.cell {
                out.push(cell);
                if cap > 0 && out.len() >= cap as usize {
                    break;
                }
            }
        }
        out
    }

    /// `MeasuredVtSwapCandidate::estimate`: the max-capacitance guard; then in a journal the
    /// driver's arrival, the replacement, the timer updated (`updateParasiticsAndTiming`), the
    /// arrival again, the journal undone and the timer updated again. Score: the arrival
    /// improvement, legal when positive.
    fn mvt_candidate_estimate(&mut self, path: &PathView, target: &StageTarget, to: &str) -> Result<Estimate, Stop> {
        let st = &path.stages[target.index];
        if !replacement_preserves_max_cap(self.ctx.libs, &target.cell, to, &st.fanin_caps) {
            self.mvt_trace(format!("cand|{}|{}|{to}|maxcap", target.inst, target.cell));
            return Ok(Estimate { legal: false, score: 0.0 });
        }
        self.design.begin_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        let before = self.mvt_arrival_delay(target);
        self.design.swap_master(&target.inst, to).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        self.retime(&[])?;
        let after = self.mvt_arrival_delay(target);
        let had_changes = self.design.restore_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        if had_changes {
            self.retime(&[])?;
        }
        let score = before - after;
        self.mvt_trace(format!("cand|{}|{}|{to}|{}|{}|{}", target.inst, target.cell, c_hex(f64::from(before)), c_hex(f64::from(after)), c_hex(f64::from(score))));
        Ok(Estimate { legal: score > 0.0, score })
    }

    /// `MeasuredVtSwapCandidate::arrivalDelay`: `Sta::arrival(driver_vertex, riseFall, {scene},
    /// max)` on the timer as it is now.
    fn mvt_arrival_delay(&self, target: &StageTarget) -> f32 {
        self.timing.driver_arrivals.get(target.scene).and_then(|m| m.get(&target.pin)).copied().unwrap_or(-vyges_sta::search::INF_SLACK)
    }

    /// `MeasuredVtSwapCandidate::apply`: `replaceCell`, one VtSwap move on the instance.
    fn mvt_candidate_apply(&mut self, target: &StageTarget, to: &str) -> Result<MoveResult, Stop> {
        self.design.swap_master(&target.inst, to).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        self.debug("opt_moves", 1, format!("ACCEPT measured_vt_swap {}: {} -> {to}", target.pin, target.cell));
        Ok(MoveResult { kind: Move::VtSwap, count: 1, insts: vec![target.inst.clone()] })
    }

    /// `MeasuredVtSwapPolicy::finishRun` → `markRunComplete`.
    fn mvt_finish_run(&mut self, p: &mut MeasuredVtSwap, _result: bool) {
        p.converged = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The instrumented reference prints its floats with C's `%a` (glibc): ours must print the
    // same text for the decision traces to be compared line by line.
    #[test]
    fn hex_floats_print_as_glibc_percent_a() {
        assert_eq!(c_hex(1.0), "0x1p+0");
        assert_eq!(c_hex(-0.5), "-0x1p-1");
        assert_eq!(c_hex(0.0), "0x0p+0");
        assert_eq!(c_hex(f64::from(1.5f32)), "0x1.8p+0");
        assert_eq!(c_hex(f64::from(1e-12f32)), "0x1.197998p-40");
        assert_eq!(c_hex(f64::NEG_INFINITY), "-inf");
    }

    // Rule (OptimizationPolicy::loadPolicyEnvars, utl::readEnvarNonNegativeInt): unset is 0
    // (unlimited); a value must be a whole integer (std::stoi, every character parsed) and not
    // negative, else the reference throws.
    #[test]
    fn policy_tunables_read_as_the_reference_reads_them() {
        use super::super::PolicyConfig;
        let vars = |kv: &[(&str, &str)]| kv.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect();
        assert_eq!(PolicyConfig::from_env(&vars(&[])), Ok(PolicyConfig { max_candidate_generation: 0, max_committed_moves: 0, delay_estimation_levels: 1, sta_slew_bias: true }));
        assert_eq!(PolicyConfig::from_env(&vars(&[("RSZ_VTSWAP_CANDIDATES", "10"), ("RSZ_VTSWAP_MAX_MOVES", "30")])), Ok(PolicyConfig { max_candidate_generation: 10, max_committed_moves: 30, delay_estimation_levels: 1, sta_slew_bias: true }));
        // RSZ_MT_DELAY_LEVELS may be negative (treated as 0 by the estimator); RSZ_MT_SLEW_BIAS is
        // on when positive.
        assert_eq!(PolicyConfig::from_env(&vars(&[("RSZ_MT_DELAY_LEVELS", "-2"), ("RSZ_MT_SLEW_BIAS", "0")])).map(|c| (c.delay_estimation_levels, c.sta_slew_bias)), Ok((-2, false)));
        assert!(PolicyConfig::from_env(&vars(&[("RSZ_VTSWAP_MAX_MOVES", "-1")])).is_err());
        assert!(PolicyConfig::from_env(&vars(&[("RSZ_VTSWAP_CANDIDATES", "10x")])).is_err());
    }
}
