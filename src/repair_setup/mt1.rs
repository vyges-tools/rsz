//! `SetupMt1Policy` (`-phases MT1`, experimental): per pass, every driver on the worst path of
//! every violating endpoint (each pin once, editable logic cells), each scored by the delay
//! estimator (VT swaps, then same-VT size-ups) on the timer as the pass found it; the best of each
//! committed in score order — one move per instance, no timer update between them — and the timer
//! updated once at the end.
//!
//! The reference fans the candidates out to a thread pool; the result is thread-count invariant
//! (scoping slice 23), so ours runs them in order. Each function is one of the reference's, in
//! its call order.

use std::collections::BTreeSet;

use vyges_sta::fuzzy;
use vyges_sta::liberty::MAX;

use super::measured_vt_swap::c_hex;
use super::{replacement_preserves_max_cap, CapCheck, MoveResult, Repair, Stop};
use crate::delay_estimator::{self, ArcDelayState, PathPin, Reader};
use crate::repair_timing::{collect_violating, Move};

/// `Target` of the path driver view, with the prepared `arc_delay` (`None`: `buildContext` failed).
struct Mt1Target {
    pin: String,
    inst: String,
    cell: String,
    endpoint: String,
    slack: f32,
    path_index: usize,
    /// The driver's fanin nets' capacitance checks (`replacementPreservesMaxCap`).
    fanin_caps: Vec<(String, Vec<CapCheck>)>,
    arc_delay: Option<ArcDelayState>,
}

/// One generated candidate: its move type and cell.
struct Candidate {
    kind: Move,
    cell: String,
}

/// `Estimate`.
#[derive(Clone, Copy)]
struct Estimate {
    legal: bool,
    score: f32,
}

/// `TargetEvaluation`: the candidates, their estimates, the best.
struct Evaluation {
    candidates: Vec<Candidate>,
    estimates: Vec<Estimate>,
    best: Option<usize>,
}

/// The run state (`committed_moves_`, `iteration_index_`, `converged_`).
#[derive(Default)]
pub(super) struct Mt1 {
    committed_moves: i32,
    iteration_index: i32,
    converged: bool,
    /// `move_sequence_`: VtSwap (with VT alternatives, not skipped), then SizeUp.
    vt_swap: bool,
}

fn rfc(rf: usize) -> &'static str {
    if rf == vyges_sta::liberty::RISE { "^" } else { "v" }
}

impl Repair<'_, '_> {
    /// One `VYGT|` decision line, as the instrumented reference prints it (`rsz-mt1-patch.py`), to
    /// `VYGES_RSZ_MT1_TRACE=<file>` (a diagnostic: no decision reads it).
    fn mt1_trace(&self, line: String) {
        let Some(path) = std::env::var_os("VYGES_RSZ_MT1_TRACE") else { return };
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = writeln!(f, "VYGT|{line}");
        }
    }

    /// `Optimizer::run` for the phase: `start()`, then `iterate()` until converged.
    pub(super) fn mt1_policy(&mut self) -> Result<(), Stop> {
        let mut p = Mt1::default();
        self.mt1_start(&mut p);
        while !p.converged {
            self.mt1_iterate(&mut p)?;
        }
        Ok(())
    }

    /// `SetupMt1Policy::start`: the base's start (RSZ-2024, the tunables), the move sequence.
    fn mt1_start(&mut self, p: &mut Mt1) {
        self.report("[WARNING RSZ-2024] Experimental repair setup policy 'SetupMt1Policy' selected. Do not use this for production.".to_string());
        p.vt_swap = !self.args.skip_vt_swap && self.ctx.vt_category_count > 1;
    }

    /// `SetupMt1Policy::iterate`.
    fn mt1_iterate(&mut self, p: &mut Mt1) -> Result<(), Stop> {
        if p.converged {
            return Ok(());
        }
        if !fuzzy::less(self.timing.worst().0, self.ctx.margin) {
            p.converged = true;
            return Ok(());
        }
        let tns_before = self.timing.tns();
        self.mt1_trace(format!("iter|{}|{}", p.iteration_index, c_hex(f64::from(tns_before))));
        // `collectWorstEndpointTargets`, then `prepareTargets` (one timer read: the stages'
        // graph indices are the builder's).
        let targets = self.mt1_collect_and_prepare_targets()?;
        if targets.is_empty() {
            // `finishIfNoValidTargetPin`.
            p.converged = true;
            return Ok(());
        }
        let evaluations = self.mt1_generate_and_estimate_targets(p, &targets);
        self.mt1_commit_and_update_timing(p, &targets, &evaluations)?;
        let tns_after = self.timing.tns();
        self.mt1_trace(format!("iterend|{}|{}", p.committed_moves, c_hex(f64::from(tns_after))));
        // `finishIfStopConditionReached`.
        let max_moves = self.ctx.policy.max_committed_moves;
        if !fuzzy::less(self.timing.worst().0, self.ctx.margin) || (max_moves > 0 && p.committed_moves >= max_moves) || !fuzzy::less(tns_before, tns_after) {
            p.converged = true;
            return Ok(());
        }
        p.iteration_index += 1;
        Ok(())
    }

    /// `collectWorstEndpointTargets` → `collectCritPathDriverPinTargets`: the violating endpoints
    /// (by slack, STA order on a tie — the fuzzy re-sort keeps that order), each one's worst path,
    /// its drivers from the start index that are not top-level ports, each pin once and only an
    /// editable logic cell's; then `prepareTargets` → `DelayEstimator::buildContext` on each.
    fn mt1_collect_and_prepare_targets(&mut self) -> Result<Vec<Mt1Target>, Stop> {
        let ends: Vec<(String, f32)> = collect_violating(&self.timing.ends, self.ctx.margin).into_iter().map(|e| (e.pin, e.slack)).collect();
        let dont_touch = self.design.net_info().dont_touch_insts.clone();
        let (ctx, levels, bias) = (self.ctx, self.ctx.policy.delay_estimation_levels, self.ctx.policy.sta_slew_bias);
        let edits = self.design.take_timer_edits();
        self.timer.trace_at = self.out.trace.len();
        let design = self.design.as_design();
        super::timed_all(ctx, design, &mut self.timer, edits, |gs, ss, cs| {
            let mut targets = Vec::new();
            let mut visited: BTreeSet<String> = BTreeSet::new();
            for (end, slack) in &ends {
                // `vertexWorstSlackPath(end, max)` over every scene, as `snapshot` picks it.
                let mut best: Option<(usize, usize, f32)> = None;
                for k in 0..gs.len() {
                    let Some(v) = gs[k].vertices.iter().position(|x| &x.name == end) else { continue };
                    let worst = ss[k].paths[v].iter().filter(|p| p.tag.mm == MAX).map(|p| p.required - p.arrival).fold(None, |m: Option<f32>, s| match m {
                        Some(m) if !fuzzy::less(s, m) => Some(m),
                        _ => Some(s),
                    });
                    if let Some(s) = worst {
                        if best.is_none_or(|(_, _, b)| fuzzy::less(s, b)) {
                            best = Some((k, v, s));
                        }
                    }
                }
                let Some((k, v, _)) = best else { continue };
                let ideal = if ctx.ideal_clock { cs[k].clone() } else { BTreeSet::new() };
                let Some(view) = super::expand(ctx, gs, ss, cs, design, k, v, &ideal)? else { continue };
                if !view.latch_segments.is_empty() {
                    return Err(Stop::refused("RSZ-ABSENT", "MT1 on a path through a latch D -> Q (its D fanin targets): not modelled".into()));
                }
                let path: Vec<PathPin> = view.stages.iter().map(|s| PathPin { vertex: s.vertex, in_edge: s.in_edge }).collect();
                let reader = Reader { libs: ctx.libs, g: &gs[k], parasitics: design.parasitics(k), scene: k, equiv: ctx.sizing.equiv, dont_use: ctx.sizing.dont_use };
                for index in view.start..view.stages.len() {
                    let st = &view.stages[index];
                    if !st.is_driver || st.top_port || visited.contains(&st.pin) {
                        continue;
                    }
                    // `isEditableLogicStdCell`.
                    let (Some(inst), Some(cell)) = (&st.inst, &st.cell) else { continue };
                    if dont_touch.contains(inst) || !ctx.sizing.masters.get(cell).is_some_and(|m| m.logic_std) {
                        continue;
                    }
                    visited.insert(st.pin.clone());
                    let arc_delay = reader.build_context(&path, view.start, index, levels, bias);
                    targets.push(Mt1Target { pin: st.pin.clone(), inst: inst.clone(), cell: cell.clone(), endpoint: end.clone(), slack: *slack, path_index: index, fanin_caps: st.fanin_caps.clone(), arc_delay });
                }
            }
            Ok(targets)
        })
        .inspect(|targets| self.mt1_trace_targets(targets))
    }

    /// The targets and their prepared contexts, as the instrumented reference prints them.
    fn mt1_trace_targets(&self, targets: &[Mt1Target]) {
        if std::env::var_os("VYGES_RSZ_MT1_TRACE").is_none() {
            return;
        }
        for (i, t) in targets.iter().enumerate() {
            self.mt1_trace(format!("tgt|{i}|{}|{}|{}|{}", t.pin, t.path_index, t.endpoint, c_hex(f64::from(t.slack))));
        }
        let libs = self.ctx.libs;
        for (i, t) in targets.iter().enumerate() {
            let Some(ad) = &t.arc_delay else {
                self.mt1_trace(format!("ctx|{i}|-"));
                continue;
            };
            self.mt1_trace(format!("ctx|{i}|{}|{}|{}", ad.target_stage_index, ad.path_stages.len(), c_hex(f64::from(ad.current_total_delay))));
            for (k, s) in ad.path_stages.iter().enumerate() {
                let Some(cell) = libs.scene_cell(s.arc.scene, &s.arc.cell) else { continue };
                let a = &cell.arc_sets[s.arc.set].arcs[s.arc.arc];
                let h = |v: f32| c_hex(f64::from(v));
                self.mt1_trace(format!("st|{i}|{k}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}", s.path_index, s.driver_pin, rfc(a.from_rf), rfc(a.to_rf), h(s.input_slew), h(s.load_cap), h(s.current_model_delay), h(s.current_delay), h(s.current_model_slew), h(s.current_slew), s.output_slew_merge_arcs.len()));
                for m in &s.output_slew_merge_arcs {
                    let set = &cell.arc_sets[m.set];
                    self.mt1_trace(format!("merge|{i}|{k}|{}|{}|{}|{}", set.from, rfc(set.arcs[m.arc].from_rf), h(m.input_slew), h(m.current_model_slew)));
                }
                let b = &s.sta_slew_bias;
                match (b.valid, b.table_worst_arc) {
                    (true, Some((ws, wa))) => {
                        let set = &cell.arc_sets[ws];
                        let mut line = format!("bias|{i}|{k}|1|{}|{}|{}", set.from, rfc(set.arcs[wa].from_rf), h(b.input_slew));
                        for x in &b.samples {
                            line.push_str(&format!("|{}|{}|{}", h(x.load_cap), h(x.table_slew), h(x.sta_slew)));
                        }
                        self.mt1_trace(line);
                    }
                    _ => self.mt1_trace(format!("bias|{i}|{k}|0")),
                }
            }
        }
    }

    /// `generateAndEstimateTargets`: each target in order — `generateCandidates` (VtSwap's, then
    /// SizeUp's), `estimateCandidates` (the best: the strictly highest legal score, the first on a
    /// tie).
    fn mt1_generate_and_estimate_targets(&self, p: &Mt1, targets: &[Mt1Target]) -> Vec<Evaluation> {
        let mut out = Vec::with_capacity(targets.len());
        for (i, t) in targets.iter().enumerate() {
            let mut candidates = Vec::new();
            if p.vt_swap {
                candidates.extend(self.mt1_vt_swap_generate(t));
            }
            candidates.extend(self.mt1_size_up_generate(t));
            let estimates: Vec<Estimate> = candidates.iter().map(|c| self.mt1_estimate(t, c)).collect();
            let mut best: Option<usize> = None;
            for (k, e) in estimates.iter().enumerate() {
                if e.legal && best.is_none_or(|b| e.score > estimates[b].score) {
                    best = Some(k);
                }
            }
            for (k, e) in estimates.iter().enumerate() {
                self.mt1_trace(format!("est|{i}|{k}|{}|{}", u8::from(e.legal), c_hex(f64::from(e.score))));
            }
            match best {
                Some(b) => self.mt1_trace(format!("best|{i}|{b}|{}", c_hex(f64::from(estimates[b].score)))),
                None => self.mt1_trace(format!("best|{i}|-|0x0p+0")),
            }
            out.push(Evaluation { candidates, estimates, best });
        }
        out
    }

    /// `VtSwapMtGenerator`: `isApplicable` (prepared; not dont_touch, a logic standard cell, two
    /// VT categories or more, VT-equivalent cells), then every VT-equivalent cell but the current
    /// one in `getVTEquivCells` order, the suffix trimmed past `RSZ_VTSWAP_CANDIDATES`.
    fn mt1_vt_swap_generate(&self, t: &Mt1Target) -> Vec<Candidate> {
        if t.arc_delay.is_none() || self.design.net_info().dont_touch_insts.contains(&t.inst) || !self.ctx.sizing.masters.get(&t.cell).is_some_and(|m| m.logic_std) || self.ctx.vt_category_count < 2 {
            return Vec::new();
        }
        let equiv = self.ctx.sizing.vt_equiv_cells(&t.cell, self.ctx.vt_category_count);
        if equiv.len() <= 1 {
            return Vec::new();
        }
        let mut cells: Vec<String> = equiv.into_iter().filter(|c| c != &t.cell).collect();
        let cap = self.ctx.policy.max_candidate_generation;
        if cap > 0 && cells.len() > cap as usize {
            cells.truncate(cap as usize);
        }
        for c in &cells {
            self.mt1_trace(format!("gen|{}|vt|{c}", t.pin));
        }
        cells.into_iter().map(|cell| Candidate { kind: Move::VtSwap, cell }).collect()
    }

    /// `SizeUpMtGenerator`: `isApplicable` (an instance not dont_touch, prepared), then
    /// `findSizeUpOptions` — the swappable cells of the arc's cell in the same VT class whose
    /// output port drives no weaker — each kept when it preserves the fanin nets' max capacitance.
    fn mt1_size_up_generate(&self, t: &Mt1Target) -> Vec<Candidate> {
        let Some(ad) = &t.arc_delay else { return Vec::new() };
        if self.design.net_info().dont_touch_insts.contains(&t.inst) {
            return Vec::new();
        }
        let target = ad.target();
        let libs = self.ctx.libs;
        let Some(cell) = libs.scene_cell(target.arc.scene, &target.arc.cell) else { return Vec::new() };
        let out_port = cell.arc_sets[target.arc.set].to.clone();
        let Ok(swappable) = self.ctx.sizing.swappable_cells(&cell.name) else { return Vec::new() };
        if swappable.is_empty() || cell.port(&out_port).is_none() {
            return Vec::new();
        }
        let drive_r = cell.drive_resistance(&out_port);
        let masters = self.ctx.sizing.masters;
        let Some(current_master) = masters.get(&cell.name) else { return Vec::new() };
        let mut out = Vec::new();
        for sw in swappable {
            if sw == cell.name {
                continue;
            }
            let Some(sw_master) = masters.get(&sw) else { continue };
            // `cellVTType(..).vt_index`: the same IMPLANT set.
            if crate::sizing::implant_set(current_master) != crate::sizing::implant_set(sw_master) {
                continue;
            }
            let Some(sw_cell) = libs.scene_cell(target.arc.scene, &sw) else { continue };
            if sw_cell.port(&out_port).is_none() {
                continue;
            }
            if sw_cell.drive_resistance(&out_port) <= drive_r && replacement_preserves_max_cap(libs, &t.cell, &sw, &t.fanin_caps) {
                self.mt1_trace(format!("gen|{}|up|{sw}", t.pin));
                out.push(Candidate { kind: Move::SizeUp, cell: sw });
            }
        }
        out
    }

    /// `VtSwapMtCandidate::estimate` (the max-capacitance guard, then the estimator; a
    /// non-improving swap keeps its score) and `SizeUpMtCandidate::estimate` (the estimator).
    fn mt1_estimate(&self, t: &Mt1Target, c: &Candidate) -> Estimate {
        let Some(ad) = &t.arc_delay else { return Estimate { legal: false, score: 0.0 } };
        if c.kind == Move::VtSwap && !replacement_preserves_max_cap(self.ctx.libs, &t.cell, &c.cell, &t.fanin_caps) {
            return Estimate { legal: false, score: 0.0 };
        }
        let d = delay_estimator::estimate(self.ctx.libs, ad, &c.cell);
        match (c.kind, d.legal) {
            (_, true) => Estimate { legal: true, score: d.arrival_impr },
            (Move::VtSwap, false) if d.non_improving => Estimate { legal: false, score: d.arrival_impr },
            (Move::VtSwap, false) => Estimate { legal: false, score: 0.0 },
            (_, false) => Estimate { legal: false, score: d.arrival_impr },
        }
    }

    /// `commitAndUpdateTiming`: the targets with a best candidate, by score descending (stable;
    /// `fuzzyLess(rhs, lhs)` — no fuzzily equal pair in the witnesses, so any stable sort gives the
    /// reference's order), each committed unless the move cap is reached or its instance already
    /// was this pass; then one timer update.
    fn mt1_commit_and_update_timing(&mut self, p: &mut Mt1, targets: &[Mt1Target], evaluations: &[Evaluation]) -> Result<(), Stop> {
        let mut ranked: Vec<usize> = (0..evaluations.len()).filter(|&i| evaluations[i].best.is_some()).collect();
        let score = |i: usize| evaluations[i].estimates[evaluations[i].best.expect("ranked has a best")].score;
        ranked.sort_by(|&a, &b| {
            if fuzzy::less(score(b), score(a)) {
                std::cmp::Ordering::Less
            } else if fuzzy::less(score(a), score(b)) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });
        let max_moves = self.ctx.policy.max_committed_moves;
        let mut committed_instances: BTreeSet<u32> = BTreeSet::new();
        let mut committed_in_iteration = 0;
        for i in ranked {
            if max_moves > 0 && p.committed_moves >= max_moves {
                break;
            }
            self.mt1_trace(format!("rank|{i}|{}", c_hex(f64::from(score(i)))));
            let t = &targets[i];
            let id = self.design.inst_id(&t.inst);
            if committed_instances.contains(&id) {
                self.mt1_trace(format!("dup|{i}"));
                continue;
            }
            let c = &evaluations[i].candidates[evaluations[i].best.expect("ranked has a best")];
            let ok = self.mt1_commit_best_candidate(p, t, c)?;
            self.mt1_trace(format!("commit|{i}|{}", u8::from(ok)));
            if !ok {
                continue;
            }
            committed_in_iteration += 1;
            committed_instances.insert(id);
        }
        if committed_in_iteration > 0 {
            self.retime(&[])?;
        }
        Ok(())
    }

    /// `commitBestCandidate`: the candidate applied (a size-up re-checks max capacitance first:
    /// against the pass's stale snapshot — no re-check rejects in the witnesses), the pending moves
    /// accepted, RSZ-2023.
    fn mt1_commit_best_candidate(&mut self, p: &mut Mt1, t: &Mt1Target, c: &Candidate) -> Result<bool, Stop> {
        if c.kind == Move::SizeUp && !replacement_preserves_max_cap(self.ctx.libs, &t.cell, &c.cell, &t.fanin_caps) {
            self.debug("opt_moves", 1, format!("REJECT size_up_mt1 {}: {} -> {} max-cap re-check failed", t.pin, t.cell, c.cell));
            return Ok(false);
        }
        self.design.swap_master(&t.inst, &c.cell).map_err(|e| Stop::error("RSZ-REPLACE", e))?;
        let tag = if c.kind == Move::VtSwap { "vt_swap_mt1" } else { "size_up_mt1" };
        self.debug("opt_moves", 1, format!("ACCEPT {tag} {}: {} -> {}", t.pin, t.cell, c.cell));
        self.commit(MoveResult { kind: c.kind, count: 1, insts: vec![t.inst.clone()] });
        self.committer.accept_pending();
        p.committed_moves += 1;
        self.report(format!("[INFO RSZ-2023] SetupMt1Policy committed {} / {} moves.", p.committed_moves, self.ctx.policy.max_committed_moves));
        Ok(true)
    }
}
