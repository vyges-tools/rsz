//! `GlobalSizingPolicy` (`-phases GLOBAL_SIZING`): Lagrangian-relaxation global sizing. Per-edge
//! multipliers λ (seeded from the arc delays) and per-endpoint μ (from the violations) are balanced
//! by a reverse-topological projection; each sweep scores every gate's swappable cells on leakage
//! plus λ-weighted delays (`LRSubproblem`) against a frozen snapshot, applies the winners, retimes,
//! and keeps or rolls back the sweep on WNS — all inside a journal the phase commits only when the
//! final WNS is no worse than at its start.
//!
//! λ is keyed by the reference's edge identity: an edge persists across a cell swap with
//! equivalent arcs (the only swaps here — a swap that is not is refused), so an edge is named by
//! its pins and arc set. The reference visits a vertex's in- and out-edges newest first; ours are
//! in creation order, so they are walked in reverse.
//!
//! Each function is one of the reference's (policy or subproblem), in its call order.

use std::collections::{BTreeSet, HashMap};

use vyges_sta::fuzzy;
use vyges_sta::graph::{EdgeKind, Graph, NetParasitics};
use vyges_sta::liberty::{Direction, Role, MAX};
use vyges_sta::netlist::Conn;
use vyges_sta::search::Search;

use super::measured_vt_swap::c_hex;
use super::{Repair, Stop};

/// `GlobalSizingConfig`, as `Resizer::initBlock` reads it from the block's `gs_*` properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalSizingConfig {
    /// 0 disabled, 1 `min_size_max_vt`, 2 `max_size_min_vt`.
    pub presize_mode: u8,
    pub include_clock_network: bool,
    pub setup_slack_margin: f32,
    pub max_iterations: i32,
    pub beta: f32,
    pub mu_exponent: f32,
    pub lambda_floor: f32,
    pub timing_bias: f32,
    pub budget_safety_factor: f32,
}

impl Default for GlobalSizingConfig {
    fn default() -> Self {
        GlobalSizingConfig { presize_mode: 0, include_clock_network: false, setup_slack_margin: 0.0, max_iterations: 20, beta: 0.6, mu_exponent: 2.0, lambda_floor: 1e-12, timing_bias: 64.0, budget_safety_factor: 1.0 }
    }
}

/// `sta::fuzzyGreaterEqual`.
fn fuzzy_greater_equal(a: f32, b: f32) -> bool {
    a >= b || fuzzy::equal(a, b)
}

const ARRIVAL_SENTINEL: f32 = 1e6;
const SLACK_SENTINEL: f32 = 1e6;

/// printf's `%.<p>g` of a double (fmt's `{:.<p>g}`): `p` significant digits, trailing zeros and a
/// trailing point dropped, scientific when the exponent is below -4 or at least `p` (`e-08`).
pub(super) fn fmt_g(v: f64, p: usize) -> String {
    if v == 0.0 {
        return "0".into();
    }
    if !v.is_finite() {
        return if v.is_nan() { "nan".into() } else if v > 0.0 { "inf".into() } else { "-inf".into() };
    }
    let sci = format!("{:.*e}", p - 1, v);
    let (mant, exp) = sci.split_once('e').expect("{:e} has an exponent");
    let exp: i32 = exp.parse().expect("an integer exponent");
    let trim = |s: &str| -> String {
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s.to_string()
        }
    };
    if exp < -4 || exp >= p as i32 {
        format!("{}e{}{:02}", trim(mant), if exp < 0 { '-' } else { '+' }, exp.abs())
    } else {
        let decimals = (p as i32 - 1 - exp).max(0) as usize;
        trim(&format!("{:.*}", decimals, v))
    }
}

/// One graph read: the scene-0 graph, its search, its clock pins and parasitics, and lookups.
struct View<'v, 'g> {
    g: &'v Graph<'g>,
    s: &'v Search<'v, 'g>,
    clocks: &'v BTreeSet<usize>,
    parasitics: &'v HashMap<String, NetParasitics>,
    /// `(instance, port) -> (driver vertex, load vertex)`.
    pins: HashMap<(usize, String), (Option<usize>, Option<usize>)>,
}

impl<'v, 'g> View<'v, 'g> {
    fn new(g: &'v Graph<'g>, s: &'v Search<'v, 'g>, clocks: &'v BTreeSet<usize>, parasitics: &'v HashMap<String, NetParasitics>) -> Self {
        let mut pins: HashMap<(usize, String), (Option<usize>, Option<usize>)> = HashMap::new();
        for (v, vx) in g.vertices.iter().enumerate() {
            if let Conn::Inst(k, p) = &vx.conn {
                let e = pins.entry((*k, p.clone())).or_default();
                if vx.is_driver {
                    e.0 = Some(v);
                } else {
                    e.1 = Some(v);
                }
            }
        }
        View { g, s, clocks, parasitics, pins }
    }

    fn inst_of(&self, v: usize) -> Option<usize> {
        match &self.g.vertices[v].conn {
            Conn::Inst(k, _) => Some(*k),
            Conn::Port(_) => None,
        }
    }

    /// The arc set role of a gate edge (`None`: a wire).
    fn role(&self, e: usize) -> Option<Role> {
        let EdgeKind::Gate { set } = self.g.edges[e].kind else { return None };
        let vx = &self.g.vertices[self.g.edges[e].to];
        let cell = self.g.libs.get(vx.lib?)?.cells.get(vx.cell.as_deref()?)?;
        Some(cell.arc_sets[set].role)
    }

    /// `isDataArc`: not a timing check, not a latch D -> Q or EN -> Q arc.
    fn is_data_arc(&self, e: usize) -> bool {
        match self.role(e) {
            None => true,
            Some(r) => !r.is_timing_check() && r != Role::LatchDtoQ && r != Role::LatchEnToQ,
        }
    }

    /// The edge's identity across timer updates (`Edge::id` in the reference).
    fn key(&self, e: usize) -> String {
        let ed = &self.g.edges[e];
        let set = match ed.kind {
            EdgeKind::Gate { set } => set.to_string(),
            EdgeKind::Wire => "w".into(),
        };
        format!("{}>{}#{set}", self.g.vertices[ed.from].name, self.g.vertices[ed.to].name)
    }

    /// The trace's edge name (`from>to`).
    fn edge_name(&self, e: usize) -> String {
        let ed = &self.g.edges[e];
        format!("{}>{}", self.g.vertices[ed.from].name, self.g.vertices[ed.to].name)
    }

    /// `edgeMaxArcDelay`: over the edge's arcs, `std::max(delay, max)` from 0.
    fn edge_max_arc_delay(&self, e: usize) -> f32 {
        let mut max_d = 0.0f32;
        for d in &self.g.delay[e] {
            let df = d[MAX];
            max_d = if df < max_d { max_d } else { df };
        }
        max_d
    }

    /// The data edges in the reference's graph order: each vertex in order, its out-edges newest
    /// first.
    fn data_edges(&self) -> Vec<usize> {
        let mut out = Vec::new();
        for v in 0..self.g.vertices.len() {
            for &e in self.g.out_edges[v].iter().rev() {
                if self.is_data_arc(e) {
                    out.push(e);
                }
            }
        }
        out
    }

    /// The vertices in `VertexIterator` order sorted by level — `std::ranges::sort`, libc++'s
    /// unstable introsort, ascending or (`descending`) not — the order `projectFlowBalance` and
    /// `computeSlackBudgets` visit them in. Any order consistent with the data edges gives the
    /// same VALUES; the permutation of one level's vertices is the reference's visit order.
    fn level_sorted(&self, descending: bool) -> Result<Vec<usize>, Stop> {
        if let Some(why) = crate::timing::unlevelable(self.g.libs, self.g.netlist) {
            return Err(Stop::refused("RSZ-LEVELS", why));
        }
        let level = self.g.levels().map_err(|e| Stop::refused("RSZ-TIMER", e))?;
        let mut order: Vec<usize> = (0..self.g.vertices.len()).collect();
        crate::order::libcxx_sort_by(&mut order, |&a, &b| if descending { level[a] > level[b] } else { level[a] < level[b] })
            .map_err(|h| Stop::refused("RSZ-ORDER", format!("GLOBAL_SIZING: the level sort over {} vertices falls back to heap sort: not modelled", h.len)))?;
        Ok(order)
    }

    /// `Sta::arrival(vertex, riseFall, scenes, max)`: the fuzzily greatest max arrival from -INF.
    fn arrival(&self, v: usize) -> f32 {
        let mut a = -vyges_sta::search::INF_SLACK;
        for p in self.s.paths[v].iter().filter(|p| p.tag.mm == MAX) {
            if fuzzy::greater(p.arrival, a) {
                a = p.arrival;
            }
        }
        a
    }

    /// `Sta::slew(vertex, riseFall, scenes, max)`: the fuzzily greatest max slew.
    fn slew(&self, v: usize) -> f32 {
        let mut m = -vyges_sta::search::INF_SLACK;
        for rf in 0..2 {
            let x = self.g.slew[v][rf][MAX];
            if fuzzy::greater(x, m) {
                m = x;
            }
        }
        m
    }
}

/// The policy's state across its reads.
#[derive(Default)]
struct Gs {
    lambda: HashMap<String, f32>,
    endpoints: Vec<String>,
    mu: Vec<f32>,
    budget: HashMap<usize, f32>,
    leak_scale: f32,
}

/// `OutputCtx`.
#[derive(Clone)]
struct OutputCtx {
    port: String,
    load_cap: f32,
    lambda_sum: f32,
    slew: f32,
    drive_res: f32,
}

/// `UpstreamCtx`: the upstream driver's cell and port, its load, this pin's cap, Σλ.
#[derive(Clone)]
struct UpstreamCtx {
    in_port: String,
    drv_cell: String,
    drv_port: String,
    load_u: f32,
    c_in_cur: f32,
    lambda_u: f32,
}

/// `DriverCapCheck`.
#[derive(Clone, Copy)]
struct DriverCapCheck {
    cap: f32,
    max_cap: f32,
    cap_slack: f32,
    corner_ok: bool,
}

/// `InputMaxCapCtx`.
#[derive(Clone)]
struct InputMaxCapCtx {
    in_port: String,
    old_cap: f32,
    drivers: Vec<DriverCapCheck>,
}

/// `GateSnapshot`.
struct GateSnapshot {
    inst: String,
    cur_cell: String,
    cur_leakage: f32,
    budget: f32,
    outputs: Vec<OutputCtx>,
    upstream: Vec<UpstreamCtx>,
    inputs: Vec<InputMaxCapCtx>,
    candidates: Vec<(String, f32)>,
}

/// `GateDecision`.
struct GateDecision {
    inst: String,
    best_cell: Option<String>,
    best_cost: f32,
    baseline_cost: f32,
    best_is_downsize: bool,
}

/// `DesignSnap`.
#[derive(Default)]
struct DesignSnap {
    total_leakage: f64,
    total_area: f64,
}

/// The policy's read-only helpers: the context and the design as it is (no timer, no edit).
struct GsEnv<'e> {
    ctx: &'e super::Ctx<'e>,
    design: &'e dyn super::SetupDesign,
}

impl GsEnv<'_> {
    fn trace(&self, line: String) {
        let Some(path) = std::env::var_os("VYGES_RSZ_GS_TRACE") else { return };
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = writeln!(f, "VYGG|{line}");
        }
    }

    /// `portInputCap`: the max over rise and fall of the port's max capacitance, 0 without it.
    fn port_input_cap(&self, cell: &str, port: &str) -> f32 {
        let Some(c) = self.ctx.libs.link_cell(cell) else { return 0.0 };
        let Some(p) = c.port(port) else { return 0.0 };
        let mut cap = 0.0f32;
        for rf in 0..2 {
            cap = cap.max(p.capacitance[rf][MAX]);
        }
        cap
    }

    /// `Resizer::cellLeakage`.
    fn cell_leakage(&self, cell: &str) -> Option<f32> {
        self.ctx.libs.link_cell(cell).and_then(crate::sizing::cell_leakage)
    }

    /// `LRSubproblem::leakageOrArea`.
    fn leakage_or_area(&self, gs: &Gs, cell: &str) -> f32 {
        if let Some(l) = self.cell_leakage(cell) {
            return l;
        }
        let a = self.design.master_area(cell) as f32;
        if gs.leak_scale > 0.0 { gs.leak_scale * a } else { a }
    }

    /// `Resizer::gateDelay(port, load, scene, max)`: the larger of the rise and fall delays over
    /// the arcs into the port at the target slews.
    fn gate_delay(&self, cell: &str, port: &str, load: f32) -> f32 {
        let Some(c) = self.ctx.libs.link_cell(cell) else { return -vyges_sta::search::INF_SLACK };
        super::gate_delay(self.ctx.sizing, c, port, load)
    }

    /// The leaf instances in network order, each with its link cell.
    fn leaf_insts(&self) -> Vec<(usize, String, String)> {
        self.design.netlist().insts.iter().enumerate().map(|(k, (n, c))| (k, n.clone(), c.clone())).collect()
    }

    /// `computeDesignSnap`.
    fn design_snap(&self) -> DesignSnap {
        let mut s = DesignSnap::default();
        for (_, _, cell) in self.leaf_insts() {
            if self.ctx.libs.link_cell(&cell).is_none() {
                continue;
            }
            if let Some(l) = self.cell_leakage(&cell) {
                s.total_leakage += f64::from(l);
            }
            s.total_area += self.design.master_area(&cell);
        }
        s
    }

    /// `selectPresizeCell`: over the swappable cells, the least (min_size_max_vt) or most leaky,
    /// a tie on leakage broken by drive resistance (more / less); the current cell kept on a full
    /// tie.
    fn select_presize_cell(&self, current: &str, mode: u8, cache: &mut HashMap<String, String>) -> String {
        if let Some(c) = cache.get(current) {
            return c.clone();
        }
        let candidates = self.ctx.sizing.swappable_cells(current).unwrap_or_default();
        let drive = |c: &str| self.ctx.libs.link_cell(c).map_or(0.0, crate::sizing::cell_drive_resistance);
        let mut best = current.to_string();
        let mut best_leak = self.cell_leakage(&best);
        let mut best_drive: Option<f32> = None;
        for cand in candidates {
            let Some(cl) = self.cell_leakage(&cand) else { continue };
            let Some(bl) = best_leak else {
                best = cand;
                best_leak = Some(cl);
                best_drive = None;
                continue;
            };
            if cl != bl {
                let better = if mode == 1 { cl < bl } else { cl > bl };
                if better {
                    best = cand;
                    best_leak = Some(cl);
                    best_drive = None;
                }
                continue;
            }
            let bd = *best_drive.get_or_insert_with(|| drive(&best));
            let cd = drive(&cand);
            if cd == bd {
                continue;
            }
            let better = if mode == 1 { cd > bd } else { cd < bd };
            if better {
                best = cand;
                best_leak = Some(cl);
                best_drive = Some(cd);
            }
        }
        cache.insert(current.to_string(), best.clone());
        best
    }

    /// `allocate`: the data edges' λ (0), the endpoints in vertex order and their μ (0).
    fn allocate(&self, gs: &mut Gs, v: &View<'_, '_>) {
        let edges = v.data_edges();
        gs.lambda = edges.iter().map(|&e| (v.key(e), 0.0)).collect();
        gs.endpoints = (0..v.g.vertices.len()).filter(|&x| v.s.is_endpoint(x)).map(|x| v.g.vertices[x].name.clone()).collect();
        gs.mu = vec![0.0; gs.endpoints.len()];
        self.trace(format!("alloc|{}|{}", edges.len(), gs.endpoints.len()));
    }

    /// The endpoints' μ: `max(0, margin − slack)^p`, normalized to a max of 1.
    fn seed_mu(&self, gs: &mut Gs, v: &View<'_, '_>, cfg: &GlobalSizingConfig) {
        let mut mu_max_raw = 0.0f32;
        for (k, name) in gs.endpoints.iter().enumerate() {
            // `Sta::slack(endpoint, max)` of the endpoint vertex.
            let slack = (0..v.g.vertices.len()).find(|&x| &v.g.vertices[x].name == name && v.s.is_endpoint(x)).map_or(vyges_sta::search::INF_SLACK, |x| v.s.slack_of(x, MAX, None));
            let gap = cfg.setup_slack_margin - slack;
            let mu = if gap > 0.0 { gap.powf(cfg.mu_exponent) } else { 0.0 };
            gs.mu[k] = mu;
            mu_max_raw = mu_max_raw.max(mu);
        }
        if mu_max_raw > 0.0 {
            for mu in &mut gs.mu {
                *mu /= mu_max_raw;
            }
        }
        for (k, name) in gs.endpoints.iter().enumerate() {
            self.trace(format!("mu|{k}|{name}|{}", c_hex(f64::from(gs.mu[k]))));
        }
    }

    /// `seedMultipliers`: λ_e = max(edge max arc delay, floor); μ seeded.
    fn seed_multipliers(&self, gs: &mut Gs, v: &View<'_, '_>, cfg: &GlobalSizingConfig) {
        for e in v.data_edges() {
            let seed = v.edge_max_arc_delay(e).max(cfg.lambda_floor);
            gs.lambda.insert(v.key(e), seed);
            self.trace(format!("seed|{}|{}", v.edge_name(e), c_hex(f64::from(seed))));
        }
        self.seed_mu(gs, v, cfg);
    }

    /// `updateMultipliers`: μ re-seeded; each constrained data edge's λ stepped by the normalized
    /// subgradient `(d − (a_to − a_from)) / max(d, floor)`, clamped to [-1, 0].
    fn update_multipliers(&self, gs: &mut Gs, v: &View<'_, '_>, cfg: &GlobalSizingConfig) {
        self.seed_mu(gs, v, cfg);
        let alpha = cfg.beta.clamp(0.0, 1.0);
        for e in v.data_edges() {
            let key = v.key(e);
            let Some(&lam) = gs.lambda.get(&key) else { continue };
            let d = v.edge_max_arc_delay(e);
            let (a_from, a_to) = (v.arrival(v.g.edges[e].from), v.arrival(v.g.edges[e].to));
            if a_from.abs() >= ARRIVAL_SENTINEL || a_to.abs() >= ARRIVAL_SENTINEL {
                self.trace(format!("upd|{}|skip", v.edge_name(e)));
                continue;
            }
            let arrival_diff = a_to - a_from;
            let denom = d.max(cfg.lambda_floor);
            let g_norm = (d - arrival_diff) / denom;
            let g_clamped = g_norm.clamp(-1.0, 0.0);
            let scale = 1.0 + alpha * g_clamped;
            let new = (lam * scale).max(cfg.lambda_floor);
            gs.lambda.insert(key, new);
            self.trace(format!("upd|{}|{}", v.edge_name(e), c_hex(f64::from(new))));
        }
    }

    /// `projectFlowBalance`: by level, descending, each vertex's in-edge λ rescaled so their sum
    /// is its out-edges' (an endpoint's: its μ); with no in-flow, an equal share.
    fn project_flow_balance(&self, gs: &mut Gs, v: &View<'_, '_>, cfg: &GlobalSizingConfig) -> Result<(), Stop> {
        let endpoints: HashMap<&str, usize> = gs.endpoints.iter().enumerate().map(|(k, n)| (n.as_str(), k)).collect();
        for x in v.level_sorted(true)? {
            let name = v.g.vertices[x].name.as_str();
            let ep = if v.s.is_endpoint(x) { endpoints.get(name).copied() } else { None };
            let target = match ep {
                Some(k) => gs.mu[k],
                None => {
                    let mut t = 0.0f32;
                    for &e in v.g.out_edges[x].iter().rev() {
                        if v.is_data_arc(e) {
                            if let Some(&l) = gs.lambda.get(&v.key(e)) {
                                t += l;
                            }
                        }
                    }
                    t
                }
            };
            let ins: Vec<(usize, String)> = v.g.in_edges[x].iter().rev().copied().filter(|&e| v.is_data_arc(e)).map(|e| (e, v.key(e))).filter(|(_, k)| gs.lambda.contains_key(k)).collect();
            let mut in_sum = 0.0f32;
            for (_, k) in &ins {
                in_sum += gs.lambda[k];
            }
            self.trace(format!("pv|{name}|{}|{}|{}|{}", u8::from(ep.is_some()), c_hex(f64::from(target)), c_hex(f64::from(in_sum)), ins.len()));
            if ins.is_empty() {
                continue;
            }
            if in_sum > 0.0 {
                let scale = target / in_sum;
                for (e, k) in &ins {
                    let l = (gs.lambda[k] * scale).max(cfg.lambda_floor);
                    gs.lambda.insert(k.clone(), l);
                    self.trace(format!("pl|{}|{}", v.edge_name(*e), c_hex(f64::from(l))));
                }
            } else if target > 0.0 {
                let share = target / ins.len() as f32;
                for (e, k) in &ins {
                    let l = share.max(cfg.lambda_floor);
                    gs.lambda.insert(k.clone(), l);
                    self.trace(format!("pl|{}|{}", v.edge_name(*e), c_hex(f64::from(l))));
                }
            }
        }
        Ok(())
    }

    /// `LRSubproblem::computeLeakageScale`: the median leakage over the median area of the
    /// instances whose cell has both (`nth_element`'s value is the sorted one's).
    fn compute_leakage_scale(&self, gs: &mut Gs) {
        let mut leakages = Vec::new();
        let mut areas = Vec::new();
        for (_, _, cell) in self.leaf_insts() {
            if self.ctx.libs.link_cell(&cell).is_none() {
                continue;
            }
            let Some(l) = self.cell_leakage(&cell) else { continue };
            let a = self.design.master_area(&cell);
            if a <= 0.0 {
                continue;
            }
            leakages.push(l);
            areas.push(a as f32);
        }
        if leakages.is_empty() {
            gs.leak_scale = 0.0;
        } else {
            let mid = leakages.len() / 2;
            leakages.sort_by(f32::total_cmp);
            areas.sort_by(f32::total_cmp);
            let (l_med, a_med) = (leakages[mid], areas[mid]);
            gs.leak_scale = if a_med > 0.0 { l_med / a_med } else { 0.0 };
        }
        self.trace(format!("leakscale|{}", c_hex(f64::from(gs.leak_scale))));
    }

    /// Σλ over the gate-internal data arcs into a driver vertex (in-edges newest first) from the
    /// same instance `inst`.
    fn internal_lambda(&self, gs: &Gs, v: &View<'_, '_>, drv: usize, inst: usize) -> f32 {
        let mut sum = 0.0f32;
        for &e in v.g.in_edges[drv].iter().rev() {
            if !v.is_data_arc(e) || v.inst_of(v.g.edges[e].from) != Some(inst) {
                continue;
            }
            if let Some(&l) = gs.lambda.get(&v.key(e)) {
                sum += l;
            }
        }
        sum
    }

    /// `computeAutoTimingWeight`: timing_bias × median leakage / median gate timing pressure.
    fn compute_auto_timing_weight(&self, gs: &Gs, v: &View<'_, '_>, cfg: &GlobalSizingConfig) -> f32 {
        let mut leakages = Vec::new();
        let mut timings = Vec::new();
        for (k, name, cell) in self.leaf_insts() {
            if self.design.net_info().dont_touch_insts.contains(&name) || self.ctx.libs.link_cell(&cell).is_none() {
                continue;
            }
            leakages.push(self.leakage_or_area(gs, &cell));
            let mut gate_t = 0.0f32;
            let mut has_pressure = false;
            for p in self.ctx.master_pins.get(&cell).map(Vec::as_slice).unwrap_or(&[]) {
                if !self.is_output(&cell, p) {
                    continue;
                }
                let Some((Some(drv), _)) = v.pins.get(&(k, p.clone())) else { continue };
                let lam_sum = self.internal_lambda(gs, v, *drv, k);
                if lam_sum <= 4.0 * cfg.lambda_floor {
                    continue;
                }
                let load = v.g.load_cap(*drv, v.parasitics);
                let d = self.gate_delay(&cell, p, load);
                gate_t += lam_sum * d;
                has_pressure = true;
            }
            if has_pressure {
                timings.push(gate_t);
            }
        }
        let tw = if leakages.is_empty() || timings.is_empty() {
            1.0
        } else {
            let (lm, tm) = (leakages.len() / 2, timings.len() / 2);
            leakages.sort_by(f32::total_cmp);
            timings.sort_by(f32::total_cmp);
            let (l_med, t_med) = (leakages[lm], timings[tm]);
            if l_med <= 0.0 || t_med <= 0.0 { 1.0 } else { cfg.timing_bias * l_med / t_med }
        };
        self.trace(format!("tw|{}", c_hex(f64::from(tw))));
        tw
    }

    fn port_direction(&self, cell: &str, port: &str) -> Option<Direction> {
        self.ctx.libs.link_cell(cell)?.port(port).map(|p| p.direction)
    }

    fn is_output(&self, cell: &str, port: &str) -> bool {
        matches!(self.port_direction(cell, port), Some(Direction::Output))
    }

    /// `computeSlackBudgets`: per vertex `max(0, slack − margin) / depth`, depth the gate count on
    /// its longest path (gate arcs: both pins on one instance); an unconstrained vertex 1e6.
    fn compute_slack_budgets(&self, gs: &mut Gs, v: &View<'_, '_>, cfg: &GlobalSizingConfig) -> Result<(), Stop> {
        let order = v.level_sorted(false)?;
        let n = v.g.vertices.len();
        let is_gate = |e: usize| {
            let (f, t) = (v.inst_of(v.g.edges[e].from), v.inst_of(v.g.edges[e].to));
            f.is_some() && f == t
        };
        let mut fwd = vec![0i32; n];
        for &x in &order {
            let mut best = 0;
            for &e in v.g.in_edges[x].iter().rev() {
                if v.is_data_arc(e) {
                    best = best.max(fwd[v.g.edges[e].from] + i32::from(is_gate(e)));
                }
            }
            fwd[x] = best;
        }
        let mut bwd = vec![0i32; n];
        for &x in order.iter().rev() {
            let mut best = 0;
            for &e in v.g.out_edges[x].iter().rev() {
                if v.is_data_arc(e) {
                    best = best.max(bwd[v.g.edges[e].to] + i32::from(is_gate(e)));
                }
            }
            bwd[x] = best;
        }
        gs.budget.clear();
        for &x in &order {
            let depth = (fwd[x] + bwd[x]).max(1);
            let slack = v.s.slack_of(x, MAX, None);
            let b = if slack >= SLACK_SENTINEL { SLACK_SENTINEL } else { (slack - cfg.setup_slack_margin).max(0.0) / depth as f32 };
            gs.budget.insert(x, b);
            self.trace(format!("bud|{}|{}|{}|{}|{}", v.g.vertices[x].name, fwd[x], bwd[x], c_hex(f64::from(slack)), c_hex(f64::from(b))));
        }
        Ok(())
    }

    /// `buildSnapshots` → `LRSubproblem::snapshot` on each leaf instance, network order.
    fn build_snapshots(&self, gs: &Gs, v: &View<'_, '_>, cfg: &GlobalSizingConfig) -> Vec<GateSnapshot> {
        let mut out = Vec::new();
        let sc = crate::timing::Scenes::new(std::slice::from_ref(v.g));
        let ideal = if self.ctx.ideal_clock { v.clocks.clone() } else { BTreeSet::new() };
        'inst: for (k, name, cell) in self.leaf_insts() {
            if self.design.net_info().dont_touch_insts.contains(&name) || self.ctx.libs.link_cell(&cell).is_none() {
                continue;
            }
            let mut s = GateSnapshot { inst: name.clone(), cur_cell: cell.clone(), cur_leakage: 0.0, budget: f32::MAX, outputs: Vec::new(), upstream: Vec::new(), inputs: Vec::new(), candidates: Vec::new() };
            for p in self.ctx.master_pins.get(&cell).map(Vec::as_slice).unwrap_or(&[]) {
                let Some(dir) = self.port_direction(&cell, p) else { continue };
                let Some(&(drv, load)) = v.pins.get(&(k, p.clone())) else { continue };
                if dir == Direction::Output {
                    let Some(d) = drv else { continue };
                    if !cfg.include_clock_network && v.clocks.contains(&d) {
                        continue 'inst;
                    }
                    let lam_sum = self.internal_lambda(gs, v, d, k);
                    let load_cap = v.g.load_cap(d, v.parasitics);
                    let drive_res = self.ctx.libs.link_cell(&cell).map_or(0.0, |c| c.drive_resistance(p));
                    s.budget = s.budget.min(gs.budget.get(&d).copied().unwrap_or(f32::MAX));
                    s.outputs.push(OutputCtx { port: p.clone(), load_cap, lambda_sum: lam_sum, slew: v.slew(d), drive_res });
                } else if dir == Direction::Input {
                    let Some(u) = load else { continue };
                    // (a) the fanin net's drivers' capacitance checks.
                    if v.g.vertex_net[u].is_some() {
                        let drivers = crate::timing::net_drivers(v.g, u).into_iter().map(|dv| {
                            let (cap, max_cap, slack, limited, _) = crate::timing::check_capacitance(&sc, dv, std::slice::from_ref(v.parasitics), &ideal);
                            DriverCapCheck { cap, max_cap, cap_slack: slack, corner_ok: max_cap > 0.0 && limited }
                        });
                        s.inputs.push(InputMaxCapCtx { in_port: p.clone(), old_cap: self.port_input_cap(&cell, p), drivers: drivers.collect() });
                    }
                    // (b) the upstream driver: the first in-edge from another pin.
                    let Some(drv_v) = v.g.in_edges[u].iter().rev().map(|&e| v.g.edges[e].from).find(|&f| f != u) else { continue };
                    let Some(up_inst) = v.inst_of(drv_v) else { continue };
                    if up_inst == k {
                        continue;
                    }
                    let up_cell = &v.design_inst_cell(up_inst);
                    if self.ctx.libs.link_cell(up_cell).is_none() {
                        continue;
                    }
                    let Some(drv_port) = v.g.vertices[drv_v].port.clone() else { continue };
                    let lambda_u = self.internal_lambda(gs, v, drv_v, up_inst);
                    if lambda_u <= 0.0 {
                        continue;
                    }
                    s.upstream.push(UpstreamCtx { in_port: p.clone(), drv_cell: up_cell.clone(), drv_port, load_u: v.g.load_cap(drv_v, v.parasitics), c_in_cur: self.port_input_cap(&cell, p), lambda_u });
                }
            }
            if s.outputs.is_empty() {
                continue;
            }
            s.cur_leakage = self.leakage_or_area(gs, &cell);
            for cand in self.ctx.sizing.swappable_cells(&cell).unwrap_or_default() {
                if cand != cell {
                    let l = self.leakage_or_area(gs, &cand);
                    s.candidates.push((cand, l));
                }
            }
            self.trace_snapshot(&s);
            out.push(s);
        }
        out
    }

    fn trace_snapshot(&self, s: &GateSnapshot) {
        if std::env::var_os("VYGES_RSZ_GS_TRACE").is_none() {
            return;
        }
        let h = |x: f32| c_hex(f64::from(x));
        self.trace(format!("snap|{}|{}|{}|{}|{}|{}|{}|{}", s.inst, s.cur_cell, h(s.cur_leakage), h(s.budget), s.outputs.len(), s.upstream.len(), s.inputs.len(), s.candidates.len()));
        for o in &s.outputs {
            self.trace(format!("out|{}|{}|{}|{}|{}|{}", s.inst, o.port, h(o.load_cap), h(o.lambda_sum), h(o.slew), h(o.drive_res)));
        }
        for u in &s.upstream {
            self.trace(format!("up|{}|{}|{}|{}|{}|{}", s.inst, u.in_port, u.drv_port, h(u.load_u), h(u.c_in_cur), h(u.lambda_u)));
        }
        for i in &s.inputs {
            let d: String = i.drivers.iter().map(|d| format!("{},{},{},{};", h(d.cap), h(d.max_cap), h(d.cap_slack), u8::from(d.corner_ok))).collect();
            self.trace(format!("in|{}|{}|{}|{d}", s.inst, i.in_port, h(i.old_cap)));
        }
        for (c, l) in &s.candidates {
            self.trace(format!("cand|{}|{c}|{}", s.inst, h(*l)));
        }
    }

    /// `LRSubproblem::evaluateCellCost`: leakage + timing_weight × (Σ over output pins λ·d at the
    /// pin's load + Σ over upstream drivers λ·d at the load with this cell's input cap).
    fn evaluate_cell_cost(&self, s: &GateSnapshot, cell: &str, leakage: f32, tw: f32) -> f32 {
        let mut cost = leakage;
        for o in &s.outputs {
            if o.lambda_sum == 0.0 {
                continue;
            }
            if self.port_direction(cell, &o.port).is_none() {
                return f32::INFINITY;
            }
            let d = self.gate_delay(cell, &o.port, o.load_cap);
            cost += tw * o.lambda_sum * d;
        }
        for u in &s.upstream {
            if u.lambda_u == 0.0 {
                continue;
            }
            let c_in = self.port_input_cap(cell, &u.in_port);
            if c_in == 0.0 {
                return f32::INFINITY;
            }
            let load = (u.load_u - u.c_in_cur + c_in).max(0.0);
            let d = self.gate_delay(&u.drv_cell, &u.drv_port, load);
            cost += tw * u.lambda_u * d;
        }
        cost
    }

    /// `candidateDrcOkSnapshot`: the fanin nets' max capacitance (as `replacementPreservesMaxCap`)
    /// and each output's max capacitance and Elmore-estimated max slew under the new cell.
    fn candidate_drc_ok(&self, s: &GateSnapshot, cell: &str) -> bool {
        for i in &s.inputs {
            let delta = self.port_input_cap(cell, &i.in_port) - i.old_cap;
            if delta <= 0.0 {
                continue;
            }
            for d in &i.drivers {
                if !d.corner_ok {
                    continue;
                }
                let ncap = d.cap + delta;
                if (d.cap_slack < 0.0 && ncap > d.cap) || (d.cap_slack >= 0.0 && ncap > d.max_cap) {
                    return false;
                }
            }
        }
        let Some(c) = self.ctx.libs.link_cell(cell) else { return false };
        for o in &s.outputs {
            let Some(port) = c.port(&o.port) else { return false };
            // `checkOutputMaxCap`.
            if port.max_capacitance.is_some_and(|m| m > 0.0 && o.load_cap > m) {
                return false;
            }
            // `checkOutputMaxSlew`.
            let factor = if o.drive_res > 0.0 && o.load_cap > 0.0 { o.slew / (o.drive_res * o.load_cap) } else { 0.0 };
            let new_slew = factor * c.drive_resistance(&o.port) * o.load_cap;
            let lib = self.ctx.libs.scene_library(0, cell).or_else(|| self.ctx.libs.default_library());
            if let Some(lib) = lib {
                let (max_slew, exists) = crate::timing::find_slew_limit(lib, port.direction, port.max_transition, self.ctx.limits);
                if exists && new_slew > max_slew {
                    return false;
                }
            }
        }
        true
    }

    /// `downsizeFitsSlackBudget`: on every output, the added delay at the frozen load within
    /// `safety × budget` (no budget: no downsize).
    fn downsize_fits(&self, s: &GateSnapshot, cell: &str, safety: f32) -> bool {
        let budget = safety * s.budget;
        if budget <= 0.0 {
            return false;
        }
        for o in &s.outputs {
            if self.port_direction(cell, &o.port).is_none() {
                return false;
            }
            let d_cur = self.gate_delay(&s.cur_cell, &o.port, o.load_cap);
            let d_cand = self.gate_delay(cell, &o.port, o.load_cap);
            if d_cand - d_cur > budget {
                return false;
            }
        }
        true
    }

    /// `LRSubproblem::evaluateSnapshot`: the strictly cheapest DRC-clean candidate (a downsize only
    /// within budget) against the current cell's cost.
    fn evaluate_snapshot(&self, s: &GateSnapshot, tw: f32, safety: f32) -> GateDecision {
        let baseline_cost = self.evaluate_cell_cost(s, &s.cur_cell, s.cur_leakage, tw);
        let mut d = GateDecision { inst: s.inst.clone(), best_cell: None, best_cost: baseline_cost, baseline_cost, best_is_downsize: false };
        let mut best_leak = s.cur_leakage;
        for (cell, leak) in &s.candidates {
            if !self.candidate_drc_ok(s, cell) {
                continue;
            }
            if *leak < s.cur_leakage && !self.downsize_fits(s, cell, safety) {
                continue;
            }
            let cost = self.evaluate_cell_cost(s, cell, *leak, tw);
            if cost < d.best_cost {
                d.best_cost = cost;
                d.best_cell = Some(cell.clone());
                best_leak = *leak;
            }
        }
        if d.best_cell.is_some() {
            d.best_is_downsize = best_leak < s.cur_leakage;
        }
        let h = |x: f32| c_hex(f64::from(x));
        self.trace(format!("dec|{}|{}|{}|{}|{}", d.inst, d.best_cell.as_deref().unwrap_or("-"), h(d.best_cost), h(d.baseline_cost), u8::from(d.best_is_downsize)));
        d
    }
}

impl Repair<'_, '_> {
    fn gs_env(&self) -> GsEnv<'_> {
        GsEnv { ctx: self.ctx, design: &*self.design }
    }

    /// `Optimizer::run` for the phase: `start()` then the one `iterate()`.
    pub(super) fn global_sizing_policy(&mut self) -> Result<(), Stop> {
        if self.ctx.libs.scene_count() > 1 {
            return Err(Stop::refused("RSZ-ABSENT", "GLOBAL_SIZING over several corners: not modelled".into()));
        }
        if self.ctx.gs.setup_slack_margin != 0.0 {
            return Err(Stop::refused("RSZ-ABSENT", "GLOBAL_SIZING with gs_setup_slack_margin set (its unit): not modelled".into()));
        }
        self.gs_iterate()
    }

    /// `GlobalSizingPolicy::iterate`.
    fn gs_iterate(&mut self) -> Result<(), Stop> {
        let cfg = self.ctx.gs;
        let pre = self.gs_env().design_snap();
        let wns_pre = self.timing.worst().0;
        let tns_pre = self.timing.tns();
        // Outer journal: presize + LR.
        self.design.begin_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        self.gs_apply_presize(cfg.presize_mode, cfg.include_clock_network)?;
        let mut gs = Gs::default();
        // `allocate`, `seedMultipliers`, `projectFlowBalance`, `subproblem_->init()`,
        // `computeAutoTimingWeight`: one read of the timer as it is.
        let timing_weight = self.gs_read(|r, v| {
            r.allocate(&mut gs, v);
            r.seed_multipliers(&mut gs, v, &cfg);
            r.project_flow_balance(&mut gs, v, &cfg)?;
            r.compute_leakage_scale(&mut gs);
            Ok(r.compute_auto_timing_weight(&gs, v, &cfg))
        })?;
        let max_iter = if cfg.max_iterations > 0 { cfg.max_iterations } else { GlobalSizingConfig::default().max_iterations };
        let wns_eps = 1e-12f32;
        let mut iter_params = cfg;
        let mut best_wns = self.timing.worst().0;
        let (mut total_committed, mut total_attempted, mut total_upsizes, mut total_downsizes) = (0, 0, 0, 0);
        let (mut accepted_iters, mut rejected_iters, mut consec_zero, mut consec_reject) = (0, 0, 0, 0);
        self.design.begin_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        for iter in 0..max_iter {
            let wns_now = self.timing.worst().0;
            if fuzzy_greater_equal(wns_now, cfg.setup_slack_margin) {
                break;
            }
            self.gs_env().trace(format!("iter|{iter}|{}|{}", c_hex(f64::from(wns_now)), c_hex(f64::from(iter_params.beta))));
            let wns0 = wns_now;
            // `updateMultipliers`, `projectFlowBalance` (from the second pass), then the sweep's
            // budgets and snapshots: one read.
            let snapshots = self.gs_read(|r, v| {
                if iter > 0 {
                    r.update_multipliers(&mut gs, v, &iter_params);
                    r.project_flow_balance(&mut gs, v, &iter_params)?;
                }
                r.compute_slack_budgets(&mut gs, v, &cfg)?;
                Ok(r.build_snapshots(&gs, v, &cfg))
            })?;
            // Phase B (in snapshot order), Phase C.
            let decisions: Vec<GateDecision> = {
                let env = self.gs_env();
                snapshots.iter().map(|s| env.evaluate_snapshot(s, timing_weight, cfg.budget_safety_factor)).collect()
            };
            let (moves, upsizes, downsizes) = self.gs_apply_decisions(&decisions)?;
            self.retime(&[])?;
            let wns1 = self.timing.worst().0;
            let reject = fuzzy::less(wns1 - wns0, -wns_eps);
            total_attempted += moves;
            total_upsizes += upsizes;
            total_downsizes += downsizes;
            if reject {
                consec_reject += 1;
                rejected_iters += 1;
                iter_params.beta *= 0.5;
            } else {
                total_committed += moves;
                accepted_iters += 1;
                consec_reject = 0;
            }
            let current_wns = self.timing.worst().0;
            let checkpoint = !reject && fuzzy_greater_equal(current_wns, best_wns);
            if checkpoint {
                // `journalEnd` (the timer is current: its update changes nothing), `journalBegin`.
                self.design.commit_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
                self.design.begin_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
                best_wns = current_wns;
            }
            self.gs_env().trace(format!("iterend|{iter}|{moves}|{}|{}|{}|{}", c_hex(f64::from(wns0)), c_hex(f64::from(wns1)), u8::from(reject), u8::from(checkpoint)));
            if consec_reject >= 3 {
                break;
            }
            if moves == 0 && !reject {
                consec_zero += 1;
                if consec_zero >= 2 {
                    break;
                }
            } else {
                consec_zero = 0;
            }
        }
        // Inner journal: back to the last checkpoint.
        if self.design.restore_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))? {
            self.retime(&[])?;
        }
        let wns_after = self.timing.worst().0;
        let outer_accept = fuzzy_greater_equal(wns_after, wns_pre);
        self.gs_env().trace(format!("outer|{}|{}", c_hex(f64::from(wns_after)), u8::from(outer_accept)));
        if outer_accept {
            self.design.commit_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))?;
        } else if self.design.restore_journal().map_err(|e| Stop::error("RSZ-JOURNAL", e))? {
            self.retime(&[])?;
        }
        let post = self.gs_env().design_snap();
        let (wns_post, tns_post) = (self.timing.worst().0, self.timing.tns());
        let rel = |after: f64, before: f64| if before > 0.0 { 100.0 * (after - before) / before } else { 0.0 };
        self.report(format!("[INFO RSZ-0400] GLOBAL_SIZING: {total_committed} cells replaced (loop); {accepted_iters}/{} sweeps accepted, {rejected_iters} rolled back; {total_attempted} replacements attempted in total ({total_upsizes} upsize, {total_downsizes} downsize).", accepted_iters + rejected_iters));
        let line = format!(
            "[INFO RSZ-0409] GLOBAL_SIZING QoR: WNS {} -> {} ({}); TNS {} -> {} ({}); leakage {} -> {}W ({:+.2}%); area {} -> {}m^2 ({:+.2}%).",
            self.ds(wns_pre, 3),
            self.ds(wns_post, 3),
            self.ds(wns_post - wns_pre, 3),
            self.ds(tns_pre, 1),
            self.ds(tns_post, 1),
            self.ds(tns_post - tns_pre, 1),
            fmt_g(pre.total_leakage, 3),
            fmt_g(post.total_leakage, 3),
            rel(post.total_leakage, pre.total_leakage),
            fmt_g(pre.total_area, 3),
            fmt_g(post.total_area, 3),
            rel(post.total_area, pre.total_area)
        );
        self.report(line);
        if total_committed == 0 && total_attempted > 0 {
            self.report(format!("[INFO RSZ-0412] GLOBAL_SIZING: nothing kept -- all {rejected_iters} sweeps tripped the WNS guard and were rolled back; the netlist is unchanged from the start of this phase. The {total_attempted} attempted replacements were tentative only."));
        }
        Ok(())
    }

    /// One read of the timer (no pending edit: the timer is current), with the scene-0 view and
    /// the read-only helpers (their borrow of the design is disjoint from the timer's).
    fn gs_read<R>(&mut self, f: impl FnOnce(&GsEnv<'_>, &View<'_, '_>) -> Result<R, Stop>) -> Result<R, Stop> {
        let edits = self.design.take_timer_edits();
        self.timer.trace_at = self.out.trace.len();
        let ctx = self.ctx;
        let design: &dyn super::SetupDesign = &*self.design;
        let env = GsEnv { ctx, design };
        let mut f = Some(f);
        let mut out: Option<Result<R, Stop>> = None;
        super::timed_all(ctx, design.as_design(), &mut self.timer, edits, |gs, ss, cs| {
            let v = View::new(&gs[0], &ss[0], &cs[0], design.parasitics(0));
            out = Some((f.take().expect("read once"))(&env, &v));
            Ok(())
        })?;
        out.expect("the read ran")
    }

    /// `applyPresize`: every editable logic cell replaced by its presize cell — as transcribed,
    /// an instance with ANY output pin that is not on the clock network is skipped when the clock
    /// network is excluded (the reference's `is_clock` test is inverted), so only all-clock gates
    /// are presized then.
    fn gs_apply_presize(&mut self, mode: u8, include_clock_network: bool) -> Result<(), Stop> {
        if mode == 0 {
            return Ok(());
        }
        let target = if mode == 1 { "smallest leakage Liberty cell" } else { "largest leakage Liberty cell" };
        self.report(format!("[INFO RSZ-0416] GLOBAL_SIZING: Presize {mode} enabled for {target}."));
        // Which instances drive a non-clock output (scene 0's clock network).
        let skip: BTreeSet<String> = if include_clock_network {
            BTreeSet::new()
        } else {
            self.gs_read(|r, v| {
                let mut s = BTreeSet::new();
                for (k, name, cell) in r.leaf_insts() {
                    let Some(pins) = r.ctx.master_pins.get(&cell) else { continue };
                    for p in pins {
                        if let Some((Some(d), _)) = v.pins.get(&(k, p.clone())) {
                            if !v.clocks.contains(d) {
                                s.insert(name.clone());
                                break;
                            }
                        }
                    }
                }
                Ok(s)
            })?
        };
        let mut editable = 0;
        let mut replacements = 0;
        let mut cache = HashMap::new();
        for (_, name, cell) in self.gs_env().leaf_insts() {
            // `isEditableLogicStdCell`.
            if self.design.net_info().dont_touch_insts.contains(&name) || !self.ctx.sizing.masters.get(&cell).is_some_and(|m| m.logic_std) || self.ctx.libs.link_cell(&cell).is_none() {
                continue;
            }
            if skip.contains(&name) {
                continue;
            }
            editable += 1;
            let replacement = self.gs_env().select_presize_cell(&cell, mode, &mut cache);
            if replacement != cell {
                self.gs_replace_cell(&name, &cell, &replacement)?;
                replacements += 1;
            }
        }
        if replacements > 0 {
            self.retime(&[])?;
        }
        self.report(format!("[INFO RSZ-0415] GLOBAL_SIZING: Presize replaced {replacements}/{editable} editable instances."));
        Ok(())
    }

    /// `Resizer::replaceCell` — refused when the swap does not keep the arcs (`equivCellsArcs`):
    /// the reference then mints new edges whose λ slot is out of range or reused.
    fn gs_replace_cell(&mut self, inst: &str, from: &str, to: &str) -> Result<(), Stop> {
        let (Some(a), Some(b)) = (self.ctx.libs.link_cell(from), self.ctx.libs.link_cell(to)) else {
            return Err(Stop::refused("RSZ-ABSENT", format!("GLOBAL_SIZING: {from} -> {to} without both cells: not modelled")));
        };
        if !vyges_sta::incr::equiv_cells_arcs(a, b) {
            return Err(Stop::refused("RSZ-ABSENT", format!("GLOBAL_SIZING: {from} -> {to} does not keep its timing arcs (the reference's new edge ids): not modelled")));
        }
        self.design.swap_master(inst, to).map_err(|e| Stop::error("RSZ-REPLACE", e))
    }

    /// `applyDecisions`: in snapshot order, a decision kept when its cost beats the baseline by
    /// 2% (upsize) or at all (downsize).
    fn gs_apply_decisions(&mut self, decisions: &[GateDecision]) -> Result<(i32, i32, i32), Stop> {
        let (mut moves, mut upsizes, mut downsizes) = (0, 0, 0);
        for r in decisions {
            let Some(best) = &r.best_cell else { continue };
            let tol = if r.best_is_downsize { 0.0f32 } else { 0.02f32 };
            if r.best_cost < r.baseline_cost * (1.0 - tol) {
                let cur = self.design.netlist().insts.iter().find(|(n, _)| n == &r.inst).map(|(_, c)| c.clone()).unwrap_or_default();
                self.gs_replace_cell(&r.inst, &cur, best)?;
                self.gs_env().trace(format!("apply|{}|{best}", r.inst));
                moves += 1;
                if r.best_is_downsize {
                    downsizes += 1;
                } else {
                    upsizes += 1;
                }
            }
        }
        Ok((moves, upsizes, downsizes))
    }
}

impl View<'_, '_> {
    /// The cell of a netlist instance.
    fn design_inst_cell(&self, k: usize) -> String {
        self.g.netlist.insts[k].1.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule (fmt `{:.3g}` = printf `%.3g`): 3 significant digits, trailing zeros dropped,
    // scientific below 1e-4 with a two-digit exponent.
    #[test]
    fn three_significant_digits_as_printf_g() {
        assert_eq!(fmt_g(7.89e-08, 3), "7.89e-08");
        assert_eq!(fmt_g(1.56e-07, 3), "1.56e-07");
        assert_eq!(fmt_g(3.4600001e-12, 3), "3.46e-12");
        assert_eq!(fmt_g(1.5e-5, 3), "1.5e-05");
        assert_eq!(fmt_g(0.00012345, 3), "0.000123");
        assert_eq!(fmt_g(1234.0, 3), "1.23e+03");
        assert_eq!(fmt_g(12.0, 3), "12");
        assert_eq!(fmt_g(0.0, 3), "0");
    }

    #[test]
    fn default_config_is_the_references() {
        let c = GlobalSizingConfig::default();
        assert_eq!((c.max_iterations, c.beta, c.mu_exponent, c.lambda_floor, c.timing_bias, c.budget_safety_factor), (20, 0.6, 2.0, 1e-12, 64.0, 1.0));
    }
}
