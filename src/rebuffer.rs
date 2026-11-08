// SPDX-License-Identifier: Apache-2.0
//! BufferMove: `Rebuffer::rebufferPin` — the driver's net rebuilt as a buffer tree chosen by slack,
//! then by area, under placement parasitics.
//!
//! One function per stage, named after the reference's and in its call order:
//! - [`characterize`] — `Rebuffer::init` (`findFastBuffers`, the sizes by input capacitance) and
//!   `characterizeBuffers` (each size's margined max cap, its long-wire asymptotics);
//! - [`Rebuf::rebuffer_pin`] — `rebufferPin`: [`Rebuf::buffer_for_timing`] three times,
//!   [`Rebuf::recover_area`] five times against the relaxed driver slack, then the export (the
//!   sequencer inserts the buffers the chosen tree names);
//! - [`Rebuf::buffer_for_timing`] — the van Ginneken pass: per node the slack/cap options, wires
//!   added, buffers inserted ([`Rebuf::insert_buffer_options`]), long wires stepped, junctions
//!   merged (with [`Rebuf::attempt_topology_rewrite`]);
//! - [`Rebuf::recover_area`] — the area pass toward a slack threshold per node.
//!
//! Delays are [`FixedDelay`] (femtoseconds, as the reference keeps them). What the pass reads from
//! the timer — the load slacks (`annotateLoadSlacks`), the driver's arcs and arrivals
//! (`drvrPinTiming`), the fanout and slew limits (`setPin`) — is taken while the path is expanded
//! ([`Probe`]); nothing it reads changes before the move commits.

use std::collections::BTreeSet;

use vyges_sta::fuzzy;
use vyges_sta::liberty::{Cell, Model};
use vyges_sta::table::AxisVar;

use crate::buffered_net::WireRc;
use crate::preamble::Libs;
use crate::repair_timing::delay_as_string;

/// `sta::INF`.
const INF: f32 = 1e30;
/// `slew_margin_`, `cap_margin_` (percent), `relaxation_factor_`.
const SLEW_MARGIN: f32 = 20.0;
const CAP_MARGIN: f32 = 20.0;
const RELAXATION_FACTOR: f32 = 0.01;
/// `kRebufferMaxFanout`.
pub const REBUFFER_MAX_FANOUT: usize = 20;

/// `FixedDelay`: a delay in whole femtoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct FixedDelay(pub i64);

impl FixedDelay {
    pub const ZERO: FixedDelay = FixedDelay(0);

    /// `FixedDelay(float, …)`: the float times 1e15 in double, truncated.
    pub fn from_secs(v: f32) -> FixedDelay {
        FixedDelay((f64::from(v) * 1e15) as i64)
    }

    /// `FixedDelay::INF`: 100 seconds.
    pub fn inf() -> FixedDelay {
        FixedDelay::from_secs(100.0)
    }

    /// `toSeconds`: the count as a float, over 1e15 in double, as a float.
    pub fn to_secs(self) -> f32 {
        (f64::from(self.0 as f32) / 1e15) as f32
    }

    /// `FixedDelay::lerp`: `b` at t = 1, else `a + (float)(b − a) · t` truncated.
    pub fn lerp(a: FixedDelay, b: FixedDelay, t: f32) -> FixedDelay {
        if t == 1.0 {
            return b;
        }
        a + FixedDelay(((b.0 - a.0) as f32 * t) as i64)
    }
}

impl std::ops::Add for FixedDelay {
    type Output = FixedDelay;
    fn add(self, o: FixedDelay) -> FixedDelay {
        FixedDelay(self.0 + o.0)
    }
}

impl std::ops::Sub for FixedDelay {
    type Output = FixedDelay;
    fn sub(self, o: FixedDelay) -> FixedDelay {
        FixedDelay(self.0 - o.0)
    }
}

impl std::ops::Neg for FixedDelay {
    type Output = FixedDelay;
    fn neg(self) -> FixedDelay {
        FixedDelay(-self.0)
    }
}

/// `RiseFallBoth` as a slack transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rfs {
    Rise,
    Fall,
    Both,
}

impl Rfs {
    pub fn of(rf: usize) -> Rfs {
        if rf == 0 { Rfs::Rise } else { Rfs::Fall }
    }
    fn range(self) -> &'static [usize] {
        match self {
            Rfs::Rise => &[0],
            Rfs::Fall => &[1],
            Rfs::Both => &[0, 1],
        }
    }
}

/// `combinedTransition`.
fn combined(a: Option<Rfs>, b: Option<Rfs>) -> Option<Rfs> {
    if a == b {
        a
    } else if a.is_none() {
        b
    } else if b.is_none() {
        a
    } else {
        Some(Rfs::Both)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Kind {
    Load { pin: String },
    Junction { r: usize, r2: usize },
    Wire { r: usize },
    Buffer { cell: String, r: usize },
    /// A global route's via, from the node's layer to `ref_layer` — only in a buffered net as built
    /// (`bufferForTiming` strips it with the wires and buffers).
    Via { r: usize, ref_layer: i32 },
}

/// `makeSteinerTree(root, sinks)` over points (`resteiner`), from the caller.
pub type SteinerOverPoints<'a> = &'a dyn Fn((i32, i32), &[(i32, i32)]) -> Option<crate::buffered_net::Tree>;

/// One `BufferedNet` node with the rebuffer annotations; an arena index is its identity, and a
/// subtree is shared by every option built on it (the reference's `shared_ptr`s).
#[derive(Debug, Clone, PartialEq)]
pub struct Node {
    pub kind: Kind,
    pub x: i32,
    pub y: i32,
    /// A wire's routing layer (`BufferedNet::null_layer` off a global route), a via's from layer.
    pub layer: i32,
    pub cap: f32,
    pub fanout: f32,
    pub max_load_slew: f32,
    pub area: f32,
    pub slack_rf: Option<Rfs>,
    pub slack: FixedDelay,
    pub delay: FixedDelay,
    pub arrival: FixedDelay,
}

/// `Rebuffer::RankedOption`: a beam option with the measures the rankings compare. `option` is
/// the tree's root (none for an empty anchor); `slack` the driver slack `evaluateOption` found
/// when the option was selected; `index` its position among the generated options (the last
/// tie-break); `signature` its structure ([`Rebuf::tree_signature`]).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RankedOption {
    pub option: Option<usize>,
    pub slack: FixedDelay,
    pub unconstrained: bool,
    pub meets_target: bool,
    pub preserves_best_slack: bool,
    pub signature: String,
    pub index: i64,
}

impl RankedOption {
    /// The option's root; every ranked option has one (only an anchor may be empty).
    fn node(&self) -> usize {
        self.option.expect("a ranked option has a tree")
    }
}

/// `kNegativeSlackLossBudget`, `kPositiveSlackLossBudget`: the slack a tree may give back for
/// area in RebufferMt planning — 1 ps while the best tree still violates, 20 ps once it has margin.
const NEGATIVE_SLACK_LOSS_BUDGET: f32 = 1.0e-12;
const POSITIVE_SLACK_LOSS_BUDGET: f32 = 20.0e-12;

/// `slackLossFloor`: the slack an option may fall to below `best_slack` and still preserve it.
pub fn slack_loss_floor(best_slack: FixedDelay) -> FixedDelay {
    let budget = if best_slack.to_secs() < 0.0 { NEGATIVE_SLACK_LOSS_BUDGET } else { POSITIVE_SLACK_LOSS_BUDGET };
    best_slack - FixedDelay::from_secs(budget)
}

/// The VYGP printer's `{:.9e}` (fmt, or printf's `%.9e`): the exponent signed, at least two
/// digits.
pub fn e9<T: Into<f64>>(v: T) -> String {
    let s = format!("{:.9e}", v.into());
    let (m, e) = s.split_once('e').expect("Rust's {:e} always has an exponent");
    let e: i32 = e.parse().expect("Rust's {:e} exponent is an integer");
    format!("{m}e{}{:02}", if e < 0 { '-' } else { '+' }, e.abs())
}

/// `BufferTreeStageType` (a via is never planned: see [`Rebuf::build_load_paths`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StageKind {
    Wire,
    Buffer,
}

/// `WireRC`: a wire stage's length (meters) and its total resistance and capacitance.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct WireRcStage {
    pub length: f32,
    pub res: f32,
    pub cap: f32,
}

/// `BufferTreePathStage`: one wire or buffer on a planned root-to-load path.
#[derive(Debug, Clone, PartialEq)]
pub struct PathStage {
    pub kind: StageKind,
    pub buffer_cell: Option<String>,
    pub wire_rc: WireRcStage,
    /// The capacitance below this stage in the planned tree.
    pub downstream_cap: f32,
    /// The stage drives more than one load branch (a wire or a branch before the next buffer).
    pub drives_distributed_load: bool,
}

/// `BufferTreeLoadPath`: the stages from the planned tree's root to one load pin.
#[derive(Debug, Clone, PartialEq)]
pub struct LoadPath {
    pub load_pin: String,
    pub stages: Vec<PathStage>,
}

/// `Rebuffer::PlanRequest`: what one planning call plans for.
#[derive(Debug, Clone)]
pub struct PlanRequest {
    pub target_load_pin: Option<String>,
    pub original_slack: FixedDelay,
}

/// `RebufferTreePlan`: a buffer-tree proposal with the paths and loading the MT estimate reads.
#[derive(Debug, Clone)]
pub struct TreePlan {
    /// The tree's root in the planner's arena.
    pub tree: usize,
    pub target_load_pin: Option<String>,
    pub load_paths: Vec<LoadPath>,
    pub tree_input_cap: f32,
    /// The tree's driver pin slack less the current net's.
    pub driver_slack_gain: f32,
    /// The lineage of top-ranked options through the beam rounds, exempt from the score gates.
    pub anchor_tree: bool,
    pub buffer_count: i64,
    pub load_count: i64,
}

impl TreePlan {
    /// `loadPath`: the load path of `pin`, if the tree reaches it.
    pub fn load_path(&self, pin: &str) -> Option<&LoadPath> {
        self.load_paths.iter().find(|p| p.load_pin == pin)
    }

    /// `targetPath`.
    pub fn target_path(&self) -> Option<&LoadPath> {
        self.target_load_pin.as_deref().and_then(|p| self.load_path(p))
    }
}

/// One buffer size (`BufferSize`) with what the passes read of its cell.
#[derive(Debug, Clone)]
pub struct BufferSize {
    pub cell: String,
    pub intrinsic_delay: FixedDelay,
    pub margined_max_cap: f32,
    pub driver_resistance: f32,
    /// `long_wire_asymptotics[-1]` (the signal wire RC): none without parasitics.
    pub asym: Option<Asymptotics>,
    /// `long_wire_asymptotics[layer]` per routing layer with a non-zero `layerRC` (global-route
    /// parasitics only); a layer missing reads all zeros, as the reference's `operator[]` does.
    pub asym_layers: std::collections::BTreeMap<i32, Asymptotics>,
    /// The input port's capacitance (`capacitance()`, the largest value), fanout load and max
    /// input slew; the output port's name; the cell's area.
    pub in_cap: f32,
    /// `portCapacitance(input, corner_)` (`scenePort`): the buffer node's cap, the asymptotics'.
    pub in_cap_cmd: f32,
    pub in_fanout: f32,
    pub in_max_slew: f32,
    pub out_port: String,
    pub area: f32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Asymptotics {
    pub buffer_spacing: f32,
    pub delay_per_meter: f32,
    pub delay_per_farad: f32,
    pub input_cap: f32,
}

/// What the passes read besides the tree: the sizes, the wire RC, the units.
pub struct Ctx<'a> {
    pub libs: &'a Libs,
    pub sizes: &'a [BufferSize],
    pub rc: WireRc,
    pub dbu: i32,
    pub slew_shape_factor: f32,
    pub tgt_slews: [f32; 2],
    pub time_scale: f32,
    pub cap_scale: f32,
    /// `corner_` (`initOnCorner(cmdScene())`): the scene of the buffered net, the characterization
    /// and `setPin`'s limits.
    pub cmd: usize,
    /// `tgt_slew_corner_`: the scene of `findFastBuffers`' delays.
    pub tgt: usize,
    /// Global-route parasitics: the estimator's per-layer RC, which a layered wire reads.
    pub layers: Option<std::sync::Arc<crate::buffered_net::LayerRc>>,
}

impl Ctx<'_> {
    fn dbu_to_meters(&self, d: i32) -> f64 {
        f64::from(d) / (f64::from(self.dbu) * 1e6)
    }
    /// `metersToDbu`: rounded, masked as the reference's.
    fn meters_to_dbu(&self, m: f64) -> i32 {
        ((m * f64::from(self.dbu) * 1e6).round() as i64 & i64::from(i32::MAX)) as i32
    }
    /// `wireSignalRC`: the mean of the horizontal and vertical values.
    fn signal_rc(&self) -> (f64, f64) {
        ((self.rc.h_res + self.rc.v_res) / 2.0, (self.rc.h_cap + self.rc.v_cap) / 2.0)
    }
    /// `layerRC(findRoutingLayer(layer), corner_)`: the estimator's table (0 where unset).
    fn layer_rc(&self, layer: i32) -> (f64, f64) {
        self.layers.as_ref().and_then(|l| l.wire.get(&layer).copied()).unwrap_or((0.0, 0.0))
    }
    /// The RC per meter a wire on `layer` reads for buffering: the signal wire's on no layer.
    fn rc_on(&self, layer: i32) -> (f64, f64) {
        if layer == crate::buffered_net::NULL_LAYER { self.signal_rc() } else { self.layer_rc(layer) }
    }
    fn cell(&self, name: &str) -> &Cell {
        self.libs.link_cell(name).expect("a buffer cell")
    }
    /// The scene's cell (`gateDelay(.., scene)` reads its arcs' `sceneArc` models; `scenePort`).
    fn scene_cell(&self, k: usize, name: &str) -> &Cell {
        self.libs.scene_cell(k, name).expect("a buffer cell")
    }
    /// `Unit::asString(float, digits)`.
    fn unit(v: f32, scale: f32, digits: usize) -> String {
        crate::repair_timing::unit_as_string(v, scale, digits)
    }
    fn delay(&self, v: f32) -> String {
        delay_as_string(v, 3, self.time_scale)
    }
    fn cap_str(&self, v: f32) -> String {
        Self::unit(v, self.cap_scale, 3)
    }
    fn dist(&self, d: i32) -> String {
        Self::unit(self.dbu_to_meters(d) as f32, 1e-6, 2)
    }
}

/// `Resizer::gateDelays(port, load, …)` at the target slews: per output transition, the largest
/// delay and slew over the non-check arcs into the port (`std::max`: the first unless the second
/// is greater — not `f32::max`, which differs on a NaN and on ±0).
pub fn gate_delays(cell: &Cell, port: &str, load_cap: f32, tgt: [f32; 2]) -> ([f32; 2], [f32; 2]) {
    let mut d = [-INF; 2];
    let mut s = [-INF; 2];
    for set in cell.arc_sets.iter().filter(|a| a.to == port && !a.role.is_timing_check()) {
        for arc in &set.arcs {
            if let Model::Gate(m) = &arc.model {
                let (gd, gs) = m.gate_delay(tgt[arc.from_rf], load_cap);
                d[arc.to_rf] = std_max(d[arc.to_rf], gd);
                s[arc.to_rf] = std_max(s[arc.to_rf], gs);
            }
        }
    }
    (d, s)
}

/// `std::max(a, b)`: `a < b ? b : a`.
pub fn std_max(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}

/// `std::min(a, b)`: `b < a ? b : a`.
pub fn std_min(a: f32, b: f32) -> f32 {
    if b < a { b } else { a }
}

/// `LibertyPort::capacitance()` (`RiseFallMinMax::maxValue`): the `std::max` of every value.
pub fn port_cap(cell: &Cell, port: &str) -> f32 {
    cell.port(port).map_or(0.0, |p| p.capacitance.iter().flatten().fold(-INF, |a, &b| std_max(a, b)))
}

/// `LibertyPort::intrinsicDelay`: the largest intrinsic delay fuzzily above 0 over the non-check
/// arcs into the port, else 0.
pub(crate) fn intrinsic_delay(cell: &Cell, port: &str) -> f32 {
    let mut best = -INF;
    let mut found = false;
    for set in cell.arc_sets.iter().filter(|a| a.to == port && !a.role.is_timing_check()) {
        for arc in &set.arcs {
            if let Model::Gate(m) = &arc.model {
                let d = m.gate_delay(0.0, 0.0).0;
                if fuzzy::greater(d, 0.0) {
                    if fuzzy::greater(d, best) {
                        best = d;
                    }
                    found = true;
                }
            }
        }
    }
    if found { best } else { 0.0 }
}

/// What characterization reads besides the cells: the slew and capacitance limits per port.
pub struct CharInputs<'a> {
    pub ctx: &'a Ctx<'a>,
    /// `bufferDriveResistance(buffer_lowest_drive_)`.
    pub r_max: f32,
    /// `findSlewLimit(port, corner, max)` → (limit, exists), and `maxInputSlew(port, corner)`.
    pub slew_limit: &'a dyn Fn(&Cell, &str) -> (f32, bool),
    pub max_input_slew: &'a dyn Fn(&Cell, &str) -> f32,
    /// `maxLoad(cell)`: the first output port's capacitance limit, 0 without one.
    pub max_load: &'a dyn Fn(&Cell) -> f32,
}

/// `Rebuffer::init` and `characterizeBuffers` over `buffer_cells_` (`findFastBuffers`). `Err`
/// when two sizes tie on input capacitance (the reference breaks it by liberty cell id, which the
/// reader here does not keep).
pub fn characterize(ci: &CharInputs<'_>, buffer_cells: &[String]) -> Result<Vec<BufferSize>, String> {
    let ctx = ci.ctx;
    let fast = find_fast_buffers(ci, buffer_cells);
    let mut sizes: Vec<BufferSize> = Vec::new();
    for name in &fast {
        let c = ctx.cell(name);
        let (inp, out) = c.buffer_ports().ok_or("not a buffer")?;
        sizes.push(BufferSize {
            cell: name.clone(),
            intrinsic_delay: FixedDelay::from_secs(intrinsic_delay(c, &out.name)),
            margined_max_cap: 0.0,
            driver_resistance: c.drive_resistance(&out.name),
            asym: None,
            asym_layers: std::collections::BTreeMap::new(),
            in_cap: port_cap(c, &inp.name),
            in_cap_cmd: port_cap(ctx.scene_cell(ctx.cmd, name), &inp.name),
            in_fanout: 0.0,
            in_max_slew: (ci.max_input_slew)(c, &inp.name),
            out_port: out.name.clone(),
            area: c.area,
        });
    }
    // By input capacitance (then cell id).
    sizes.sort_by(|a, b| a.in_cap.partial_cmp(&b.in_cap).unwrap_or(std::cmp::Ordering::Equal));
    if sizes.windows(2).any(|w| w[0].in_cap == w[1].in_cap) {
        return Err("rebuffer: two buffer sizes with the same input capacitance (the cell-id tie break is not modelled)".into());
    }
    for s in sizes.iter_mut() {
        let c = ctx.cell(&s.cell);
        let (inp, out) = c.buffer_ports().expect("a buffer");
        s.in_fanout = inp.fanout_load.or(ctx.libs.link_library(&s.cell).and_then(|l| l.default_fanout_load)).unwrap_or(0.0);
        let cap_limit = out.max_capacitance;
        let slew_cap = find_buffer_load_limit_implied_by_driver_slew(ci, c);
        s.margined_max_cap = cap_limit.map_or(INF, max_cap_margined).min(slew_cap);
        s.asym = find_long_wire_asymptotics(ctx, s);
        // `characterizeBuffers` then runs every routing layer (1..N) on its `layerRC`.
        if let Some(l) = ctx.layers.as_ref() {
            for (&level, &(res, cap)) in &l.wire {
                if let Some(a) = long_wire_asymptotics_at(ctx, s, res, cap) {
                    s.asym_layers.insert(level, a);
                }
            }
        }
    }
    Ok(sizes)
}

/// `Resizer::findFastBuffers`: by input capacitance (stable), each size kept unless the last kept
/// one outmatches it, popping the kept ones it outmatches.
pub fn find_fast_buffers(ci: &CharInputs<'_>, buffer_cells: &[String]) -> Vec<String> {
    let ctx = ci.ctx;
    let mut by_cap: Vec<&String> = buffer_cells.iter().collect();
    let cin = |n: &str| ctx.cell(n).buffer_ports().map_or(0.0, |(i, _)| port_cap(ctx.cell(n), &i.name));
    by_cap.sort_by(|a, b| cin(a).partial_cmp(&cin(b)).unwrap_or(std::cmp::Ordering::Equal));
    let mut fast: Vec<String> = Vec::new();
    for size in by_cap {
        match fast.last() {
            None => fast.push(size.clone()),
            Some(last) => {
                if !buffer_size_outmatched(ci, size, last) {
                    while fast.last().is_some_and(|l| buffer_size_outmatched(ci, l, size)) {
                        fast.pop();
                    }
                    fast.push(size.clone());
                }
            }
        }
    }
    fast
}

/// `Resizer::bufferSizeOutmatched(worse, better, R_max)`: `better` (plus `R_max` times its extra
/// input cap) is no slower than `worse` at every capacitance test point of either's
/// combinational delay tables, within `worse`'s max load and under 20 × the smaller input cap.
fn buffer_size_outmatched(ci: &CharInputs<'_>, worse: &str, better: &str) -> bool {
    let ctx = ci.ctx;
    let (w, b) = (ctx.cell(worse), ctx.cell(better));
    let (Some((win, wout)), Some((bin, bout))) = (w.buffer_ports(), b.buffer_ports()) else { return false };
    let (wc, bc) = (port_cap(w, &win.name), port_cap(b, &bin.name));
    let extra = (bc - wc).max(0.0);
    let penalty = ci.r_max * extra;
    let wlimit = (ci.max_load)(w);
    let mut points: Vec<f32> = Vec::new();
    for (c, i, o) in [(w, &win.name, &wout.name), (b, &bin.name, &bout.name)] {
        for set in c.arc_sets.iter().filter(|s| s.role == vyges_sta::liberty::Role::Combinational && &s.from == i && &s.to == o) {
            for arc in &set.arcs {
                if let Model::Gate(m) = &arc.model {
                    if let Some(t) = &m.delay {
                        for ax in t.axes.iter().take(3).filter(|a| a.var == AxisVar::TotalOutputNetCapacitance) {
                            points.extend(ax.values.iter().copied());
                        }
                    }
                }
            }
        }
    }
    if points.is_empty() {
        return false;
    }
    points.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    points.dedup();
    for p in points {
        if (wlimit == 0.0 || p <= wlimit) && p < bc.min(wc) * 20.0 {
            // `bufferDelay(.., tgt_slew_corner_)`.
            let bd = buffer_delay_max(ctx, ctx.scene_cell(ctx.tgt, better), p);
            let wd = buffer_delay_max(ctx, ctx.scene_cell(ctx.tgt, worse), p);
            if bd + penalty > wd {
                return false;
            }
        }
    }
    true
}

/// `Resizer::bufferDelay(cell, load)`: the larger of rise and fall at the target slews.
fn buffer_delay_max(ctx: &Ctx<'_>, c: &Cell, load: f32) -> f32 {
    let Some((_, out)) = c.buffer_ports() else { return -INF };
    let (d, _) = gate_delays(c, &out.name, load, ctx.tgt_slews);
    d[0].max(d[1])
}

fn max_slew_margined(s: f32) -> f32 {
    (f64::from(s) * (1.0 - f64::from(SLEW_MARGIN) / 100.0)) as f32
}

fn max_cap_margined(c: f32) -> f32 {
    (f64::from(c) * (1.0 - f64::from(CAP_MARGIN) / 100.0)) as f32
}

/// `findBufferLoadLimitImpliedByDriverSlew`: the load at which the cell's slowest output slew
/// (over all its non-check arcs, at the margined max input slew) reaches the margined slew
/// limit — doubling, then halving to 1 %.
fn find_buffer_load_limit_implied_by_driver_slew(ci: &CharInputs<'_>, c: &Cell) -> f32 {
    let (inp, outp) = c.buffer_ports().expect("a buffer");
    let (limit, _) = (ci.slew_limit)(c, &outp.name);
    let max_slew = max_slew_margined(limit);
    let in_slew = max_slew_margined((ci.max_input_slew)(c, &inp.name));
    // `gateDelay(arc, .., corner_)`: the scene's arcs (a max over all of them: the scene cell's own
    // sets are the link sets' scene arcs).
    let sc = ci.ctx.scene_cell(ci.ctx.cmd, &c.name);
    let objective = |load: f32| -> f32 {
        let mut slew = -INF;
        for set in sc.arc_sets.iter().filter(|s| !s.role.is_timing_check()) {
            for arc in &set.arcs {
                if let Model::Gate(m) = &arc.model {
                    slew = slew.max(m.gate_delay(in_slew, load).1);
                }
            }
        }
        slew - max_slew
    };
    let drvr_res = f64::from(c.drive_resistance(&outp.name));
    if drvr_res == 0.0 {
        return INF;
    }
    let mut cap1 = 0.0f64;
    let mut cap2 = f64::from(max_slew) / drvr_res * 2.0;
    let tol = 0.01;
    let mut diff1 = f64::from(objective(cap2 as f32));
    while (cap1 - cap2).abs() > cap1.max(cap2) * tol {
        if diff1 < 0.0 {
            cap1 = cap2;
            cap2 *= 2.0;
            diff1 = f64::from(objective(cap2 as f32));
        } else {
            let cap3 = (cap1 + cap2) / 2.0;
            let diff2 = f64::from(objective(cap3 as f32));
            if diff2 < 0.0 {
                cap1 = cap3;
            } else {
                cap2 = cap3;
                diff1 = diff2;
            }
        }
    }
    cap1 as f32
}

/// `findLongWireAsymptotics(-1, size)` on the signal wire RC: three fixed-point steps toward the
/// buffer spacing that balances wire and buffer delay; none without parasitics.
fn find_long_wire_asymptotics(ctx: &Ctx<'_>, s: &BufferSize) -> Option<Asymptotics> {
    let (wire_res, wire_cap) = ctx.signal_rc();
    long_wire_asymptotics_at(ctx, s, wire_res, wire_cap)
}

/// `findLongWireAsymptotics(layer, size)` on a wire RC per meter (the layer's, or the signal wire's).
fn long_wire_asymptotics_at(ctx: &Ctx<'_>, s: &BufferSize, wire_res: f64, wire_cap: f64) -> Option<Asymptotics> {
    if wire_res <= 0.0 || wire_cap <= 0.0 {
        return None;
    }
    // `characterizationDelay`: `gateDelays` at `corner_`; the input cap its `scenePort`'s.
    let c = ctx.scene_cell(ctx.cmd, &s.cell);
    let char_delay = |load: f32| buffer_delay_max(ctx, c, load);
    let mut length = 0.0f32;
    let mut buffer_delay = 0.0f32;
    for _ in 0..3 {
        let buffer_load = (wire_cap * f64::from(length) + f64::from(s.in_cap_cmd)) as f32;
        let eps = 0.01f32;
        buffer_delay = char_delay(buffer_load);
        let slope = (f64::from((char_delay(buffer_load * (1.0 + eps)) - buffer_delay) / (buffer_load * eps)) * wire_cap) as f32;
        let intrinsic = (buffer_delay - slope * length).max(0.0);
        length = (2.0 * f64::from(intrinsic) / (wire_res * wire_cap)).sqrt() as f32;
    }
    let wire_delay = (wire_res * wire_cap / 2.0 * f64::from(length) * f64::from(length)) as f32;
    let delay_per_meter = (buffer_delay + wire_delay) / length;
    Some(Asymptotics { buffer_spacing: length, delay_per_meter, delay_per_farad: (f64::from(delay_per_meter) / wire_cap) as f32, input_cap: s.in_cap_cmd })
}

/// The driver's arc into the net for one transition (`drvrPinTiming`'s inputs): the arc's gate
/// model, the slew at its from pin (`edgeFromSlew`), the from path's arrival (`clkPathArrival` for
/// a clock path) and the driver's arrival. `None`: the driver path has no previous path.
#[derive(Debug, Clone)]
pub struct DriverArc {
    pub model: Model,
    pub from_slew: f32,
    pub prev_arrival: f32,
    pub arrival: f32,
}

/// What `rebufferPin` reads from the timer, taken at expand time.
#[derive(Debug, Clone)]
pub struct Probe {
    /// The initial buffered net (`makeBufferedNet`) with its load slacks (`annotateLoadSlacks`);
    /// its root.
    pub nodes: Vec<Node>,
    pub root: usize,
    /// Per transition, the arrival path at the driver (`arrival_paths_`): its arc, or `None` for a
    /// path with no previous path; absent when no load's path took that transition.
    pub arcs: [Option<Option<DriverArc>>; 2],
    /// Per transition, the scene of that arrival path (`bufferDelay` reads its delays there).
    pub arc_scenes: [usize; 2],
    /// `drvr_port_->capacitance()`, `driveResistance()`.
    pub drvr_port_cap: f32,
    pub drvr_resistance: f32,
    /// `setPin`: `fanout_limit_`, `drvr_pin_max_slew_`.
    pub fanout_limit: f32,
    pub drvr_pin_max_slew: f32,
    /// The driver pin's name (`network_->name(pin_)`).
    pub pin: String,
    /// Warnings `annotateLoadSlacks` logs (RSZ-2006), in order.
    pub warnings: Vec<String>,
}

/// One pass's state: the arena, the probe, the high-water mark, the debug lines.
pub struct Rebuf<'c> {
    pub ctx: &'c Ctx<'c>,
    pub nodes: Vec<Node>,
    pub probe: &'c Probe,
    drvr_load_high_water_mark: f32,
    /// `(level, line)` of the `rebuffer` debug group, in order; a `None` level is a report line
    /// (a warning the reference logs between them).
    pub trace: Vec<(Option<i64>, String)>,
    /// The reference's error, where it stops (RSZ-0501).
    pub failed: Option<String>,
    /// A step the reference takes that is not modelled (refused by the caller).
    pub refused: Option<String>,
    /// `pin_`, for the warnings that name it.
    pub pin: String,
}

/// `BufferedNet::to_string`, indented `level` spaces.
impl Rebuf<'_> {
    pub fn new<'c>(ctx: &'c Ctx<'c>, probe: &'c Probe) -> Rebuf<'c> {
        Rebuf { ctx, nodes: probe.nodes.clone(), probe, drvr_load_high_water_mark: 0.0, trace: Vec::new(), failed: None, refused: None, pin: probe.pin.clone() }
    }

    fn debug(&mut self, level: i64, line: String) {
        self.trace.push((Some(level), line));
    }

    fn warn(&mut self, line: String) {
        self.trace.push((None, line));
    }

    fn to_string(&self, n: usize) -> String {
        let nd = &self.nodes[n];
        let (x, y) = (self.ctx.dist(nd.x), self.ctx.dist(nd.y));
        let cap = self.ctx.cap_str(nd.cap);
        let slack = self.ctx.delay(nd.slack.to_secs());
        let sl = self.ctx.delay(nd.max_load_slew);
        let buffers = self.buffer_count(n);
        let s = match &nd.kind {
            Kind::Load { pin } => load_text(pin, &x, &y, &cap, &slack, &sl),
            Kind::Wire { .. } => format!("wire ({x}, {y}) cap {cap} slack {slack} buffers {buffers} load sl {sl}"),
            Kind::Buffer { cell, .. } => format!("buffer ({x}, {y}) {cell} cap {cap} slack {slack} buffers {buffers} load sl {sl}"),
            Kind::Junction { .. } => format!("junction ({x}, {y}) cap {cap} slack {slack} buffers {buffers} load sl {sl}"),
            Kind::Via { ref_layer, .. } => format!("via ({x}, {y}) layer {} -> {ref_layer}", nd.layer),
        };
        // A diagnostic: the raw slack (fs) and load slew bits (`VYG_RAW`), as the instrumented
        // reference appends them.
        if std::env::var_os("VYG_RAW").is_some() {
            format!("{s} RAW {} {:08x}", nd.slack.0, nd.max_load_slew.to_bits())
        } else {
            s
        }
    }

    pub fn buffer_count(&self, n: usize) -> i64 {
        match &self.nodes[n].kind {
            Kind::Buffer { r, .. } => self.buffer_count(*r) + 1,
            Kind::Wire { r } | Kind::Via { r, .. } => self.buffer_count(*r),
            Kind::Junction { r, r2 } => self.buffer_count(*r) + self.buffer_count(*r2),
            Kind::Load { .. } => 0,
        }
    }

    fn loc(&self, n: usize) -> (i32, i32) {
        (self.nodes[n].x, self.nodes[n].y)
    }

    fn push(&mut self, n: Node) -> usize {
        self.nodes.push(n);
        self.nodes.len() - 1
    }

    /// The wire node's RC per meter on no layer (`BufferedNet::wireRC`): each direction's share.
    /// On a routing layer: the estimator's `layerRC` for it, whatever the length.
    fn wire_rc_of(&self, at: (i32, i32), r: usize, layer: i32) -> (f64, f64, i32) {
        let rl = self.loc(r);
        let len = (at.0 - rl.0).abs() + (at.1 - rl.1).abs();
        if layer != crate::buffered_net::NULL_LAYER {
            let (res, cap) = self.ctx.layer_rc(layer);
            return (res, cap, len);
        }
        if len == 0 {
            return (0.0, 0.0, 0);
        }
        let c = self.ctx;
        let dx = c.dbu_to_meters((at.0 - rl.0).abs()) / c.dbu_to_meters(len);
        let dy = c.dbu_to_meters((at.1 - rl.1).abs()) / c.dbu_to_meters(len);
        (dx * c.rc.h_res + dy * c.rc.v_res, dx * c.rc.h_cap + dy * c.rc.v_cap, len)
    }

    /// The wire constructor: the ref's cap plus the wire's, fanout, slew limit and area.
    fn new_wire(&mut self, at: (i32, i32), r: usize, layer: i32) -> usize {
        let (_, wire_cap, len) = self.wire_rc_of(at, r, layer);
        let p = &self.nodes[r];
        let cap = (f64::from(p.cap) + self.ctx.dbu_to_meters(len) * wire_cap) as f32;
        let node = Node { kind: Kind::Wire { r }, x: at.0, y: at.1, layer, cap, fanout: p.fanout, max_load_slew: p.max_load_slew, area: p.area, slack_rf: None, slack: FixedDelay::ZERO, delay: FixedDelay::ZERO, arrival: FixedDelay::ZERO };
        self.push(node)
    }

    /// `createBnetJunction`: the junction constructor, the combined transition, the worse slack.
    fn create_junction(&mut self, p: usize, q: usize, at: (i32, i32)) -> usize {
        let (a, b) = (&self.nodes[p], &self.nodes[q]);
        let node = Node {
            kind: Kind::Junction { r: p, r2: q },
            x: at.0,
            y: at.1,
            layer: crate::buffered_net::NULL_LAYER,
            cap: a.cap + b.cap,
            fanout: a.fanout + b.fanout,
            max_load_slew: a.max_load_slew.min(b.max_load_slew),
            area: a.area + b.area,
            slack_rf: combined(a.slack_rf, b.slack_rf),
            slack: a.slack.min(b.slack),
            delay: FixedDelay::ZERO,
            arrival: FixedDelay::ZERO,
        };
        self.push(node)
    }

    /// The buffer constructor: the input port's cap, fanout and slew limit; the area added.
    fn new_buffer(&mut self, at: (i32, i32), size: usize, r: usize) -> usize {
        let s = &self.ctx.sizes[size];
        let node = Node {
            kind: Kind::Buffer { cell: s.cell.clone(), r },
            x: at.0,
            y: at.1,
            layer: crate::buffered_net::NULL_LAYER,
            cap: s.in_cap_cmd,
            fanout: s.in_fanout,
            max_load_slew: s.in_max_slew,
            area: self.nodes[r].area + s.area,
            slack_rf: None,
            slack: FixedDelay::ZERO,
            delay: FixedDelay::ZERO,
            arrival: FixedDelay::ZERO,
        };
        self.push(node)
    }

    /// `Rebuffer::addWire`: a wire from `end` to `p`, its delay `R·(C/2 + Cload)` taken off the
    /// slack and its slew degradation off the slew limit.
    fn add_wire(&mut self, p: usize, end: (i32, i32), layer: i32, level: Option<usize>) -> usize {
        let z = self.new_wire(end, p, layer);
        let (layer_res, layer_cap, len) = self.wire_rc_of(end, p, layer);
        let wire_length = self.ctx.dbu_to_meters(len);
        let wire_res = wire_length * layer_res;
        let wire_cap = wire_length * layer_cap;
        let pc = f64::from(self.nodes[p].cap);
        let wire_delay = FixedDelay::from_secs((wire_res * (wire_cap / 2.0 + pc)) as f32);
        let (ps, prf, pml) = (self.nodes[p].slack, self.nodes[p].slack_rf, self.nodes[p].max_load_slew);
        let nz = &mut self.nodes[z];
        nz.delay = wire_delay;
        nz.slack = ps - wire_delay;
        nz.slack_rf = prf;
        nz.max_load_slew = (f64::from(pml) - wire_res * (wire_cap / 2.0 + pc) * f64::from(self.ctx.slew_shape_factor)) as f32;
        if let Some(level) = level {
            let line = format!("{:level$}wire wl {} {}", "", len, self.to_string(z));
            self.debug(3, line);
        }
        z
    }

    /// `Rebuffer::bufferDelay(cell, rf, load)`: over the transitions, the largest delay.
    fn buffer_delay(&self, size: usize, rf: Option<Rfs>, load: f32) -> FixedDelay {
        let mut delay = FixedDelay::ZERO;
        if let Some(rf) = rf {
            let s = &self.ctx.sizes[size];
            // Each transition at its arrival path's scene.
            for &k in rf.range() {
                let (d, _) = gate_delays(self.ctx.scene_cell(self.probe.arc_scenes[k], &s.cell), &s.out_port, load, self.ctx.tgt_slews);
                delay = delay.max(FixedDelay::from_secs(d[k]));
            }
        }
        delay
    }

    /// `bufferSizeCanDriveLoad`: with `extra` dbu of signal wire, the size's slew into the load
    /// within its margined limit and the load within the size's margined max cap.
    fn can_drive(&self, size: usize, n: usize, extra: i32) -> bool {
        let s = &self.ctx.sizes[size];
        let (wire_res, wire_cap) = self.ctx.signal_rc();
        let seg_cap = (self.ctx.dbu_to_meters(extra) * wire_cap) as f32;
        let seg_res = (self.ctx.dbu_to_meters(extra) * wire_res) as f32;
        let nd = &self.nodes[n];
        let load_slew = s.driver_resistance * (nd.cap + seg_cap) + seg_cap * seg_res * self.ctx.slew_shape_factor / 2.0;
        load_slew <= max_slew_margined(nd.max_load_slew) && (nd.cap + seg_cap) <= s.margined_max_cap
    }

    /// `drvrPinTiming`: per slack transition, the driver's arc at the option's load plus its own
    /// pin cap — the largest delay and slew, the smallest arrival correction.
    fn drvr_pin_timing(&self, n: usize) -> (f32, f32, f32) {
        let nd = &self.nodes[n];
        let Some(rfs) = nd.slack_rf else { return (0.0, 0.0, 0.0) };
        let (mut delay, mut correction, mut slew) = (0.0f32, INF, 0.0f32);
        for &rf in rfs.range() {
            let (rf_delay, rf_correction, rf_slew) = match self.probe.arcs[rf].as_ref().and_then(|a| a.as_ref()) {
                Some(a) => {
                    let Model::Gate(m) = &a.model else { unreachable!("a gate arc") };
                    let (gd, gs) = m.gate_delay(a.from_slew, nd.cap + self.probe.drvr_port_cap);
                    (gd, a.arrival - (a.prev_arrival + gd), gs)
                }
                None => (0.0, 0.0, 0.0),
            };
            delay = delay.max(rf_delay);
            correction = correction.min(rf_correction);
            slew = slew.max(rf_slew);
        }
        (delay, correction, slew)
    }

    fn slack_at_driver_pin(&self, n: usize) -> FixedDelay {
        self.nodes[n].slack + FixedDelay::from_secs(self.drvr_pin_timing(n).1)
    }

    /// `evaluateOption`: the slack at the driver into the option, unless the driver cannot drive
    /// it (load slew) or its pin slew exceeds the limit at a load above any seen to pass.
    fn evaluate_option(&mut self, n: usize, index: i64) -> Option<FixedDelay> {
        let (_, correction, slew) = self.drvr_pin_timing(n);
        let slack = self.nodes[n].slack + FixedDelay::from_secs(correction);
        let (cap, mls) = (self.nodes[n].cap, self.nodes[n].max_load_slew);
        if self.probe.drvr_resistance * cap > max_slew_margined(mls) {
            return None;
        }
        if slew > self.probe.drvr_pin_max_slew && cap > self.drvr_load_high_water_mark {
            return None;
        }
        self.drvr_load_high_water_mark = cap.max(self.drvr_load_high_water_mark);
        let line = format!("option {:3}: {:2} buffers {:.2} area slack {} cap {}", index, self.buffer_count(n), f64::from(self.nodes[n].area), self.ctx.delay(slack.to_secs()), self.ctx.cap_str(cap));
        self.debug(2, line);
        Some(slack)
    }

    fn strip_wire(&self, mut n: usize) -> usize {
        while let Kind::Wire { r } | Kind::Via { r, .. } = self.nodes[n].kind {
            n = r;
        }
        n
    }

    fn strip_wires_and_buffers(&self, mut n: usize) -> usize {
        loop {
            match self.nodes[n].kind {
                Kind::Wire { r } | Kind::Buffer { r, .. } | Kind::Via { r, .. } => n = r,
                _ => return n,
            }
        }
    }

    /// `findWireLayer`: past buffers and vias to the first wire — its layer — else none.
    fn find_wire_layer(&self, mut n: usize) -> i32 {
        loop {
            match self.nodes[n].kind {
                Kind::Buffer { r, .. } | Kind::Via { r, .. } => n = r,
                Kind::Wire { .. } => return self.nodes[n].layer,
                _ => return crate::buffered_net::NULL_LAYER,
            }
        }
    }

    /// `attemptTopologyRewrite`: when the critical side is a junction and a buffer sits on one of
    /// the other two branches, the two non-critical branches are joined at their median point
    /// behind the smallest buffer that keeps the critical branch critical.
    fn attempt_topology_rewrite(&mut self, node: usize, left: usize, right: usize, best_cap: f32) -> Option<usize> {
        let junc_slack = self.nodes[left].slack.min(self.nodes[right].slack);
        let (crit1, mut aux1) = if self.nodes[left].slack < self.nodes[right].slack { (self.strip_wire(left), self.strip_wire(right)) } else { (self.strip_wire(right), self.strip_wire(left)) };
        let Kind::Junction { r: c2, r2: a2 } = self.nodes[crit1].kind else { return None };
        let (mut crit2, mut aux2) = (c2, a2);
        if self.nodes[crit2].slack > self.nodes[aux2].slack {
            std::mem::swap(&mut crit2, &mut aux2);
        }
        aux2 = self.strip_wire(aux2);
        let is_buf = |k: &Kind| matches!(k, Kind::Buffer { .. });
        if !(is_buf(&self.nodes[aux1].kind) || is_buf(&self.nodes[aux2].kind)) {
            return None;
        }
        aux1 = self.strip_wires_and_buffers(aux1);
        aux2 = self.strip_wires_and_buffers(aux2);
        crit2 = self.strip_wires_and_buffers(crit2);
        let (p1, p2, p3) = (self.loc(aux1), self.loc(aux2), self.loc(crit2));
        let jp = (middle_value(p1.0, p2.0, p3.0), middle_value(p1.1, p2.1, p3.1));
        let node_loc = self.loc(node);
        let in1 = self.add_wire(aux1, jp, crate::buffered_net::NULL_LAYER, None);
        let in2 = self.add_wire(aux2, jp, crate::buffered_net::NULL_LAYER, None);
        let j = self.create_junction(in1, in2, jp);
        let junc1 = self.add_wire(j, node_loc, crate::buffered_net::NULL_LAYER, None);
        let in3 = self.add_wire(crit2, node_loc, crate::buffered_net::NULL_LAYER, None);
        let lr_cap = self.nodes[left].cap + self.nodes[right].cap;
        for size in 0..self.ctx.sizes.len() {
            let s = &self.ctx.sizes[size];
            let in3_cap = s.in_cap + self.nodes[in3].cap;
            if !fuzzy::less(in3_cap, best_cap) || !fuzzy::less(in3_cap, lr_cap) || self.nodes[junc1].slack - s.intrinsic_delay < junc_slack {
                break;
            }
            let buffer_delay = self.buffer_delay(size, self.nodes[junc1].slack_rf, self.nodes[junc1].cap);
            let buffer_slack = self.nodes[junc1].slack - buffer_delay;
            if buffer_slack >= junc_slack && self.can_drive(size, junc1, 0) {
                let b = self.new_buffer(node_loc, size, junc1);
                let rf = self.nodes[junc1].slack_rf;
                let nb = &mut self.nodes[b];
                nb.slack = buffer_slack;
                nb.slack_rf = rf;
                nb.delay = buffer_delay;
                return Some(self.create_junction(b, in3, node_loc));
            }
        }
        None
    }

    /// `timingOrder`: unconstrained first, then the larger slack (compared in seconds), the
    /// smaller cap, the smaller area, the fewer buffers.
    pub fn timing_order(&self, lhs: &RankedOption, rhs: &RankedOption) -> bool {
        if lhs.unconstrained != rhs.unconstrained {
            return lhs.unconstrained;
        }
        if lhs.slack.to_secs() != rhs.slack.to_secs() {
            return lhs.slack > rhs.slack;
        }
        let (l, r) = (&self.nodes[lhs.node()], &self.nodes[rhs.node()]);
        if l.cap != r.cap {
            return l.cap < r.cap;
        }
        if l.area != r.area {
            return l.area < r.area;
        }
        self.buffer_count(lhs.node()) < self.buffer_count(rhs.node())
    }

    /// `areaOrder`: meets the target, preserves the best slack, the larger slack among those that
    /// do not, the smaller area among those that meet and preserve, then cap, buffer count, slack
    /// and generation index.
    pub fn area_order(&self, lhs: &RankedOption, rhs: &RankedOption) -> bool {
        if lhs.meets_target != rhs.meets_target {
            return lhs.meets_target;
        }
        if lhs.preserves_best_slack != rhs.preserves_best_slack {
            return lhs.preserves_best_slack;
        }
        if !lhs.preserves_best_slack && lhs.slack.to_secs() != rhs.slack.to_secs() {
            return lhs.slack > rhs.slack;
        }
        let (l, r) = (&self.nodes[lhs.node()], &self.nodes[rhs.node()]);
        if lhs.meets_target && lhs.preserves_best_slack && l.area != r.area {
            return l.area < r.area;
        }
        if l.cap != r.cap {
            return l.cap < r.cap;
        }
        let (lb, rb) = (self.buffer_count(lhs.node()), self.buffer_count(rhs.node()));
        if lb != rb {
            return lb < rb;
        }
        if lhs.slack.to_secs() != rhs.slack.to_secs() {
            return lhs.slack > rhs.slack;
        }
        lhs.index < rhs.index
    }

    /// `std::ranges::sort(options, order)`: libc++'s unstable sort, so fully tied options keep the
    /// order it leaves them in. A range that reaches its heap fallback is refused.
    fn sort_ranked(&mut self, options: &mut [RankedOption], area: bool) {
        let sorted = {
            let me = &*self;
            crate::order::libcxx_sort_by(options, |a, b| if area { me.area_order(a, b) } else { me.timing_order(a, b) })
        };
        if let Err(h) = sorted {
            self.refused.get_or_insert(format!("rebuffer option ranking: {} options reach the sort's heap fallback, which is not modelled", h.len));
        }
    }

    /// `selectTimingOptions`: each option the driver can drive, with its driver slack; for one plan
    /// the best (the serial rule: the first, an unconstrained one, or a strictly larger slack),
    /// else the `max_plans` first in timing order.
    pub fn select_timing_options(&mut self, top: &[usize], max_plans: usize) -> Vec<RankedOption> {
        if max_plans == 1 {
            let mut best_slack = -FixedDelay::inf();
            let mut best: Option<usize> = None;
            let mut best_index = 0;
            let mut i = 1;
            self.debug(2, "timing-optimized options".into());
            for &p in top {
                let Some(slack) = self.evaluate_option(p, i) else { continue };
                if best.is_none() || self.nodes[p].slack_rf.is_none() || slack > best_slack {
                    best_slack = slack;
                    best = Some(p);
                    best_index = i;
                }
                i += 1;
            }
            let Some(b) = best else {
                self.debug(2, "no available option".into());
                return Vec::new();
            };
            self.debug(2, format!("best option {best_index}"));
            return vec![RankedOption { option: Some(b), slack: best_slack, unconstrained: self.nodes[b].slack_rf.is_none(), index: best_index, ..RankedOption::default() }];
        }
        let mut ranked: Vec<RankedOption> = Vec::with_capacity(top.len());
        let mut i = 1;
        self.debug(2, "timing-optimized options".into());
        for &p in top {
            let Some(slack) = self.evaluate_option(p, i) else { continue };
            ranked.push(RankedOption { option: Some(p), slack, unconstrained: self.nodes[p].slack_rf.is_none(), index: i, ..RankedOption::default() });
            i += 1;
        }
        self.sort_ranked(&mut ranked, false);
        match ranked.first() {
            Some(f) => {
                let line = format!("best option {}", f.index);
                self.debug(2, line);
            }
            None => self.debug(2, "no available option".into()),
        }
        ranked.truncate(max_plans);
        ranked
    }

    /// `bufferForTimingTopK`: the timing options at the root (the visitor below), then
    /// `selectTimingOptions`; none, with RSZ-2009, when the root has no option.
    pub fn buffer_for_timing_top_k(&mut self, tree: usize, allow_topology_rewrite: bool, max_plans: usize) -> Vec<RankedOption> {
        let top = self.timing_options(tree, 1, allow_topology_rewrite);
        if top.is_empty() {
            let line = format!("[WARNING RSZ-2009] Skipping buffering because no valid buffering solution satisfying the design rules can be found for net connected to pin {}.", self.pin);
            self.warn(line);
            return Vec::new();
        }
        self.select_timing_options(&top, max_plans)
    }

    /// `bufferForTiming`: the single best of `bufferForTimingTopK`.
    pub fn buffer_for_timing(&mut self, tree: usize, allow_topology_rewrite: bool) -> Option<usize> {
        self.buffer_for_timing_top_k(tree, allow_topology_rewrite, 1).first().and_then(|o| o.option)
    }

    /// `bufferForTiming`'s visitor at `node` (`level` from 1 at the root).
    fn timing_options(&mut self, node: usize, level: usize, rewrite: bool) -> Vec<usize> {
        match self.nodes[node].kind.clone() {
            Kind::Wire { r } | Kind::Buffer { r, .. } | Kind::Via { r, .. } => {
                let layer = self.find_wire_layer(node);
                let rf_ = self.strip_wires_and_buffers(r);
                let ref_loc = self.loc(rf_);
                let target = self.loc(node);
                let opts = self.timing_options(rf_, level + 1, rewrite);
                let segment_wl = (target.0 - ref_loc.0).abs() + (target.1 - ref_loc.1).abs();
                let strong = self.ctx.sizes.len() - 1;
                let mut wired: Vec<usize> = Vec::with_capacity(opts.len());
                for &opt in &opts {
                    if self.can_drive(strong, opt, segment_wl) {
                        wired.push(self.add_wire(opt, target, layer, None));
                    }
                }
                let (_, wire_cap) = self.ctx.rc_on(layer);
                for size in 0..self.ctx.sizes.len() {
                    let sz = &self.ctx.sizes[size];
                    let asym = if layer == crate::buffered_net::NULL_LAYER { sz.asym.unwrap_or_default() } else { sz.asym_layers.get(&layer).copied().unwrap_or_default() };
                    let target_load = (f64::from(asym.input_cap) + f64::from(asym.buffer_spacing) * wire_cap) as f32;
                    let mut best_appraisal = f32::NEG_INFINITY;
                    let mut head: Option<usize> = None;
                    for &opt in &opts {
                        let o = &self.nodes[opt];
                        let appraisal = o.slack.to_secs() - asym.delay_per_farad * o.cap;
                        if appraisal > best_appraisal && o.cap < target_load {
                            head = Some(opt);
                            best_appraisal = appraisal;
                        }
                    }
                    let Some(mut h) = head else { continue };
                    if wire_cap <= 0.0 {
                        continue;
                    }
                    let mut inserted = false;
                    loop {
                        let hl = self.loc(h);
                        let remaining = (target.0 - hl.0).abs() + (target.1 - hl.1).abs();
                        let step = self.ctx.meters_to_dbu(f64::from(target_load - self.nodes[h].cap) / wire_cap);
                        if step >= remaining {
                            break;
                        }
                        let (mut dx, mut dy) = (target.0 - hl.0, target.1 - hl.1);
                        if dx.abs() + dy.abs() >= step {
                            let ratio = dx.abs() as f32 / (dx.abs() + dy.abs()) as f32;
                            let dx_abs = ((ratio * step as f32) as i32).min(step);
                            let dy_abs = step - dx_abs;
                            dx = if dx > 0 { dx_abs } else { -dx_abs };
                            dy = if dy > 0 { dy_abs } else { -dy_abs };
                        }
                        let next = (hl.0 + dx, hl.1 + dy);
                        let wire_head = self.add_wire(h, next, layer, None);
                        let bd = self.buffer_delay(size, self.nodes[wire_head].slack_rf, self.nodes[wire_head].cap);
                        if !self.can_drive(size, wire_head, 0) {
                            inserted = false;
                            break;
                        }
                        let b = self.new_buffer(next, size, wire_head);
                        let (ws, wrf) = (self.nodes[wire_head].slack, self.nodes[wire_head].slack_rf);
                        let nb = &mut self.nodes[b];
                        nb.slack = ws - bd;
                        nb.slack_rf = wrf;
                        nb.delay = bd;
                        h = b;
                        inserted = true;
                    }
                    if inserted {
                        let hl = self.loc(h);
                        let remaining = (target.0 - hl.0).abs() + (target.1 - hl.1).abs();
                        if self.can_drive(strong, h, remaining) {
                            let hw = self.add_wire(h, target, layer, None);
                            let hc = self.nodes[hw].cap;
                            let at = wired.iter().position(|&o| self.nodes[o].cap >= hc).unwrap_or(wired.len());
                            wired.insert(at, hw);
                        }
                    }
                }
                self.insert_buffer_options(&mut wired, level, 0, false, FixedDelay::ZERO, None, true);
                wired
            }
            Kind::Junction { r, r2 } => {
                let left = self.timing_options(r, level + 1, rewrite);
                let right = self.timing_options(r2, level + 1, rewrite);
                let mut opts = Vec::new();
                let mut best_cap = INF;
                // Reverse iterators over both lists.
                let (lend, rend) = (left.len(), right.len());
                let (mut li, mut ri) = (0usize, 0usize);
                let lat = |k: usize| left[lend - 1 - k];
                let rat = |k: usize| right[rend - 1 - k];
                while li < lend && ri < rend {
                    while li + 1 < lend && self.nodes[lat(li + 1)].slack >= self.nodes[rat(ri)].slack {
                        li += 1;
                    }
                    while ri + 1 < rend && self.nodes[rat(ri + 1)].slack >= self.nodes[lat(li)].slack {
                        ri += 1;
                    }
                    let mut rewrote = false;
                    let mut junc = None;
                    if rewrite {
                        junc = self.attempt_topology_rewrite(node, lat(li), rat(ri), best_cap);
                        rewrote = junc.is_some();
                    }
                    let at = self.loc(node);
                    let junc = junc.unwrap_or_else(|| self.create_junction(lat(li), rat(ri), at));
                    if self.nodes[junc].fanout <= self.probe.fanout_limit {
                        let line = format!("{:level$}{}{}", "", if rewrote { "(rewritten) " } else { "" }, self.to_string(junc));
                        self.debug(3, line);
                        best_cap = self.nodes[junc].cap;
                        opts.push(junc);
                    }
                    loop {
                        let next_l = if li + 1 < lend { self.nodes[lat(li + 1)].slack } else { -FixedDelay::inf() };
                        let next_r = if ri + 1 < rend { self.nodes[rat(ri + 1)].slack } else { -FixedDelay::inf() };
                        if next_l > next_r {
                            li += 1;
                        } else {
                            ri += 1;
                        }
                        if li == lend || ri == rend || self.nodes[lat(li)].cap + self.nodes[rat(ri)].cap < best_cap {
                            break;
                        }
                    }
                }
                opts.reverse();
                opts
            }
            Kind::Load { .. } => {
                let line = format!("{:level$}{}", "", self.to_string(node));
                self.debug(3, line);
                vec![node]
            }
        }
    }

    /// `fitsEnvelope`.
    fn fits_envelope(&self, n: usize, env: &Envelope) -> bool {
        let nd = &self.nodes[n];
        nd.slack >= env.slack && !fuzzy::greater(nd.cap, env.cap) && !fuzzy::less(nd.max_load_slew, env.max_load_slew) && !fuzzy::greater(nd.fanout, env.fanout)
    }

    fn metrics(&self, n: usize, slack: FixedDelay) -> Envelope {
        let nd = &self.nodes[n];
        Envelope { slack, cap: nd.cap, max_load_slew: nd.max_load_slew, fanout: nd.fanout }
    }

    /// `insertAssuredOption`: before the first option with at least its cap.
    fn insert_assured(&mut self, opts: &mut Vec<usize>, assured: usize, level: usize) {
        let c = self.nodes[assured].cap;
        let at = opts.iter().position(|&o| self.nodes[o].cap >= c).unwrap_or(opts.len());
        let line = format!("{:level$}assured fixup: {}", "", self.to_string(assured));
        self.debug(3, line);
        opts.insert(at, assured);
    }

    /// `insertBufferOptions`: the options passed through when they improve on the last kept
    /// (slack, or area above the threshold), each buffer size placed on the best option it can
    /// drive; in area mode an option fitting the exemplar's envelope is assured.
    /// `strict_assured`: without an assured option the reference errors (RSZ-0501); otherwise the
    /// options are cleared and `false` returned.
    #[allow(clippy::too_many_arguments)]
    fn insert_buffer_options(&mut self, opts: &mut Vec<usize>, level: usize, next_segment_wl: i32, area_oriented: bool, slack_threshold: FixedDelay, exemplar: Option<usize>, strict_assured: bool) -> bool {
        if opts.is_empty() {
            return true;
        }
        let strong = self.ctx.sizes.len() - 1;
        let envelope = if area_oriented { Some(self.metrics(exemplar.expect("an exemplar"), slack_threshold)) } else { None };
        let mut assured_satisfied = !area_oriented;
        let mut best_area = INF;
        let mut best_slack = -FixedDelay::inf();
        let mut new_opts: Vec<usize> = Vec::with_capacity(opts.len() * 2);
        let mut it = 0usize;
        let pass_through = |me: &mut Self, it: &mut usize, new_opts: &mut Vec<usize>, threshold_cap: f32, best_area: &mut f32, best_slack: &mut FixedDelay, assured_satisfied: &mut bool| {
            while *it < opts.len() && me.nodes[opts[*it]].cap <= threshold_cap {
                let opt = opts[*it];
                let mut keep = if area_oriented { fuzzy::less(me.nodes[opt].area, *best_area) && me.nodes[opt].slack >= slack_threshold } else { me.nodes[opt].slack > *best_slack };
                if !me.can_drive(strong, opt, next_segment_wl) {
                    keep = false;
                }
                if keep {
                    new_opts.push(opt);
                    let line = format!("{:level$}{}", "", me.to_string(opt));
                    me.debug(3, line);
                    if !*assured_satisfied && me.fits_envelope(opt, envelope.as_ref().expect("area mode")) {
                        *assured_satisfied = true;
                    }
                    *best_slack = me.nodes[opt].slack;
                    *best_area = me.nodes[opt].area;
                }
                *it += 1;
            }
        };
        for size in 0..self.ctx.sizes.len() {
            let in_cap = self.ctx.sizes[size].in_cap;
            pass_through(self, &mut it, &mut new_opts, in_cap, &mut best_area, &mut best_slack, &mut assured_satisfied);
            let mut load_opt: Option<usize> = None;
            let mut load_opt_delay = FixedDelay::ZERO;
            let start = if new_opts.is_empty() && it == opts.len() && it > 0 { it - 1 } else { it };
            for &opt in &opts[start..] {
                let s = &self.ctx.sizes[size];
                let o = &self.nodes[opt];
                let cand = if area_oriented { o.slack - s.intrinsic_delay >= slack_threshold && fuzzy::less(o.area + s.area, best_area) } else { (o.slack - s.intrinsic_delay) > best_slack };
                if cand && self.can_drive(size, opt, 0) {
                    let bd = self.buffer_delay(size, self.nodes[opt].slack_rf, self.nodes[opt].cap);
                    let slack = self.nodes[opt].slack - bd;
                    if if area_oriented { slack >= slack_threshold } else { slack > best_slack } {
                        load_opt = Some(opt);
                        load_opt_delay = bd;
                        best_slack = slack;
                        best_area = self.nodes[opt].area + self.ctx.sizes[size].area;
                    }
                }
            }
            if let Some(lo) = load_opt {
                let at = self.loc(lo);
                let z = self.new_buffer(at, size, lo);
                let rf = self.nodes[lo].slack_rf;
                {
                    let nz = &mut self.nodes[z];
                    nz.slack = best_slack;
                    nz.slack_rf = rf;
                    nz.delay = load_opt_delay;
                }
                if !assured_satisfied && self.fits_envelope(z, envelope.as_ref().expect("area mode")) {
                    assured_satisfied = true;
                }
                let line = format!("{:level$}buffer {} load {} delay {}: {}", "", self.ctx.sizes[size].cell, self.ctx.cap_str(self.nodes[lo].cap), self.ctx.delay(load_opt_delay.to_secs()), self.to_string(z));
                self.debug(3, line);
                new_opts.push(z);
            }
        }
        pass_through(self, &mut it, &mut new_opts, INF, &mut best_area, &mut best_slack, &mut assured_satisfied);
        if !assured_satisfied {
            let env = envelope.expect("area mode");
            let ex = exemplar.expect("an exemplar");
            if let Kind::Buffer { cell, .. } = self.nodes[ex].kind.clone() {
                let size = self.ctx.sizes.iter().position(|s| s.cell == cell).expect("a characterized size");
                let mut best_area = INF;
                let mut best_option: Option<usize> = None;
                for &lo in opts.iter() {
                    if self.nodes[lo].area >= best_area {
                        continue;
                    }
                    let bd = self.buffer_delay(size, self.nodes[lo].slack_rf, self.nodes[lo].cap);
                    if self.can_drive(size, lo, 0) && self.nodes[lo].slack - bd >= slack_threshold {
                        let at = self.loc(lo);
                        let z = self.new_buffer(at, size, lo);
                        let (ls, lrf) = (self.nodes[lo].slack, self.nodes[lo].slack_rf);
                        let nz = &mut self.nodes[z];
                        nz.slack = ls - bd;
                        nz.slack_rf = lrf;
                        nz.delay = bd;
                        if self.fits_envelope(z, &env) {
                            best_area = self.nodes[lo].area;
                            best_option = Some(z);
                        }
                    }
                }
                if let Some(b) = best_option {
                    self.insert_assured(&mut new_opts, b, level);
                    assured_satisfied = true;
                }
            } else {
                for &o in opts.iter() {
                    if self.fits_envelope(o, &env) {
                        self.insert_assured(&mut new_opts, o, level);
                        assured_satisfied = true;
                        break;
                    }
                }
            }
            if !assured_satisfied {
                if !strict_assured {
                    opts.clear();
                    return false;
                }
                self.error_area_recovery_unreproducible();
            }
        }
        *opts = new_opts;
        true
    }

    /// `errorAreaRecoveryUnreproducible` (RSZ-0501): where the reference stops.
    fn error_area_recovery_unreproducible(&mut self) {
        if self.failed.is_none() {
            self.failed = Some(format!("RSZ-0501: buffering pin {} failed: area recovery cannot reproduce solution", self.pin));
        }
    }

    /// `recoverArea`: the arrival spread down from the root, then per node the cheapest options
    /// meeting a slack threshold that moves from the node's slack toward the target by `alpha`.
    pub fn recover_area(&mut self, root: usize, slack_target: FixedDelay, alpha: f32) -> Option<usize> {
        let slack_target = self.annotate_arrival_delays(root, slack_target);
        let top = self.area_recovery_options(root, slack_target, alpha, true);
        let mut best_slack = -FixedDelay::inf();
        let mut best_area = f32::MAX;
        let (mut best_slack_option, mut best_area_option) = (None, None);
        let (mut best_slack_index, mut best_area_index) = (0, 0);
        let mut i = 1;
        self.debug(2, "area-optimized options".into());
        for p in top {
            let Some(slack) = self.evaluate_option(p, i) else { continue };
            let unconstrained = self.nodes[p].slack_rf.is_none();
            if best_slack_option.is_none() || unconstrained || slack > best_slack {
                best_slack = slack;
                best_slack_option = Some(p);
                best_slack_index = i;
            }
            if (slack >= slack_target || unconstrained) && (best_area_option.is_none() || fuzzy::less(self.nodes[p].area, best_area)) {
                best_area = self.nodes[p].area;
                best_area_option = Some(p);
                best_area_index = i;
            }
            i += 1;
        }
        if best_area_option.is_some() {
            self.debug(2, format!("best option {best_area_index} (area optimized)"));
            return best_area_option;
        }
        if best_slack_option.is_some() {
            self.debug(2, format!("best option {best_slack_index} (closest to meeting timing target)"));
            return best_slack_option;
        }
        self.debug(2, "no available option".into());
        None
    }

    /// `recoverAreaTopK`: the area options of `root` with their driver slacks; the `max_plans`
    /// first in area order when the first meets the target, else the one with the best slack
    /// (the serial rule).
    pub fn recover_area_top_k(&mut self, root: usize, slack_target: FixedDelay, alpha: f32, max_plans: usize, strict_assured: bool) -> Vec<RankedOption> {
        let slack_target = self.annotate_arrival_delays(root, slack_target);
        let top = self.area_recovery_options(root, slack_target, alpha, strict_assured);
        let mut ranked: Vec<RankedOption> = Vec::with_capacity(top.len());
        let mut best_slack = -FixedDelay::inf();
        let mut best_slack_option: Option<usize> = None;
        let mut best_slack_index = 0;
        let mut i = 1;
        self.debug(2, "area-optimized options".into());
        for p in top {
            let Some(slack) = self.evaluate_option(p, i) else { continue };
            let unconstrained = self.nodes[p].slack_rf.is_none();
            if best_slack_option.is_none() || unconstrained || slack > best_slack {
                best_slack = slack;
                best_slack_option = Some(p);
                best_slack_index = i;
            }
            ranked.push(RankedOption { option: Some(p), slack, unconstrained, meets_target: slack >= slack_target || unconstrained, preserves_best_slack: false, index: i, ..RankedOption::default() });
            i += 1;
        }
        let slack_floor = slack_loss_floor(best_slack);
        for o in &mut ranked {
            o.preserves_best_slack = o.slack >= slack_floor || o.unconstrained;
        }
        self.sort_ranked(&mut ranked, true);
        if ranked.first().is_some_and(|o| o.meets_target) {
            let line = format!("best option {} (area optimized)", ranked[0].index);
            self.debug(2, line);
            ranked.truncate(max_plans);
            return ranked;
        }
        if let Some(b) = best_slack_option {
            self.debug(2, format!("best option {best_slack_index} (closest to meeting timing target)"));
            return vec![RankedOption { option: Some(b), slack: best_slack, unconstrained: self.nodes[b].slack_rf.is_none(), index: best_slack_index, ..RankedOption::default() }];
        }
        self.debug(2, "no available option".into());
        Vec::new()
    }

    /// `treeSignature`: a type-tagged pre-order encoding of the tree's structure — locations,
    /// layers, buffer cells and load pins (the reference encodes the pin and cell pointers,
    /// which within one planning call identify them as their names do).
    pub fn tree_signature(&self, n: usize) -> String {
        let nd = &self.nodes[n];
        match &nd.kind {
            Kind::Load { pin } => format!("L{pin}"),
            Kind::Wire { r } => format!("W{},{},{}[{}]", nd.x, nd.y, nd.layer, self.tree_signature(*r)),
            Kind::Via { r, ref_layer } => format!("V{},{},{},{ref_layer}[{}]", nd.x, nd.y, nd.layer, self.tree_signature(*r)),
            Kind::Buffer { cell, r } => format!("B{cell},{},{}[{}]", nd.x, nd.y, self.tree_signature(*r)),
            Kind::Junction { r, r2 } => format!("J{},{}[{}][{}]", nd.x, nd.y, self.tree_signature(*r), self.tree_signature(*r2)),
        }
    }

    /// `keepAnchorTree`: the anchor replaces the selected option that is its structural twin's
    /// tree; else it takes a free slot, else the last.
    pub fn keep_anchor_tree(ranked: &[RankedOption], anchor: &RankedOption, anchor_signature: &str, plan_limit: usize, selected: &mut Vec<RankedOption>) {
        let Some(twin) = ranked.iter().find(|o| o.signature == anchor_signature).map(|o| o.option) else { return };
        if let Some(s) = selected.iter_mut().find(|s| s.option == twin) {
            *s = anchor.clone();
            return;
        }
        if selected.len() < plan_limit {
            selected.push(anchor.clone());
        } else if let Some(last) = selected.last_mut() {
            *last = anchor.clone();
        }
    }

    /// `uniqueTreeOptions`: each option's signature recorded; an option whose structure an earlier
    /// one has is dropped.
    pub fn unique_tree_options(&self, options: Vec<RankedOption>) -> Vec<RankedOption> {
        let mut seen = std::collections::HashSet::new();
        let mut unique = Vec::with_capacity(options.len());
        for mut o in options {
            o.signature = self.tree_signature(o.node());
            if seen.insert(o.signature.clone()) {
                unique.push(o);
            }
        }
        unique
    }

    /// `limitTimingOptions`: the unique options in timing order, the `plan_limit` first, the
    /// anchor kept.
    pub fn limit_timing_options(&mut self, options: Vec<RankedOption>, anchor: &RankedOption, plan_limit: usize) -> Vec<RankedOption> {
        let mut ranked = self.unique_tree_options(options);
        self.sort_ranked(&mut ranked, false);
        let mut selected: Vec<RankedOption> = ranked.iter().take(plan_limit).cloned().collect();
        if let Some(a) = anchor.option {
            let sig = self.tree_signature(a);
            Self::keep_anchor_tree(&ranked, anchor, &sig, plan_limit, &mut selected);
        }
        selected
    }

    /// `limitAreaOptions`: the unique options with this round's target (meets) and order position,
    /// the best slack's loss floor (preserves), in area order, the `plan_limit` first, the anchor
    /// kept.
    pub fn limit_area_options(&mut self, options: Vec<RankedOption>, slack_target: FixedDelay, anchor: &RankedOption, plan_limit: usize) -> Vec<RankedOption> {
        let mut ranked = self.unique_tree_options(options);
        let mut best_slack = -FixedDelay::inf();
        for (index, o) in ranked.iter_mut().enumerate() {
            best_slack = best_slack.max(o.slack);
            o.meets_target = o.slack >= slack_target || o.unconstrained;
            o.index = index as i64;
        }
        let slack_floor = slack_loss_floor(best_slack);
        for o in &mut ranked {
            o.preserves_best_slack = o.slack >= slack_floor || o.unconstrained;
        }
        self.sort_ranked(&mut ranked, true);
        let mut selected: Vec<RankedOption> = ranked.iter().take(plan_limit).cloned().collect();
        if let Some(a) = anchor.option {
            let sig = self.tree_signature(a);
            Self::keep_anchor_tree(&ranked, anchor, &sig, plan_limit, &mut selected);
        }
        selected
    }

    /// `areaRecoveryTarget`: the tree's driver slack less 1% of its driver gate delay (floored at
    /// 0) plus its critical path delay.
    pub fn area_recovery_target(&self, tree: usize) -> FixedDelay {
        let (gd, _, _) = self.drvr_pin_timing(tree);
        let relaxation = (gd.max(0.0) + self.critical_path_delay(tree).to_secs()) * RELAXATION_FACTOR;
        self.slack_at_driver_pin(tree) - FixedDelay::from_secs(relaxation)
    }

    /// `runTimingBeam`: three rounds of `bufferForTimingTopK` over the beam; the anchor follows the
    /// top option of the previous anchor's expansion. `None`, with RSZ-2021, when a round leaves
    /// no option.
    pub fn run_timing_beam(&mut self, bnet: usize, allow_topology_rewrite: bool, plan_limit: usize, trace: &mut Vec<String>) -> Option<(Vec<RankedOption>, RankedOption)> {
        let mut beam = vec![RankedOption { option: Some(bnet), ..RankedOption::default() }];
        let mut anchor = RankedOption { option: Some(bnet), ..RankedOption::default() };
        for i in 0..3 {
            let mut next_options = Vec::new();
            let mut next_anchor = RankedOption::default();
            for beam_option in beam.clone() {
                let timing_options = self.buffer_for_timing_top_k(beam_option.node(), allow_topology_rewrite, plan_limit);
                trace.push(self.vyg_ranked("rbtk", i, &timing_options, None));
                if anchor.option.is_some() && beam_option.option == anchor.option && !timing_options.is_empty() {
                    next_anchor = timing_options[0].clone();
                }
                next_options.extend(timing_options);
            }
            beam = self.limit_timing_options(next_options, &next_anchor, plan_limit);
            trace.push(self.vyg_ranked("rbtbeam", i, &beam, Some(&next_anchor)));
            anchor = next_anchor;
            if beam.is_empty() {
                let line = format!("[WARNING RSZ-2021] cannot find a viable buffering solution on pin {} after {} rounds of buffering (no solution meets design rules)", self.pin, i + 1);
                self.warn(line);
                return None;
            }
        }
        Some((beam, anchor))
    }

    /// `runAreaRecoveryBeam`: five rounds of `recoverAreaTopK` (alpha (1 + i) / 5) over the beam
    /// against the first option's target; the anchor recovered against its own target, strictly
    /// and alone; with more than one plan the beam's options stay candidates.
    pub fn run_area_recovery_beam(&mut self, plan_limit: usize, beam: &mut Vec<RankedOption>, anchor: &mut RankedOption, trace: &mut Vec<String>) {
        let target = self.area_recovery_target(beam[0].node());
        let anchor_target = match anchor.option {
            Some(a) => self.area_recovery_target(a),
            None => target,
        };
        trace.push(format!("VYGP|rbatgt|{}|target={}|anchor_target={}|anchor={}", self.pin, e9(target.to_secs()), e9(anchor_target.to_secs()), u8::from(anchor.option.is_some())));
        let mut i = 0;
        while i < 5 && !beam.is_empty() {
            let alpha = (1 + i) as f32 / 5.0;
            let mut next_options = if plan_limit > 1 { beam.clone() } else { Vec::new() };
            let mut next_anchor = RankedOption::default();
            for beam_option in beam.clone() {
                if anchor.option.is_some() && beam_option.option == anchor.option {
                    let anchor_area_options = self.recover_area_top_k(beam_option.node(), anchor_target, alpha, 1, true);
                    trace.push(self.vyg_ranked("rbaa", i, &anchor_area_options, None));
                    if let Some(f) = anchor_area_options.first() {
                        next_anchor = f.clone();
                    }
                    next_options.extend(anchor_area_options);
                }
                let area_options = self.recover_area_top_k(beam_option.node(), target, alpha, plan_limit, false);
                trace.push(self.vyg_ranked("rbak", i, &area_options, None));
                next_options.extend(area_options);
            }
            *beam = self.limit_area_options(next_options, target, &next_anchor, plan_limit);
            trace.push(self.vyg_ranked("rbabeam", i, beam, Some(&next_anchor)));
            *anchor = next_anchor;
            i += 1;
        }
    }

    /// The printer's ranked-option line (`VYGP|<tag>|drv|r<round>|n=|anchor=|options`), as the
    /// instrumented reference writes it.
    fn vyg_ranked(&self, tag: &str, round: usize, options: &[RankedOption], anchor: Option<&RankedOption>) -> String {
        let one = |o: &RankedOption| {
            let n = o.node();
            let nd = &self.nodes[n];
            format!("{}:u{}:i{}:m{}:p{}:c{}:a{}:b{}:{}", e9(o.slack.to_secs()), u8::from(o.unconstrained), o.index, u8::from(o.meets_target), u8::from(o.preserves_best_slack), e9(nd.cap), e9(nd.area), self.buffer_count(n), self.vyg_tree(n))
        };
        let list: Vec<String> = options.iter().map(one).collect();
        let an = match anchor {
            Some(a) if a.option.is_some() => {
                let at = options.iter().rposition(|o| o.option == a.option).map_or(-1, |k| k as i64);
                format!("{}@{at}", one(a))
            }
            _ => "-".into(),
        };
        format!("VYGP|{tag}|{}|r{round}|n={}|anchor={an}|{}", self.pin, options.len(), list.join(","))
    }

    /// The printer's tree: `L<pin>`, `W(x,y,layer)`, `V(x,y,layer,ref)`, `B(cell,x,y)`, `J(x,y)`,
    /// each followed by its children in brackets.
    pub fn vyg_tree(&self, n: usize) -> String {
        let nd = &self.nodes[n];
        match &nd.kind {
            Kind::Load { pin } => format!("L{pin}"),
            Kind::Wire { r } => format!("W({},{},{})[{}]", nd.x, nd.y, nd.layer, self.vyg_tree(*r)),
            Kind::Via { r, ref_layer } => format!("V({},{},{},{ref_layer})[{}]", nd.x, nd.y, nd.layer, self.vyg_tree(*r)),
            Kind::Buffer { cell, r } => format!("B({cell},{},{})[{}]", nd.x, nd.y, self.vyg_tree(*r)),
            Kind::Junction { r, r2 } => format!("J({},{})[{}][{}]", nd.x, nd.y, self.vyg_tree(*r), self.vyg_tree(*r2)),
        }
    }

    /// `buildLoadPaths`: per load, in tree order, the stages from the root — a wire its length and
    /// RC with the cap below it; a buffer its cell, the cap below plus its output pin's, and
    /// whether that is a distributed load (`drivesDistributedLoadBeforeNextBuffer`). A via is
    /// refused (`bufferForTiming`'s trees carry none).
    pub fn build_load_paths(&mut self, n: usize, prefix: &mut Vec<PathStage>, out: &mut Vec<LoadPath>) {
        match self.nodes[n].kind.clone() {
            Kind::Load { pin } => out.push(LoadPath { load_pin: pin, stages: prefix.clone() }),
            Kind::Wire { r } => {
                let (layer_res, layer_cap, _) = self.wire_rc_of(self.loc(n), r, self.nodes[n].layer);
                let len = {
                    let (a, b) = (self.loc(n), self.loc(r));
                    (a.0 - b.0).abs() + (a.1 - b.1).abs()
                };
                let length = self.ctx.dbu_to_meters(len) as f32;
                let rc = WireRcStage { length, res: (f64::from(length) * layer_res) as f32, cap: (f64::from(length) * layer_cap) as f32 };
                prefix.push(PathStage { kind: StageKind::Wire, buffer_cell: None, wire_rc: rc, downstream_cap: self.nodes[r].cap, drives_distributed_load: false });
                self.build_load_paths(r, prefix, out);
                prefix.pop();
            }
            Kind::Via { r, .. } => {
                self.refused.get_or_insert_with(|| "buildLoadPaths: a via in a planned tree (viaResistance is not modelled)".into());
                self.build_load_paths(r, prefix, out);
            }
            Kind::Buffer { cell, r } => {
                let c = self.ctx.cell(&cell);
                let output_cap = c.buffer_ports().map_or(0.0, |(_, out)| port_cap(c, &out.name));
                let distributed = !matches!(self.nodes[r].kind, Kind::Load { .. } | Kind::Buffer { .. });
                prefix.push(PathStage { kind: StageKind::Buffer, buffer_cell: Some(cell), wire_rc: WireRcStage::default(), downstream_cap: self.nodes[r].cap + output_cap, drives_distributed_load: distributed });
                self.build_load_paths(r, prefix, out);
                prefix.pop();
            }
            Kind::Junction { r, r2 } => {
                self.build_load_paths(r, prefix, out);
                self.build_load_paths(r2, prefix, out);
            }
        }
    }

    /// `makePlanFromTree`: the tree's load paths, its input cap, buffers and loads, and its driver
    /// slack less the original net's. The reference clones the tree (ownership only).
    pub fn make_plan_from_tree(&mut self, tree: usize, request: &PlanRequest) -> TreePlan {
        let mut load_paths = Vec::new();
        self.build_load_paths(tree, &mut Vec::new(), &mut load_paths);
        let final_slack = self.slack_at_driver_pin(tree);
        TreePlan {
            tree,
            target_load_pin: request.target_load_pin.clone(),
            load_paths,
            tree_input_cap: self.nodes[tree].cap,
            driver_slack_gain: (final_slack - request.original_slack).to_secs(),
            anchor_tree: false,
            buffer_count: self.buffer_count(tree),
            load_count: self.load_count(tree),
        }
    }

    /// `BufferedNet::loadCount`.
    pub fn load_count(&self, n: usize) -> i64 {
        match &self.nodes[n].kind {
            Kind::Load { .. } => 1,
            Kind::Wire { r } | Kind::Buffer { r, .. } | Kind::Via { r, .. } => self.load_count(*r),
            Kind::Junction { r, r2 } => self.load_count(*r) + self.load_count(*r2),
        }
    }

    /// `plansFromBeam`: one plan per beam option that reaches the target load, the anchor's
    /// marked.
    pub fn plans_from_beam(&mut self, beam: &[RankedOption], anchor: &RankedOption, request: &PlanRequest, trace: &mut Vec<String>) -> Vec<TreePlan> {
        let mut plans = Vec::with_capacity(beam.len());
        for o in beam {
            let mut plan = self.make_plan_from_tree(o.node(), request);
            plan.anchor_tree = anchor.option.is_some() && o.option == anchor.option;
            trace.push(self.vyg_plan(&plan));
            if plan.target_path().is_some() {
                plans.push(plan);
            }
        }
        plans
    }

    /// The printer's plan line (`VYGP|rbplan`).
    fn vyg_plan(&self, plan: &TreePlan) -> String {
        let mut paths = Vec::new();
        for lp in &plan.load_paths {
            let st: Vec<String> = lp
                .stages
                .iter()
                .map(|s| {
                    let t = match s.kind {
                        StageKind::Wire => "w",
                        StageKind::Buffer => "b",
                    };
                    format!("{t}:{}:{}:{}:{}:{}:{}", s.buffer_cell.as_deref().unwrap_or("-"), e9(s.wire_rc.length), e9(s.wire_rc.res), e9(s.wire_rc.cap), e9(s.downstream_cap), u8::from(s.drives_distributed_load))
                })
                .collect();
            paths.push(format!("{}{{{}}}", lp.load_pin, st.join(";")));
        }
        format!(
            "VYGP|rbplan|{}|anchor={}|bufs={}|loads={}|dsg={}|cap={}|target={}|tree={}|paths={}",
            self.pin,
            u8::from(plan.anchor_tree),
            plan.buffer_count,
            plan.load_count,
            e9(plan.driver_slack_gain),
            e9(plan.tree_input_cap),
            u8::from(plan.target_path().is_some()),
            self.vyg_tree(plan.tree),
            paths.join(",")
        )
    }

    /// `makeRebufferTreePlans` after `setPin` and `annotateLoadSlacks` (the probe): the timing
    /// beam, the area-recovery beam, the plans. Empty when the timing beam finds nothing; an empty
    /// area beam is RSZ-2022.
    pub fn make_rebuffer_tree_plans(&mut self, target_load_pin: Option<&str>, max_plans: usize, trace: &mut Vec<String>) -> Vec<TreePlan> {
        let plan_limit = max_plans.max(1);
        let root = self.probe.root;
        let request = PlanRequest { target_load_pin: target_load_pin.map(str::to_string), original_slack: self.slack_at_driver_pin(root) };
        trace.push(format!("VYGP|rbreq|{}|tgt={}|limit={plan_limit}|orig={}|tree={}", self.pin, target_load_pin.unwrap_or("-"), e9(request.original_slack.to_secs()), self.vyg_tree(root)));
        // `allow_topology_rewrite`: placement parasitics only.
        let allow_topology_rewrite = self.ctx.layers.is_none();
        let Some((mut beam, mut anchor)) = self.run_timing_beam(root, allow_topology_rewrite, plan_limit, trace) else { return Vec::new() };
        self.run_area_recovery_beam(plan_limit, &mut beam, &mut anchor, trace);
        if beam.is_empty() {
            self.failed.get_or_insert_with(|| "RSZ-2022: failed area recovery".into());
            return Vec::new();
        }
        self.plans_from_beam(&beam, &anchor, &request, trace)
    }

    /// `annotateArrivalDelays`: the driver's arrival correction spread down the tree as each
    /// node's arrival delay; the target area recovery works against (-INF, and no correction,
    /// for an unconstrained tree).
    fn annotate_arrival_delays(&mut self, root: usize, mut slack_target: FixedDelay) -> FixedDelay {
        let (_, mut correction, _) = self.drvr_pin_timing(root);
        if self.nodes[root].slack_rf.is_none() {
            correction = 0.0;
            slack_target = -FixedDelay::inf();
        }
        self.spread_arrival(root, -FixedDelay::from_secs(correction));
        slack_target
    }

    /// `areaRecoveryOptions`: the area options at the root (the visitor below); empty when a
    /// non-strict node has no assured option.
    fn area_recovery_options(&mut self, root: usize, slack_target: FixedDelay, alpha: f32, strict_assured: bool) -> Vec<usize> {
        self.area_options(root, 1, 0, slack_target, alpha, strict_assured)
    }

    fn spread_arrival(&mut self, n: usize, arrival: FixedDelay) {
        self.nodes[n].arrival = arrival;
        match self.nodes[n].kind.clone() {
            Kind::Wire { r } | Kind::Buffer { r, .. } | Kind::Via { r, .. } => {
                let d = self.nodes[n].delay;
                self.spread_arrival(r, arrival + d);
            }
            Kind::Junction { r, r2 } => {
                self.spread_arrival(r, arrival);
                self.spread_arrival(r2, arrival);
            }
            Kind::Load { .. } => {}
        }
    }

    fn area_options(&mut self, node: usize, level: usize, upstream_wl: i32, slack_target: FixedDelay, alpha: f32, strict_assured: bool) -> Vec<usize> {
        match self.nodes[node].kind.clone() {
            // `recoverArea` runs on `bufferForTiming`'s trees, which carry no via (the reference
            // aborts on one).
            Kind::Via { .. } => {
                self.failed.get_or_insert_with(|| "recoverArea: a via in a buffered tree (unhandled BufferedNet type)".into());
                Vec::new()
            }
            Kind::Buffer { .. } | Kind::Wire { .. } => {
                let inner = match self.nodes[node].kind { Kind::Buffer { r, .. } => r, _ => node };
                let mut opts = match self.nodes[inner].kind.clone() {
                    Kind::Wire { r } => {
                        let len = {
                            let (a, b) = (self.loc(inner), self.loc(r));
                            (a.0 - b.0).abs() + (a.1 - b.1).abs()
                        };
                        let o = self.area_options(r, level + 1, len, slack_target, alpha, strict_assured);
                        let (at, layer) = (self.loc(inner), self.nodes[inner].layer);
                        o.into_iter().map(|opt| self.add_wire(opt, at, layer, Some(level))).collect()
                    }
                    _ => self.area_options(inner, level + 1, 0, slack_target, alpha, strict_assured),
                };
                let threshold = FixedDelay::lerp(self.nodes[node].slack, slack_target + self.nodes[node].arrival, alpha);
                if !self.insert_buffer_options(&mut opts, level, upstream_wl, true, threshold, Some(node), strict_assured) {
                    return Vec::new();
                }
                opts
            }
            Kind::Junction { r, r2 } => {
                let left = self.area_options(r, level + 1, upstream_wl, slack_target, alpha, strict_assured);
                let right = self.area_options(r2, level + 1, upstream_wl, slack_target, alpha, strict_assured);
                let threshold = FixedDelay::lerp(self.nodes[node].slack, slack_target + self.nodes[node].arrival, alpha);
                let envelope = self.metrics(node, threshold);
                let mut assured_fallback = None;
                let mut opts = Vec::with_capacity(left.len() * right.len());
                let at = self.loc(node);
                for &l in &left {
                    for &rr in &right {
                        let j = self.create_junction(l, rr, at);
                        if assured_fallback.is_none() && self.fits_envelope(j, &envelope) {
                            assured_fallback = Some(j);
                        }
                        if self.nodes[j].fanout <= self.probe.fanout_limit {
                            opts.push(j);
                        }
                    }
                }
                self.prune_cap_vs_area(&mut opts);
                let line = format!("{:level$}junction: {} options", "", opts.len());
                self.debug(3, line);
                for &o in &opts {
                    let line = format!("{:level$} - {}", "", self.to_string(o));
                    self.debug(3, line);
                }
                let assured_found = opts.iter().any(|&o| self.fits_envelope(o, &envelope));
                if !assured_found && assured_fallback.is_none() {
                    if !strict_assured {
                        return Vec::new();
                    }
                    self.error_area_recovery_unreproducible();
                }
                if !assured_found {
                    if let Some(f) = assured_fallback {
                        self.insert_assured(&mut opts, f, level);
                    }
                }
                opts
            }
            Kind::Load { .. } => {
                let line = format!("{:level$}{}", "", self.to_string(node));
                self.debug(3, line);
                vec![node]
            }
        }
    }

    /// `pruneCapVsAreaOptions`: by (area, cap) ascending, keep each option whose cap is fuzzily
    /// below every kept one's, then reverse.
    /// `pruneCapVsAreaOptions`. Its sort is `std::ranges::sort` — not stable: options equal in
    /// (area, cap) keep the order libc++'s introsort leaves them in, and the first survives.
    // `std::tuple`'s `<` is `a0 < b0 || (!(b0 < a0) && a1 < b1)`: the negation is the reference's
    // and differs from `>=` on a NaN — kept as written.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    fn prune_cap_vs_area(&mut self, opts: &mut Vec<usize>) {
        let nodes = &self.nodes;
        // `std::tuple` `<`: area, then cap.
        let sorted = crate::order::libcxx_sort_by(opts, |&a, &b| {
            let (x, y) = (&nodes[a], &nodes[b]);
            x.area < y.area || (!(y.area < x.area) && x.cap < y.cap)
        });
        if let Err(h) = sorted {
            self.refused.get_or_insert(format!("pruneCapVsAreaOptions: {} options reach the sort's heap fallback, which is not modelled", h.len));
            return;
        }
        if opts.is_empty() {
            return;
        }
        let mut lowest = self.nodes[opts[0]].cap;
        let mut si = 1;
        for pi in 1..opts.len() {
            let c = self.nodes[opts[pi]].cap;
            if fuzzy::less(c, lowest) {
                opts[si] = opts[pi];
                si += 1;
                lowest = c;
            }
        }
        opts.truncate(si);
        opts.reverse();
    }

    /// `criticalPathDelay`: the worst load slack less the root's.
    fn critical_path_delay(&self, root: usize) -> FixedDelay {
        let mut worst = FixedDelay::inf();
        let mut stack = vec![root];
        while let Some(n) = stack.pop() {
            match &self.nodes[n].kind {
                Kind::Wire { r } | Kind::Buffer { r, .. } | Kind::Via { r, .. } => stack.push(*r),
                Kind::Junction { r, r2 } => {
                    stack.push(*r2);
                    stack.push(*r);
                }
                Kind::Load { .. } => worst = worst.min(self.nodes[n].slack),
            }
        }
        worst - self.nodes[root].slack
    }

    /// `rebufferPin` from the annotated net to the chosen tree: three timing passes, the target,
    /// five area passes. `Err` with the reference's message where it stops.
    pub fn rebuffer_pin(&mut self) -> Result<Option<usize>, String> {
        let mut bnet = Some(self.probe.root);
        for i in 0..3 {
            // `allow_topology_rewrite`: placement parasitics only.
            bnet = self.buffer_for_timing(bnet.expect("a tree"), self.ctx.layers.is_none());
            if bnet.is_none() {
                let line = format!("[WARNING RSZ-2021] cannot find a viable buffering solution on pin {} after {} rounds of buffering (no solution meets design rules)", self.pin, i + 1);
                self.warn(line);
                break;
            }
        }
        let Some(b) = bnet else { return Ok(None) };
        let (gd, _, _) = self.drvr_pin_timing(b);
        let relaxation = (gd.max(0.0) + self.critical_path_delay(b).to_secs()) * RELAXATION_FACTOR;
        let target = self.slack_at_driver_pin(b) - FixedDelay::from_secs(relaxation);
        let mut bnet = Some(b);
        for i in 0..5 {
            let Some(b) = bnet else { break };
            bnet = self.recover_area(b, target, (1 + i) as f32 / 5.0);
        }
        if let Some(e) = &self.failed {
            return Err(e.clone());
        }
        bnet.map(Some).ok_or_else(|| "RSZ-2022: failed area recovery".into())
    }

    /// `Rebuffer::bufferDelay(cell, rf, load)` for any buffer cell (an imported tree's buffers
    /// need not be characterized sizes): over the transitions, the largest gate delay at its
    /// arrival path's scene.
    fn buffer_delay_cell(&self, cell: &str, rf: Option<Rfs>, load: f32) -> FixedDelay {
        let mut delay = FixedDelay::ZERO;
        if let Some(rf) = rf {
            for &k in rf.range() {
                let c = self.ctx.scene_cell(self.probe.arc_scenes[k], cell);
                let Some((_, out)) = c.buffer_ports() else { continue };
                let (d, _) = gate_delays(c, &out.name, load, self.ctx.tgt_slews);
                delay = delay.max(FixedDelay::from_secs(d[k]));
            }
        }
        delay
    }

    /// `annotateTiming`: bottom-up over an imported tree — a junction the combined transition, the
    /// worse slack, the summed cap, the tighter slew limit; a wire its `R·(C/2 + Cload)` delay off
    /// the slack and its slew degradation off the limit (nothing for no length); a via its
    /// resistance times the load; a buffer its gate delay at the load. Loads keep their slacks.
    pub fn annotate_timing(&mut self, n: usize) {
        match self.nodes[n].kind.clone() {
            Kind::Load { .. } => {}
            Kind::Junction { r: p, r2: q } => {
                self.annotate_timing(p);
                self.annotate_timing(q);
                let (a, b) = (&self.nodes[p], &self.nodes[q]);
                let (rf, slack, cap, mls) = (combined(a.slack_rf, b.slack_rf), a.slack.min(b.slack), a.cap + b.cap, a.max_load_slew.min(b.max_load_slew));
                let nd = &mut self.nodes[n];
                nd.slack_rf = rf;
                nd.slack = slack;
                nd.cap = cap;
                nd.max_load_slew = mls;
            }
            Kind::Wire { r: p } => {
                self.annotate_timing(p);
                let (layer_res, layer_cap, len) = self.wire_rc_of(self.loc(n), p, self.nodes[n].layer);
                let wire_length = self.ctx.dbu_to_meters(len);
                let (mut wire_res, mut wire_cap) = (wire_length * layer_res, wire_length * layer_cap);
                let pc = f64::from(self.nodes[p].cap);
                let mut wire_delay = FixedDelay::from_secs((wire_res * (wire_cap / 2.0 + pc)) as f32);
                if len == 0 {
                    wire_res = 0.0;
                    wire_cap = 0.0;
                    wire_delay = FixedDelay::ZERO;
                }
                let (ps, prf, pml) = (self.nodes[p].slack, self.nodes[p].slack_rf, self.nodes[p].max_load_slew);
                let nd = &mut self.nodes[n];
                nd.delay = wire_delay;
                nd.slack = ps - wire_delay;
                nd.slack_rf = prf;
                nd.max_load_slew = (f64::from(pml) - wire_res * (wire_cap / 2.0 + pc) * f64::from(self.ctx.slew_shape_factor)) as f32;
            }
            Kind::Via { r: p, .. } => {
                self.annotate_timing(p);
                // `viaResistance`: placement parasitics carry no vias
                let (ps, prf) = (self.nodes[p].slack, self.nodes[p].slack_rf);
                let nd = &mut self.nodes[n];
                nd.delay = FixedDelay::ZERO;
                nd.slack = ps;
                nd.slack_rf = prf;
            }
            Kind::Buffer { cell, r: p } => {
                self.annotate_timing(p);
                let (prf, pcap, ps) = (self.nodes[p].slack_rf, self.nodes[p].cap, self.nodes[p].slack);
                let d = self.buffer_delay_cell(&cell, prf, pcap);
                let nd = &mut self.nodes[n];
                nd.delay = d;
                nd.slack = ps - d;
                nd.slack_rf = prf;
            }
        }
    }

    /// `stripTreeBuffers`: the tree rebuilt without its buffers — each wire and junction made
    /// again (the constructors' cap, fanout, slew limit and area), a load kept as it is.
    pub fn strip_tree_buffers(&mut self, n: usize) -> usize {
        match self.nodes[n].kind.clone() {
            Kind::Load { .. } => n,
            Kind::Buffer { r, .. } => self.strip_tree_buffers(r),
            Kind::Wire { r } => {
                let inner = self.strip_tree_buffers(r);
                self.new_wire(self.loc(n), inner, self.nodes[n].layer)
            }
            Kind::Via { r, ref_layer } => {
                let inner = self.strip_tree_buffers(r);
                let p = self.nodes[inner].clone();
                let node = Node { kind: Kind::Via { r: inner, ref_layer }, x: self.nodes[n].x, y: self.nodes[n].y, layer: self.nodes[n].layer, cap: p.cap, fanout: p.fanout, max_load_slew: p.max_load_slew, area: p.area, slack_rf: None, slack: FixedDelay::ZERO, delay: FixedDelay::ZERO, arrival: FixedDelay::ZERO };
                self.push(node)
            }
            Kind::Junction { r, r2 } => {
                let (a, b) = (self.strip_tree_buffers(r), self.strip_tree_buffers(r2));
                self.new_junction(self.loc(n), a, b)
            }
        }
    }

    /// The junction constructor (`BufferedNet(junction, …)`): the summed cap, fanout and area, the
    /// tighter slew limit; no timing.
    fn new_junction(&mut self, at: (i32, i32), p: usize, q: usize) -> usize {
        let (a, b) = (&self.nodes[p], &self.nodes[q]);
        let node = Node { kind: Kind::Junction { r: p, r2: q }, x: at.0, y: at.1, layer: crate::buffered_net::NULL_LAYER, cap: a.cap + b.cap, fanout: a.fanout + b.fanout, max_load_slew: a.max_load_slew.min(b.max_load_slew), area: a.area + b.area, slack_rf: None, slack: FixedDelay::ZERO, delay: FixedDelay::ZERO, arrival: FixedDelay::ZERO };
        self.push(node)
    }

    /// The sinks `resteiner` re-routes: each load, and each buffer kept (with its own subtree
    /// re-steinered) — in visit order.
    fn resteiner_sinks(&mut self, n: usize, steiner: SteinerOverPoints<'_>, sinks: &mut Vec<usize>) {
        match self.nodes[n].kind.clone() {
            Kind::Wire { r } | Kind::Via { r, .. } => self.resteiner_sinks(r, steiner, sinks),
            Kind::Junction { r, r2 } => {
                self.resteiner_sinks(r, steiner, sinks);
                self.resteiner_sinks(r2, steiner, sinks);
            }
            Kind::Load { .. } => sinks.push(n),
            Kind::Buffer { cell, r } => {
                let inner = self.resteiner(r, steiner);
                let cell_name = cell.clone();
                let mut b = self.nodes[n].clone();
                b.kind = Kind::Buffer { cell, r: inner.unwrap_or(r) };
                b.area = self.nodes[inner.unwrap_or(r)].area + self.ctx.cell(&cell_name).area;
                b.slack_rf = None;
                b.slack = FixedDelay::ZERO;
                b.delay = FixedDelay::ZERO;
                let k = self.push(b);
                sinks.push(k);
            }
        }
    }

    /// `resteiner`: a Steiner tree from the tree's root over its sinks' locations
    /// (`makeBufferedNetSteinerOverBnets`), the sinks stitched in at their points.
    pub fn resteiner(&mut self, n: usize, steiner: SteinerOverPoints<'_>) -> Option<usize> {
        let mut sinks = Vec::new();
        self.resteiner_sinks(n, steiner, &mut sinks);
        let root = self.loc(n);
        let points: Vec<(i32, i32)> = sinks.iter().map(|&k| self.loc(k)).collect();
        let mut sink_map: std::collections::BTreeMap<(i32, i32), Vec<usize>> = std::collections::BTreeMap::new();
        for &k in &sinks {
            sink_map.entry(self.loc(k)).or_default().push(k);
        }
        let tree = steiner(root, &points)?;
        let drvr_pt = tree.drvr_pt?;
        let branch_count = tree.branch.len();
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); branch_count];
        for i in 0..branch_count {
            let j = tree.branch[i].2;
            if j != i {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
        let mut visited = std::collections::BTreeSet::new();
        self.stitch_tree2(&tree, None, drvr_pt, &adj, &mut visited, &sink_map)
    }

    /// `makeBufferedNetFromTree2`: at a point the sinks there (the first visit only), joined in
    /// order; each other adjacent point's subtree joined after; a wire up to the parent point
    /// unless it is the same location.
    fn stitch_tree2(&mut self, tree: &crate::buffered_net::Tree, from: Option<usize>, to: usize, adj: &[Vec<usize>], visited: &mut std::collections::BTreeSet<(i32, i32)>, sink_map: &std::collections::BTreeMap<(i32, i32), Vec<usize>>) -> Option<usize> {
        let at = |k: usize| (tree.branch[k].0, tree.branch[k].1);
        let to_loc = at(to);
        let mut bnet: Option<usize> = None;
        if let Some(list) = sink_map.get(&to_loc) {
            if visited.insert(to_loc) {
                for &sink in list {
                    bnet = Some(match bnet {
                        Some(b) => self.new_junction(to_loc, b, sink),
                        None => sink,
                    });
                }
            }
        }
        for &k in &adj[to] {
            if Some(k) != from {
                if let Some(b1) = self.stitch_tree2(tree, Some(to), k, adj, visited, sink_map) {
                    bnet = Some(match bnet {
                        Some(b) => self.new_junction(at(to), b, b1),
                        None => b1,
                    });
                }
            }
        }
        match (bnet, from) {
            (Some(b), Some(f)) if at(to) != at(f) => Some(self.new_wire(at(f), b, crate::buffered_net::NULL_LAYER)),
            _ => bnet,
        }
    }

    /// `fullyRebuffer`'s passes on one driver's imported tree (`self.probe.root`, its loads'
    /// slacks annotated): timing annotated; the tree's slack error against the timer's
    /// (`sta_slack`); buffers stripped and the tree re-steinered; three timing passes (topology
    /// rewrite allowed; RSZ-2005 when none fits); the area target — a quarter of the projected
    /// slack less the model's optimism, plus 1% of the critical delay — and five area passes.
    /// `Ok(None)` when no timing solution was found; `Err` where the reference stops (RSZ-2004).
    pub fn fully_rebuffer_tree(&mut self, sta_slack: f32, steiner: SteinerOverPoints<'_>) -> Result<Option<(usize, usize)>, String> {
        let original = self.probe.root;
        self.annotate_timing(original);
        let slack = self.slack_at_driver_pin(original).to_secs();
        let error = slack - sta_slack;
        let stripped = self.strip_tree_buffers(original);
        let Some(unbuffered) = self.resteiner(stripped, steiner) else { return Err("resteiner: no Steiner tree".into()) };
        let mut timing = Some(unbuffered);
        for i in 0..3 {
            timing = self.buffer_for_timing(timing.expect("a tree"), true);
            if timing.is_none() {
                let line = format!("[WARNING RSZ-2005] cannot find a viable buffering solution on pin {} after {} rounds of buffering (no solution meets design rules)", self.pin, i + 1);
                self.warn(line);
                break;
            }
        }
        let Some(t) = timing else { return Ok(None) };
        let (gate_delay, _, _) = self.drvr_pin_timing(t);
        let mut target = -FixedDelay::inf();
        if self.nodes[t].slack_rf.is_some() {
            let relaxation = std_max(0.0, self.slack_at_driver_pin(t).to_secs() - std_min(error, 0.0)) / 4.0 + (std_max(gate_delay, 0.0) + self.critical_path_delay(t).to_secs()) * RELAXATION_FACTOR;
            target = self.slack_at_driver_pin(t) - FixedDelay::from_secs(relaxation);
        }
        let mut area = Some(t);
        for i in 0..5 {
            let Some(a) = area else { break };
            area = self.recover_area(a, target, (1 + i) as f32 / 5.0);
        }
        if let Some(e) = &self.failed {
            return Err(e.clone());
        }
        match area {
            Some(a) => Ok(Some((a, unbuffered))),
            None => Err("RSZ-2004: failed area recovery".into()),
        }
    }

    /// The export order (`exportBufferTree`): each buffer, bottom-up, with the loads it drives —
    /// load pins and the inputs of the buffers below it (by name; the caller maps a buffer to the
    /// instance it inserted).
    pub fn export_order(&self, root: usize, out: &mut Vec<(usize, Vec<Load>)>) -> Vec<Load> {
        match &self.nodes[root].kind {
            Kind::Wire { r } | Kind::Via { r, .. } => self.export_order(*r, out),
            Kind::Junction { r, r2 } => {
                let mut l = self.export_order(*r, out);
                l.extend(self.export_order(*r2, out));
                l
            }
            Kind::Load { pin } => vec![Load::Pin(pin.clone())],
            Kind::Buffer { r, .. } => {
                let child = self.export_order(*r, out);
                if child.is_empty() {
                    return Vec::new();
                }
                out.push((root, child));
                vec![Load::Buffer(root)]
            }
        }
    }
}

/// A load in the export: a pin of the design, or the input of a buffer inserted below.
#[derive(Debug, Clone, PartialEq)]
pub enum Load {
    Pin(String),
    Buffer(usize),
}

/// `BufferedNet::Metrics`.
#[derive(Debug, Clone, Copy)]
struct Envelope {
    slack: FixedDelay,
    cap: f32,
    max_load_slew: f32,
    fanout: f32,
}

/// `middleValue`: the median of three.
fn middle_value(mut a: i32, mut b: i32, c: i32) -> i32 {
    if b < a {
        std::mem::swap(&mut a, &mut b);
    }
    if c > b {
        b
    } else if c > a {
        c
    } else {
        a
    }
}

/// The dont_touch loads the export skips are named by the caller; this keeps the set type in one
/// place.
pub type DontTouch = BTreeSet<String>;

/// `BufferedNet::to_string` for a load: its pin by `sdcNetwork()->pathName` — the SDC spelling,
/// a lone escape dropped (`staToSdc`), where the other lines use the db network's name.
fn load_text(pin: &str, x: &str, y: &str, cap: &str, slack: &str, sl: &str) -> String {
    format!("load {} ({x}, {y}) cap {cap} slack {slack} load sl {sl}", crate::repair_timing::sta_to_sdc(pin))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule (BufferedNet::to_string, load case): the pin prints through the SDC network, so an
    // escaped bus bracket prints bare.
    #[test]
    fn a_load_prints_its_sdc_name() {
        assert_eq!(load_text(r"dpath.a_reg.out\[7\]$_DFFE_PP_/D", "1", "2", "0.693", "-2.990", "320.000"), "load dpath.a_reg.out[7]$_DFFE_PP_/D (1, 2) cap 0.693 slack -2.990 load sl 320.000");
    }

    // Rule (FixedDelay): float seconds × 1e15 in double, truncated; back as a float over 1e15.
    // 1.5e-12 is just below 1.5 ps as a float, so it truncates to 1499 fs.
    #[test]
    fn fixed_delay_is_truncated_femtoseconds() {
        assert_eq!(FixedDelay::from_secs(1.5e-12).0, 1499);
        assert_eq!(FixedDelay::from_secs(-2.5e-15).0, -2);
        assert_eq!(FixedDelay::inf().to_secs(), 100.0);
        assert_eq!(FixedDelay::lerp(FixedDelay(0), FixedDelay(10), 0.25).0, 2);
        assert_eq!(FixedDelay::lerp(FixedDelay(0), FixedDelay(10), 1.0).0, 10);
    }

    // Rule (middleValue): the median of three.
    #[test]
    fn middle_value_is_the_median() {
        assert_eq!(middle_value(1, 5, 3), 3);
        assert_eq!(middle_value(5, 1, 9), 5);
        assert_eq!(middle_value(5, 1, 0), 1);
    }

    // Rule (combinedTransition): equal stays, one missing takes the other, else both.
    #[test]
    fn transitions_combine() {
        assert_eq!(combined(Some(Rfs::Rise), Some(Rfs::Rise)), Some(Rfs::Rise));
        assert_eq!(combined(None, Some(Rfs::Fall)), Some(Rfs::Fall));
        assert_eq!(combined(Some(Rfs::Rise), Some(Rfs::Fall)), Some(Rfs::Both));
    }
}
