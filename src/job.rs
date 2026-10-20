// SPDX-License-Identifier: Apache-2.0
//! The resizer's job runner: the steps of a session (libraries, the design, constraints, RC,
//! estimates) and its commands (`repair_design`, `buffer_ports`, `repair_timing`,
//! `fully_rebuffer`, …) run in order over a database, the JSON report built from them. The
//! `vyges-rsz` command reads a job file and calls [`run`]; an engine that repairs its own design
//! mid-command calls [`run_owned`].
#![allow(clippy::too_many_lines)]

use std::collections::{BTreeMap, BTreeSet, HashMap};

use serde_json::{json, Value};
use vyges_opendb::Db;
use crate::buffer_ports;
use crate::preamble::Libs;
use crate::repair_design::{self, Args, Inputs};
use crate::trace::Trace;
use crate::env::{estimate_state, master_pins, masters, no_signal_cap, port_caps_at, sdc_env, search_sdc_on, sim_constants};
use crate::Stop;
use crate::odb_design::{clamp_loc_to_core, gr_layer_rc, grt_flutes, grt_stt, net_info, sdc_cap_pins, sdc_port_cap_limits, sizing_limits, CliDesign, GrLive, TimerLog};
use crate::timing::{user_time_to_sta, Limits};
use vyges_sta::graph::NetParasitics;
use vyges_sta::liberty::Library;



/// The estimator's `set_layer_rc` table as the global router's parasitics read it: per routing
/// level (ohm/m, F/m), and per cut layer name its resistance — only the entries the table has.
pub fn grt_layer_rc(db: &Db, rc: &vyges_est::rc::Rc) -> (BTreeMap<i32, (f64, f64)>, BTreeMap<String, f64>) {
    let (mut layers, mut vias) = (BTreeMap::new(), BTreeMap::new());
    for name in db.tech_get_layers() {
        let number = db.layer_get_number(&name);
        if !rc.layer_res.contains_key(&number) && !rc.layer_cap.contains_key(&number) {
            continue;
        }
        let (res, cap) = rc.layer_rc(number, 0);
        let level = db.layer_get_routing_level(&name);
        if level > 0 {
            layers.insert(level, (res, cap));
        } else {
            vias.insert(name, res);
        }
    }
    (layers, vias)
}







/// `dbPlacementStatus::isPlaced`.
pub fn is_placed(status: &str) -> bool {
    matches!(status, "PLACED" | "LOCKED" | "FIRM" | "COVER" | "FIXED")
}

/// `Resizer::metersToDbu(separation)` as `repairTieFanout` calls it: `-separation` is microns
/// (`distance_ui_sta` makes meters), then `lround(m · dbu_ · 1e6)` masked to a non-negative int.
///
/// ⛔ The call comes BEFORE `initDesignArea()`, and `dbu_` is 0 until some resizer command has run
/// `initBlock` — so as the session's first resizer command the separation is 0 and ties land on
/// their loads' pins (upstream's repair_tie2 golden shows exactly that). After `repair_design`, as
/// LibreLane runs it, the database's units apply.
pub fn separation_dbu(separation_um: f64, dbu: f64) -> i32 {
    ((separation_um * 1e-6 * dbu * 1e6).round() as i64 & i64::from(i32::MAX)) as i32
}

/// `Resizer::tieLocation` for an instance pin at `(x, y)` on an instance with bounding box `b`
/// (`[x_min, y_min, x_max, y_max]`): moved `separation` out through the side STRICTLY nearest the
/// pin; a pin equally near two sides is not moved. A port's location is used as is.
pub fn tie_location((x, y): (i32, i32), b: [i32; 4], separation: i32) -> (i32, i32) {
    let (l, r, bo, t) = ((x - b[0]).abs(), (x - b[2]).abs(), (y - b[1]).abs(), (y - b[3]).abs());
    let (mut tx, mut ty) = (x, y);
    if l < r && l < bo && l < t {
        tx -= separation;
    }
    if r < l && r < bo && r < t {
        tx += separation;
    }
    if bo < l && bo < r && bo < t {
        ty -= separation;
    }
    if t < l && t < r && t < bo {
        ty += separation;
    }
    (tx, ty)
}

/// `repair_tie_fanout lib_port [-separation dist] [-max_fanout n] [-verbose]` —
/// `Resizer::repairTieFanout`: every load of a tie cell's output gets a tie cell of its own.
///
/// The call sequence, as the reference makes it:
///  1. the tie instances (`findCellInstances`): every instance of the port's cell, sorted by name;
///  2. per tie, skipped when it is dont_touch, its pin has no net, or the net is dont_touch;
///  3. its loads: every other pin on the net, sorted by path name — a dont_touch load stays on the
///     original tie (`keep_tie`) and is not given one;
///  4. per load (`createNewTieCellForLoadPin`): a tie named after the load's instance (the block's
///     name for a port), uniquified `IF_NEEDED_WITH_UNDERSCORE`, source TIMING, placed at
///     `tieLocation` clamped to the core, UNPLACED when the load is not placed; an instance pin is
///     disconnected and hierarchically connected (connection "net"), a port is moved onto a new net
///     "net" (uniquified `IF_NEEDED`) with the tie's output;
///  5. unless kept (`deleteTieCellAndNet`): the original net destroyed, then the tie instance too
///     when none of its other pins is on a signal net (a cell with both HI and LO keeps the other);
///  6. `RSZ-0042` when any tie was inserted.
///
/// `-max_fanout` and `-verbose` are parsed and unused, as upstream's are. `block_inited`: whether an
/// earlier resizer command in the session ran `initBlock` (see [`separation_dbu`]).
pub fn repair_tie_fanout(db: &mut Db, libs: &Libs, args: &[String], block_inited: bool) -> Result<Value, String> {
    let e = |x: vyges_opendb::Error| x.to_string();
    let mut separation = 0.0f64;
    let mut positional = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "-separation" | "-max_fanout" => {
                let v = args.get(i + 1).ok_or_else(|| format!("repair_tie_fanout {} needs a value", args[i]))?;
                if args[i] == "-separation" {
                    // `check_positive_float`: a double, and >= 0 (STA-0572 otherwise).
                    separation = v.parse::<f64>().ok().filter(|x| *x >= 0.0).ok_or_else(|| format!("STA-0572: -separation '{v}' is not a positive float."))?;
                }
                i += 2;
            }
            "-verbose" => i += 1,
            f if f.starts_with('-') => return Err(format!("repair_tie_fanout {f}: not modelled")),
            _ => {
                positional.push(args[i].clone());
                i += 1;
            }
        }
    }
    let [lib_port] = positional.as_slice() else { return Err("repair_tie_fanout takes one lib_port".into()) };
    let (cell, port) = lib_port.rsplit_once('/').ok_or_else(|| format!("repair_tie_fanout {lib_port}: not a cell/port (not modelled)"))?;
    if [cell, port].iter().any(|x| x.contains(['*', '?'])) {
        return Err(format!("repair_tie_fanout {lib_port}: a pattern (not modelled)"));
    }
    if !libs.link_cell(cell).is_some_and(|c| c.port(port).is_some()) {
        return Err(format!("repair_tie_fanout {lib_port}: no such liberty pin (not modelled)"));
    }
    if !db.block_get_mod_insts().is_empty() {
        return Err("repair_tie_fanout on a hierarchical design: not modelled".into());
    }
    let separation_dbu = separation_dbu(separation, if block_inited { f64::from(db.tech_get_db_units_per_micron()) } else { 0.0 });
    let core = (db.block_get_core_area_x_min(), db.block_get_core_area_y_min(), db.block_get_core_area_x_max(), db.block_get_core_area_y_max());
    let core = (core != (0, 0, 0, 0)).then_some(core);
    let size = (db.master_get_width(cell) as i32, db.master_get_height(cell) as i32);
    let block = db.block_name();

    let mut insts: Vec<String> = db.block_get_insts().into_iter().filter(|i| db.inst_master(i) == cell).collect();
    insts.sort();
    let (mut tie_count, mut ties_checked) = (0usize, 0usize);
    for tie in &insts {
        if db.inst_is_do_not_touch(tie) {
            continue;
        }
        let net = db.net_of(tie, port);
        if net.is_empty() || db.net_is_do_not_touch(&net) {
            continue;
        }
        ties_checked += 1;
        let mut keep_tie = false;
        let mut loads: Vec<(String, Option<(String, String)>)> = Vec::new();
        for it in db.net_iterms(&net) {
            let (inst, pin) = it.rsplit_once('/').ok_or("an instance pin")?;
            if db.inst_is_do_not_touch(inst) {
                keep_tie = true;
                continue;
            }
            if inst == tie && pin == port {
                continue;
            }
            loads.push((it.clone(), Some((inst.to_string(), pin.to_string()))));
        }
        for bt in db.net_bterms(&net) {
            loads.push((bt, None));
        }
        loads.sort_by(|a, b| a.0.cmp(&b.0));
        for (path, iterm) in &loads {
            // `tieLocation`: the load pin's location, moved `separation` out of the nearest side
            // of its instance (strictly nearest: a tie between two sides is not moved).
            let (base, (lx, ly), placed) = match iterm {
                Some((inst, pin)) => {
                    let loc = db.iterm_avg_xy(inst, pin).unwrap_or((db.inst_get_origin_x(inst), db.inst_get_origin_y(inst)));
                    (inst.clone(), loc, is_placed(&db.inst_get_placement_status(inst)))
                }
                None => (block.clone(), db.bterm_first_pin_location(path).unwrap_or((0, 0)), is_placed(&db.bterm_get_first_pin_placement_status(path))),
            };
            let (tx, ty) = match iterm {
                Some((inst, _)) => {
                    let b = db.inst_bbox(inst).map_err(e)?;
                    tie_location((lx, ly), [b[0], b[1], b[2], b[3]], separation_dbu)
                }
                None => (lx, ly),
            };
            // `makeInstance`: named, TIMING, `setLocation` (clamped to the core, PLACED).
            let name = db.make_new_inst_name(&base, "IF_NEEDED_WITH_UNDERSCORE").map_err(e)?;
            db.create_inst(cell, &name).map_err(e)?;
            db.inst_set_source_type(&name, "TIMING").map_err(e)?;
            let at = clamp_loc_to_core(core, size, (tx, ty));
            db.set_inst_location(&name, at.0, at.1).map_err(e)?;
            if !placed {
                db.inst_set_placement_status(&name, "UNPLACED").map_err(e)?;
            }
            match iterm {
                Some((inst, pin)) => {
                    db.disconnect(inst, pin).map_err(e)?;
                    db.hierarchical_connect(&name, port, inst, pin, "net").map_err(e)?;
                }
                None => {
                    let n = db.make_new_net_name("net", "IF_NEEDED").map_err(e)?;
                    db.create_net(&n).map_err(e)?;
                    db.connect(&name, port, &n).map_err(e)?;
                    db.bterm_connect(path, &n).map_err(e)?;
                }
            }
            tie_count += 1;
        }
        if !keep_tie {
            db.destroy_net(&net).map_err(e)?;
            let other = db.master_mterms(cell).map_err(e)?.into_iter().any(|(t, _)| {
                let n = db.net_of(tie, &t);
                t != port && !n.is_empty() && !matches!(db.net_get_sig_type(&n).as_str(), "POWER" | "GROUND")
            });
            if !other {
                db.destroy_inst(tie).map_err(e)?;
            }
        }
    }
    let lines: Vec<Value> = if tie_count > 0 { vec![json!({ "code": "RSZ-0042", "message": format!("Inserted {tie_count} tie {cell} instances.") })] } else { Vec::new() };
    Ok(json!({ "lib_port": lib_port, "tie_instances": insts.len(), "ties_checked": ties_checked, "inserted": tie_count, "lines": lines }))
}

/// `PatternMatch` without regexp: `*` matches any run of characters, `?` any one; the rest exactly.
pub fn glob_match(pat: &str, s: &str) -> bool {
    let (p, t): (Vec<char>, Vec<char>) = (pat.chars().collect(), s.chars().collect());
    let (mut pi, mut ti, mut star, mut mark) = (0usize, 0usize, None::<usize>, 0usize);
    while ti < t.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == t[ti]) {
            pi += 1;
            ti += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = Some(pi);
            mark = ti;
            pi += 1;
        } else if let Some(sp) = star {
            pi = sp + 1;
            mark += 1;
            ti = mark;
        } else {
            return false;
        }
    }
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}






/// `Resizer::initBlock`'s sizing restrictions from the block's properties (`set_opt_config`
/// writes them before the repair): a limit absent is the default; `keep_sizing_site` absent is
/// off, as is `keep_sizing_vt`.
/// `Resizer::initBlock`'s global sizing knobs: the block's `gs_*` properties (written by
/// `set_global_sizing_config`, so a captured database carries them) over the defaults.
pub fn global_sizing_config(db: &Db) -> Result<crate::repair_setup::GlobalSizingConfig, String> {
    let mut c = crate::repair_setup::GlobalSizingConfig::default();
    if let Some(v) = db.block_string_property("gs_presize_mode").map_err(|e| e.to_string())? {
        c.presize_mode = match v.as_str() {
            "disabled" => 0,
            "min_size_max_vt" => 1,
            "max_size_min_vt" => 2,
            other => return Err(format!("gs_presize_mode '{other}' (an invalid value, which the reference warns about and ignores): not modelled")),
        };
    }
    if let Some(v) = db.block_bool_property("gs_include_clock_network").map_err(|e| e.to_string())? {
        c.include_clock_network = v;
    }
    let f = |name: &str| -> Result<Option<f32>, String> { Ok(db.block_double_property(name).map_err(|x| x.to_string())?.map(|v| v as f32)) };
    if let Some(v) = f("gs_setup_slack_margin")? {
        c.setup_slack_margin = v;
    }
    if let Some(v) = db.block_int_property("gs_max_iterations").map_err(|e| e.to_string())? {
        c.max_iterations = v;
    }
    if let Some(v) = f("gs_beta")? {
        c.beta = v;
    }
    if let Some(v) = f("gs_mu_exponent")? {
        c.mu_exponent = v;
    }
    if let Some(v) = f("gs_lambda_floor")? {
        c.lambda_floor = v;
    }
    if let Some(v) = f("gs_timing_bias")? {
        c.timing_bias = v;
    }
    if let Some(v) = f("gs_budget_safety_factor")? {
        c.budget_safety_factor = v;
    }
    Ok(c)
}

/// The checks of repair_design / buffer_ports read the LINK library through scene 0: a first
/// library read for another corner is refused there.
pub fn link_corner_is_first(libs: &Libs) -> Result<(), String> {
    if libs.link_scene.is_some_and(|k| k != 0) {
        return Err("the first liberty library read is not for the first corner: the link library's corner is not modelled".into());
    }
    Ok(())
}


pub fn read_text(path: &str) -> Result<String, String> {
    if path.ends_with(".gz") {
        use std::io::Read;
        let mut s = String::new();
        flate2::read::MultiGzDecoder::new(std::fs::File::open(path).map_err(|e| format!("{path}: {e}"))?)
            .read_to_string(&mut s)
            .map_err(|e| format!("{path}: {e}"))?;
        Ok(s)
    } else {
        std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))
    }
}



/// `set_max_capacitance V <objects>` beyond the design: on ports (`set_port_capacitance_limit`) and
/// on instance pins (`set_pin_capacitance_limit`), the two forms `write_sdc` writes besides the
/// design's. A cell — named bare, `find_cells_matching` — or any other form is refused, as is a
/// value that is not a number.
pub fn sdc_cap_objects_modelled(s: &vyges_loom::sdc::Sdc) -> Result<(), String> {
    for (v, o) in &s.max_capacitance_objects {
        if v.is_nan() || !matches!(o.accessor.as_str(), "get_ports" | "get_pins") {
            return Err(format!("set_max_capacitance {v} on [{} {}]: not modelled (the design, ports and pins are)", o.accessor, o.names.join(" ")));
        }
    }
    Ok(())
}

/// `repair_design`'s Tcl arguments. `-max_wire_length` is in microns (the default distance
/// unit) and reaches the repair in meters.
pub fn parse_args(args: &[String]) -> Result<Args, String> {
    let mut a = Args::default();
    let mut i = 0;
    let num = |v: Option<&String>, flag: &str| -> Result<f64, String> {
        v.ok_or(format!("{flag} needs a value"))?.parse::<f64>().map_err(|_| format!("{flag}: not a number"))
    };
    while i < args.len() {
        match args[i].as_str() {
            // `sta::distance_ui_sta`: microns times the distance unit's FLOAT scale, in double.
            "-max_wire_length" => {
                a.max_wire_length = num(args.get(i + 1), "-max_wire_length")? * f64::from(1e-6f32);
                i += 1;
            }
            "-slew_margin" => {
                a.slew_margin = num(args.get(i + 1), "-slew_margin")?;
                i += 1;
            }
            "-cap_margin" => {
                a.cap_margin = num(args.get(i + 1), "-cap_margin")?;
                i += 1;
            }
            // Deprecated: it only turns the early sizing round on.
            "-buffer_gain" => {
                a.pre_placement = true;
                i += 1;
            }
            "-pre_placement" => a.pre_placement = true,
            // internal: `findResizeSlacks`' repair (global placement's timing-driven mode)
            "-vyges_resize_slack" => a.resize_slack = true,
            "-verbose" => {}
            f @ ("-match_cell_footprint" | "-reroute" | "-max_utilization") => return Err(format!("repair_design {f}: not modelled")),
            other => return Err(format!("repair_design argument {other}: not modelled")),
        }
        i += 1;
    }
    Ok(a)
}



pub fn run(job: &Value) -> Result<Value, String> {
    run_owned(job, Db::new()).map(|(report, _)| report)
}

/// [`run`] on a database already open (the steps may still read one over it), handing the
/// database back as the steps left it — for an engine that runs the resizer on its own design
/// mid-command (global placement's timing-driven iterations).
pub fn run_owned(job: &Value, db: Db) -> Result<(Value, Db), String> {
    let mut db = db;
    let mut libs = Libs::default();
    let mut set_dont_use: BTreeSet<String> = BTreeSet::new();
    let mut liberty = vyges_est::liberty::LibertyClocks::default();
    let mut rc = vyges_est::rc::Rc::new();
    let mut sdc: Option<vyges_loom::sdc::Sdc> = None;
    let mut estimated = false;
    // `set_routing_alpha` (the Steiner builder's; 0.3 by default): the estimate reads the value
    // in force when it ran, the repair's trees the value in force when repair_design does.
    let mut routing_alpha = 0.3f32;
    // `set_global_routing_layer_adjustment * F`: the capacity every routing layer keeps back
    let mut routing_adjustment = 0.0f32;
    let mut estimate_alpha = routing_alpha;
    // The design the estimate saw, when it differs from the one repaired (a job's `db` on the
    // estimate step: the reference keeps an estimate across a later cell MOVE — est has no callback
    // for one — so it repairs on the earlier placement's parasitics).
    let mut estimate_db: Option<String> = None;
    // The constraints as the estimate saw them (an estimate step's "sdc"), when a port load was
    // set after it: the estimate's pi models were reduced without it.
    let mut estimate_sdc: Option<String> = None;
    // The RC in force when the estimate ran (later re-estimates read the RC in force then).
    let mut estimate_rc: Option<vyges_est::rc::Rc> = None;
    let units = |l: &vyges_est::liberty::LibertyClocks| -> Result<vyges_est::rc::Units, String> {
        let u = l.units.ok_or("RC before a liberty library: the timer's default units are not modelled")?;
        Ok(vyges_est::rc::Units { resistance: u.resistance, capacitance: u.capacitance, distance: u.distance })
    };
    let mut trace = Trace::default();
    let mut outcome: Option<Result<repair_design::Outcome, Stop>> = None;
    // The estimator's per-net state after an edit made inside its incremental guard (buffer_ports):
    // a later repair starts from it rather than from a fresh estimate.
    let mut carried: Option<Vec<HashMap<String, NetParasitics>>> = None;
    let mut buffered: Vec<buffer_ports::Outcome> = Vec::new();
    // Each repair_tie_fanout's result. A later repair is refused: the ties' new nets would reach
    // it through the estimator's callbacks, which are not modelled across this command.
    let mut tie_runs: Vec<Value> = Vec::new();
    // Each repair_clock_nets' result, and the resizer's clock buffer list (`clk_buffers_`) as the
    // session left it — `clock_buffers`, standing for what clock_tree_synthesis set.
    let mut clock_runs: Vec<Value> = Vec::new();
    let mut clk_buffers: Vec<String> = Vec::new();
    let mut resizer_ran = false;
    // `set_propagated_clock` on the clock, and as the estimate saw it.
    let mut propagated = false;
    let mut estimate_propagated = false;
    let mut sdc_propagated = false;
    // `set_max_delay` / `set_min_delay` in the file: read by repair_timing's search, which says
    // which forms it models; every other command refuses them.
    let mut sdc_path_delays = false;
    // Each repair_timing's lines; a refusal after them stops the job.
    let mut timing_runs: Vec<Value> = Vec::new();
    // `set_debug_level`: each (tool, group) and its level.
    let mut debug_levels: BTreeMap<(String, String), i64> = BTreeMap::new();
    // `setenv <name> <value>`: the environment variables a script set (`set ::env(..)`) that a
    // policy reads (`loadPolicyEnvars`), as the capture logged them at its repair_timing.
    let mut env_vars: BTreeMap<String, String> = BTreeMap::new();
    let mut timing_stop: Option<String> = None;
    // The last repair_design's progress table (`printProgress`), as the reference prints it.
    let mut repair_progress: Vec<String> = Vec::new();
    // `resize_slack_preamble`: what `Resizer::resizeSlackPreamble` logged — each corner without a
    // wire resistance (RSZ-0088), and RSZ-0089 when no corner has one.
    let mut slack_preamble: Option<Value> = None;
    // `sta_levels`: the timer's levels a caller carries from an earlier job (its session), given
    // to each timing step and returned as the last left them; whether a repair_design edited the
    // netlist under them (their incremental relevelization is not modelled across the steps).
    let mut sta_levels: Option<BTreeMap<String, i32>> = match &job["sta_levels"] {
        Value::Object(m) => Some(m.iter().filter_map(|(k, v)| v.as_i64().map(|l| (k.clone(), l as i32))).collect()),
        _ => None,
    };
    let mut repair_design_edited = false;
    // `clock_latencies`: global placement's virtual clock tree for this reweight (`virtual_cts`'s
    // result, set in the timer as `setClockLatency(clk, pin)` before every timing step).
    let virtual_latencies: Vec<(String, String, f32)> = job["clock_latencies"]
        .as_array()
        .map(|a| a.iter().filter_map(|x| Some((x["clock"].as_str()?.to_string(), x["pin"].as_str()?.to_string(), x["delay"].as_f64()? as f32))).collect())
        .unwrap_or_default();
    let mut virtual_cts_report: Option<Value> = None;
    // `global_route`: the router's result (its routes, and the parasitics
    // `estimate_parasitics -global_routing` builds from them); `gr_estimated` when that estimate is
    // the one in force (`estimated` stays for the placement estimate's).
    let mut groute: Option<vyges_grt::global_route::RouteResult> = None;
    let mut groute_opts: Option<vyges_grt::global_route::RouteOptions> = None;
    let mut gr_estimated = false;
    for step in job["steps"].as_array().ok_or("steps")? {
        let cmd = step["cmd"].as_str().ok_or("cmd")?;
        let args: Vec<String> = step["args"].as_array().map(|a| a.iter().filter_map(Value::as_str).map(String::from).collect()).unwrap_or_default();
        // Whether a resizer command before this one ran `initBlock` (each of these does, early).
        let block_inited = resizer_ran;
        resizer_ran |= matches!(cmd, "repair_design" | "buffer_ports" | "repair_timing" | "repair_tie_fanout" | "repair_clock_nets");
        match cmd {
            "read_lef" => {
                let path = args.last().ok_or("read_lef path")?;
                db.read_lef(path).map_err(|e| format!("{path}: {e}"))?
            }
            "read_def" => {
                let path = args.last().ok_or("read_def path")?;
                db.read_def(path, "default").map_err(|e| format!("{path}: {e}"))?
            }
            "read_db" => {
                let path = args.last().ok_or("read_db path")?;
                db = Db::open(path).map_err(|e| format!("{path}: {e}"))?
            }
            "read_liberty" => {
                if args.iter().any(|a| a == "-min" || a == "-max") {
                    return Err("read_liberty -min / -max: min/max libraries are not modelled".into());
                }
                // `-corner c`: the library serves that scene only; without it, every scene.
                let scene = match args.iter().position(|a| a == "-corner") {
                    Some(i) => {
                        let c = args.get(i + 1).ok_or("read_liberty -corner needs a corner")?;
                        Some(libs.scenes.iter().position(|s| s == c).ok_or_else(|| format!("read_liberty -corner {c}: corner not defined"))?)
                    }
                    None => None,
                };
                let path = args.last().ok_or("read_liberty path")?;
                let text = read_text(path)?;
                liberty.read(&text).map_err(|e| format!("{path}: {e}"))?;
                let group = vyges_sta::liberty_parse::parse(&text).map_err(|e| format!("{path}: {e}"))?;
                // A model's value is scaled by the library's k_ factors at the operating
                // conditions; only an unscaled library is modelled.
                if group.simple.iter().any(|(n, _)| n.starts_with("k_")) || group.groups_of("scaling_factors").next().is_some() {
                    return Err(format!("{path}: liberty k_ scale factors are not modelled"));
                }
                let lib = Library::read(&group).map_err(|e| format!("{path}: {e} (not modelled)"))?;
                if !libs.scene_libs.is_empty() {
                    if libs.libs.is_empty() {
                        libs.link_scene = scene;
                    }
                    for (k, sl) in libs.scene_libs.iter_mut().enumerate() {
                        if scene.is_none_or(|c| c == k) {
                            sl.push(lib.clone());
                        }
                    }
                } else if scene.is_some() {
                    return Err("read_liberty -corner without define_corners (not modelled)".into());
                }
                libs.libs.push(lib);
            }
            // `set_dont_use_cmd`: each element of the list through `get_lib_cells` — a cell-name
            // glob (`*`, `?`) over every library read so far; no match only warns.
            "set_dont_use" => {
                // The buffer list is found once and kept (findBuffers returns early when it has one).
                if !buffered.is_empty() {
                    return Err("set_dont_use after buffer_ports: the kept buffer list is not modelled".into());
                }
                for a in &args {
                    for pat in a.split_whitespace() {
                        // A SWIG object handle (`_<hex>_p_LibertyCell`) is a cell object logged
                        // without its name: as a pattern it would silently match nothing.
                        if pat.starts_with('_') && pat.contains("_p_") {
                            return Err(format!("set_dont_use {pat}: an object handle, not a cell name (not modelled)"));
                        }
                        if pat.contains(['/', '[', ']', '\\']) {
                            return Err(format!("set_dont_use {pat}: library-qualified or bracketed patterns are not modelled"));
                        }
                        for lib in &libs.libs {
                            set_dont_use.extend(lib.cells.keys().filter(|c| glob_match(pat, c)).cloned());
                        }
                    }
                }
            }
            // The scenes, in order; before any library (a later one would remap them).
            "define_corners" => {
                if !libs.libs.is_empty() {
                    return Err("define_corners after read_liberty: not modelled".into());
                }
                rc.define_corners(&args);
                libs.scenes = args.clone();
                libs.scene_libs = vec![Vec::new(); args.len()];
            }
            "set_layer_rc" => rc.set_layer_rc(&mut db, units(&liberty)?, &args)?,
            // The argument checks first: the reference raises them before converting any value.
            "set_wire_rc" => {
                rc.check_set_wire_rc(&db, &args)?;
                rc.set_wire_rc(&db, units(&liberty)?, &args)?
            }
            // `-global_routing`: `estimateGlobalRouteRC` — every routed net's network from its route
            // (`MakeWireParasitics`), over the router's routes as they stand.
            "estimate_parasitics" if args.iter().any(|a| a == "-global_routing") => {
                if args.len() != 1 {
                    return Err(format!("estimate_parasitics {}: options with -global_routing are not modelled", args.join(" ")));
                }
                if groute.is_none() {
                    return Err("estimate_parasitics -global_routing before global_route: not modelled".into());
                }
                estimated = false;
                gr_estimated = true;
                carried = None;
            }
            "estimate_parasitics" => {
                if !args.iter().any(|a| a == "-placement") {
                    return Err("estimate_parasitics without -placement: not modelled".into());
                }
                estimated = true;
                gr_estimated = false;
                carried = None;
                estimate_alpha = routing_alpha;
                estimate_db = step["db"].as_str().map(String::from);
                estimate_sdc = step["sdc"].as_str().map(String::from);
                estimate_rc = Some(rc.clone());
                estimate_propagated = propagated;
            }
            // One clock is modelled, so the clock it names is that one.
            "set_propagated_clock" => propagated = true,
            "setenv" => {
                let [name, value] = args.as_slice() else { return Err(format!("setenv {}: needs a name and a value", args.join(" "))) };
                env_vars.insert(name.clone(), value.clone());
            }
            // `set_debug_level <tool> <group> <level>`: debug lines are not output this engine
            // scores, except a group that adds report lines (the move tracker's, RSZ-0211 on).
            "set_debug_level" => {
                let [tool, group, level] = args.as_slice() else { return Err(format!("set_debug_level {}: needs a tool, a group and a level", args.join(" "))) };
                let level: i64 = level.parse().map_err(|_| format!("set_debug_level {level}: not an integer"))?;
                debug_levels.insert((tool.clone(), group.clone()), level);
            }
            // `set_global_routing_layer_adjustment * F`: every routing layer's capacity reduced by
            // the fraction F, as the router's global adjustment. One layer's own adjustment (a layer
            // name in place of `*`) is not modelled.
            "set_global_routing_layer_adjustment" => {
                let [layers, v] = args.as_slice() else { return Err(format!("set_global_routing_layer_adjustment {}: needs layers and a value", args.join(" "))) };
                if layers != "*" {
                    return Err(format!("set_global_routing_layer_adjustment {layers}: one layer's adjustment is not modelled (all layers, `*`, is)"));
                }
                routing_adjustment = v.parse::<f32>().map_err(|_| format!("set_global_routing_layer_adjustment {v}: not a number"))?;
            }
            // `set_routing_layers -signal lo-hi [-clock lo-hi]`: the block's routing layer range.
            "set_routing_layers" => {
                let mut i = 0;
                while i < args.len() {
                    let range = args.get(i + 1).ok_or_else(|| format!("set_routing_layers {}: a range", args[i]))?;
                    let (lo, hi) = range.split_once('-').ok_or_else(|| format!("set_routing_layers {range}: lo-hi"))?;
                    let level = |n: &str| -> Result<i32, String> {
                        match db.layer_get_routing_level(n) {
                            0 => Err(format!("set_routing_layers: {n} is not a routing layer")),
                            l => Ok(l),
                        }
                    };
                    let (lo, hi) = (level(lo)?, level(hi)?);
                    match args[i].as_str() {
                        "-signal" => {
                            db.block_set_min_routing_layer(lo).map_err(|e| e.to_string())?;
                            db.block_set_max_routing_layer(hi).map_err(|e| e.to_string())?;
                        }
                        "-clock" => {
                            db.block_set_min_layer_for_clock(lo).map_err(|e| e.to_string())?;
                            db.block_set_max_layer_for_clock(hi).map_err(|e| e.to_string())?;
                        }
                        other => return Err(format!("set_routing_layers {other}: not modelled")),
                    }
                    i += 2;
                }
            }
            // `global_route` (no options): FastRoute over the design, timing-driven on this
            // design's constraints (the router reads its net slacks from the timer, as the
            // reference's does). Its routes stay for `estimate_parasitics -global_routing`.
            "global_route" => {
                if !args.is_empty() {
                    return Err(format!("global_route {}: options are not modelled", args.join(" ")));
                }
                if estimated || gr_estimated {
                    return Err("global_route after estimate_parasitics: the router's timer on those parasitics is not modelled".into());
                }
                if libs.scene_count() > 1 {
                    return Err("global_route over several corners: not modelled".into());
                }
                let mut opts = vyges_grt::global_route::RouteOptions::new();
                opts.liberty = Some(liberty.clone());
                opts.alpha = routing_alpha;
                opts.adjustment = routing_adjustment;
                (opts.layer_rc, opts.via_rc) = grt_layer_rc(&db, &rc);
                if let Some(s) = sdc.as_ref() {
                    let lib0 = libs.default_library().ok_or("global_route before any liberty library")?;
                    opts.clock_sources = s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect();
                    let ssdc = search_sdc_on(s, &vyges_grt::timer::netlist(&db), lib0.time_scale, propagated || sdc_propagated, false)?;
                    opts.timing = Some(vyges_grt::timer::Timing { libs: libs.libs.clone(), constraints: Default::default(), sdc: Some(ssdc) });
                }
                let res = vyges_grt::global_route::route_design(&mut db, &opts, &grt_stt, &grt_flutes).map_err(|e| format!("global_route: {e}"))?;
                if res.total_overflow > 0 {
                    return Err("global_route left overflow (GRT-0116): not modelled".into());
                }
                groute = Some(res);
                groute_opts = Some(opts);
            }
            "set_routing_alpha" => {
                if args.iter().any(|a| a.starts_with('-')) {
                    return Err(format!("set_routing_alpha {}: per-net / min_fanout / min_hpwl alphas are not modelled", args.join(" ")));
                }
                let [v] = args.as_slice() else { return Err("set_routing_alpha needs one value".into()) };
                routing_alpha = v.parse::<f32>().map_err(|_| format!("set_routing_alpha {v}: not a number"))?;
            }
            // The design's constraints as the reference held them when repair_design ran.
            "read_sdc" => {
                let path = args.last().ok_or("read_sdc path")?;
                // Read against the linked design, as the reference does: object queries
                // (-filter, -of_objects) select from the database's ports, instances and pins.
                let s = if db.inst_names().is_empty() {
                    vyges_loom::sdc::Sdc::load(path)
                } else {
                    vyges_loom::sdc::Sdc::load_with(path, &crate::env::sdc_design(&db, &libs))
                }
                .map_err(|e| format!("{path}: {}", e.0))?;
                // A command whose object query could still not be evaluated was not applied:
                // refuse rather than repair against constraints the file did not mean.
                if let Some(q) = s.unevaluated_queries.first() {
                    return Err(format!("{path}: {q}: object query not evaluated, not modelled"));
                }
                // set_max_transition / set_max_fanout / set_max_capacitance on the design are
                // modelled (the limits); on any other object they are not.
                let unmodelled: Vec<&str> = s
                    .ignored_affecting_timing()
                    .into_iter()
                    .filter(|c| (*c != "set_max_transition" || s.max_transition_on_objects) && (*c != "set_max_fanout" || s.max_fanout_on_objects) && *c != "set_max_capacitance" && *c != "set_driving_cell" && *c != "set_propagated_clock" && *c != "set_max_delay" && *c != "set_min_delay")
                    .collect();
                if !unmodelled.is_empty() {
                    return Err(format!("{path}: {} not modelled", unmodelled.join(", ")));
                }
                sdc_cap_objects_modelled(&s).map_err(|e| format!("{path}: {e}"))?;
                // `set_propagated_clock` in the file: read by repair_timing's search; the other
                // commands refuse it when they run.
                sdc_propagated = !s.propagated.is_empty();
                sdc_path_delays = !s.path_delays.is_empty();
                sdc = Some(s);
            }
            "buffer_ports" | "repair_design" | "repair_timing" if !tie_runs.is_empty() => return Err(format!("{cmd} after repair_tie_fanout: not modelled")),
            "repair_tie_fanout" if gr_estimated => return Err("repair_tie_fanout on global-route parasitics: not modelled".into()),
            "repair_tie_fanout" => tie_runs.push(repair_tie_fanout(&mut db, &libs, &args, block_inited)?),
            // `set_dont_touch`: each object an instance, else a net (dbInst / dbNet setDoNotTouch).
            "set_dont_touch" => {
                for o in &args {
                    let (inst, net) = (db.block_get_insts().contains(o), db.block_get_nets().contains(o));
                    match (inst, net) {
                        (true, false) => db.inst_set_do_not_touch(o, true).map_err(|e| e.to_string())?,
                        (false, true) => db.net_set_do_not_touch(o, true).map_err(|e| e.to_string())?,
                        _ => return Err(format!("set_dont_touch {o}: not exactly one instance or net (not modelled)")),
                    }
                }
            }
            "buffer_ports" if gr_estimated => return Err("buffer_ports on global-route parasitics: not modelled".into()),
            "repair_design" if gr_estimated => return Err("repair_design on global-route parasitics: not modelled".into()),
            "buffer_ports" => {
                if sdc_propagated {
                    return Err("set_propagated_clock: not modelled for buffer_ports".into());
                }
                if sdc_path_delays {
                    return Err("set_max_delay / set_min_delay: not modelled for buffer_ports".into());
                }
                link_corner_is_first(&libs)?;
                let o = buffer_ports::Options::parse(&args)?;
                let mut dont_use = set_dont_use.clone();
                for lib in &libs.libs {
                    dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
                }
                let m = masters(&db)?;
                if !db.block_get_mod_insts().is_empty() {
                    return Err("buffer_ports on a hierarchical design: not modelled".into());
                }
                let netlist = vyges_grt::timer::netlist(&db);
                // `Sim`'s constants: a tie cell's output and what it holds (case analysis is refused
                // at read_sdc, so tie cells are the only source).
                let constants = sim_constants(&db, &liberty)?;
                // selectBufferCell: the user's cell (a buffer), else findBuffers' weakest.
                let weakest = match &o.buffer_cell {
                    Some(c) => {
                        if !libs.link_cell(c).is_some_and(|c| c.is_buffer()) {
                            return Err(format!("buffer_ports -buffer_cell {c}: not a buffer"));
                        }
                        c.clone()
                    }
                    None => crate::preamble::find_buffers(&libs, &m, &dont_use, true).map_err(|e| e.message().to_string())?.lowest,
                };
                let clock_sources: Vec<String> = sdc.as_ref().map(|s| s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect()).unwrap_or_default();
                rc.sort_clk_and_signal_layers();
                // Inside the guard every edit marks its nets invalid — once parasitics exist.
                let parasitics = match (carried.take(), estimated) {
                    (Some(p), _) => p,
                    (None, true) => {
                        let mut est_rc = estimate_rc.clone().expect("estimated");
                        est_rc.sort_clk_and_signal_layers();
                        let caps = port_caps_at(estimate_sdc.as_deref(), sdc.as_ref(), &libs, &netlist)?;
                        estimate_state(&db, estimate_db.as_deref(), &netlist, &est_rc, &liberty, &clock_sources, estimate_propagated, estimate_alpha, libs.scene_count(), &caps)?
                    }
                    (None, false) => vec![HashMap::new(); libs.scene_count()],
                };
                let info = net_info(&db, &netlist)?;
                let estimating = estimated && !no_signal_cap(&db, &rc);
                let core = (db.block_get_core_area_x_min(), db.block_get_core_area_y_min(), db.block_get_core_area_x_max(), db.block_get_core_area_y_max());
                let core = (core != (0, 0, 0, 0)).then_some(core);
                let port_caps = port_caps_at(step["sdc"].as_str(), sdc.as_ref(), &libs, &netlist)?;
                let mut design = CliDesign { db: &mut db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, constants: constants.clone(), alpha: routing_alpha, estimating, netlist: std::sync::Arc::new(netlist), netlist_gen: 0, parasitics, invalid: BTreeSet::new(), info, core, port_caps, propagated: false, journal: Vec::new(), sdc_nets: BTreeSet::new(), sdc_pins: sdc.as_ref().map(sdc_cap_pins).unwrap_or_default(), timer: TimerLog::default(), stale_out_arcs: BTreeMap::new(), gr: None, gr_error: None, undoing: false, skips: None, skip_log: false };
                let r = buffer_ports::buffer_ports(&mut design, &o, &weakest);
                let parasitics = std::mem::take(&mut design.parasitics);
                match r {
                    Ok(b) => buffered.push(b),
                    Err(Stop::Refused { msg, .. }) => return Err(format!("{msg} (not modelled)")),
                    Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                }
                if estimated {
                    carried = Some(parasitics);
                }
                // The design as buffer_ports left it.
                if let Some(path) = step["write_def"].as_str() {
                    db.write_def(path).map_err(|e| format!("{path}: {e}"))?;
                }
            }
            "repair_design" => {
                if outcome.is_some() {
                    return Err("a second repair_design: not modelled".into());
                }
                link_corner_is_first(&libs)?;
                if sdc_propagated {
                    return Err("set_propagated_clock: not modelled for repair_design".into());
                }
                if sdc_path_delays {
                    return Err("set_max_delay / set_min_delay: not modelled for repair_design".into());
                }
                let a = parse_args(&args)?;
                // dont_use_: every liberty dont_use cell (copyDontUseFromLiberty), then set_dont_use.
                let mut dont_use = set_dont_use.clone();
                for lib in &libs.libs {
                    dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
                }
                let m = masters(&db)?;
                let lib0 = libs.default_library().ok_or("repair_design before any liberty library")?;
                // The SDC's times are in the first library's unit (`user_time_to_sta`, here and in
                // `sdc_env`). Witnessed in another unit only by global placement's reweight (an
                // asap7 ps run); the command on its own keeps the refusal until a case shows it.
                if lib0.time_scale != 1e-9f32 && !a.resize_slack {
                    return Err("a library time unit other than 1ns: user-unit conversion is not modelled".into());
                }
                let limits = Limits {
                    design_max_transition: sdc.as_ref().and_then(|s| s.max_transition).map(|v| user_time_to_sta(v, lib0.time_scale)),
                    // `set_max_fanout` takes a float, unscaled.
                    design_max_fanout: sdc.as_ref().and_then(|s| s.max_fanout).map(|v| v as f32),
                    // In the first library's capacitance unit, like the time unit.
                    design_max_capacitance: sdc.as_ref().and_then(|s| s.max_capacitance).map(|v| crate::timing::user_cap_to_sta(v, lib0.cap_scale)),
                    port_max_capacitance: sdc.as_ref().map(|s| sdc_port_cap_limits(s, lib0.cap_scale)).unwrap_or_default(),
                };
                let clock_sources: Vec<String> = sdc.as_ref().map(|s| s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect()).unwrap_or_default();
                let netlist = vyges_grt::timer::netlist(&db);
                if !estimated {
                    return Err("repair_design without estimate_parasitics -placement: not modelled".into());
                }
                // The estimate runs at the alpha in force when estimate_parasitics did.
                rc.sort_clk_and_signal_layers();
                let mut est_rc = estimate_rc.clone().expect("estimated");
                est_rc.sort_clk_and_signal_layers();
                // The estimator's state as an earlier edit (buffer_ports) left it, else the estimate.
                let parasitics = match carried.take() {
                    Some(p) => p,
                    None => {
                        let caps = port_caps_at(estimate_sdc.as_deref(), sdc.as_ref(), &libs, &netlist)?;
                        estimate_state(&db, estimate_db.as_deref(), &netlist, &est_rc, &liberty, &clock_sources, estimate_propagated, estimate_alpha, libs.scene_count(), &caps)?
                    }
                };
                let wire_rc: Vec<crate::buffered_net::WireRc> = (0..libs.scene_count())
                    .map(|k| {
                        let v = rc.resolved(&db.tech_get_name(), k);
                        crate::buffered_net::WireRc { h_res: v[0], v_res: v[1], h_cap: v[2], v_cap: v[3] }
                    })
                    .collect();
                let constants = sim_constants(&db, &liberty)?;
                let info = net_info(&db, &netlist)?;
                let netlist_for_env = netlist.clone();
                let mut env = match &sdc {
                    Some(s) => sdc_env(s, &libs, &netlist_for_env)?,
                    None => vyges_sta::graph::SdcEnv::default(),
                };
                env.constants = constants.clone();
                let dbu = db.tech_get_db_units_per_micron();
                let core = (db.block_get_core_area_x_min(), db.block_get_core_area_y_min(), db.block_get_core_area_x_max(), db.block_get_core_area_y_max());
                let core = (core != (0, 0, 0, 0)).then_some(core);
                let estimating = !no_signal_cap(&db, &rc);
                // A diagnostic: the timer's delay calculation over the design as read, one line per
                // DMP gate call and per load (`Graph::find_delays`' trace), before any repair.
                let mpins = master_pins(&db)?;
                if let Some(path) = job["dcalc_trace"].as_str() {
                    let mut g = crate::repair_design::timer_graph(&libs, 0, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
                    let mut lines = Vec::new();
                    g.find_delays(&parasitics[0], Some(&mut lines))?;
                    std::fs::write(path, lines.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                }
                let port_caps = env.port_pin_cap.clone();
                let limits_of_block = sizing_limits(&db)?;
                let mut design = CliDesign { db: &mut db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, constants: constants.clone(), alpha: routing_alpha, estimating, netlist: std::sync::Arc::new(netlist), netlist_gen: 0, parasitics, invalid: BTreeSet::new(), info, core, port_caps, propagated: false, journal: Vec::new(), sdc_nets: BTreeSet::new(), sdc_pins: sdc.as_ref().map(sdc_cap_pins).unwrap_or_default(), timer: TimerLog::default(), stale_out_arcs: BTreeMap::new(), gr: None, gr_error: None, undoing: false, skips: None, skip_log: false };
                let inputs = Inputs { libs: &libs, masters: &m, dont_use: &dont_use, limits, clock_sources: &clock_sources, dbu, wire_rc, sdc: env, master_pins: mpins, sizing_limits: limits_of_block, clock_pattern: Default::default() };
                let area0 = crate::odb_design::design_area(design.db);
                let r = repair_design::repair_design(&inputs, &mut design, &a, &mut trace);
                // `RepairDesign::printProgress` (not verbose): the header, row 0, the final row and
                // the closing rule — area growth against the design area when the repair started.
                // Reported for gpl's reweight (`-vyges_resize_slack`), which prints it.
                if let Ok(o) = &r {
                    repair_design_edited |= o.inserted_buffers > 0 || o.resized > 0;
                }
                if let (Ok(o), true) = (&r, a.resize_slack) {
                    let area1 = crate::odb_design::design_area(design.db);
                    let row = |label: &str, area: f64, resized: usize, buffers: usize, repaired: usize, left: usize| format!("{label:>9} | {:>+8.1}% | {resized:>7} | {buffers:>7} | {repaired:>13} | {left:>9}", (area - area0) / area0 * 1e2);
                    repair_progress = vec![
                        "Iteration |   Area    | Resized | Buffers | Nets repaired | Remaining".to_string(),
                        "---------------------------------------------------------------------".to_string(),
                        row("0", area0, 0, 0, 0, o.drivers),
                        row("final", area1, o.resized, o.inserted_buffers, o.nets_repaired, 0),
                        "---------------------------------------------------------------------".to_string(),
                    ];
                }
                outcome = Some(r);
            }
            // The resizer's clock buffer list as clock_tree_synthesis left it in the session
            // (`setClockBuffersList` from its -buf_list): not a Tcl command, the session's state.
            "clock_buffers" => clk_buffers = args.clone(),
            "repair_clock_nets" if gr_estimated => return Err("repair_clock_nets on global-route parasitics: not modelled".into()),
            "repair_clock_nets" => {
                // `parse_max_wire_length`: -max_wire_length in microns, a positive float.
                let mut max_wire_length = 0.0f64;
                let mut it = args.iter();
                while let Some(a) = it.next() {
                    match a.as_str() {
                        "-max_wire_length" => {
                            // `sta::check_positive_float`: a number >= 0 (0 means "use the default
                            // length", which LibreLane's CTS_CLK_MAX_WIRE_LENGTH=0 relies on).
                            let raw = it.next().ok_or("repair_clock_nets -max_wire_length: a value is required")?;
                            let v: f64 = match raw.parse::<f64>() {
                                Ok(v) if v >= 0.0 => v,
                                _ => return Err(format!("STA-0572: -max_wire_length '{raw}' is not a positive float.")),
                            };
                            max_wire_length = v * 1e-6;
                        }
                        other => return Err(format!("repair_clock_nets {other}: not modelled")),
                    }
                }
                // `est::check_parasitics`.
                if !estimated {
                    return Err("repair_clock_nets without estimate_parasitics -placement: not modelled".into());
                }
                link_corner_is_first(&libs)?;
                // `repairClkNets` infers the list (inferClockBufferList) only when the session has none.
                if clk_buffers.is_empty() {
                    return Err("repair_clock_nets with no clock buffer list in the session (inferClockBufferList): not modelled".into());
                }
                let mut dont_use = set_dont_use.clone();
                for lib in &libs.libs {
                    dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
                }
                let m = masters(&db)?;
                let lib0 = libs.default_library().ok_or("repair_clock_nets before any liberty library")?;
                if lib0.time_scale != 1e-9f32 {
                    return Err("a library time unit other than 1ns: user-unit conversion is not modelled".into());
                }
                let limits = Limits {
                    design_max_transition: sdc.as_ref().and_then(|s| s.max_transition).map(|v| user_time_to_sta(v, lib0.time_scale)),
                    design_max_fanout: sdc.as_ref().and_then(|s| s.max_fanout).map(|v| v as f32),
                    design_max_capacitance: sdc.as_ref().and_then(|s| s.max_capacitance).map(|v| crate::timing::user_cap_to_sta(v, lib0.cap_scale)),
                    port_max_capacitance: sdc.as_ref().map(|s| sdc_port_cap_limits(s, lib0.cap_scale)).unwrap_or_default(),
                };
                let clock_sources: Vec<String> = sdc.as_ref().map(|s| s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect()).unwrap_or_default();
                // Per SDC clock, in the SDC's order: its source pins (a virtual clock has none).
                let clocks: Vec<Vec<String>> = sdc.as_ref().map(|s| s.clocks.iter().map(|c| c.sources.clone()).collect()).unwrap_or_default();
                let netlist = vyges_grt::timer::netlist(&db);
                rc.sort_clk_and_signal_layers();
                let mut est_rc = estimate_rc.clone().expect("estimated");
                est_rc.sort_clk_and_signal_layers();
                let clock_propagated = propagated || sdc_propagated;
                let parasitics = match carried.take() {
                    Some(p) => p,
                    None => {
                        let caps = port_caps_at(estimate_sdc.as_deref(), sdc.as_ref(), &libs, &netlist)?;
                        estimate_state(&db, estimate_db.as_deref(), &netlist, &est_rc, &liberty, &clock_sources, estimate_propagated, estimate_alpha, libs.scene_count(), &caps)?
                    }
                };
                let wire_rc: Vec<crate::buffered_net::WireRc> = (0..libs.scene_count())
                    .map(|k| {
                        let v = rc.resolved(&db.tech_get_name(), k);
                        crate::buffered_net::WireRc { h_res: v[0], v_res: v[1], h_cap: v[2], v_cap: v[3] }
                    })
                    .collect();
                // `check_max_wire_length $max_wire_length true`: with a wire resistance at the
                // command scene, the length past which buffering pays — RSZ-0065 when a shorter
                // one was given, RSZ-0058 and that length when none was.
                let mut lines: Vec<Value> = Vec::new();
                let buffers = crate::preamble::find_buffers(&libs, &m, &dont_use, true).map_err(|e| e.message().to_string())?;
                let (tgt_slews, _, _) = crate::preamble::find_target_loads(&libs, &buffers.cells, &dont_use);
                let mpins = master_pins(&db)?;
                if wire_rc.first().is_some_and(|rc| (rc.h_res + rc.v_res) / 2.0 > 0.0) {
                    let ctx = crate::max_wire_length::Ctx { libs: &libs, buffers: &buffers.cells, tgt_slews, wire_rc: &wire_rc, master_pins: &mpins };
                    let mut lengths = Vec::new();
                    let min_delay = crate::max_wire_length::find_max_wire_length(&ctx, &mut lengths).map_err(|e| e.message().to_string())?;
                    if max_wire_length > 0.0 {
                        if max_wire_length < min_delay {
                            lines.push(json!({ "code": "RSZ-0065", "severity": "warning", "message": format!("max wire length less than {:.0}u increases wire delays.", min_delay / 1e-6) }));
                        }
                    } else {
                        max_wire_length = min_delay;
                        lines.push(json!({ "code": "RSZ-0058", "severity": "info", "message": format!("Using max wire length {:.0}um.", max_wire_length / 1e-6) }));
                    }
                }
                let constants = sim_constants(&db, &liberty)?;
                let info = net_info(&db, &netlist)?;
                let mut env = match &sdc {
                    Some(s) => sdc_env(s, &libs, &netlist)?,
                    None => vyges_sta::graph::SdcEnv::default(),
                };
                env.constants = constants.clone();
                let dbu = db.tech_get_db_units_per_micron();
                let core = (db.block_get_core_area_x_min(), db.block_get_core_area_y_min(), db.block_get_core_area_x_max(), db.block_get_core_area_y_max());
                let core = (core != (0, 0, 0, 0)).then_some(core);
                let estimating = !no_signal_cap(&db, &rc);
                let port_caps = env.port_pin_cap.clone();
                let limits_of_block = sizing_limits(&db)?;
                // dbNetwork::id(pin) — an instance terminal's odb id << 1, a port's << 1 | 1 — is
                // the order of the clock network's `PinSet`.
                let mut ids: HashMap<String, u64> = HashMap::new();
                for (inst, master) in &netlist.insts {
                    for t in mpins.get(master).into_iter().flatten() {
                        if let Ok(id) = db.iterm_id(inst, t) {
                            ids.insert(format!("{inst}/{t}"), u64::from(id) << 1);
                        }
                    }
                }
                for b in db.block_get_b_terms() {
                    if let Ok(id) = db.bterm_id(&b) {
                        ids.insert(b, (u64::from(id) << 1) | 1);
                    }
                }
                let sdc_pins = sdc.as_ref().map(sdc_cap_pins).unwrap_or_default();
                let mut design = CliDesign { db: &mut db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, constants: constants.clone(), alpha: routing_alpha, estimating, netlist: std::sync::Arc::new(netlist), netlist_gen: 0, parasitics, invalid: BTreeSet::new(), info, core, port_caps, propagated: clock_propagated, journal: Vec::new(), sdc_nets: BTreeSet::new(), sdc_pins, timer: TimerLog::default(), stale_out_arcs: BTreeMap::new(), gr: None, gr_error: None, undoing: false, skips: None, skip_log: false };
                let inputs = Inputs { libs: &libs, masters: &m, dont_use: &dont_use, limits, clock_sources: &clock_sources, dbu, wire_rc, sdc: env, master_pins: mpins, sizing_limits: limits_of_block, clock_pattern: Default::default() };
                let order = |p: &str| ids.get(p).copied().unwrap_or(u64::MAX);
                let r = repair_design::repair_clk_nets(&inputs, &mut design, &clk_buffers, &clocks, &order, max_wire_length, &mut trace);
                if estimated {
                    carried = Some(std::mem::take(&mut design.parasitics));
                }
                match r {
                    Ok(o) => {
                        lines.extend(o.clk_summary().into_iter().map(|(code, text)| json!({ "code": code, "severity": "info", "message": text })));
                        clock_runs.push(json!({ "lines": lines, "max_wire_length_um": max_wire_length / 1e-6, "nets_checked": o.nets_checked, "nets_repaired": o.nets_repaired, "inserted_buffers": o.inserted_buffers, "length_violations": o.length_violations }));
                    }
                    Err(Stop::Refused { msg, .. }) => return Err(format!("{msg} (not modelled)")),
                    Err(Stop::Error { code, msg }) => {
                        lines.push(json!({ "code": code, "severity": "error", "message": msg }));
                        clock_runs.push(json!({ "lines": lines, "error": code }));
                        break;
                    }
                }
            }
            // `Resizer::resizeSlackPreamble` (global placement's timing-driven mode, at its start):
            // `findMaxWireLength1(issue_error = true)` — a warning per corner whose wire signal
            // resistance is not positive, the error when none is left.
            // `ClockBase::executeVirtualCts` (global placement's `-virtual_cts`, at a reweight): per
            // clock (finite positive period, a source), its register clock pins at their placed
            // instances' origins, each without a user latency given the tree's latency. Reported
            // for the next job of the reweight to carry; nothing is set here.
            "virtual_cts" => {
                let fraction: f32 = args.first().ok_or("virtual_cts needs the skew fraction")?.parse().map_err(|_| "virtual_cts: the skew fraction is not a number".to_string())?;
                let mut out = Vec::new();
                let mut clock_count = 0;
                if let Some(s) = sdc.as_ref() {
                    let lib0 = libs.default_library().ok_or("virtual_cts before any liberty library")?;
                    let ssdc = search_sdc_on(s, &vyges_grt::timer::netlist(&db), lib0.time_scale, propagated || sdc_propagated, false)?;
                    clock_count = ssdc.clocks.len();
                    if clock_count > 0 {
                        let netlist = vyges_grt::timer::netlist(&db);
                        let mpins = master_pins(&db)?;
                        let mut env = sdc_env(s, &libs, &netlist)?;
                        env.constants = sim_constants(&db, &liberty)?;
                        let g = repair_design::timer_graph(&libs, 0, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
                        let info = net_info(&db, &netlist)?;
                        for c in &ssdc.clocks {
                            if c.period <= 0.0 || c.period.is_infinite() || c.source.is_empty() {
                                continue;
                            }
                            let mut sinks: Vec<(String, (i32, i32))> = Vec::new();
                            for p in crate::virtual_cts::register_clk_pins(&g, &c.source, &info.pin_id) {
                                let Some((inst, _)) = p.rsplit_once('/') else { continue };
                                if is_placed(&db.inst_get_placement_status(inst)) {
                                    sinks.push((p.clone(), db.inst_location(inst)));
                                }
                            }
                            let locs: Vec<(i32, i32)> = sinks.iter().map(|x| x.1).collect();
                            let Some(l) = crate::virtual_cts::latencies(&locs, fraction, c.period) else { continue };
                            for (i, (p, _)) in sinks.iter().enumerate() {
                                // `hasUserClockLatency`: any transition and min/max set on the pin
                                if (0..2).any(|rf| (0..2).any(|mm| c.pin_latency(p, rf, mm).is_some())) {
                                    continue;
                                }
                                out.push(json!({ "clock": c.name, "pin": p, "delay": l[i] }));
                            }
                        }
                    }
                }
                virtual_cts_report = Some(json!({ "clocks": clock_count, "latencies": out }));
            }
            "resize_slack_preamble" => {
                let mut sorted = rc.clone();
                sorted.sort_clk_and_signal_layers();
                let mut warnings = Vec::new();
                let mut any = false;
                for k in 0..libs.scene_count() {
                    let v = sorted.resolved(&db.tech_get_name(), k);
                    if (v[0] + v[1]) / 2.0 <= 0.0 {
                        warnings.push(json!({ "code": "RSZ-0088", "message": format!("Corner: {} has no wire signal resistance value.", libs.scene_name(k)) }));
                    } else {
                        any = true;
                    }
                }
                let error = (!any).then(|| json!({ "code": "RSZ-0089", "message": "Could not find a resistance value for any corner. Cannot evaluate max wire length for buffer. Check over your `set_wire_rc` configuration" }));
                slack_preamble = Some(json!({ "warnings": warnings, "error": error }));
            }
            "repair_timing" | "fully_rebuffer" | "resize_slacks" => {
                use crate::repair_timing as rt;
                // `rsz::fully_rebuffer <pin | NULL>`: `resizeSlackPreamble` (the setup preamble's
                // buffers, target loads, fast buffers) and the rebuffer characterization, then the
                // rebuffering — no repair preamble lines, every driver whatever the slacks.
                let fr: Option<String> = match cmd {
                    "fully_rebuffer" => Some(args.iter().find(|a| !a.starts_with("-vyges")).cloned().unwrap_or_else(|| "NULL".into())),
                    // `findResizeSlacks1`'s net slacks, on the resizer's setup context
                    "resize_slacks" => Some(String::new()),
                    _ => None,
                };
                let mut a = if fr.is_some() { rt::Args::parse(&["-setup".to_string(), "-sequence".to_string(), "buffer".to_string()])? } else { rt::Args::parse(&args)? };
                if sta_levels.is_some() && repair_design_edited {
                    return Err("carried timer levels across a repair_design edit: their relevelization is not modelled".into());
                }
                a.sta_levels = sta_levels.clone();
                // `-recover_power` dispatches to `rsz::recover_power` in place of setup and hold:
                // -phases, -sequence and the setup flags never reach it.
                let recover = a.recover_power;
                if recover && a.match_cell_footprint {
                    return Err("repair_timing -recover_power -match_cell_footprint: not modelled".into());
                }
                if recover && gr_estimated {
                    return Err("repair_timing -recover_power on global-route parasitics (initMacrosAndGrid, legalCellPos in replaceCell): not modelled".into());
                }
                // `-phases`: LEGACY alone is the default pipeline without LAST_GASP; another policy
                // after the shared preamble is refused there; an empty list or an unknown first
                // phase is the reference's error, before any line.
                let plan = a.phases.as_deref().filter(|_| !recover).map(rt::phase_plan);
                match &plan {
                    Some(rt::PhasePlan::Error { code, line }) if a.setup => {
                        timing_runs.push(json!({ "lines": [line], "endpoints": 0, "violating_endpoints": 0, "resized": 0, "removed": 0, "inserted": 0, "error": code }));
                        break;
                    }
                    Some(rt::PhasePlan::Other) => return Err("repair_timing -phases: GLOBAL_SIZING, MT1, and MEASURED_VT_SWAP with other phases, are not modelled".into()),
                    Some(rt::PhasePlan::MeasuredVtSwap | rt::PhasePlan::Mt1 | rt::PhasePlan::GlobalSizing) if !a.setup => return Err("repair_timing -phases MEASURED_VT_SWAP / MT1 / GLOBAL_SIZING without -setup: not modelled".into()),
                    _ => {}
                }
                if !estimated && !gr_estimated {
                    return Err("repair_timing without estimate_parasitics -placement: not modelled".into());
                }
                let m = masters(&db)?;
                // VT categories (IMPLANT obstructions) are read where the reference reads them: the
                // buffer list, the hold buffer's VT, VtSwapMove, CRIT_VT_SWAP.
                let lib0 = libs.default_library().ok_or("repair_timing before any liberty library")?;
                let time_scale = lib0.time_scale;
                let s = sdc.as_ref().ok_or("repair_timing without constraints: not modelled")?;
                let netlist = vyges_grt::timer::netlist(&db);
                let constants = sim_constants(&db, &liberty)?;
                let clock_sources: Vec<String> = s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect();
                let parasitics = match carried.take() {
                    Some(p) => p,
                    None if gr_estimated => {
                        // `estimateGlobalRouteRC`: one scene (checked at global_route).
                        let caps = port_caps_at(None, sdc.as_ref(), &libs, &netlist)?;
                        let res = groute.as_ref().expect("estimated from routes");
                        let map: HashMap<String, NetParasitics> = res
                            .routed_parasitics
                            .iter()
                            .map(|(net, n)| {
                                let mut p = vyges_grt::timer::timer_network(net, n);
                                p.port_pin_caps = Some(caps.clone());
                                (net.clone(), p)
                            })
                            .collect();
                        // A diagnostic: each net's network as built (`<net> <total ground cap F> <nodes>
                        // <resistors>`), against the reference's `report_net` wire capacitance.
                        if let Ok(path) = std::env::var("VYGES_RSZ_PAR_DUMP") {
                            let mut nets: Vec<_> = map.iter().collect();
                            nets.sort_by(|a, b| a.0.cmp(b.0));
                            let text: String = nets.iter().map(|(n, p)| format!("{n} {:e} {} {}\n", p.network.node_caps.iter().map(|&c| f64::from(c)).sum::<f64>(), p.network.node_caps.len(), p.network.resistors.len())).collect();
                            std::fs::write(&path, text).map_err(|e| format!("{path}: {e}"))?;
                        }
                        vec![map]
                    }
                    None => {
                        let mut est_rc = estimate_rc.clone().expect("estimated");
                        est_rc.sort_clk_and_signal_layers();
                        let caps = port_caps_at(estimate_sdc.as_deref(), sdc.as_ref(), &libs, &netlist)?;
                        estimate_state(&db, estimate_db.as_deref(), &netlist, &est_rc, &liberty, &clock_sources, estimate_propagated, estimate_alpha, libs.scene_count(), &caps)?
                    }
                };
                let mut env = sdc_env(s, &libs, &netlist)?;
                env.constants = constants.clone();
                let mpins = master_pins(&db)?;
                let mut g = repair_design::timer_graph(&libs, 0, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
                let clock_propagated = propagated || sdc_propagated;
                let clocks = crate::timing::clock_pins(&g, &clock_sources);
                if s.ignored.iter().any(|c| c == "set_clock_transition") {
                    return Err("set_clock_transition: not modelled".into());
                }
                // An ideal clock's network (`ClkNetwork::isIdealClock`): its registers read the
                // ideal clock slew, 0.
                if !clock_propagated {
                    g.ideal_clock = clocks.iter().copied().collect();
                }
                // A diagnostic: the delay calculation's trace at the repair's start
                // (`VYGES_RSZ_DCALC_TRACE=<file>`: each gate call and load), as the repair_design
                // job's `dcalc_trace` writes it.
                if let Ok(path) = std::env::var("VYGES_RSZ_DCALC_TRACE") {
                    let mut lines = Vec::new();
                    g.find_delays(&parasitics[0], Some(&mut lines))?;
                    std::fs::write(&path, lines.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                } else {
                    g.find_delays(&parasitics[0], None)?;
                }
                let mut ssdc = search_sdc_on(s, &netlist, time_scale, clock_propagated, a.hold)?;
                crate::env::add_virtual_latencies(&mut ssdc, &virtual_latencies)?;
                let mut search = vyges_sta::search::Search::in_graph_order(&g, &ssdc);
                search.constraints_modelled()?;
                search.find_arrivals()?;
                search.find_requireds()?;
                // A diagnostic: every vertex's paths' arrivals, requireds and slews (seconds),
                // to set against the reference's `report_checks -fields {slew}` along a path.
                if let Ok(path) = std::env::var("VYGES_RSZ_TIMING_DUMP") {
                    let mut out = String::new();
                    for (v, vx) in g.vertices.iter().enumerate() {
                        for p in &search.paths[v] {
                            out.push_str(&format!("{} mm={} rf={} clk={} edge={:?} states={} arr={:e} req={:e} slew={:e}\n", vx.name, p.tag.mm, p.tag.rf, p.tag.is_clock, p.tag.clk_edge, p.tag.states.0, p.arrival, p.required, g.slew[v][p.tag.rf][p.tag.mm]));
                        }
                    }
                    std::fs::write(&path, out).map_err(|e| format!("{path}: {e}"))?;
                }
                let (ends, starts) = rt::timing_points(&g, &search, &libs, &clocks)?;
                // Several corners: every scene timed on its own libraries and parasitics, each
                // point's slack the least over the scenes (`Sta::slack` over every path).
                let (ends, starts) = if libs.scene_count() > 1 {
                    let mut per_scene = Vec::new();
                    for (k, par) in parasitics.iter().enumerate().take(libs.scene_count()) {
                        let mut gk = repair_design::timer_graph(&libs, k, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
                        let ck = crate::timing::clock_pins(&gk, &clock_sources);
                        if !clock_propagated {
                            gk.ideal_clock = ck.iter().copied().collect();
                        }
                        gk.find_delays(par, None)?;
                        let mut sk = vyges_sta::search::Search::in_graph_order(&gk, &ssdc);
                        sk.find_arrivals()?;
                        sk.find_requireds()?;
                        per_scene.push(rt::timing_points(&gk, &sk, &libs, &ck)?);
                    }
                    (rt::least_over_scenes(per_scene.iter().map(|(e, _)| e.as_slice()).collect())?, rt::least_over_scenes(per_scene.iter().map(|(_, s)| s.as_slice()).collect())?)
                } else {
                    (ends, starts)
                };
                // `-hold` alone: `Resizer::repairHold` (its preamble with clock buffers allowed),
                // over the endpoints' min and max slacks.
                let hold_only = !a.setup;
                let margin = vyges_sta::sdc::user_to_sta(a.setup_margin, time_scale);
                let violating = rt::collect_violating(&ends, margin);
                let violating_starts = rt::collect_violating(&starts, margin);
                // `hasVtSwapCells`: more than one VT category among the buffers (`getBufferList` in
                // the preamble, clock buffers excluded for -setup).
                let vt_category_count = {
                    let mut du = set_dont_use.clone();
                    for lib in &libs.libs {
                        du.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
                    }
                    crate::preamble::get_buffer_list(&libs, &m, &du, !hold_only).map_err(|e| format!("{}: {}", e.code(), e.message()))?.sorted_vt.len()
                };
                let seq = if recover { Vec::new() } else { rt::move_sequence(&a, vt_category_count > 1) };
                // `MeasuredVtSwapPolicy` and `SetupMt1Policy` have no preamble (no
                // `SetupLegacyBase::start`): their lines start with their own RSZ-2024.
                let measured = matches!(plan, Some(rt::PhasePlan::MeasuredVtSwap | rt::PhasePlan::Mt1 | rt::PhasePlan::GlobalSizing));
                let mut lines = if recover || measured || fr.is_some() { Vec::new() } else { rt::preamble(&seq, violating.len(), a.repair_tns_end_percent, a.phases.as_deref()) };
                // `SetupLegacyMtPolicy::start` warns (RSZ-2024) before `SetupLegacyBase::start`'s lines.
                if !recover && !measured && rt::phase_names(a.phases.as_deref()).first().is_some_and(|n| n == "LEGACY_MT") {
                    lines.insert(0, "[WARNING RSZ-2024] Experimental repair setup policy 'SetupLegacyMtPolicy' selected. Do not use this for production.".to_string());
                }
                // The moves modelled, over one corner or several (each reads the scenes the
                // reference's does: see the moves), LEGACY and LAST_GASP.
                let startpoint_rows = matches!(plan, Some(rt::PhasePlan::LegacyPreamble { startpoints: true }));
                let unmodelled = if matches!(plan, Some(rt::PhasePlan::LegacyPreamble { .. })) && !a.phases.as_deref().is_some_and(rt::phases_modelled) {
                    Some(format!("repair_timing -phases {}: not modelled", a.phases.as_deref().unwrap_or_default()))
                } else if let Some(m) = seq.iter().find(|m| !matches!(m, rt::Move::SizeUp | rt::Move::SizeDownFanout | rt::Move::Unbuffer | rt::Move::SwapPins | rt::Move::Buffer | rt::Move::Clone | rt::Move::SplitLoad | rt::Move::SizeUpMatch | rt::Move::VtSwap | rt::Move::Reroute | rt::Move::BufferToInverters)) {
                    Some(format!("repair_timing: {} is not modelled", m.name()))
                } else if a.match_cell_footprint {
                    Some("repair_timing -match_cell_footprint: not modelled".into())
                } else {
                    None
                };
                // Nothing to repair: the preamble alone — but a policy with none still runs (its
                // `iterate` finds the design clean; the final report prints).
                if !recover && !hold_only && !measured && violating.is_empty() && fr.is_none() {
                    timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": 0 }));
                } else if let Some(why) = unmodelled.filter(|_| !hold_only && !recover) {
                    lines.extend(rt::row0_with(&ends, violating.len(), &starts, &violating_starts, time_scale, startpoint_rows));
                    timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": violating.len() }));
                    timing_stop = Some(why);
                    break;
                } else {
                    // Resizer::resizePreamble: the equivalent cells, the buffers, the target slews.
                    let mut dont_use = set_dont_use.clone();
                    for lib in &libs.libs {
                        dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
                    }
                    let equiv = crate::sizing::make_equiv_cells(&libs);
                    let db_hierarchy = db.has_hierarchy();
                    let site_heights: BTreeMap<String, i32> = m.values().map(|mm| mm.site.clone()).filter(|s| !s.is_empty()).map(|s| (s.clone(), db.site_get_height(&s))).collect();
                    let db_dbu = db.tech_get_db_units_per_micron();
                    let buffers = crate::preamble::find_buffers(&libs, &m, &dont_use, !hold_only).map_err(|e| format!("{}: {}", e.code(), e.message()))?;
                    let (tgt_slews, tgt_scene, target_loads) = crate::preamble::find_target_loads(&libs, &buffers.cells, &dont_use);
                    let sizing = crate::sizing::Sizing { libs: &libs, masters: &m, dont_use: &dont_use, equiv: &equiv, target_loads: &target_loads, tgt_slews, tgt_scene, limits: sizing_limits(&db)? };
                    let limits = Limits {
                        design_max_transition: s.max_transition.map(|v| user_time_to_sta(v, time_scale)),
                        design_max_fanout: s.max_fanout.map(|v| v as f32),
                        design_max_capacitance: s.max_capacitance.map(|v| crate::timing::user_cap_to_sta(v, lib0.cap_scale)),
                        port_max_capacitance: sdc_port_cap_limits(s, lib0.cap_scale),
                    };
                    let wire_rc = {
                        rc.sort_clk_and_signal_layers();
                        let v = rc.resolved(&db.tech_get_name(), 0);
                        crate::buffered_net::WireRc { h_res: v[0], v_res: v[1], h_cap: v[2], v_cap: v[3] }
                    };
                    let slew_shape_factor = crate::preamble::compute_slew_shape_factor(lib0).map_err(|e| format!("{}: {}", e.code(), e.message()))?;
                    // Global-route parasitics: the per-layer RC a layered wire reads in buffering.
                    let gr_layers = gr_estimated.then(|| std::sync::Arc::new(gr_layer_rc(&db, &rc)));
                    // The reference's pin addresses, when the gate supplies them (debug order only).
                    let pin_addr = match job["pin_address"].as_str() {
                        Some(path) => Some(crate::unbuffer::PinAddr::parse(&read_text(path)?)?),
                        None => None,
                    };
                    // Rebuffer::init / initOnCorner, when the sequence has BufferMove; the fast
                    // buffers (`resizePreamble` → `findFastBuffers`) SizeDownFanout reads.
                    let rb_sizes_store;
                    let rb_ctx_store;
                    let mut fast_buffers: Vec<String> = Vec::new();
                    let rb_ctx = if !hold_only && (seq.contains(&rt::Move::Buffer) || seq.contains(&rt::Move::SizeDownFanout)) {
                        // `corner_` is `cmdScene()`, the first corner: scene 0.
                        let base = crate::rebuffer::Ctx { libs: &libs, sizes: &[], rc: wire_rc, dbu: db.tech_get_db_units_per_micron(), slew_shape_factor, tgt_slews, time_scale, cap_scale: lib0.cap_scale, cmd: 0, tgt: tgt_scene, layers: gr_layers.clone() };
                        let lowest = libs.link_cell(&buffers.lowest).ok_or("no lowest-drive buffer")?;
                        let r_max = lowest.buffer_ports().map_or(0.0, |(_, o)| lowest.drive_resistance(&o.name));
                        // `findSlewLimit(port, corner_)` / `maxInputSlew(port, corner_)`: the scene
                        // port and its library (the library default for an input is the link's).
                        let slew_limit = |c: &vyges_sta::liberty::Cell, port: &str| {
                            let lib = libs.scene_library(0, &c.name).unwrap_or(lib0);
                            let p = libs.scene_cell(0, &c.name).unwrap_or(c).port(port);
                            crate::timing::find_slew_limit(lib, p.map_or(vyges_sta::liberty::Direction::Output, |p| p.direction), p.and_then(|p| p.max_transition), &limits)
                        };
                        let max_input_slew = |c: &vyges_sta::liberty::Cell, port: &str| {
                            let link_lib = libs.link_library(&c.name).unwrap_or(lib0);
                            let lib = libs.scene_library(0, &c.name).unwrap_or(lib0);
                            let p = libs.scene_cell(0, &c.name).unwrap_or(c).port(port);
                            crate::timing::max_input_slew_at(link_lib, lib, p.map_or(vyges_sta::liberty::Direction::Input, |p| p.direction), p.and_then(|p| p.max_transition), &limits)
                        };
                        let max_load = |c: &vyges_sta::liberty::Cell| crate::timing::max_load(c, &limits);
                        let ci = crate::rebuffer::CharInputs { ctx: &base, r_max, slew_limit: &slew_limit, max_input_slew: &max_input_slew, max_load: &max_load };
                        if seq.contains(&rt::Move::SizeDownFanout) {
                            fast_buffers = crate::rebuffer::find_fast_buffers(&ci, &buffers.cells);
                        }
                        if seq.contains(&rt::Move::Buffer) {
                            rb_sizes_store = match crate::rebuffer::characterize(&ci, &buffers.cells) {
                                Ok(s) => s,
                                Err(e) => {
                                    lines.extend(rt::row0(&ends, violating.len(), &starts, &violating_starts, time_scale));
                                    timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": violating.len() }));
                                    timing_stop = Some(format!("{e} (not modelled)"));
                                    break;
                                }
                            };
                            rb_ctx_store = crate::rebuffer::Ctx { sizes: &rb_sizes_store, ..base };
                            Some(&rb_ctx_store)
                        } else {
                            None
                        }
                    } else {
                        None
                    };
                    let ctx = crate::repair_setup::Ctx {
                        libs: &libs,
                        sizing: &sizing,
                        env: &env,
                        master_pins: &mpins,
                        ssdc: &ssdc,
                        clock_sources: &clock_sources,
                        ideal_clock: !clock_propagated,
                        margin,
                        time_scale,
                        sequence: &seq,
                        limits: &limits,
                        dbu: db.tech_get_db_units_per_micron(),
                        wire_rc,
                        slew_shape_factor,
                        debug: &debug_levels,
                        rebuffer: rb_ctx,
                        lowest_buffer: &buffers.lowest,
                        fast_buffers: &fast_buffers,
                        vt_category_count,
                        pin_addr: pin_addr.as_ref(),
                        hierarchy: db_hierarchy,
                        policy: crate::repair_setup::PolicyConfig::from_env(&env_vars).map_err(|e| format!("{e} (the reference throws): not modelled"))?,
                        gs: global_sizing_config(&db)?,
                    };
                    drop(search);
                    drop(g);
                    let info = net_info(&db, &netlist)?;
                    // Global-route parasitics are estimated whatever the wire RC says.
                    let estimating = gr_estimated || !no_signal_cap(&db, &rc);
                    // `IncrementalParasiticsGuard` on a global-route source: the router alive over
                    // the repair (`IncrementalGRoute`), from the state `global_route` left.
                    let gr = match (gr_estimated, groute.as_ref(), groute_opts.as_ref()) {
                        (true, Some(r), Some(o)) => {
                            // `initMacrosAndGrid`: `importDb`, `initGrid`, `setFixedGridCells`.
                            let grid = vyges_dpl::grid::Grid::build(&db)?;
                            let fixed = vyges_dpl::grid::FixedCells::build(&db, &grid)?;
                            let layers = std::sync::Arc::new(gr_layer_rc(&db, &rc));
                            Some(GrLive { after: r.after.clone(), opts: o.clone(), dirty: BTreeSet::new(), grid, fixed, layers, slacks: None })
                        }
                        _ => None,
                    };
                    rc.sort_clk_and_signal_layers();
                    let core = (db.block_get_core_area_x_min(), db.block_get_core_area_y_min(), db.block_get_core_area_x_max(), db.block_get_core_area_y_max());
                    let core = (core != (0, 0, 0, 0)).then_some(core);
                    let port_caps = env.port_pin_cap.clone();
                    let sdc_nets: BTreeSet<String> = s.env.iter().filter(|e| e.cmd == "set_load" && e.accessor == "get_nets").flat_map(|e| e.objects.iter().cloned()).collect();
                    let mut design = CliDesign { db: &mut db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, constants: constants.clone(), alpha: routing_alpha, estimating, netlist: std::sync::Arc::new(netlist), netlist_gen: 0, parasitics, invalid: BTreeSet::new(), info, core, port_caps, propagated: clock_propagated, journal: Vec::new(), sdc_nets, sdc_pins: sdc_cap_pins(s), timer: TimerLog::default(), stale_out_arcs: BTreeMap::new(), gr, gr_error: None, undoing: false, skips: None, skip_log: false };
                    if recover {
                        // `Resizer::recoverPower`: `resizePreamble`, then `RecoverPower::recoverPower`.
                        let max_area = match a.max_utilization.as_deref() {
                            None => 0.0,
                            Some(v) => {
                                let u: f64 = v.parse().map_err(|_| "repair_timing -max_utilization: not a number".to_string())?;
                                if !(0.0..=100.0).contains(&u) {
                                    return Err("RSZ-0004: -max_utilization must be between 0 and 100%.".into());
                                }
                                crate::repair_setup::SetupDesign::core_area(&design) * (u / 100.0)
                            }
                        };
                        let pa = crate::repair_setup::PowerArgs { percent: a.recover_power_percent as f32, verbose: a.verbose, max_area };
                        let r = crate::repair_setup::recover_power(&ctx, &mut design, &pa);
                        carried = Some(std::mem::take(&mut design.parasitics));
                        let o = match r {
                            Ok(o) => o,
                            Err(Stop::Refused { msg, .. }) => {
                                timing_stop = Some(format!("{msg} (not modelled)"));
                                break;
                            }
                            Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                        };
                        if let Some(path) = job["timing_trace"].as_str() {
                            std::fs::write(path, o.trace.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                        }
                        timing_runs.push(json!({ "lines": o.lines, "endpoints": ends.len(), "violating_endpoints": violating.len(), "resized": o.resized, "error": o.error }));
                        if o.error.is_some() {
                            break;
                        }
                        continue;
                    }
                    if hold_only {
                        let mut trace_head = Vec::new();
                        if debug_levels.get(&("RSZ".to_string(), "resizer".to_string())).is_some_and(|&l| l >= 1) {
                            // `findFastBuffers`' list (`pre-selected buffers`).
                            let base = crate::rebuffer::Ctx { libs: &libs, sizes: &[], rc: wire_rc, dbu: db_dbu, slew_shape_factor, tgt_slews, time_scale, cap_scale: lib0.cap_scale, cmd: 0, tgt: tgt_scene, layers: gr_layers.clone() };
                            let lowest = libs.link_cell(&buffers.lowest).ok_or("no lowest-drive buffer")?;
                            let r_max = lowest.buffer_ports().map_or(0.0, |(_, o)| lowest.drive_resistance(&o.name));
                            let slew_limit = |c: &vyges_sta::liberty::Cell, port: &str| {
                                let lib = libs.link_library(&c.name).unwrap_or(lib0);
                                let p = c.port(port);
                                crate::timing::find_slew_limit(lib, p.map_or(vyges_sta::liberty::Direction::Output, |p| p.direction), p.and_then(|p| p.max_transition), &limits)
                            };
                            let max_input_slew = |c: &vyges_sta::liberty::Cell, port: &str| {
                                let lib = libs.link_library(&c.name).unwrap_or(lib0);
                                let p = c.port(port);
                                crate::timing::max_input_slew_at(lib, lib, p.map_or(vyges_sta::liberty::Direction::Input, |p| p.direction), p.and_then(|p| p.max_transition), &limits)
                            };
                            let max_load = |c: &vyges_sta::liberty::Cell| crate::timing::max_load(c, &limits);
                            let ci = crate::rebuffer::CharInputs { ctx: &base, r_max, slew_limit: &slew_limit, max_input_slew: &max_input_slew, max_load: &max_load };
                            trace_head.push("[DEBUG RSZ-resizer] pre-selected buffers:".to_string());
                            for b in crate::rebuffer::find_fast_buffers(&ci, &buffers.cells) {
                                trace_head.push(format!("[DEBUG RSZ-resizer]  - {b}"));
                            }
                        }
                        let ha = crate::repair_hold::HoldArgs {
                            setup_margin: a.setup_margin * f64::from(time_scale),
                            hold_margin: a.hold_margin * f64::from(time_scale),
                            allow_setup_violations: a.allow_setup_violations,
                            max_buffer_percent: (a.max_buffer_percent / 100.0) as f32,
                            max_passes: a.max_passes,
                            max_iterations: a.max_iterations,
                            verbose: a.verbose,
                            // `parse_max_util`: 0..100 percent, as a fraction.
                            max_utilization: match a.max_utilization.as_deref() {
                                None => None,
                                Some(v) => {
                                    let u: f64 = v.parse().map_err(|_| "repair_timing -max_utilization: not a number".to_string())?;
                                    if !(0.0..=100.0).contains(&u) {
                                        return Err("RSZ-0004: -max_utilization must be between 0 and 100%.".into());
                                    }
                                    Some(u / 100.0)
                                }
                            },
                        };
                        let hctx = crate::repair_hold::HoldCtx { libs: &libs, masters: &m, site_heights: &site_heights, dont_use: &dont_use, tgt_slews, time_scale, debug: &debug_levels };
                        let r = crate::repair_setup::repair_hold(&ctx, &hctx, &mut design, &ha);
                        carried = Some(std::mem::take(&mut design.parasitics));
                        let o = match r {
                            Ok(o) => o,
                            Err(Stop::Refused { msg, .. }) => {
                                timing_stop = Some(format!("{msg} (not modelled)"));
                                break;
                            }
                            Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                        };
                        if let Some(path) = job["timing_trace"].as_str() {
                            let all: Vec<String> = trace_head.into_iter().chain(o.trace.iter().cloned()).collect();
                            std::fs::write(path, all.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                        }
                        timing_runs.push(json!({ "hold": true, "lines": o.lines, "endpoints": ends.len(), "violating_endpoints": o.violating, "inserted": o.inserted, "resized": o.resized, "stopped": o.stopped, "error": o.error }));
                        if let Some(why) = o.stopped {
                            timing_stop = Some(why);
                            break;
                        }
                        // The reference's error ends the command (it throws): no later step runs.
                        if o.error.is_some() {
                            break;
                        }
                        continue;
                    }
                    if cmd == "resize_slacks" {
                        let r = crate::repair_setup::resize_slacks(&ctx, &mut design, &a);
                        carried = Some(std::mem::take(&mut design.parasitics));
                        let slacks = match r {
                            Ok((m, levels)) => {
                                sta_levels = Some(levels);
                                m
                            }
                            Err(Stop::Refused { msg, .. }) => {
                                timing_stop = Some(format!("{msg} (not modelled)"));
                                break;
                            }
                            Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                        };
                        // in the block's net order (its id order: the tie-break of the worst nets)
                        let order: Vec<Value> = design.db.block_get_nets().into_iter().filter_map(|n| slacks.get(&n).map(|s| json!({ "net": n, "slack": s }))).collect();
                        timing_runs.push(json!({ "resize_slacks": order }));
                        continue;
                    }
                    if let Some(pin) = &fr {
                        // `-vyges_resize_slacks`: global placement's reweight — the net slacks read
                        // on the timer the rebuffering left (`findResizeSlacks`), not a fresh one
                        let with_slacks = args.iter().any(|x| x == "-vyges_resize_slacks");
                        let user_pin = (pin != "NULL").then_some(pin.as_str());
                        let mut slacks = None;
                        let r = if with_slacks {
                            crate::repair_setup::fully_rebuffer_with_slacks(&ctx, &mut design, &a, user_pin).map(|(o, s)| {
                                slacks = Some(s);
                                o
                            })
                        } else {
                            crate::repair_setup::fully_rebuffer(&ctx, &mut design, &a, user_pin)
                        };
                        carried = Some(std::mem::take(&mut design.parasitics));
                        let o = match r {
                            Ok(o) => o,
                            Err(Stop::Refused { msg, .. }) => {
                                timing_runs.push(json!({ "fully_rebuffer": pin, "lines": lines }));
                                timing_stop = Some(format!("{msg} (not modelled)"));
                                break;
                            }
                            Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                        };
                        if let Some(path) = job["timing_trace"].as_str() {
                            std::fs::write(path, o.trace.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                        }
                        sta_levels = Some(o.levels.clone());
                        timing_runs.push(json!({ "fully_rebuffer": pin, "lines": o.lines, "inserted": o.inserted, "removed": o.removed }));
                        if let Some(s) = slacks {
                            let order: Vec<Value> = design.db.block_get_nets().into_iter().filter_map(|n| s.get(&n).map(|x| json!({ "net": n, "slack": x }))).collect();
                            timing_runs.push(json!({ "resize_slacks": order }));
                        }
                        continue;
                    }
                    let r = crate::repair_setup::repair_setup(&ctx, &mut design, &a);
                    carried = Some(std::mem::take(&mut design.parasitics));
                    let o = match r {
                        Ok(o) => o,
                        Err(Stop::Refused { msg, .. }) if measured => {
                            // A policy without a preamble has nothing to score before the refusal.
                            timing_stop = Some(format!("{msg} (not modelled)"));
                            break;
                        }
                        Err(Stop::Refused { msg, .. }) => {
                            // A step refused during the repair: scored, as a refusal before it is,
                            // on the preamble and the progress table's header and row 0.
                            lines.extend(rt::row0(&ends, violating.len(), &starts, &violating_starts, time_scale));
                            timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": violating.len() }));
                            timing_stop = Some(format!("{msg} (not modelled)"));
                            break;
                        }
                        Err(Stop::Error { code, msg }) => return Err(format!("{code}: {msg}")),
                    };
                    if let Some(path) = job["timing_trace"].as_str() {
                        std::fs::write(path, o.trace.join("\n") + "\n").map_err(|e| format!("{path}: {e}"))?;
                    }
                    // A carried timer (global placement's reweight) keeps the levels the repair left.
                    if sta_levels.is_some() {
                        sta_levels = Some(o.levels.clone());
                    }
                    // The preamble's own lines from the repair (the move tracker's capture) print
                    // right after RSZ-0099, before RSZ-0221.
                    if let Some(k) = lines.iter().position(|l| l.starts_with("[INFO RSZ-0099]")) {
                        lines.splice(k + 1..k + 1, o.preamble);
                    }
                    lines.extend(o.lines);
                    timing_runs.push(json!({ "lines": lines, "endpoints": ends.len(), "violating_endpoints": violating.len(), "resized": o.resized, "removed": o.removed, "inserted": o.inserted }));
                }
                if a.hold {
                    timing_stop = Some("repair_timing -hold: not modelled".into());
                    break;
                }
            }
            other => return Err(format!("step {other}: not modelled")),
        }
    }
    if let Some(path) = job["trace"].as_str() {
        std::fs::write(path, trace.text()).map_err(|e| format!("{path}: {e}"))?;
    }
    // The design as the repair left it.
    if let Some(path) = job["write_def"].as_str() {
        db.write_def(path).map_err(|e| format!("{path}: {e}"))?;
    }
    // A diagnostic: the database itself (odb ids preserved), to set against the reference's.
    if let Some(path) = job["write_db"].as_str() {
        db.write(path).map_err(|e| format!("{path}: {e}"))?;
    }
    let tags: Vec<&str> = trace.tags.iter().copied().collect();
    let mut report = match outcome {
        // repair_timing alone: its preamble, then the repair it made, the moves it would make
        // (refused), or a CHECKED nothing-to-do with the endpoints it checked.
        None if buffered.is_empty() && !timing_runs.is_empty() => match &timing_stop {
            Some(reason) => json!({ "tool": "vyges-rsz", "status": "refused", "reason": reason }),
            None => {
                let checked: u64 = timing_runs.iter().filter_map(|r| r["endpoints"].as_u64()).sum();
                let sum = |k: &str| -> u64 { timing_runs.iter().filter_map(|r| r[k].as_u64()).sum() };
                let resized = sum("resized");
                let changed = resized + sum("removed") + sum("inserted");
                let violating = sum("violating_endpoints");
                let error = timing_runs.iter().find_map(|r| r["error"].as_str().map(String::from));
                // A repair that ran: what it changed. Nothing to repair: a CHECKED nothing-to-do.
                // The reference's error (RSZ-0050, RSZ-0060) ends the command: an error, with it.
                let status = if error.is_some() { "error" } else if changed > 0 { "repaired" } else if violating > 0 { "unrepaired" } else if checked > 0 { "up_to_date" } else { "vacuous" };
                let mut r = json!({ "tool": "vyges-rsz", "status": status, "endpoints_checked": checked, "resized": resized, "inserted": sum("inserted") });
                if let Some(e) = error {
                    r["reason"] = json!(e);
                }
                r
            }
        },
        // repair_tie_fanout alone: the ties it inserted, else the tie instances it CHECKED.
        None if buffered.is_empty() && !tie_runs.is_empty() => {
            let sum = |k: &str| -> u64 { tie_runs.iter().filter_map(|r| r[k].as_u64()).sum() };
            let status = if sum("inserted") > 0 { "repaired" } else if sum("ties_checked") > 0 { "up_to_date" } else { "vacuous" };
            json!({ "tool": "vyges-rsz", "status": status, "ties_checked": sum("ties_checked"), "inserted_ties": sum("inserted") })
        }
        // repair_clock_nets alone: the buffers it inserted, else the clock drivers it CHECKED.
        None if buffered.is_empty() && !clock_runs.is_empty() => {
            let sum = |k: &str| -> u64 { clock_runs.iter().filter_map(|r| r[k].as_u64()).sum() };
            let error = clock_runs.iter().find_map(|r| r["error"].as_str().map(String::from));
            let status = if error.is_some() { "error" } else if sum("inserted_buffers") > 0 { "repaired" } else if sum("nets_checked") > 0 { "up_to_date" } else { "vacuous" };
            let mut r = json!({ "tool": "vyges-rsz", "status": status, "nets_checked": sum("nets_checked"), "inserted_buffers": sum("inserted_buffers") });
            if let Some(e) = error {
                r["reason"] = json!(e);
            }
            r
        }
        None if buffered.is_empty() => json!({ "tool": "vyges-rsz", "status": "vacuous", "reason": "no repair_design or buffer_ports step" }),
        // buffer_ports alone: the ports it walked are what it checked.
        None => {
            let inserted: usize = buffered.iter().map(|b| b.inserted_inputs + b.inserted_outputs).sum();
            let checked: usize = buffered.iter().map(|b| b.ports_checked).sum();
            let status = if inserted > 0 { "repaired" } else if checked > 0 { "up_to_date" } else { "vacuous" };
            json!({ "tool": "vyges-rsz", "status": status, "ports_checked": checked, "inserted_buffers": inserted })
        }
        // Every driver checked and none needed a repair: a CHECKED nothing-to-do, with its count.
        Some(Ok(o)) => {
            let summary: Vec<Value> = o.summary().into_iter().map(|(code, text)| json!({ "code": code, "message": text })).collect();
            json!({
                "tool": "vyges-rsz",
                "status": o.settle(),
                "nets_checked": o.nets_checked,
                "nets_repaired": o.nets_repaired,
                "inserted_buffers": o.inserted_buffers,
                "resized": o.resized,
                "drivers_skipped": o.drivers_skipped,
                "violations": { "slew": o.slew_violations, "capacitance": o.cap_violations, "fanout": o.fanout_violations, "length": o.length_violations },
                "summary": summary,
                "warnings": o.warnings.iter().map(|(code, text)| json!({ "code": code, "message": text })).collect::<Vec<_>>(),
                "max_wire_lengths": o.max_wire_lengths.iter().map(|(k, b, m)| json!({ "scene": k, "buffer": b, "meters": m })).collect::<Vec<_>>(),
            })
        }
        Some(Err(Stop::Refused { code, msg })) => json!({ "tool": "vyges-rsz", "status": "refused", "code": code, "reason": msg }),
        Some(Err(Stop::Error { code, msg })) => json!({ "tool": "vyges-rsz", "status": "error", "code": code, "reason": msg }),
    };
    report["traced"] = json!(tags);
    if let Some(p) = slack_preamble {
        report["resize_slack_preamble"] = p;
    }
    if let Some(l) = sta_levels {
        report["sta_levels"] = json!(l);
    }
    if let Some(v) = virtual_cts_report {
        report["virtual_cts"] = v;
    }
    if !repair_progress.is_empty() {
        report["repair_design_progress"] = json!(repair_progress);
    }
    if !clock_runs.is_empty() {
        report["repair_clock_nets"] = json!(clock_runs);
    }
    if !timing_runs.is_empty() {
        report["repair_timing"] = json!(timing_runs);
    }
    if !tie_runs.is_empty() {
        // A command that left nothing to do (or had nothing to check), then ties inserted: the
        // design was changed. Ties checked after a vacuous command: a CHECKED nothing-to-do.
        let inserted = tie_runs.iter().any(|r| r["inserted"].as_u64() > Some(0));
        let checked = tie_runs.iter().any(|r| r["ties_checked"].as_u64() > Some(0));
        if inserted && (report["status"] == "up_to_date" || report["status"] == "vacuous") {
            report["status"] = json!("repaired");
        } else if checked && report["status"] == "vacuous" {
            report["status"] = json!("up_to_date");
        }
        report["repair_tie_fanout"] = json!(tie_runs);
    }
    if !buffered.is_empty() {
        report["buffer_ports"] = buffered
            .iter()
            .map(|b| {
                let lines: Vec<Value> = b.lines.iter().map(|l| json!({ "code": l.code, "severity": if l.warning { "warning" } else { "info" }, "message": l.text })).collect();
                json!({ "inserted_inputs": b.inserted_inputs, "inserted_outputs": b.inserted_outputs, "ports_checked": b.ports_checked, "lines": lines })
            })
            .collect();
    }
    Ok((report, db))
}
