// SPDX-License-Identifier: Apache-2.0
//! `vyges-rsz` — electrical repair from a JSON job.
//!
//! ```text
//! vyges-rsz repair_design <job.json> [-o FILE]
//! ```
//!
//! The job: `steps`, the case's commands in order as `{cmd, args}` with the arguments as Tcl
//! evaluated them (`read_lef`, `read_def`, `read_db`, `read_liberty`, `read_sdc`, `set_dont_use`,
//! `set_layer_rc`, `set_wire_rc`, `estimate_parasitics -placement`, `repair_design …`), and
//! `trace` (where to write one `tag|…` line per decision, in the order they are made).

use std::process::ExitCode;

use serde_json::{json, Value};
use vyges_rsz::job::run;

const USAGE: &str = "\
vyges physical rsz — electrical repair of a placed design: repeaters along each net's Steiner
tree, drivers resized, where a wire is too long or a capacitance, fanout or transition limit is
broken

USAGE:
  vyges physical rsz repair_design <job.json> [-o FILE]
  vyges physical rsz buffer_ports <job.json> [-o FILE]  (the same job runner; a job may hold either)
  vyges physical rsz --describe
  vyges physical rsz --help
  vyges physical rsz --version

JOB FIELDS:
  steps        required — the commands in order, each {\"cmd\": ..., \"args\": [...]}, as a flow
               script passes them:
                 read_lef, read_def, read_db, define_corners, read_liberty [-corner C],
                 read_sdc, set_dont_use, set_layer_rc, set_wire_rc, set_routing_alpha,
                 set_routing_layers -signal LO-HI [-clock LO-HI], global_route,
                 estimate_parasitics -placement | -global_routing, set_propagated_clock,
                 buffer_ports [options],
                 repair_design [options], repair_timing [options],
                 clock_buffers CELL..., repair_clock_nets [-max_wire_length L]
               clock_buffers states the resizer's clock buffer list as clock_tree_synthesis
               left it in the session (its -buf_list); repair_clock_nets repairs long wires in
               every clock network with it (L microns; without it, the length past which
               buffering pays, RSZ-0058)
               an estimate_parasitics step may carry \"db\": the database as the estimate saw it,
               when cells were moved between it and the repair
               a buffer_ports step may carry \"write_def\": the design as it left it, as DEF
               an estimate_parasitics or buffer_ports step may carry \"sdc\": the constraints in
               force when it ran, when a port's set_load comes after it
  trace        write one line per decision, in the order the repair makes them, to this path
  write_def    write the design as the repair left it, as DEF, to this path
  dcalc_trace  (diagnostic) write the timer's delay-calculation trace of the design as read

REPAIR_DESIGN OPTIONS:
  -max_wire_length L    the longest wire, microns (0: none)
  -slew_margin P        percent taken off every slew limit
  -cap_margin P         percent taken off every capacitance limit
  -verbose              accepted
  refused: -pre_placement / -buffer_gain, -match_cell_footprint, -reroute, -max_utilization

BUFFER_PORTS OPTIONS:
  -inputs / -outputs    which side (neither: both) — a buffer after each input port, before
                        each output port, unless its net is dont-touch, special or pinless, an
                        input is a clock source, an input's loads are all buffers or one is
                        dont-touch, or an output's driver is tristate or dont-touch
  -buffer_cell C        the buffer to use (default: the weakest buffer the repair would pick)
  -verbose              each port's decision in the report's lines
  refused: -max_utilization, a hierarchical design

REPAIR_TIMING:
  -setup: every move of the default sequence (UnbufferMove, SizeUpMove, SwapPinsMove,
  BufferMove, CloneMove, SplitLoadMove; SizeUpMatchMove) in the LEGACY phase and LAST_GASP —
  the move sequence (RSZ-0100), RSZ-0094 / RSZ-0099 (and RSZ-0221) or RSZ-0098, every progress
  row, the summary (RSZ-0051, RSZ-0062) in the report's repair_timing[].lines, and the design it
  leaves (write_def). A job's timing_trace file gets the pass-by-pass decisions.
  -hold (alone): the hold buffer, RSZ-0046 or RSZ-0033, every progress row, RSZ-0064 / RSZ-0066,
  RSZ-0132, RSZ-0032 and the buffers it inserts; -max_utilization or -max_buffer_percent reached
  ends it with RSZ-0050 / RSZ-0060 (status error).
  status repaired (the design changed), unrepaired (violations, nothing kept), up_to_date or
  error. Clocks (one or several, real or virtual), each ideal or propagated, with their I/O
  delays; latches (time borrowing); VT libraries (VtSwapMove); pins tied to supply nets.
  -setup and -hold together (or neither): the setup repair, then refused. Phases LEGACY, WNS,
  TNS, ENDPOINT_FANIN, STARTPOINT_FANOUT, LAST_GASP, CRIT_VT_SWAP and REROUTE are modelled (an
  empty list or an unknown first phase is the command's own error), on placement or global-route
  parasitics (the live incremental router; a resistance-aware re-route reads the timer's net
  slacks mid-update); the experimental policies
  GLOBAL_SIZING, MT1 and MEASURED_VT_SWAP each alone (tunables from RSZ_* environment variables,
  a setenv step), and LEGACY_MT as the first phase. -recover_power runs in full (RecoverPower: every row, the trace, the design; -match_cell_footprint
  and global-route parasitics refused). Refused before the lines: an experimental policy with
  other phases, LEGACY_MT after another phase or with SizeDownFanoutMove, parasitics other than -placement and -global_routing, setup clock
  uncertainty, clock latency / transition, derates, false and multicycle paths, clock groups,
  path delays other than the forms the timer models (see --describe); refused during the repair:
  -setup with -max_utilization, more than one repair per pass.

CONSTRAINTS READ FROM SDC:
  create_clock (-waveform: a rise then a fall within one period), set_input_delay /
  set_output_delay (-clock, one delay per port and clock; -add_delay adds a clock or merges),
  set_max_transition and set_max_fanout on the design, set_max_capacitance on the design, ports
  and pins, set_load on nets and ports, set_input_transition, set_driving_cell, set_max_delay /
  set_min_delay (repair_timing only, the forms above); any other timing-affecting command is
  refused

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

REPORT:
  buffer_ports (per step: inserted_inputs, inserted_outputs, ports_checked, lines — each with its
  code and severity), status, nets_checked, nets_repaired, inserted_buffers, resized, drivers_skipped, violations
  {slew, capacitance, fanout, length}, summary — the repair's closing lines, each with its code —
  warnings (RSZ-0065: -max_wire_length shorter than the length at which a buffer pays for itself),
  and max_wire_lengths (that length per buffer and scene, meters, as the check computed it)

EXIT STATUS:
  0  repaired     the design changed: buffers inserted or drivers resized
  0  up_to_date   drivers were checked and none needed a change (nets_checked)
  2  vacuous      no driver was checked. NOT a pass.
  2  error        usage, unreadable input, or an error the repair raises
  3  refused      an input or an option this engine does not model — see `reason`
";

const PIN_TOKEN: &str = "@OPENROAD_PIN@";

fn describe() -> String {
    DESCRIBE.replace(PIN_TOKEN, vyges_opendb::OPENROAD_PIN)
}

const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "openroad_pin": "@OPENROAD_PIN@",
  "name": "rsz",
  "summary": "electrical repair of a placed design: repeaters inserted along each net's Steiner tree for long wires, max capacitance and max slew; and port buffering",
  "maturity": "structured",
  "provenance_limitations": [
    "input_hash covers the argument vector, not the content of the job file or of the design files it names.",
    "status is one of repaired, up_to_date, vacuous, refused or error. repaired means the design changed (buffers inserted or drivers resized); up_to_date means drivers were checked and none needed a change (nets_checked says how many; for a job with buffer_ports and no repair_design, ports_checked); vacuous means nothing was checked and is NOT a pass. The declared assertion passes on repaired or up_to_date. Exit status is 0 for repaired and up_to_date, 2 for vacuous and for error, 3 for refused.",
    "Modelled: placement parasitics, one or more corners, flat and hierarchical netlists, the default buffer selection, the SDC constraints the usage lists, buffer_ports before the repair (the estimate it leaves carried into it). Global-route parasitics (set_routing_layers, global_route with no options on one corner, estimate_parasitics -global_routing): repair_timing -hold on them in full, with the router alive over the repair (the dirty nets re-routed before each estimate, new nets added, each new instance legalized to a site and row); -setup on them in full (every move, RerouteMove and the REROUTE phase; a journal undo restores the routes from their guides; a resistance-aware re-route reads the timer's net slacks mid-update); repair_design on them from each net's route (its buffered net from the route, not a Steiner tree; a route with a via, repairNetVia, and -reroute refused); a design with a block instance and buffer_ports on them are refused; overflow global_route leaves is its error, GRT-0116. Refused rather than guessed: the early sizing round, footprint matching, any other netlist edit between the estimate and the repair, buffer_ports on a hierarchical design, a tristate driver or a bidirect pin on a net, and any other timing-affecting SDC command.",
    "repair_timing -setup is modelled for every move of the default sequence and VtSwapMove in the LEGACY, WNS, TNS, ENDPOINT_FANIN, STARTPOINT_FANOUT, LAST_GASP, CRIT_VT_SWAP and REROUTE phases (RerouteMove too) on placement or global-route parasitics, the experimental policies GLOBAL_SIZING, MT1 and MEASURED_VT_SWAP each alone and LEGACY_MT as the first phase, and repair_timing -hold alone and repair_timing -recover_power in full (ending with RSZ-0050 / RSZ-0060 / RSZ-0125 as the command does): every progress row, the summary and the design left, for one or several clocks (real or virtual, each ideal or propagated), latches with time borrowing, VT libraries, pins tied to supply nets, set_max_delay / set_min_delay in the forms the timer models, over one corner or several; -setup with -hold runs the setup part and is refused after it; GLOBAL_SIZING, MT1 or MEASURED_VT_SWAP with other phases, LEGACY_MT after another phase or with SizeDownFanoutMove, setup clock uncertainty, clock latency or transition, derates, false and multicycle paths and clock groups are refused before the lines."
  ],
  "invocation": {
    "args_template": ["repair_design", "{job}"],
    "optional": [ { "arg": "out", "flag": "-o" } ],
    "emits_json": true
  },
  "inputs": {
    "type": "object",
    "required": ["job"],
    "properties": {
      "job": { "type": "string", "description": "path to a JSON job: {steps: [{cmd, args}], trace, write_def}" },
      "out": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["job"],
  "artifacts": [],
  "assertion": {
    "id": "design-repaired",
    "field": "status",
    "pass_when": { "in": ["repaired", "up_to_date"] }
  }
}"#;

fn link(flag: &str) -> Option<(&'static str, &'static str)> {
    Some(match flag {
        "--bug-report" => ("Report a bug", "https://github.com/vyges/community/issues/new?template=bug_report_template.yaml"),
        "--feature-request" => ("Request a feature", "https://github.com/vyges/community/issues/new?labels=enhancement"),
        "--sponsor" => ("Sponsor Vyges", "https://github.com/sponsors/vyges-ip"),
        "--star" => ("Star this tool", "https://github.com/vyges-tools/rsz"),
        _ => return None,
    })
}

fn exit_for(status: &str) -> u8 {
    match status {
        "repaired" | "up_to_date" => 0,
        "refused" => 3,
        _ => 2,
    }
}

mod events {
    use vyges_events::{emit, Event, Severity};

    /// The run's closing events: the repair's own summary lines (one per non-zero count, under
    /// their message codes), then `RSZ-DONE` with the census — what was checked, what was
    /// changed, and what was passed over — or the refusal / error.
    pub fn outcome(report: &serde_json::Value) {
        let status = report["status"].as_str().unwrap_or("error");
        for line in report["summary"].as_array().into_iter().flatten() {
            if let (Some(code), Some(msg)) = (line["code"].as_str(), line["message"].as_str()) {
                emit(&Event::new("vyges-rsz", Severity::Info, msg.to_string()).with_code(code));
            }
        }
        let (code, severity) = match status {
            "refused" => ("RSZ-REFUSED", Severity::Error),
            "error" => ("RSZ-ERROR", Severity::Error),
            "repaired" | "up_to_date" => ("RSZ-DONE", Severity::Info),
            _ => ("RSZ-DONE", Severity::Warn),
        };
        let text = match report["reason"].as_str() {
            Some(r) => format!("{status}: {r}"),
            None if report["nets_checked"].is_u64() => format!(
                "{status}: {} driver(s) checked, {} passed over; {} net(s) repaired, {} buffer(s) inserted, {} driver(s) resized",
                report["nets_checked"], report["drivers_skipped"].as_u64().unwrap_or(0), report["nets_repaired"].as_u64().unwrap_or(0), report["inserted_buffers"].as_u64().unwrap_or(0), report["resized"].as_u64().unwrap_or(0)
            ),
            None => status.to_string(),
        };
        emit(&Event::new("vyges-rsz", severity, text).with_code(code));
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut out: Option<String> = None;
    let mut positional: Vec<&str> = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "-o" => match args.get(i + 1) {
                Some(v) => {
                    out = Some(v.clone());
                    i += 1;
                }
                None => {
                    eprintln!("vyges-rsz: -o needs a FILE");
                    return ExitCode::from(2);
                }
            },
            "--json" => {}
            a if a.starts_with('-') && !positional.is_empty() => {
                eprintln!("vyges-rsz: unknown option {a}\n\n{USAGE}");
                return ExitCode::from(2);
            }
            a => positional.push(a),
        }
        i += 1;
    }
    match positional.first().copied() {
        None | Some("-h") | Some("--help") => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Some("-V") | Some("--version") => {
            println!("vyges-rsz {} ({})\nCopyright (c) Vyges. Apache-2.0.", env!("CARGO_PKG_VERSION"), env!("VYGES_GIT_SHA"));
            return ExitCode::SUCCESS;
        }
        Some("--describe") => {
            println!("{}", describe());
            return ExitCode::SUCCESS;
        }
        Some(f) if link(f).is_some() => {
            let (label, url) = link(f).unwrap();
            println!("{label}:\n  {url}");
            return ExitCode::SUCCESS;
        }
        Some("repair_design") | Some("buffer_ports") if positional.len() == 2 => {}
        _ => {
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    }
    vyges_opendb::init_events_logging();
    let path = positional[1];
    let report = match std::fs::read_to_string(path)
        .map_err(|e| format!("{path}: {e}"))
        .and_then(|t| serde_json::from_str::<Value>(&t).map_err(|e| format!("{path}: {e}")))
    {
        Err(e) => json!({ "tool": "vyges-rsz", "status": "error", "reason": e }),
        Ok(job) => match run(&job) {
            Ok(r) => r,
            Err(e) => json!({ "tool": "vyges-rsz", "status": if e.contains("not modelled") { "refused" } else { "error" }, "reason": e }),
        },
    };
    let status = report["status"].as_str().unwrap_or("error").to_string();
    events::outcome(&report);
    let text = format!("{report}\n");
    match &out {
        Some(f) => {
            if let Err(e) = std::fs::write(f, &text) {
                eprintln!("vyges-rsz: {f}: {e}");
                return ExitCode::from(2);
            }
        }
        None => print!("{text}"),
    }
    ExitCode::from(exit_for(&status))
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_rsz::env::search_sdc;
    use vyges_rsz::job::*;
    use vyges_rsz::odb_design::{clamp_loc_to_core, sdc_cap_pins, sdc_port_cap_limits};

    /// Rules (`Resizer::tieLocation`): out through the strictly nearest side; a tie between two
    /// sides stays on the pin. (`metersToDbu` before `initBlock`): no units yet, no separation.
    #[test]
    fn a_tie_moves_out_through_the_nearest_side_only() {
        let b = [10000, 10000, 10760, 11400];
        assert_eq!(tie_location((10125, 10612), b, 2000), (8125, 10612), "left side nearest");
        assert_eq!(tie_location((10635, 10612), b, 2000), (12635, 10612), "right side nearest");
        assert_eq!(tie_location((10380, 10100), b, 2000), (10380, 8100), "bottom nearest");
        assert_eq!(tie_location((10380, 11300), b, 2000), (10380, 13300), "top nearest");
        assert_eq!(tie_location((10100, 10100), b, 2000), (10100, 10100), "left and bottom tie: not moved");
        assert_eq!(separation_dbu(2.0, 1000.0), 2000);
        assert_eq!(separation_dbu(2.0, 0.0), 0, "the session's first resizer command: dbu_ is still 0");
    }


    /// Rule (`GlobalRouter::globalRoute` → `isCongested`): overflow left after the congestion
    /// loop, without `-allow_congestion`, is error GRT-0116 — the reference stops, so a run that
    /// reaches it is an ERROR (as the reference's is), never a refusal of something unmodelled.
    #[test]
    fn global_route_overflow_is_the_references_error() {
        assert_eq!(global_route_verdict(0), Ok(()));
        let e = global_route_verdict(3).unwrap_err();
        assert!(e.starts_with("GRT-0116: Global routing finished with congestion"), "{e}");
        assert!(!e.contains("not modelled"), "an error, not a refusal: {e}");
    }

    #[test]
    fn the_descriptor_is_valid_json_and_names_the_engine() {
        let d: Value = serde_json::from_str(&describe()).expect("--describe must be valid JSON");
        assert_eq!(d["name"], "rsz");
        assert_eq!(d["assertion"]["pass_when"]["in"], serde_json::json!(["repaired", "up_to_date"]));
    }

    // ⛔ `maturity` has exactly THREE legal values — discovered, structured, workflow-validated —
    // and the parser maps anything else to `discovered`, where the verdict is suppressed to
    // `unknown`. This engine said `experimental`, which threw every verdict away.
    #[test]
    fn the_maturity_is_a_legal_rung() {
        let d: Value = serde_json::from_str(&describe()).expect("--describe must be valid JSON");
        let m = d["maturity"].as_str().expect("maturity is a string");
        assert!(["discovered", "structured", "workflow-validated"].contains(&m), "{m}");
        assert_eq!(m, "structured");
    }

    /// One port delay as (port, clock, delay[rf][min/max]).
    type DelayRow = (String, usize, [[f32; 2]; 2]);

    /// The port delays [`search_sdc`] hands the timer: (inputs, outputs).
    fn port_delays_of(text: &str) -> (Vec<DelayRow>, Vec<DelayRow>) {
        let s = vyges_loom::sdc::Sdc::parse(text).unwrap();
        let ss = search_sdc(&s, 1e-9, false, false).unwrap();
        let rows = |l: &[vyges_sta::sdc::PortDelay]| l.iter().map(|d| (d.port.clone(), d.clock, d.delay)).collect::<Vec<_>>();
        (rows(&ss.input_delays), rows(&ss.output_delays))
    }

    const TWO_CLOCKS: &str = "create_clock -name a -period 10 [get_ports clk]\ncreate_clock -name v -period 5\n";

    /// Rules (`Sdc::setInputDelay` / `setOutputDelay`): a delay is keyed by port and clock edge.
    /// `-add_delay` on another clock adds a second delay; without it, the new delay deletes the
    /// port's delays on other clocks. Delays stay in the order they were made.
    #[test]
    fn io_delays_are_kept_per_port_and_clock() {
        let ns = |x: f64| vyges_sta::sdc::user_to_sta(x, 1e-9);
        let (i, o) = port_delays_of(&format!(
            "{TWO_CLOCKS}set_input_delay 1 -clock a [get_ports x]\nset_input_delay 2 -clock v -add_delay [get_ports x]\n\
             set_output_delay 3 -clock v [get_ports y]\nset_output_delay 4 -clock a [get_ports y]\n"
        ));
        assert_eq!(i, [("x".into(), 0, [[ns(1.0); 2]; 2]), ("x".into(), 1, [[ns(2.0); 2]; 2])]);
        assert_eq!(o, [("y".into(), 0, [[ns(4.0); 2]; 2])], "without -add_delay the clock a delay replaces the clock v one");
    }

    /// Rule (`RiseFallMinMax::mergeValue`): `-add_delay` on the same clock merges — a max kept if
    /// larger, a min if smaller; without it the named values are overwritten.
    #[test]
    fn add_delay_on_the_same_clock_merges_by_min_max() {
        let ns = |x: f64| vyges_sta::sdc::user_to_sta(x, 1e-9);
        let (i, _) = port_delays_of(&format!(
            "{TWO_CLOCKS}set_input_delay 2 -clock a [get_ports x]\nset_input_delay 1 -clock a -add_delay [get_ports x]\n\
             set_input_delay 3 -clock a -add_delay [get_ports x]\n\
             set_input_delay 5 -clock a [get_ports z]\nset_input_delay 1 -clock a -max [get_ports z]\n"
        ));
        // x: max 2 → 2 (1 not larger) → 3; min 2 → 1 → 1 (3 not smaller).
        assert_eq!(i[0], ("x".into(), 0, [[ns(1.0), ns(3.0)], [ns(1.0), ns(3.0)]]));
        // z: the -max overwrite keeps the min.
        assert_eq!(i[1], ("z".into(), 0, [[ns(5.0), ns(1.0)], [ns(5.0), ns(1.0)]]));
    }

    /// The oracle (probe `sdc_io2_replace`): the reference read these hand-written delays and its
    /// `write_sdc` gave back the second text. Read both ways, they hand the timer the same delays.
    #[test]
    fn hand_written_replace_and_merge_read_as_the_reference_wrote_them() {
        let clocks = "create_clock -name clk -period 0.1 [get_ports clk]\ncreate_clock -name vclk -period 0.1 -waveform {0.08 0.09}\n";
        let hand = format!(
            "{clocks}set_input_delay -clock clk 0.02 [get_ports a1]\nset_output_delay -clock clk 0.01 [get_ports y2]\n\
             set_output_delay -clock vclk 0.0 [get_ports y2]\nset_output_delay -clock clk 0.012 [get_ports y2]\n\
             set_input_delay -clock clk 0.03 -add_delay [get_ports a1]\nset_input_delay -clock clk 0.01 -add_delay [get_ports a1]\n"
        );
        let written = format!(
            "{clocks}set_input_delay 0.0100 -clock [get_clocks {{clk}}] -min -add_delay [get_ports {{a1}}]\n\
             set_input_delay 0.0300 -clock [get_clocks {{clk}}] -max -add_delay [get_ports {{a1}}]\n\
             set_output_delay 0.0120 -clock [get_clocks {{clk}}] -add_delay [get_ports {{y2}}]\n"
        );
        assert_eq!(port_delays_of(&hand), port_delays_of(&written));
    }

    /// Rules (`set_capacitance_limit` → `parse_cell_port_pin_args`; `write_sdc` writes the design,
    /// port and pin limits): ports and pins are read — a port's value per port, the last command's
    /// — and a cell named bare (`find_cells_matching`) or any other form is refused.
    #[test]
    fn object_cap_limits_read_ports_and_pins_and_refuse_the_rest() {
        let s = vyges_loom::sdc::Sdc::parse(
            "set_max_capacitance 2 [get_ports {a b}]\nset_max_capacitance 3 [get_ports {b}]\nset_max_capacitance 4 [get_pins {u1/A}]\n",
        )
        .unwrap();
        assert!(sdc_cap_objects_modelled(&s).is_ok());
        let ports = sdc_port_cap_limits(&s, 1e-15);
        let cap = |v: f64| vyges_rsz::timing::user_cap_to_sta(v, 1e-15);
        assert_eq!(ports.iter().map(|(p, c)| (p.as_str(), *c)).collect::<Vec<_>>(), [("a", cap(2.0)), ("b", cap(3.0))]);
        assert_eq!(sdc_cap_pins(&s).into_iter().collect::<Vec<_>>(), ["u1/A"]);
        for refused in ["set_max_capacitance 2 BUF_X1\n", "set_max_capacitance 2 [get_cells u1]\n", "set_max_capacitance 2 [get_lib_cells BUF_X1]\n"] {
            let s = vyges_loom::sdc::Sdc::parse(refused).unwrap();
            assert!(sdc_cap_objects_modelled(&s).is_err(), "{refused}");
        }
    }

    // Rule: only `repaired` exits 0; a refusal is never a pass.
    #[test]
    fn only_repaired_exits_zero() {
        assert_eq!(exit_for("repaired"), 0);
        assert_eq!(exit_for("up_to_date"), 0);
        assert_eq!(exit_for("refused"), 3);
        assert_eq!(exit_for("vacuous"), 2);
        assert_eq!(exit_for("error"), 2);
    }

    // Rule (get_lib_cells' PatternMatch): `*` any run, `?` one character, the rest literal.
    #[test]
    fn dont_use_patterns_are_globs() {
        assert!(glob_match("sky130_fd_sc_hd__probe_p_*", "sky130_fd_sc_hd__probe_p_8"));
        assert!(!glob_match("sky130_fd_sc_hd__probe_p_*", "sky130_fd_sc_hd__probec_p_8"));
        assert!(glob_match("BUF_X?", "BUF_X4") && !glob_match("BUF_X?", "BUF_X16"));
        assert!(glob_match("*", "") && glob_match("BUF_X1", "BUF_X1") && !glob_match("BUF_X1", "BUF_X16"));
    }

    // Rule (Resizer::clampLocToCore): inside the core with room for the master; a master wider
    // than the core at the core's lower edge; no core, no clamp.
    #[test]
    fn a_repeater_is_clamped_into_the_core() {
        let core = Some((100, 200, 1100, 1200));
        assert_eq!(clamp_loc_to_core(core, (50, 50), (0, 5000)), (100, 1150));
        assert_eq!(clamp_loc_to_core(core, (50, 50), (500, 500)), (500, 500));
        assert_eq!(clamp_loc_to_core(core, (2000, 50), (900, 500)), (100, 500));
        assert_eq!(clamp_loc_to_core(None, (50, 50), (-7, 9)), (-7, 9));
    }

    // Rule (the command's argument parsing): -buffer_gain only sets pre_placement; -max_wire_length is microns.
    #[test]
    fn repair_design_arguments() {
        let a = parse_args(&["-max_wire_length".into(), "800".into(), "-buffer_gain".into(), "4".into()]).unwrap();
        assert_eq!((a.max_wire_length, a.pre_placement), (800.0 * f64::from(1e-6f32), true));
        assert!(parse_args(&["-reroute".into()]).unwrap_err().contains("not modelled"));
    }
}
