// SPDX-License-Identifier: Apache-2.0
//! `repair_clock_nets` times its drivers on the incremental timer: one graph set, timed whole
//! once, then each edit taken in place — and the repair is the one whole re-timing makes.
//!
//! Reference rule (OpenROAD `RepairDesign::repairClkNets`): `findDelays()` once, then per clock
//! network driver `repairNet` on the session's one `sta::Graph`, whose `GraphDelayCalc` takes each
//! inserted buffer through the network-edit callbacks; no graph is built again per edit. The
//! timer's graph sets are counted (`VYGES_RSZ_EDIT_STATS`), and the result is checked against the
//! whole re-timing per driver (`VYGES_RSZ_DCALC`).
#![cfg(feature = "cli")]

use std::path::Path;
use std::process::Command;

/// The `VYGEDIT|stats|…` counts: (in place, fallback, built).
fn stats(stderr: &str) -> (u64, u64, u64) {
    let line = stderr.lines().find(|l| l.starts_with("VYGEDIT|stats|")).unwrap_or_else(|| panic!("no stats line:\n{stderr}"));
    let n = |key: &str| -> u64 { line.split('|').find_map(|f| f.strip_prefix(key)).and_then(|v| v.trim().parse().ok()).unwrap_or_else(|| panic!("{key} in {line}")) };
    (n("in_place "), n("fallback "), n("built "))
}

/// One run in its own directory: (exit code, stderr, report, written DEF).
fn run(name: &str, whole: bool) -> (Option<i32>, String, String, String) {
    let example = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/long_wire");
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    for f in ["cells.lef", "cells.lib", "long_wire.def"] {
        std::fs::copy(example.join(f), dir.join(f)).unwrap();
    }
    // the 1.9 mm wire is a clock net: `in1` is the clock's source, `drv` and `load` its buffers
    std::fs::write(dir.join("c.sdc"), "create_clock -name clk -period 1 [get_ports in1]\n").unwrap();
    let job = r#"{ "steps": [
        { "cmd": "read_lef", "args": ["cells.lef"] },
        { "cmd": "read_def", "args": ["long_wire.def"] },
        { "cmd": "read_liberty", "args": ["cells.lib"] },
        { "cmd": "read_sdc", "args": ["c.sdc"] },
        { "cmd": "set_wire_rc", "args": ["-resistance", "0.0008", "-capacitance", "0.0002"] },
        { "cmd": "estimate_parasitics", "args": ["-placement"] },
        { "cmd": "clock_buffers", "args": ["BUF_X1", "BUF_X2", "BUF_X4"] },
        { "cmd": "repair_clock_nets", "args": ["-max_wire_length", "400"] } ],
      "write_def": "out.def" }"#;
    std::fs::write(dir.join("job.json"), job).unwrap();
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_vyges-rsz"));
    cmd.args(["repair_design", "job.json", "-o", "report.json"]).env("VYGES_RSZ_EDIT_STATS", "1").env_remove("VYGES_RSZ_GRAPH_REBUILD").current_dir(&dir);
    if whole {
        cmd.env("VYGES_RSZ_DCALC", "1");
    } else {
        cmd.env_remove("VYGES_RSZ_DCALC");
    }
    let out = cmd.output().unwrap();
    let read = |f: &str| std::fs::read_to_string(dir.join(f)).unwrap_or_default();
    (out.status.code(), String::from_utf8_lossy(&out.stderr).into_owned(), read("report.json"), read("out.def"))
}

#[test]
fn clock_nets_are_repaired_on_one_graph_set_edited_in_place() {
    let (rc, stderr, report, def) = run("clock_nets_incremental", false);
    assert_eq!(rc, Some(0), "{stderr}\n{report}");
    let r: serde_json::Value = serde_json::from_str(&report).unwrap();
    assert!(r["inserted_buffers"].as_u64().unwrap_or(0) >= 2, "the long clock wire was buffered: {r}");
    let (in_place, fallback, built) = stats(&stderr);
    assert_eq!((built, fallback), (1, 0), "one graph set, timed once; in place {in_place}\n{stderr}");
    assert!(in_place >= 1, "the drivers after an edit on the edited graphs: {in_place}");
    // the same repair as re-timing every graph whole per driver
    let (rc2, stderr2, _, def2) = run("clock_nets_whole", true);
    assert_eq!(rc2, Some(0), "{stderr2}");
    let body = |d: &str| d.lines().filter(|l| !l.starts_with('#')).collect::<Vec<_>>().join("\n");
    assert!(!def.is_empty() && body(&def) == body(&def2), "incremental and whole re-timing differ");
}
