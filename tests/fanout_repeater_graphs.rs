// SPDX-License-Identifier: Apache-2.0
//! A fanout repeater's own net is repaired on the graphs the outer net's repair holds.
//!
//! Reference rule (OpenROAD 4155ac31 `RepairDesign::makeFanoutRepeater`): after `makeRepeater`
//! the new buffer's net is repaired at once with `repairNet(out_net, …)`, nested inside the outer
//! net's `makeRegionRepeaters`, on the session's one `sta::Graph` and `GraphDelayCalc`: the
//! netlist-edit callbacks have already patched that graph, and no second graph is made. Here the
//! timer's graph sets are counted (`VYGES_RSZ_EDIT_STATS`): the one set built for the driver loop
//! is the only one, every later bring-up an edit in place.
#![cfg(feature = "cli")]

use std::path::Path;
use std::process::Command;

/// `drv` drives twelve buffers in two rows: a fanout-12 net under a fanout limit of 4. The rows
/// are 7 microns apart: `subdivideRegion` cuts only a box wider and taller than a micron.
fn def() -> String {
    let loads: Vec<String> = (0..12).map(|k| format!("    - l{k} BUF_X1 + PLACED ( {} {} ) N ;\n", 40000 + 8000 * (k / 2), 14000 * (k % 2))).collect();
    let pins: String = (0..12).map(|k| format!(" ( l{k} A )")).collect();
    format!(
        "VERSION 5.8 ;
DIVIDERCHAR \"/\" ;
BUSBITCHARS \"[]\" ;
DESIGN fanout12 ;
UNITS DISTANCE MICRONS 2000 ;
DIEAREA ( 0 0 ) ( 200000 28000 ) ;
ROW ROW_0 core 0 0 N DO 500 BY 1 STEP 400 0 ;
COMPONENTS 13 ;
    - drv BUF_X1 + PLACED ( 20000 0 ) N ;
{}END COMPONENTS
PINS 2 ;
    - in1 + NET in1 + DIRECTION INPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 200 1400 ) N ;
    - out1 + NET out1 + DIRECTION OUTPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 199800 1400 ) N ;
END PINS
NETS 3 ;
    - in1 ( PIN in1 ) ( drv A ) + USE SIGNAL ;
    - n1 ( drv Z ){pins} + USE SIGNAL ;
    - out1 ( l0 Z ) ( PIN out1 ) + USE SIGNAL ;
END NETS
END DESIGN
",
        loads.concat()
    )
}

/// The `VYGEDIT|stats|…` counts: (in place, fallback, built).
fn stats(stderr: &str) -> (u64, u64, u64) {
    let line = stderr.lines().find(|l| l.starts_with("VYGEDIT|stats|")).unwrap_or_else(|| panic!("no stats line:\n{stderr}"));
    let n = |key: &str| -> u64 { line.split('|').find_map(|f| f.strip_prefix(key)).and_then(|v| v.trim().parse().ok()).unwrap_or_else(|| panic!("{key} in {line}")) };
    (n("in_place "), n("fallback "), n("built "))
}

#[test]
fn a_fanout_repeaters_net_is_repaired_on_the_outer_graphs() {
    let example = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/long_wire");
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("fanout_repeater_graphs");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::copy(example.join("cells.lef"), dir.join("cells.lef")).unwrap();
    let lib = std::fs::read_to_string(example.join("cells.lib")).unwrap();
    let lib = lib.replacen("default_max_transition : 0.2 ;", "default_max_transition : 0.2 ;\n  default_fanout_load : 1 ;", 1);
    std::fs::write(dir.join("cells.lib"), lib).unwrap();
    std::fs::write(dir.join("d.def"), def()).unwrap();
    std::fs::write(dir.join("c.sdc"), "set_max_fanout 4 [current_design]\n").unwrap();
    let job = r#"{ "steps": [
        { "cmd": "read_lef", "args": ["cells.lef"] },
        { "cmd": "read_def", "args": ["d.def"] },
        { "cmd": "read_liberty", "args": ["cells.lib"] },
        { "cmd": "read_sdc", "args": ["c.sdc"] },
        { "cmd": "set_wire_rc", "args": ["-resistance", "0.0008", "-capacitance", "0.0002"] },
        { "cmd": "estimate_parasitics", "args": ["-placement"] },
        { "cmd": "repair_design", "args": [] } ] }"#;
    std::fs::write(dir.join("job.json"), job).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-rsz"))
        .args(["repair_design", "job.json", "-o", "report.json"])
        .env("VYGES_RSZ_EDIT_STATS", "1")
        .env_remove("VYGES_RSZ_GRAPH_REBUILD")
        .env_remove("VYGES_RSZ_DCALC")
        .current_dir(&dir)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&out.stderr);
    let report = std::fs::read_to_string(dir.join("report.json")).unwrap_or_default();
    assert_eq!(out.status.code(), Some(0), "{stderr}\n{report}");
    let r: serde_json::Value = serde_json::from_str(&report).unwrap();
    // the fanout repair ran and made repeaters, each of whose nets was then repaired
    assert!(r["violations"]["fanout"].as_u64().unwrap_or(0) > 0, "{r}");
    assert!(r["inserted_buffers"].as_u64().unwrap_or(0) >= 2, "{r}");
    let (in_place, fallback, built) = stats(&stderr);
    assert_eq!((built, fallback), (1, 0), "one graph set, the driver loop's; in place {in_place}\n{stderr}");
    assert!(in_place >= 2, "every repeater's net on the outer graphs, edited in place: {in_place}");
}
