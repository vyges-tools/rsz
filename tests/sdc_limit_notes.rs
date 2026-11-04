// SPDX-License-Identifier: Apache-2.0
//! Two design-rule limits that cannot act as written, and what `repair_design` says about each:
//! a `set_max_fanout` with no fanout loads to count (an info note, not silence), and a
//! `set_max_transition` no buffer can reach (an error, exit 2).
#![cfg(feature = "cli")]

use std::path::{Path, PathBuf};
use std::process::Command;

/// One driver with three loads a few microns away.
const DEF: &str = "VERSION 5.8 ;
DIVIDERCHAR \"/\" ;
BUSBITCHARS \"[]\" ;
DESIGN fanout3 ;
UNITS DISTANCE MICRONS 2000 ;
DIEAREA ( 0 0 ) ( 200000 28000 ) ;
ROW ROW_0 core 0 0 N DO 500 BY 1 STEP 400 0 ;
COMPONENTS 4 ;
    - drv BUF_X1 + PLACED ( 20000 0 ) N ;
    - l1 BUF_X1 + PLACED ( 40000 0 ) N ;
    - l2 BUF_X1 + PLACED ( 60000 0 ) N ;
    - l3 BUF_X1 + PLACED ( 80000 0 ) N ;
END COMPONENTS
PINS 2 ;
    - in1 + NET in1 + DIRECTION INPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 200 1400 ) N ;
    - out1 + NET out1 + DIRECTION OUTPUT + USE SIGNAL
      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 199800 1400 ) N ;
END PINS
NETS 3 ;
    - in1 ( PIN in1 ) ( drv A ) + USE SIGNAL ;
    - n1 ( drv Z ) ( l1 A ) ( l2 A ) ( l3 A ) + USE SIGNAL ;
    - out1 ( l1 Z ) ( PIN out1 ) + USE SIGNAL ;
END NETS
END DESIGN
";

fn example() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/long_wire")
}

/// `repair_design` with `sdc`, on the example library with or without a `default_fanout_load`:
/// the exit code, the report (if written) and stderr.
fn repair(name: &str, sdc: &str, fanout_load: bool) -> (i32, serde_json::Value, String) {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("sdc_limit_notes").join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::copy(example().join("cells.lef"), dir.join("cells.lef")).unwrap();
    let mut lib = std::fs::read_to_string(example().join("cells.lib")).unwrap();
    assert!(!lib.contains("fanout_load"), "the example library now sets fanout loads");
    if fanout_load {
        lib = lib.replacen("default_max_transition : 0.2 ;", "default_max_transition : 0.2 ;\n  default_fanout_load : 1 ;", 1);
        assert!(lib.contains("default_fanout_load"), "the library header changed");
    }
    std::fs::write(dir.join("cells.lib"), lib).unwrap();
    std::fs::write(dir.join("d.def"), DEF).unwrap();
    std::fs::write(dir.join("c.sdc"), sdc).unwrap();
    let job = r#"{ "steps": [
        { "cmd": "read_lef", "args": ["cells.lef"] },
        { "cmd": "read_def", "args": ["d.def"] },
        { "cmd": "read_liberty", "args": ["cells.lib"] },
        { "cmd": "read_sdc", "args": ["c.sdc"] },
        { "cmd": "set_wire_rc", "args": ["-resistance", "0.0008", "-capacitance", "0.0002"] },
        { "cmd": "estimate_parasitics", "args": ["-placement"] },
        { "cmd": "repair_design", "args": [] } ] }"#;
    std::fs::write(dir.join("job.json"), job).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-rsz")).args(["repair_design", "job.json", "-o", "report.json"]).current_dir(&dir).output().unwrap();
    let r = std::fs::read_to_string(dir.join("report.json")).ok().and_then(|t| serde_json::from_str(&t).ok()).unwrap_or(serde_json::Value::Null);
    (out.status.code().unwrap_or(-1), r, String::from_utf8_lossy(&out.stderr).into_owned())
}

fn fanout_notes(r: &serde_json::Value) -> usize {
    r["notes"].as_array().map_or(0, |n| n.iter().filter(|n| n["code"] == "RSZ-FANOUT-LOAD").count())
}

/// A design `set_max_fanout` counts each load's `fanout_load`, else its library's
/// `default_fanout_load`, else 0. With neither in the library, every count is 0 and the limit
/// never acts; the report says so at info level, once. With a default, the limit acts and there
/// is no note.
#[test]
fn a_fanout_limit_with_no_fanout_loads_is_noted() {
    let (rc, r, err) = repair("no_loads", "set_max_fanout 2 [current_design]\n", false);
    assert_eq!(rc, 0, "{err}");
    assert_eq!(r["violations"]["fanout"], 0, "{r}");
    assert_eq!(fanout_notes(&r), 1, "{r}");
    let n = &r["notes"][0];
    assert_eq!(n["severity"], "info");
    assert!(n["message"].as_str().unwrap().contains("never acts"), "{n}");
    let (rc, r, err) = repair("with_loads", "set_max_fanout 2 [current_design]\n", true);
    assert_eq!(rc, 0, "{err}");
    assert!(r["violations"]["fanout"].as_u64() > Some(0), "{r}");
    assert_eq!(fanout_notes(&r), 0, "{r}");
    // no limit, no note
    let (_, r, _) = repair("no_limit", "", false);
    assert_eq!(fanout_notes(&r), 0, "{r}");
}

/// A `set_max_transition` below the best transition a buffer can give a load is not met by
/// repair: `repair_design` stops with an error (exit 2) naming the pin, the limit and the best
/// achievable transition.
#[test]
fn an_unreachable_transition_limit_stops_repair() {
    let (rc, _, err) = repair("unreachable", "set_max_transition 0.001 [current_design]\n", true);
    assert_eq!(rc, 2, "{err}");
    assert!(err.contains("Max transition time for pin") && err.contains("Best achievable transition time is"), "{err}");
}
