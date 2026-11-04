// SPDX-License-Identifier: Apache-2.0
//! The SDC design-rule limits are applied by `repair_design`: each one, read from the job's SDC,
//! changes what the repair finds on a design that is clean without it, and the same job with the
//! command left out finds nothing. SDC: `set_max_capacitance value object_list`, `set_max_fanout
//! value object_list` and `set_max_transition value object_list` on `[current_design]` set the
//! limit for every net of the design, on top of the library's own (the tighter one holds).
#![cfg(feature = "cli")]

use std::path::{Path, PathBuf};
use std::process::Command;

/// One driver with three loads a few microns away: no library limit is near.
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

/// `repair_design` on the design with `sdc` as its constraints: the report's violation counts
/// (capacitance, fanout, slew) and the buffers inserted.
fn repair(name: &str, sdc: &str) -> (u64, u64, u64, u64) {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("sdc_limits").join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::copy(example().join("cells.lef"), dir.join("cells.lef")).unwrap();
    // each input pin counts one fanout load (a library without the attribute counts none)
    let lib = std::fs::read_to_string(example().join("cells.lib")).unwrap();
    let lib = lib.replacen("default_max_transition : 0.2 ;", "default_max_transition : 0.2 ;\n  default_fanout_load : 1 ;", 1);
    assert!(lib.contains("default_fanout_load"), "the library header changed");
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
    assert_eq!(out.status.code(), Some(0), "{name}: {}", String::from_utf8_lossy(&out.stderr));
    let r: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(dir.join("report.json")).unwrap()).unwrap();
    let n = |v: &serde_json::Value| v.as_u64().unwrap_or_else(|| panic!("{name}: {r}"));
    let v = &r["violations"];
    (n(&v["capacitance"]), n(&v["fanout"]), n(&v["slew"]), n(&r["inserted_buffers"]))
}

/// The same job with the limit left out (its own directory: the tests run in parallel).
fn without(name: &str) -> (u64, u64, u64, u64) {
    let r = repair(&format!("{name}_without"), "");
    assert_eq!(r, (0, 0, 0, 0), "the design is clean without an SDC limit");
    r
}

/// `set_max_capacitance 0.001 [current_design]`: every driver's load is over 1 fF, so each net is
/// a capacitance violation; without the command, none is.
#[test]
fn sdc_max_capacitance_is_applied_by_repair_design() {
    let (cap, ..) = repair("max_cap", "set_max_capacitance 0.001 [current_design]\n");
    assert!(cap > 0, "the SDC capacitance limit found no violation");
    without("max_cap");
}

/// `set_max_fanout 2 [current_design]`: the driver of three loads (one fanout load each) is a
/// fanout violation; without the command (the library sets no fanout limit), it is not.
#[test]
fn sdc_max_fanout_is_applied_by_repair_design() {
    let (_, fanout, ..) = repair("max_fanout", "set_max_fanout 2 [current_design]\n");
    assert!(fanout > 0, "the SDC fanout limit found no violation");
    without("max_fanout");
}

/// `set_max_transition 0.045 [current_design]`: slews here are over 45 ps (and a buffer can reach
/// it), so pins are slew violations; without the command (the library's 0.2 ns default holds),
/// none is.
#[test]
fn sdc_max_transition_is_applied_by_repair_design() {
    let (_, _, slew, _) = repair("max_transition", "set_max_transition 0.045 [current_design]\n");
    assert!(slew > 0, "the SDC transition limit found no violation");
    without("max_transition");
}
