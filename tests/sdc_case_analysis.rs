// SPDX-License-Identifier: Apache-2.0
//! The SDC's constraint values seed the logic simulation every command reads through
//! `Sta::isConstant`: `set_case_analysis` / `set_logic_*` on a port make it, and what it alone
//! decides, constant. A constant driver's net is not repaired (`RepairDesign::repairNet`: "Exclude
//! tie hi/low cells and supply nets", `!sta_->isConstant(drvr_pin)`), and a constant port gets no
//! buffer (`Resizer::bufferInputs` / `bufferOutputs`, `!sta_->isConstant(pin)`). The forms the
//! timer does not model (a `rise` / `fall` case value) refuse at `read_sdc`.
#![cfg(feature = "cli")]

use std::path::{Path, PathBuf};
use std::process::Command;

/// `in1` drives `drv`, whose output fans out to three buffers: a fanout-3 net behind the port.
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

/// One job (`cmd`: its last step) on the design with `sdc` as its constraints: the exit code
/// and the report.
fn run(name: &str, sdc: &str, cmd: &str) -> (Option<i32>, serde_json::Value) {
    run_def(name, DEF, sdc, cmd)
}

/// [`run`] on another design.
fn run_def(name: &str, def: &str, sdc: &str, cmd: &str) -> (Option<i32>, serde_json::Value) {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("sdc_case_analysis").join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::copy(example().join("cells.lef"), dir.join("cells.lef")).unwrap();
    let lib = std::fs::read_to_string(example().join("cells.lib")).unwrap();
    let lib = lib.replacen("default_max_transition : 0.2 ;", "default_max_transition : 0.2 ;\n  default_fanout_load : 1 ;", 1);
    std::fs::write(dir.join("cells.lib"), lib).unwrap();
    std::fs::write(dir.join("d.def"), def).unwrap();
    std::fs::write(dir.join("c.sdc"), sdc).unwrap();
    let last = match cmd {
        "repair_design" => r#"{ "cmd": "set_wire_rc", "args": ["-resistance", "0.0008", "-capacitance", "0.0002"] },
        { "cmd": "estimate_parasitics", "args": ["-placement"] },
        { "cmd": "repair_design", "args": [] }"#,
        _ => r#"{ "cmd": "buffer_ports", "args": [] }"#,
    };
    let job = format!(
        r#"{{ "steps": [
        {{ "cmd": "read_lef", "args": ["cells.lef"] }},
        {{ "cmd": "read_def", "args": ["d.def"] }},
        {{ "cmd": "read_liberty", "args": ["cells.lib"] }},
        {{ "cmd": "read_sdc", "args": ["c.sdc"] }},
        {last} ] }}"#
    );
    std::fs::write(dir.join("job.json"), job).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-rsz")).args([cmd, "job.json", "-o", "report.json"]).current_dir(&dir).output().unwrap();
    let r = std::fs::read_to_string(dir.join("report.json")).ok().and_then(|t| serde_json::from_str(&t).ok()).unwrap_or(serde_json::Value::Null);
    (out.status.code(), r)
}

/// Rule (`RepairDesign::repairNet` asks `!sta_->isConstant(drvr_pin)`; `Sim` propagates a case
/// value through the buffer): with `in1` held at 0, `drv/Z` is constant and its fanout-3 net is
/// passed over; without the case value the same net is a fanout violation.
#[test]
fn a_case_value_makes_the_driven_net_constant_for_repair_design() {
    let limit = "set_max_fanout 2 [current_design]\n";
    let (code, r) = run("rd_free", limit, "repair_design");
    assert_eq!(code, Some(0), "{r}");
    assert!(r["violations"]["fanout"].as_u64().unwrap() > 0, "{r}");
    let (code, r) = run("rd_case", &format!("{limit}set_case_analysis 0 [get_ports {{in1}}]\n"), "repair_design");
    assert!(matches!(code, Some(0) | Some(2)), "{r}");
    assert_eq!((r["violations"]["fanout"].as_u64(), r["inserted_buffers"].as_u64()), (Some(0), Some(0)), "{r}");
    // `set_logic_one` seeds the simulation the same way
    let (_, r) = run("rd_logic", &format!("{limit}set_logic_one [get_ports {{in1}}]\n"), "repair_design");
    assert_eq!(r["violations"]["fanout"].as_u64(), Some(0), "{r}");
}

/// Rule (`Resizer::bufferOutputs`: `!sta_->isConstant(pin)` on the port pin): `out1` is driven
/// through two buffers from `in1`, so a case value on `in1` makes it constant and it gets no
/// buffer; without one it does. (`in1` itself is never buffered here: its loads are all buffers.)
#[test]
fn a_constant_port_gets_no_buffer() {
    let inserted = |r: &serde_json::Value| r["buffer_ports"][0]["inserted_outputs"].as_u64().unwrap_or_else(|| panic!("{r}"));
    let (code, free) = run("bp_free", "", "buffer_ports");
    assert_eq!(code, Some(0), "{free}");
    let (code, held) = run("bp_case", "set_case_analysis 1 [get_ports {in1}]\n", "buffer_ports");
    assert_eq!(code, Some(0), "{held}");
    assert_eq!((inserted(&free), inserted(&held)), (1, 0), "free {free}\nheld {held}");
}

/// Rule (`Resizer::bufferInputs`: `!dontTouch(net) && !sta_->isConstant(pin) && !isClock(pin)
/// ...`): an input port the SDC holds constant gets no buffer. Here `in1`'s net also reaches a
/// second output port — another port counts as a non-buffer load — so without a case value it is
/// buffered.
#[test]
fn a_constant_input_port_gets_no_buffer() {
    let def = DEF
        .replace("PINS 2 ;", "PINS 3 ;\n    - out2 + NET in1 + DIRECTION OUTPUT + USE SIGNAL\n      + LAYER M1 ( -100 -100 ) ( 100 100 ) + PLACED ( 199800 9400 ) N ;")
        .replace("- in1 ( PIN in1 ) ( drv A )", "- in1 ( PIN in1 ) ( drv A ) ( PIN out2 )");
    let inserted = |r: &serde_json::Value| r["buffer_ports"][0]["inserted_inputs"].as_u64().unwrap_or_else(|| panic!("{r}"));
    let (code, free) = run_def("bpi_free", &def, "", "buffer_ports");
    assert_eq!(code, Some(0), "{free}");
    let (code, held) = run_def("bpi_case", &def, "set_case_analysis 0 [get_ports {in1}]\n", "buffer_ports");
    assert_eq!(code, Some(0), "{held}");
    assert_eq!((inserted(&free), inserted(&held)), (1, 0), "free {free}\nheld {held}");
}

/// A `rise` / `fall` case value limits the transitions the search takes, which this timer does not
/// model: refused when the constraints are read.
#[test]
fn a_transition_case_value_is_refused() {
    let (code, r) = run("bp_rise", "set_case_analysis rise [get_ports {in1}]\n", "buffer_ports");
    assert_ne!(code, Some(0), "{r}");
    assert!(r["reason"].as_str().unwrap_or_default().contains("transition value"), "{r}");
}
