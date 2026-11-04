# vyges-rsz

Electrical repair of a placed design: repeaters inserted along each net's Steiner tree where a
wire is too long, a driver sees too much capacitance or fans out too far, or a load's transition
is too slow — and a driver resized where its own transition is.

```sh
vyges install physical
vyges physical rsz repair_design job.json
vyges physical rsz --describe
```

Documentation: [`docs/src/rsz.md`](docs/src/rsz.md) (an mdBook; `mdbook build docs`), with the
[CLI reference](docs/src/reference/vyges-rsz.md).

## The job

The commands a flow script would run, in order, with the arguments as it passed them:

```json
{
  "steps": [
    { "cmd": "read_liberty",  "args": ["cells.lib"] },
    { "cmd": "read_db",       "args": ["placed.odb"] },
    { "cmd": "read_sdc",      "args": ["constraints.sdc"] },
    { "cmd": "set_wire_rc",   "args": ["-layer", "metal3"] },
    { "cmd": "estimate_parasitics", "args": ["-placement"] },
    { "cmd": "repair_design", "args": ["-max_wire_length", "800"] }
  ],
  "write_def": "repaired.def"
}
```

```json
{ "tool": "vyges-rsz", "status": "repaired", "nets_checked": 4, "nets_repaired": 1,
  "inserted_buffers": 3, "resized": 0, "drivers_skipped": 0,
  "violations": { "slew": 1, "capacitance": 0, "fanout": 0, "length": 1 },
  "summary": [ { "code": "RSZ-0034", "message": "Found 1 slew violations." }, … ] }
```

### SDC design-rule limits

`set_max_capacitance`, `set_max_fanout` and `set_max_transition` on `[current_design]` are read
from the job's SDC and checked by `repair_design` alongside the library's own limits (the
tighter one holds). Two cases where a limit cannot act as written:

- **`set_max_fanout` needs fanout loads.** Each load counts its pin's `fanout_load`, else its
  library's `default_fanout_load`, else 0. If no library read gives either, every count is 0 and
  the limit never acts. The report then carries an info note (also an event), and fanout is not
  checked:

  ```json
  "notes": [ { "code": "RSZ-FANOUT-LOAD", "severity": "info",
               "message": "set_max_fanout 2: no library read gives a fanout_load or a default_fanout_load, so every load counts 0 and the limit never acts" } ]
  ```

- **An unreachable `set_max_transition`.** A limit below the best transition a buffer can give a
  load stops `repair_design` with exit 2 and an error such as:

  ```text
  Max transition time for pin l1/A at corner default is 1e-12s. Best achievable transition time is 3.6939996e-11s with a load of 0e0F
  ```

  Raise the limit to one a buffer can reach.

A worked example — a 1.9 mm net between two buffers, repaired with five repeaters — is in
[`examples/long_wire`](examples/long_wire), with the report, design and trace it must produce.

## Build

The command-line engine reads the design database through `vyges-opendb`, which builds the
database library from source. Check out `vyges-tools/opendb`, `opendb-lib`, `sta`, `grt`, `est`,
`stt`, `loom` and `layout` beside this repository (see `.github/workflows/ci.yml`), then:

```sh
cargo build --release --features cli
cargo test  --release --features cli
```

Without `--features cli` the library builds with no C++ dependency, and the pure rules are tested
on their own.

Licensed under Apache-2.0.
