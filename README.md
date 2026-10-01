# vyges-rsz

Electrical repair of a placed design: repeaters inserted along each net's Steiner tree where a
wire is too long, a driver sees too much capacitance or fans out too far, or a load's transition
is too slow — and a driver resized where its own transition is.

```sh
vyges install loom
vyges loom rsz repair_design job.json
vyges loom rsz --describe
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
