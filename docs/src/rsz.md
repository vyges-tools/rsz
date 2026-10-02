# vyges-rsz — electrical repair of a placed design

> **Part of the Vyges Loom suite.** Install once with `vyges install loom`, then run
> `vyges loom rsz`. It's also a standalone `vyges-rsz` binary on your PATH (the
> integration contract for flow authors).

`vyges-rsz` answers: **after placement, which nets break an electrical rule, and what buffers or
driver sizes fix them?** A wire that is too long, a driver that sees more capacitance than its
cell allows, a net that fans out further than its limit, or a pin whose transition is slower than
its limit all make a design unsafe to time and route. The repair walks each driver's net, decides
where a repeater belongs and which size it takes, and edits the design database in place.

What the repair does, in the order it is decided:

| stage | decides |
| --- | --- |
| **the libraries** | the buffers it may insert (one footprint class, no clock buffers, not `dont_use`), each cell's equivalent sizes, each cell's target load at the target slews, the slew shape factor |
| **the forward pass** | each load whose transition already breaks its limit is held at the limit, so the excess is repaired where it is and does not reach later stages |
| **the driver order** | every driver, levelized once, repaired from the last level back to the first |
| **fanout** | a net over its fanout limit: its loads cut into regions, each region buffered by the weakest buffer at the load nearest the driver, and each new buffer's net repaired in turn |
| **the driver's slew** | a driver over its slew limit: the smallest equivalent size that fits, else the size that violates least; still over, the load capacitance that would fit becomes the net's capacitance limit |
| **load slews and capacitance** | a load over its slew limit, or a driver over its capacitance limit |
| **the buffered net** | the net's Steiner tree, walked from the loads to the driver: a repeater wherever the wire's length, its capacitance or the Elmore slew it builds breaks the limit, sized to the load it then drives |

Every check runs in every timing corner; a violation repairs the net at the corner it names. The
timer underneath is `vyges-sta`, on parasitics estimated by `vyges-est` from the placement.

## Run it

```sh
vyges install loom                                  # one-time
vyges loom rsz repair_design job.json               # -> a JSON report
vyges loom rsz repair_design job.json -o report.json
```

A job is the list of commands a flow script would run, in order, with the arguments as the script
passed them, plus where to write the result:

```json
{
  "steps": [
    { "cmd": "define_corners", "args": ["slow", "fast"] },
    { "cmd": "read_liberty",   "args": ["-corner", "slow", "cells_slow.lib"] },
    { "cmd": "read_liberty",   "args": ["-corner", "fast", "cells_fast.lib"] },
    { "cmd": "read_db",        "args": ["placed.odb"] },
    { "cmd": "read_sdc",       "args": ["constraints.sdc"] },
    { "cmd": "set_wire_rc",    "args": ["-layer", "metal3"] },
    { "cmd": "estimate_parasitics", "args": ["-placement"] },
    { "cmd": "repair_design",  "args": ["-max_wire_length", "800", "-slew_margin", "10"] }
  ],
  "write_def": "repaired.def",
  "trace": "repair.trace"
}
```

The report counts what was checked and what changed, and carries the repair's own closing lines,
each with its message code:

```json
{ "tool": "vyges-rsz", "status": "repaired", "nets_checked": 123, "nets_repaired": 1,
  "inserted_buffers": 9, "resized": 5, "drivers_skipped": 2,
  "violations": { "slew": 1, "capacitance": 0, "fanout": 1, "length": 0 },
  "summary": [
    { "code": "RSZ-0034", "message": "Found 1 slew violations." },
    { "code": "RSZ-0035", "message": "Found 1 fanout violations." },
    { "code": "RSZ-0039", "message": "Resized 5 instances." },
    { "code": "RSZ-0038", "message": "Inserted 9 buffers in 1 nets." } ] }
```

`write_def` writes the design as the repair left it. `trace` writes one line per decision, in the
order the repair makes them — the checks of each driver, its Steiner tree and buffered net, each
point of the walk, each repeater with its location and size — so a run can be read step by step.

A complete job on synthetic files, with the outputs a correct build writes, is in
`examples/long_wire`: a 1.9 mm net between two buffers, repaired with five repeaters and two
resizes.

See the full [CLI reference](./reference/vyges-rsz.md) (generated from `--help`).

## Exit status

| code | status | meaning |
| --- | --- | --- |
| 0 | `repaired` | the design changed: buffers inserted or drivers resized |
| 0 | `up_to_date` | drivers were checked and none needed a change; `nets_checked` says how many |
| 2 | `vacuous` | no driver was checked. **Not a pass.** |
| 2 | `error` | usage, unreadable input, or an error the repair raises (an unreasonably small capacitance limit, a slew limit no buffer can meet) |
| 3 | `refused` | an input or an option this engine does not model; `reason` names it |

⛔ **`vacuous` is not success.** The declared assertion passes on `repaired` or `up_to_date` only.

## The rules, stage by stage

### The libraries

- **Buffers**: every buffer cell that is not `dont_use`, not a clock buffer, and fits the design's
  sites, grouped by drive strength; the weakest is the one fanout repair and wire repair start
  from.
- **Target slews**: per corner, the mean slew of every buffer driving ten times its own input
  capacitance; the corner with the largest is the **target-slew corner**, where every later sizing
  decision is timed.
- **Target loads**: per cell, the mean over its arcs of the load at which the arc reaches the
  target slew, by bisection to 1 %.
- **Equivalent sizes**: cells with the same ports, functions, sequential behaviour and power
  pins. A size more than four times larger, or leaking more than four times more, is not a swap.

### The checks

- **Slew**: per corner and transition, the limit of the pin's port (or the design's
  `set_max_transition`, whichever is tighter; an output's library default where the port sets
  none), less `-slew_margin`. Load slews take the running minimum of the limits seen.
- **Capacitance**: the driver's load — pin capacitance plus the wire's share of its reduced
  network — against the port's `max_capacitance`, less `-cap_margin`.
- **Fanout**: the loads' fanout load against the port's limit (or the design's `set_max_fanout`,
  or the library default); with no limit, the number of load pins against 50.
- A top-level port with `set_driving_cell` takes its limits and its drive resistance from the
  driving cell's output pin.

### The walk

The buffered net is the Steiner tree from the driver, built at the routing `alpha`, read from the
loads up:

- **A wire** carries its length's resistance and capacitance (horizontal and vertical weighted). A
  repeater goes in where the remaining length exceeds `-max_wire_length`, where the wire's
  capacitance plus the load beyond it exceeds the limit, or where the slew it builds — solved as a
  quadratic in its length — reaches the load's limit. It is placed 5 % short of that split length,
  measured from the load end, and the walk continues from it with the rest of the wire.
- **A junction** joins two branches; a repeater goes on the side with less slack or more
  capacitance or more wire.
- **A repeater** starts as the size chosen for the load at that point, then is resized to the load
  it actually drives, timed at the target-slew corner: of its equivalent sizes, one that is faster
  with a target load within 10 % as close, or closer with a delay within 10 %. Its input becomes the
  load the walk carries upward.

### Fanout regions

The loads' bounding box is cut in two along its longer side until no region holds more than the
limit; from the leaves up, each full group of loads, and each leftover group of at least half the
limit, is buffered at the load nearest the driver, and that buffer's net is repaired with the
fanout check off.

## buffer_ports

A `buffer_ports` step puts a buffer after each input port and before each output port, so every
port is driven by, or drives, one known cell (`vyges loom rsz buffer_ports job.json` runs the same
job runner). Ports are walked in the database's order. A port is left alone when its net is
dont-touch, special or has no instance pins, when an input is a clock source or its loads are all
buffers (or one is dont-touch), and when an output's driver is tristate or dont-touch. The buffer
is the weakest one the repair would choose, or `-buffer_cell`.

Run after `estimate_parasitics`, it keeps the estimate as an incremental estimator would: the two
nets each insertion touches are estimated again when the walk ends, and every other net keeps its
estimate. A later `repair_design` starts from that state. Each estimate is reduced against the
port loads in force when it was made. Give a step the constraints it saw, as `"sdc"`, when a
port's `set_load` comes later. That later load is then compared against the earlier estimate, and
an estimate smaller than the load is set aside (the driver is timed against the load alone).

## Correlation

Scored against the reference implementation of the same command on its 35 regression cases, at
the build pin `--describe` publishes, three ways: the call-sequence trace line for line, the
repaired design (every component, pin and net), and the repair's closing summary lines.

- **35 of 35** cases match on all three, across Nangate45 and sky130, one and two corners, flat
  and hierarchical netlists, with up to 84 repeaters in a run.
- Two of them run `buffer_ports` between the parasitic estimate and the repair. Its stage is
  scored on its own as well: the design it leaves, and its closing lines.

⚠️ **A number here means nothing without the build.** The reference's own answer moves between
releases; the pin is part of the claim.

## What it refuses

A refusal is named in `reason`. Nothing is approximated:

- global-route or detailed-route parasitics; only `estimate_parasitics -placement` is modelled
- `-pre_placement` / `-buffer_gain` (the early sizing round), `-match_cell_footprint`, `-reroute`,
  `-max_utilization`
- a netlist edited between `estimate_parasitics` and `repair_design` by anything but a
  `buffer_ports` step (a cell MOVED in between is modelled: give the estimate step the database
  it saw, as `"db"`)
- `buffer_ports` on a hierarchical design, or with `-max_utilization`
- a tristate driver or a bidirect pin on a net
- liberty `bus` and `bundle` pins; `ff_bank`, `latch_bank` and statetable cells where their
  equivalence decides a swap
- any timing-affecting SDC command beyond those the [CLI reference](./reference/vyges-rsz.md)
  lists, `set_load -wire_load`, and library-qualified `set_dont_use` patterns

Not reported: the advisory warning that `-max_wire_length` is shorter than the length at which a
buffer starts to pay for itself. It does not change the repair.

## Where it sits

`rsz` reads the design database, the timing libraries and the constraints, times the design with
`vyges-sta` on `vyges-est`'s placement parasitics, builds Steiner trees with `vyges-stt`, and edits
the database through `vyges-opendb`. It runs after global placement and before clock-tree
synthesis and detailed placement, which legalize the repeaters it adds.

## Licence

Apache-2.0.
