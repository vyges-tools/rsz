# vyges loom rsz — CLI reference

_Generated from `vyges loom rsz --help` — this page is the tool's own output, verbatim._

```text
vyges loom rsz — electrical repair of a placed design: repeaters along each net's Steiner tree,
drivers resized, where a wire is too long or a capacitance, fanout or transition limit is broken

USAGE:
  vyges loom rsz repair_design <job.json> [-o FILE]
  vyges loom rsz buffer_ports <job.json> [-o FILE]     (the same job runner; a job may hold either)
  vyges loom rsz --describe
  vyges loom rsz --help
  vyges loom rsz --version

JOB FIELDS:
  steps        required — the commands in order, each {"cmd": ..., "args": [...]}, as a flow
               script passes them:
                 read_lef, read_def, read_db, define_corners, read_liberty [-corner C],
                 read_sdc, set_dont_use, set_layer_rc, set_wire_rc, set_routing_alpha,
                 estimate_parasitics -placement, set_propagated_clock, buffer_ports [options],
                 repair_design [options], repair_timing [options]
               an estimate_parasitics step may carry "db": the database as the estimate saw it,
               when cells were moved between it and the repair
               a buffer_ports step may carry "write_def": the design as it left it, as DEF
               an estimate_parasitics or buffer_ports step may carry "sdc": the constraints in
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
  error. One clock, ideal or propagated, with its I/O delays. -setup and -hold together (or
  neither): the setup repair, then refused. Refused before the lines: -phases other than
  LEGACY, -recover_power, several corners, VT libraries, a latch, a virtual clock, clock
  uncertainty / latency / transition, derates, path exceptions; refused during the repair:
  -setup with -max_utilization, more than one repair per pass.

CONSTRAINTS READ FROM SDC:
  create_clock, set_max_transition and set_max_fanout on the design, set_load on nets and ports,
  set_input_transition, set_driving_cell; any other timing-affecting command is refused

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
```
