![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Sequential 2x2 Matrix Solver ASIC (TinyTapeout, SKY130)

A tiny hardware solver for 2x2 linear systems (Ax = b), designed in Verilog for the [Tiny Tapeout](https://tinytapeout.com) SKY130 flow. Give it a signed 4-bit matrix and vector, and it returns the solution in Q4.4 fixed point, with singular-matrix detection and overflow saturation handled in hardware.

Status: RTL and gate-level cocotb tests passing, hardened to GDS through the OpenLane-based TinyTapeout toolchain (1x1 tile, 50 MHz target). Designed for a SKY130 shuttle; not submitted for fabrication.

## What it does

Solves:

```
| a  b |   | x0 |   | e |
| c  d | * | x1 | = | f |
```

using Cramer's rule:

```
det = a*d - b*c
x0  = (d*e - b*f) / det
x1  = (a*f - c*e) / det
```

Inputs are signed 4-bit integers. Results are 8-bit Q4.4 fixed point (4 integer bits, 4 fractional bits), saturated to the int8 range on overflow.

## Architecture

The design is built around doing a lot with very little area:

- One shared signed 4x4 multiplier. A small mux time-multiplexes it across all four products (a*d, b*c, d*e - b*f numerators, a*f - c*e numerators) under FSM control, instead of instantiating four multipliers.
- Sequential restoring divider (13-bit dividend / 9-bit divisor), one quotient bit per cycle, run twice (once per unknown). This avoids a large combinational divider.
- 12-state control FSM sequencing load, determinant, numerators, the two division passes, and result latch.
- Hardware edge cases: if det == 0 the `singular` flag is raised and outputs are zeroed; if a quotient exceeds int8 range the result saturates to +127/-128 and `overflow` is raised. `valid` signals result-ready.

## Pinout

| Pin | Direction | Function |
|---|---|---|
| ui[3:0] | in | 4-bit signed data stream (a, b, c, d, e, f loaded in sequence) |
| uo[7:0] | out | 8-bit Q4.4 result (x0 or x1, chosen by `sel`) |
| uio[0] | in/out | `start` (in): pulse to begin load; `valid` (out): result ready |
| uio[1] | in/out | `sel` (in): 0 = x0, 1 = x1; `singular` (out): det == 0 |
| uio[2] | out | `overflow`: result saturated |

## Operation

1. Pulse `start` high with `a` on ui[3:0].
2. Feed `b`, `c`, `d`, `e`, `f` on the following clock cycles.
3. Wait for `valid` to go high.
4. Read x0 on uo[7:0], set `sel` high to read x1.
5. Check `singular` and `overflow` flags.

## Verification

- cocotb testbench driving the full load/solve/read sequence
- Passing at RTL and at gate level (GL_TEST with SKY130 power pins)
- Waveforms dumped to FST for inspection in GTKWave/Surfer

## Files

- `project.v` - top module `tt_um_solve2x2` and the `signed_div13by9` divider
- `info.yaml` - TinyTapeout project metadata and pinout
- `test/` - cocotb testbench

## Author

Azam Mohamed, UC Santa Cruz Computer Engineering. Built for CSE 122 (VLSI Design).

## Use of GenAI

This project was developed with assistance from Gemini 3 Flash, which was used to optimize the RTL from a parallel design to a sequential accumulator architecture, debug signed-arithmetic promotion issues, fix FSM race conditions, refactor the cocotb testbench for variable-latency handshaking, and analyze synthesis logs to remove redundant register bits.
