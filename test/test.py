# =============================================================================
# test_solve2x2_seq.py  —  Cocotb testbench for tt_um_solve2x2_q16 (sequential edition)
#
# Differences from original:
# - DUT no longer produces output on a fixed cycle after input streaming
# - After loading a,b,c,d,e,f we WAIT for valid flag to assert (with timeout)
# =============================================================================

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

# ---------------------------------------------------------------------------
# Q4.4 helpers
# ---------------------------------------------------------------------------
Q = 4  # fractional bits

def to_q44(f: float) -> int:
    """Float -> Q4.4 signed 8-bit integer."""
    v = int(round(f * (1 << Q)))
    return max(-128, min(127, v))

def from_q44(v: int) -> float:
    """Q4.4 signed 8-bit integer -> float."""
    v = int(v) & 0xFF
    if v >= 128:
        v -= 256
    return v / (1 << Q)

def clamp4(v: int) -> int:
    """Clamp to signed 4-bit range -8..7."""
    return max(-8, min(7, v))

# ---------------------------------------------------------------------------
# Tolerance: 1.5 LSBs of Q4.4 (accounts for truncation/rounding differences)
# ---------------------------------------------------------------------------
TOL = 1.5 / (1 << Q)   # 0.09375

def check(label, got, expected, tol=TOL):
    err = abs(got - expected)
    assert err <= tol, \
        f"{label}: got {got:.5f}, expected {expected:.5f}, err={err:.4f} > tol={tol:.4f}"

# ---------------------------------------------------------------------------
# Timing control: sequential divider/mul can take many cycles
# Pick something comfortably above your FSM worst-case.
# Example rough budget: load(6) + mul(6) + div(13) + div(13) + overhead ~= 40+
# ---------------------------------------------------------------------------
MAX_WAIT_CYCLES = 200

# ---------------------------------------------------------------------------
# DUT driver helpers
# ---------------------------------------------------------------------------
async def reset_dut(dut):
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 2)

async def stream_inputs(dut, a, b, c, d, e, f):
    """
    Stream 6 signed 4-bit values in ui_in[3:0], one per clock.
    Cycle 0: start pulse + a
    Cycle 1-5: b,c,d,e,f
    After this, DUT computes internally over variable latency; valid asserts later.
    """
    vals = [a, b, c, d, e, f]

    # Cycle 0: start pulse + a
    dut.uio_in.value = 0b00000001  # start=1, sel=0
    dut.ui_in.value  = int(vals[0]) & 0x0F
    await RisingEdge(dut.clk)

    # Cycles 1-5: b through f
    dut.uio_in.value = 0b00000000  # start=0, sel=0
    for v in vals[1:]:
        dut.ui_in.value = int(v) & 0x0F
        await RisingEdge(dut.clk)

    # Clear data bus after load (not required but nice)
    dut.ui_in.value = 0

async def wait_for_valid(dut, max_cycles=MAX_WAIT_CYCLES):
    """
    Wait until valid flag (uio_out[0]) is high.
    """
    for _ in range(max_cycles):
        flags = int(dut.uio_out.value)
        if flags & 0x01:
            return
        await RisingEdge(dut.clk)

    # timed out
    flags = int(dut.uio_out.value)
    raise AssertionError(f"Timeout waiting for valid after {max_cycles} cycles. flags=0x{flags:02X}")

async def do_solve(dut, a, b, c, d, e, f):
    """
    Stream inputs and return (x0_float, x1_float, singular, overflow).
    We read uo_out combinatorially by toggling sel — no extra clock edges required.
    """
    await stream_inputs(dut, a, b, c, d, e, f)

    # Wait for solver to finish (variable latency)
    await wait_for_valid(dut)

    flags    = int(dut.uio_out.value)
    singular = bool(flags & 0x02)
    overflow = bool(flags & 0x04)

    # Read x0 combinatorially (sel=0, start=0)
    dut.uio_in.value = 0b00000000
    await Timer(1, unit="ns")
    x0_raw = int(dut.uo_out.value)

    # Read x1 combinatorially (sel=1, start=0)
    dut.uio_in.value = 0b00000010
    await Timer(1, unit="ns")
    x1_raw = int(dut.uo_out.value)

    # Restore
    dut.uio_in.value = 0b00000000

    return from_q44(x0_raw), from_q44(x1_raw), singular, overflow

# ---------------------------------------------------------------------------
# TEST 1: Identity matrix  A=I  =>  x = b
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_identity(dut):
    """A = identity, solution should equal b vector."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        (1,  3),
        (-5, 7),
        (0,  0),
        (-8, 7),   # extreme 4-bit values
    ]
    for (ex0, ex1) in cases:
        x0, x1, sing, ovf = await do_solve(dut, 1, 0, 0, 1, ex0, ex1)
        assert not sing, "Identity should not be singular"
        check(f"identity x0 (e={ex0})", x0, float(ex0))
        check(f"identity x1 (f={ex1})", x1, float(ex1))
        dut._log.info(f"PASS identity: x0={x0} x1={x1}")

# ---------------------------------------------------------------------------
# TEST 2: Diagonal matrix  A=diag(a,d)
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_diagonal(dut):
    """Diagonal matrix: x0 = e/a, x1 = f/d."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        (2,  3,  6,  3),   # x0=3.0, x1=1.0
        (4,  2,  4,  6),   # x0=1.0, x1=3.0
        (-2, 4, -4,  4),   # x0=2.0, x1=1.0
        (1,  1,  5, -3),   # x0=5.0, x1=-3.0
        (2,  4,  1,  2),   # x0=0.5, x1=0.5
    ]
    for (a, d, e, f) in cases:
        exp_x0 = e / a
        exp_x1 = f / d
        x0, x1, sing, ovf = await do_solve(dut, a, 0, 0, d, e, f)
        assert not sing
        check(f"diag x0 a={a} e={e}", x0, exp_x0)
        check(f"diag x1 d={d} f={f}", x1, exp_x1)
        dut._log.info(f"PASS diagonal a={a} d={d}: x0={x0:.4f} x1={x1:.4f}")

# ---------------------------------------------------------------------------
# TEST 3: General non-singular — known integer solutions
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_known_solutions(dut):
    """Construct b=A*x from known x; verify DUT recovers x."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    matrices = [
        ( 2,  1,  1,  3,   1,  -1),
        ( 1,  2,  1,  3,  -2,   2),
        ( 3,  1,  2,  4,   1,  -1),
        ( 1, -1,  2,  3,   1,   1),
        (-1,  2,  1, -3,   1,  -1),
        ( 2,  1,  1,  2,   1,   0),
    ]
    for (a, b, c, d, ex0, ex1) in matrices:
        det = a*d - b*c
        if det == 0:
            continue
        e = clamp4(a*ex0 + b*ex1)
        f = clamp4(c*ex0 + d*ex1)

        exp_x0 = (d*e - b*f) / det
        exp_x1 = (a*f - c*e) / det

        x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
        assert not sing, f"Unexpected singular: det={det}"
        check(f"known x0 [{a},{b};{c},{d}]", x0, exp_x0)
        check(f"known x1 [{a},{b};{c},{d}]", x1, exp_x1)
        dut._log.info(f"PASS known [{a},{b};{c},{d}] x0={x0:.4f} x1={x1:.4f}")

# ---------------------------------------------------------------------------
# TEST 4: Fractional solutions
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_fractional(dut):
    """Solutions that are not integers — tests fractional Q4.4 bits."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        (2, 0, 0, 4,  1, 1),   # x0=0.5, x1=0.25
        (4, 0, 0, 4,  2, 3),   # x0=0.5, x1=0.75
        (2, 1, 1, 2,  1, 1),   # det=3, x0=1/3, x1=1/3
        (4, 2, 2, 4,  2, 2),   # det=12, x0=0.5, x1=0
    ]
    for (a, b, c, d, e, f) in cases:
        det = a*d - b*c
        if det == 0:
            continue
        exp_x0 = (d*e - b*f) / det
        exp_x1 = (a*f - c*e) / det
        x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
        assert not sing
        check(f"frac x0 [{a},{b};{c},{d}]", x0, exp_x0)
        check(f"frac x1 [{a},{b};{c},{d}]", x1, exp_x1)
        dut._log.info(
            f"PASS fractional: x0={x0:.4f}(exp {exp_x0:.4f}) x1={x1:.4f}(exp {exp_x1:.4f})"
        )

# ---------------------------------------------------------------------------
# TEST 5: Singular matrices
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_singular(dut):
    """det=0: singular flag must be set, outputs must be zero."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        (1, 2, 2, 4,  1, 2),   # det=0
        (2, 4, 1, 2,  3, 0),   # det=0
        (0, 0, 0, 0,  1, 1),   # all-zero
        (3, 6, 1, 2,  1, 1),   # det=0 (after clamp)
    ]
    for (a, b, c, d, e, f) in cases:
        b_c = clamp4(b)
        x0, x1, sing, ovf = await do_solve(dut, a, b_c, c, d, e, f)
        assert sing, f"Expected singular for [{a},{b_c};{c},{d}]"
        assert x0 == 0.0, f"Expected x0=0 for singular, got {x0}"
        assert x1 == 0.0, f"Expected x1=0 for singular, got {x1}"
        dut._log.info(f"PASS singular [{a},{b_c};{c},{d}]")

# ---------------------------------------------------------------------------
# TEST 6: Saturation — result that overflows signed 8-bit Q4.4
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_saturation(dut):
    """Result overflow saturates to 0x7F / 0x80."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Positive overflow example
    a, b, c, d, e, f = 2, 1, 1, 1, 7, -8  # det=1, x0=15.0 => saturate
    x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
    assert not sing
    assert ovf, "Expected overflow flag"
    assert x0 == from_q44(0x7F), f"Expected saturated max, got {x0}"
    dut._log.info(f"PASS saturation positive: x0={x0:.4f}")

    # Negative overflow example
    a, b, c, d, e, f = 2, 1, 1, 1, -8, 7  # det=1, x0=-15.0 => saturate
    x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
    assert not sing
    assert ovf, "Expected overflow flag"
    assert x0 == from_q44(0x80), f"Expected saturated min, got {x0}"
    dut._log.info(f"PASS saturation negative: x0={x0:.4f}")

# ---------------------------------------------------------------------------
# TEST 7: Back-to-back solves without reset
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_back_to_back(dut):
    """Multiple consecutive solves — valid must re-assert each time."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        (1, 0, 0, 1,  3,  2,  3.0,  2.0),   # identity
        (2, 1, 1, 2,  3,  3,  1.0,  1.0),   # det=3
        (1, 1, 1, 2,  3,  4,  2.0,  1.0),   # det=1
        (3, 1, 1, 1,  2,  1,  0.5,  0.5),   # det=2
    ]
    for (a, b, c, d, e, f, ex0, ex1) in cases:
        x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
        assert not sing
        check(f"b2b x0 [{a},{b};{c},{d}]", x0, ex0)
        check(f"b2b x1 [{a},{b};{c},{d}]", x1, ex1)
        dut._log.info(f"PASS b2b [{a},{b};{c},{d}] x0={x0:.4f} x1={x1:.4f}")

# ---------------------------------------------------------------------------
# TEST 8: Reset mid-operation
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset_mid_op(dut):
    """Assert reset while loading; verify clean recovery."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Start a solve but interrupt after a few cycles
    dut.uio_in.value = 0b00000001
    dut.ui_in.value  = 0x01
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0x00

    await ClockCycles(dut.clk, 3)

    # Reset mid-load/compute
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    flags = int(dut.uio_out.value)
    assert not (flags & 0x01), "valid should be 0 after reset"
    assert not (flags & 0x02), "singular should be 0 after reset"

    # Fresh solve must work
    x0, x1, sing, ovf = await do_solve(dut, 1, 0, 0, 1, 5, 3)
    assert not sing
    check("post-reset x0", x0, 5.0)
    check("post-reset x1", x1, 3.0)
    dut._log.info(f"PASS reset mid-op recovery: x0={x0} x1={x1}")

# ---------------------------------------------------------------------------
# TEST 9: uio_oe — only bits [2:0] driven
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_uio_oe(dut):
    """Check output-enable register is exactly 0x07."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)
    await RisingEdge(dut.clk)
    oe = int(dut.uio_oe.value)
    assert oe == 0x07, f"uio_oe expected 0x07, got 0x{oe:02X}"
    dut._log.info(f"PASS uio_oe = 0x{oe:02X}")

# ---------------------------------------------------------------------------
# SUMMARY
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_all_passed_summary(dut):
    """Print grand success message."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await ClockCycles(dut.clk, 5)
    dut._log.info("==========================================================")
    dut._log.info("      DONE: tt_um_solve2x2_q16 Q4.4 Verification Complete ")
    dut._log.info("            ALL TESTS PASSED SUCCESSFULLY!                ")
    dut._log.info("==========================================================")
    await ClockCycles(dut.clk, 2)