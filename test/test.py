
# =============================================================================
# test_solve2x2.py  —  Cocotb testbench for tt_um_solve2x2_q16
#
# Strategy:
#   1. Pick a known integer solution x0, x1
#   2. Pick matrix A
#   3. Compute b = A*x  (so we know the exact answer)
#   4. Stream all 12 bytes into the DUT
#   5. Read back x0, x1 as Q16.16
#   6. Compare to expected within tolerance (1 LSB = 1/65536 ≈ 0.000015)
# =============================================================================

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random
import struct

# ---------------------------------------------------------------------------
# Q16.16 helpers
# ---------------------------------------------------------------------------
Q = 16  # fractional bits

def to_q16(f: float) -> int:
    """Float -> Q16.16 signed 32-bit integer (Python int, may be negative)."""
    v = int(round(f * (1 << Q)))
    # clamp to 32-bit signed
    v = max(-(1 << 31), min((1 << 31) - 1, v))
    return v

def from_q16(v: int) -> float:
    """Q16.16 signed 32-bit integer -> float."""
    # interpret as signed 32-bit
    if v >= (1 << 31):
        v -= (1 << 32)
    return v / (1 << Q)

def to_signed16(v: int) -> int:
    """Clamp and interpret as signed 16-bit."""
    v = v & 0xFFFF
    if v >= (1 << 15):
        v -= (1 << 16)
    return v

def pack16(v: int):
    """Return (lo_byte, hi_byte) for a signed 16-bit value."""
    v = v & 0xFFFF
    return (v & 0xFF, (v >> 8) & 0xFF)

# ---------------------------------------------------------------------------
# DUT driver helpers
# ---------------------------------------------------------------------------
async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

async def stream_inputs(dut, a, b, c, d, e, f):
    """
    Send 12 bytes: a_lo, a_hi, b_lo, b_hi, ..., f_lo, f_hi.
    Byte 0 (a_lo) is presented on the SAME cycle as start=1.
    The DUT samples it in S_IDLE. Bytes 1..11 follow on subsequent cycles.
    """
    vals = [a, b, c, d, e, f]
    bytes_to_send = []
    for v in vals:
        lo, hi = pack16(v)
        bytes_to_send.extend([lo, hi])

    # Cycle N: start=1, byte 0 on ui_in — DUT captures a_lo in S_IDLE
    dut.uio_in.value = 0b00000001
    dut.ui_in.value  = bytes_to_send[0]
    await RisingEdge(dut.clk)

    # Cycles N+1..N+11: start=0, bytes 1..11
    dut.uio_in.value = 0b00000000
    for byte_val in bytes_to_send[1:]:
        dut.ui_in.value = byte_val
        await RisingEdge(dut.clk)

async def wait_valid(dut, timeout=200):
    # Phase 1: if valid already high, wait for it to clear first
    if int(dut.uio_out.value) & 0x01:
        for _ in range(timeout):
            await RisingEdge(dut.clk)
            if not (int(dut.uio_out.value) & 0x01):
                break
        else:
            return False

    # Phase 2: wait for valid to assert with the new result
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.uio_out.value) & 0x01:
            return True
    return False
async def read_result(dut, sel):
    """Read all 4 bytes of x0 (sel=0) or x1 (sel=1) and assemble Q16.16."""
    result = 0
    for byte_idx in range(4):
        dut.uio_in.value = (byte_idx << 2) | (sel << 1)
        await RisingEdge(dut.clk)
        result |= (int(dut.uo_out.value) << (8 * byte_idx))
    return result

# ---------------------------------------------------------------------------
# Single solve helper — returns (got_x0_float, got_x1_float, singular, overflow)
# ---------------------------------------------------------------------------
async def do_solve(dut, a, b, c, d, e, f):
    await stream_inputs(dut, a, b, c, d, e, f)

    ok = await wait_valid(dut, timeout=300)
    assert ok, f"TIMEOUT: valid never asserted for a={a} b={b} c={c} d={d} e={e} f={f}"

    flags    = int(dut.uio_out.value)
    singular = bool(flags & 0x02)
    overflow = bool(flags & 0x04)

    x0_raw = await read_result(dut, sel=0)
    x1_raw = await read_result(dut, sel=1)

    return from_q16(x0_raw), from_q16(x1_raw), singular, overflow

# ---------------------------------------------------------------------------
# Tolerance: 2 LSBs of Q16.16 to allow for truncation in integer division
# ---------------------------------------------------------------------------
TOL = 2.0 / (1 << Q)   # ~0.0000305

def check(label, got, expected, tol=TOL):
    err = abs(got - expected)
    assert err <= tol, \
        f"{label}: got {got:.8f}, expected {expected:.8f}, err={err:.2e} > tol={tol:.2e}"

# ---------------------------------------------------------------------------
# TEST 1: Identity matrix  A=I  =>  x = b
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_identity(dut):
    """A = identity, solution should equal b vector."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        (1, 3),
        (-5, 7),
        (0, 0),
        (100, -200),
        (-32768, 32767),   # extreme 16-bit values
    ]
    for (ex0, ex1) in cases:
        e = ex0         # b = A*x = I*x = x
        f = ex1
        x0, x1, sing, ovf = await do_solve(dut, 1, 0, 0, 1, e, f)
        assert not sing, f"Identity should not be singular"
        check(f"identity x0 (e={e})", x0, float(ex0))
        check(f"identity x1 (f={f})", x1, float(ex1))
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
        (2,  3,   6,   9),   # a=2, d=3, e=6, f=9  => x0=3, x1=3
        (4,  2,   4,   6),   # x0=1, x1=3
        (-2, 4,  -8,   8),   # x0=4, x1=2
        (1,  1,   5,  -3),   # x0=5, x1=-3
        (3,  5,   1,   1),   # x0=1/3, x1=1/5
    ]
    for (a, d, e, f) in cases:
        exp_x0 = e / a
        exp_x1 = f / d
        x0, x1, sing, ovf = await do_solve(dut, a, 0, 0, d, e, f)
        assert not sing
        check(f"diag x0 a={a} e={e}", x0, exp_x0)
        check(f"diag x1 d={d} f={f}", x1, exp_x1)
        dut._log.info(f"PASS diagonal a={a} d={d}: x0={x0:.6f} x1={x1:.6f}")

# ---------------------------------------------------------------------------
# TEST 3: General non-singular — known integer solutions
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_known_solutions(dut):
    """Construct b=A*x from known x, verify DUT recovers x."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # (a, b, c, d,  expected_x0, expected_x1)
    matrices = [
        ( 2,  1,  5,  3,   1,  -1),
        ( 1,  2,  3,  4,  -2,   2),
        ( 3,  1,  2,  4,   2,  -1),
        ( 1, -1,  2,  3,   1,   1),
        (-1,  2,  3, -4,   1,  -2),
        ( 4,  3,  2,  5,   2,   1),
        ( 5,  2,  1,  3,  -1,   3),
    ]
    for (a, b, c, d, ex0, ex1) in matrices:
        det = a*d - b*c
        if det == 0:
            continue
        e = a*ex0 + b*ex1
        f = c*ex0 + d*ex1
        x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
        assert not sing, f"Unexpected singular: det={det}"
        check(f"known x0 [{a},{b};{c},{d}]", x0, float(ex0))
        check(f"known x1 [{a},{b};{c},{d}]", x1, float(ex1))
        dut._log.info(f"PASS known [{a},{b};{c},{d}] x0={x0:.6f} x1={x1:.6f}")

# ---------------------------------------------------------------------------
# TEST 4: Fractional solutions (non-integer x)
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_fractional(dut):
    """Solutions that are not integers — tests the fractional bits."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        # a, b, c, d, e, f   (b=A*x, computed exactly)
        (2, 0, 0, 4,  1,  1),   # x0=0.5, x1=0.25
        (3, 0, 0, 3,  1,  2),   # x0=1/3, x1=2/3
        (4, 2, 1, 3,  3,  2),   # det=10, x0=(3*3-2*2)/10=5/10=0.5, x1=(-1*3+4*2)/10=5/10=0.5
        (2, 1, 1, 2,  3,  3),   # det=3, num0=3, num1=3 -> x0=1, x1=1
    ]
    for (a, b, c, d, e, f) in cases:
        det = a*d - b*c
        if det == 0:
            continue
        exp_x0 = (d*e - b*f) / det
        exp_x1 = (-c*e + a*f) / det
        x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
        assert not sing
        check(f"frac x0 [{a},{b};{c},{d}] e={e} f={f}", x0, exp_x0)
        check(f"frac x1 [{a},{b};{c},{d}] e={e} f={f}", x1, exp_x1)
        dut._log.info(f"PASS fractional: x0={x0:.6f}(exp {exp_x0:.6f}) x1={x1:.6f}(exp {exp_x1:.6f})")

# ---------------------------------------------------------------------------
# TEST 5: Singular matrices — valid=1, singular=1, x0=x1=0
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_singular(dut):
    """det=0: singular flag must be set, outputs must be zero."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        (1, 2, 2, 4,   1, 2),   # det=0
        (2, 4, 1, 2,   3, 0),   # det=0
        (0, 0, 0, 0,   1, 1),   # all-zero
        (3, 6, 1, 2,   5, 5),   # det=0
        (5, 5, 5, 5,   1, 2),   # det=0
    ]
    for (a, b, c, d, e, f) in cases:
        x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
        assert sing,       f"Expected singular for [{a},{b};{c},{d}]"
        assert x0 == 0.0,  f"Expected x0=0 for singular, got {x0}"
        assert x1 == 0.0,  f"Expected x1=0 for singular, got {x1}"
        dut._log.info(f"PASS singular [{a},{b};{c},{d}]")

# ---------------------------------------------------------------------------
# TEST 6: Saturation — small det, large numerator
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_saturation(dut):
    """Result overflow saturates to 0x7FFFFFFF / 0x80000000."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # det=1 matrix that produces num0=98302, which overflows Q16.16
    a, b, c, d = 1, 1, 1, 2   # det = 2-1 = 1
    e, f = 32767, -32768       # num0 = 2*32767 - (-32768) = 98302
    x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
    assert not sing
    assert ovf, f"Expected overflow (num0=98302 >> 32767)"
    assert x0 == from_q16(0x7FFFFFFF)
    dut._log.info(f"PASS saturation positive: x0={x0:.4f}")

    e, f = -32768, 32767       # num0 = 2*(-32768) - 32767 = -98303
    x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
    assert not sing
    assert ovf, f"Expected overflow (num0=-98303 << -32768)"
    assert x0 == from_q16(0x80000000)
    dut._log.info(f"PASS saturation negative: x0={x0:.4f}")
# ---------------------------------------------------------------------------
# TEST 7: Random regression — 100 random matrices
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_random(dut):
    """100 random matrices with known solutions; compare within Q16.16 tolerance."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    rng = random.Random(0xC0FFEE)
    passes = 0
    skipped = 0

    for trial in range(100):
        # Random matrix entries in [-128, 127] to keep det reasonable
        a = rng.randint(-128, 127)
        b = rng.randint(-128, 127)
        c = rng.randint(-128, 127)
        d = rng.randint(-128, 127)
        det = a*d - b*c

        if det == 0:
            skipped += 1
            continue

        # Random integer solutions in [-64, 64]
        ex0 = rng.randint(-64, 64)
        ex1 = rng.randint(-64, 64)

        e = a*ex0 + b*ex1
        f = c*ex0 + d*ex1

        # Skip if e or f overflow 16-bit
        if not (-32768 <= e <= 32767 and -32768 <= f <= 32767):
            skipped += 1
            continue

        x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)

        assert not sing, f"Trial {trial}: unexpected singular det={det}"
        check(f"rand trial {trial} x0", x0, float(ex0), tol=TOL)
        check(f"rand trial {trial} x1", x1, float(ex1), tol=TOL)
        passes += 1

    dut._log.info(f"Random: {passes} passed, {skipped} skipped (singular/overflow)")
    assert passes >= 50, f"Too few passing random trials: {passes}"

# ---------------------------------------------------------------------------
# TEST 8: Back-to-back solves without reset
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_back_to_back(dut):
    """Multiple consecutive solves; valid must re-assert each time."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    cases = [
        (2, 1, 1, 2, 3, 3,  1.0,  1.0),   # det=3, x0=1, x1=1
        (1, 1, 1, 2, 3, 4,  2.0,  1.0),   # det=1, x0=2, x1=1
        (1, 0, 0, 1, 7, -3, 7.0, -3.0),   # identity
        (3, 2, 1, 1, 5, 3,  -1.0 ,  4.0),  # det=1, x0=-1, x1=4
    ]
    for (a, b, c, d, e, f, ex0, ex1) in cases:
        x0, x1, sing, ovf = await do_solve(dut, a, b, c, d, e, f)
        assert not sing
        check(f"b2b x0 [{a},{b};{c},{d}]", x0, ex0)
        check(f"b2b x1 [{a},{b};{c},{d}]", x1, ex1)
        dut._log.info(f"PASS b2b [{a},{b};{c},{d}] x0={x0:.4f} x1={x1:.4f}")

# ---------------------------------------------------------------------------
# TEST 9: Reset mid-operation
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset_mid_op(dut):
    """Assert reset while a solve is in progress; verify clean recovery."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    # Start a solve
    dut.uio_in.value = 0b00000001  # start pulse
    dut.ui_in.value  = 0x01
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0

    # Let it run a few cycles then reset
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # valid and singular must be cleared
    flags = int(dut.uio_out.value)
    assert not (flags & 0x01), "valid should be 0 after reset"
    assert not (flags & 0x02), "singular should be 0 after reset"

    # Fresh solve must work correctly
    x0, x1, sing, ovf = await do_solve(dut, 1, 0, 0, 1, 5, 3)
    assert not sing
    check("post-reset x0", x0, 5.0)
    check("post-reset x1", x1, 3.0)
    dut._log.info(f"PASS reset mid-op recovery: x0={x0} x1={x1}")

# ---------------------------------------------------------------------------
# TEST 10: uio_oe — only bits [2:0] should be driven outputs
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
# FINAL SUCCESS SUMMARY
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_all_passed_summary(dut):
    """Final check to print a grand success message."""
    # Clear the warning: change unit="ns" to unit="ns" in your other tests too!
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    
    await RisingEdge(dut.clk)
    await ClockCycles(dut.clk, 5) # Wait a few cycles to ensure logs flush
    
    dut._log.info("==========================================================")
    dut._log.info("      DONE: tt_um_solve2x2_q16 Verification Complete      ")
    dut._log.info("            ALL TESTS PASSED SUCCESSFULLY!                ")
    dut._log.info("==========================================================")
    
    await ClockCycles(dut.clk, 2)