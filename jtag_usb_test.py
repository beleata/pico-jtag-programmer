"""Probe the C USB-Blaster firmware directly.

Reads the DR three ways so we can tell our protocol usage apart from the
firmware's fast shift engine:
  1. bit-banged only (every clock is a USB round trip, slow but transparent)
  2. via the shift-mode engine
  3. after explicitly loading the IDCODE instruction
"""
import time

from jtag_usb import Blaster, Jtag


def dr_bitbang(jt, nbits):
    jt._clock_bits(3, 0b001, 0, False)                    # Idle -> Shift-DR
    val = jt._clock_bits(nbits, 1 << (nbits - 1), 0, readback=True)
    jt.exit_to_idle()
    return val


def dr_shiftmode(jt, nbits):
    jt._clock_bits(3, 0b001, 0, False)
    val = jt.shift(nbits, 0, readback=True)
    jt.exit_to_idle()
    return val


bl = Blaster()
jt = Jtag(bl)

t0 = time.time()
jt.reset()
print("1. bit-banged DR only        = 0x%08X   (%.1fs)" % (dr_bitbang(jt, 32), time.time() - t0))

t0 = time.time()
jt.reset()
print("2. shift-mode DR             = 0x%08X   (%.1fs)" % (dr_shiftmode(jt, 32), time.time() - t0))

t0 = time.time()
jt.reset()
jt.shift_ir(10, 0x006)
print("3. IDCODE instr + bit-banged = 0x%08X   (%.1fs)" % (dr_bitbang(jt, 32), time.time() - t0))

jt.reset()
jt.shift_ir(10, 0x3FF)                                   # BYPASS -> DR is one 0 bit
print("4. BYPASS  + bit-banged      = 0x%08X   (expect all zeros)" % dr_bitbang(jt, 32))

# what do the raw input pins look like?  nCS low makes the firmware return
# DATAOUT/nSTATUS instead of TDO, and OE off releases our outputs
print("5. reading inputs with nCS low, outputs released:")
from jtag_usb import NCS, OE, TMS, TDI, TCK
for label, pins in (("all low ", 0), ("nCS high", NCS), ("TMS high", TMS),
                    ("TDI high", TDI), ("TCK high", TCK)):
    tdo, ns = bl.bitbang(pins, read=True)                # OE not set: high-Z
    print("   %s -> tdo=%d nstatus=%d" % (label, tdo, ns))
