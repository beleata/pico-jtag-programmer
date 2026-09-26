"""Does the instruction register actually load now?

IDCODE (IR=0x6) must give the 32-bit IDCODE; BYPASS (IR=all ones) must give a
single 0 bit, so a 32-bit DR scan reads back as all zeros.  If BYPASS still shows
the IDCODE, the IR is not loading and no configuration can possibly work.
"""
import time

import serial

import jtag_do as D
from jtag_run import Jtag

port = D.find_port()
s = serial.Serial(port, 115200, timeout=0.2)
time.sleep(0.8)
s.reset_input_buffer()
ok, out = D.console_alive(s)
print("console:", ok)
if not ok:
    raise SystemExit("console dead - replug the Pico")
if not D.upload_slave(s) or not D.boot_slave(s):
    raise SystemExit("no slave")

jt = Jtag(s)
for name, ir, nbits in (("IDCODE", 0x006, 32), ("BYPASS", 0x3FF, 32),
                        ("IDCODE again", 0x006, 32)):
    jt.reset()
    jt.shift_ir(10, ir)
    val = jt.shift_dr(nbits, 0, readback=True)
    print("%-13s IR=0x%03X  DR = 0x%08X" % (name, ir, val))
D.quit_slave(s)
