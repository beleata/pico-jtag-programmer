"""Read the IDCODE while listening on every candidate TDO pin at once.

Loads the IDCODE instruction (IR = 10 bits, 0x6 - straight out of the .prg) and
then shifts the DR while sampling GP16, GP17, GP13, GP14, GP18-GP22.  Whichever
pin shows something other than all-zeros or all-ones is where the TDO wire is.
"""
import time

import serial

import jtag_do as D
from jtag_run import SCAN_PINS, Jtag

port = D.find_port()
print("port:", port)
s = serial.Serial(port, 115200, timeout=0.2)
time.sleep(0.8)
s.reset_input_buffer()

if D.slave_ping(s):
    print("slave already running")
else:
    ok, out = D.console_alive(s)
    print("console:", "reachable" if ok else "NO ANSWER")
    if not ok:
        raise SystemExit("console dead")
    if not D.upload_slave(s) or not D.boot_slave(s):
        raise SystemExit("could not start the slave")

jt = Jtag(s)
jt.reset()
jt.shift_ir(10, 0x6)            # IDCODE instruction, IR is 10 bits wide
jt.goto_shift_dr()
print("pin    value       verdict")
for pin, val in jt.scan_candidates(32):
    if val == 0:
        verdict = "stuck low"
    elif val == 0xFFFFFFFF:
        verdict = "stuck high / floating"
    elif val in (0x01000001, 0x01000011):
        verdict = "*** THIS IS THE IDCODE ***"
    else:
        verdict = "*** data! ***"
    print("GP%-3d  0x%08X  %s" % (pin, val, verdict))
D.quit_slave(s)
