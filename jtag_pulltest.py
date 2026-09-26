"""Is the FPGA driving TDO at all, and is the slave reading it correctly?

With the internal pull-down on GP16 the line must read all zeros unless something
external drives it high; with the pull-up it must read all ones.  So:
  pull-down -> 00000000 and pull-up -> FFFFFFFF  = readback path fine, FPGA silent
  pull-down -> real data                         = the FPGA *is* driving TDO
"""
import time

import serial

import jtag_do as D
from jtag_run import Jtag

port = D.find_port()
print("port:", port)
s = serial.Serial(port, 115200, timeout=0.2)
time.sleep(0.8)
s.reset_input_buffer()

if D.slave_ping(s):
    print("slave already running")
else:
    ok, out = D.console_alive(s)
    print("console:", "reachable" if ok else "NO ANSWER",
          "|", out.decode("utf-8", "replace").strip()[-60:])
    if not ok:
        raise SystemExit("console dead")
    if not D.upload_slave(s) or not D.boot_slave(s):
        raise SystemExit("could not start the slave")

jt = Jtag(s)
print("pull      ack   IDCODE")
for pull, name in ((1, "down  "), (2, "up    "), (0, "none  ")):
    s.write(b"U %d\n" % pull)
    ack = jt._reply()
    jt.reset()
    idc = jt.shift_dr(32, 0, readback=True)
    print(f"{name}   {ack!r:6} 0x{idc:08X}")

# and one long shift, to see whether anything at all moves
jt.reset()
jit = jt.shift_dr(256, 0, readback=True)
print(f"256-bit DR scan = 0x{jit:064X}  (all same nibble = line is stuck)")
D.quit_slave(s)
