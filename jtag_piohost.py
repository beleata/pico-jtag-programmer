"""Streaming speed test of the chunked PIO path, with every step printed as it
happens so a hang shows up immediately instead of after a ten minute timeout."""
import sys
import time

import serial

import jtag_do as D
from jtag_run import Jtag

print("opening port...", flush=True)
s = serial.Serial(D.find_port(), 115200, timeout=0.2)
time.sleep(0.8)
s.reset_input_buffer()
print("waking the console...", flush=True)
ok, out = D.console_alive(s)
print("console:", ok, flush=True)
if not ok:
    raise SystemExit("console dead - replug the Pico")
print("uploading the slave...", flush=True)
D.upload_slave(s)
D.boot_slave(s)

jt = Jtag(s)
print("reset + IDCODE check...", flush=True)
jt.reset()
jt.shift_ir(10, 0x6)
print("IDCODE = 0x%08X" % jt.shift_dr(32, 0, readback=True), flush=True)

for nbits in (4128, 131072, 1048576, 4418528):
    print("shifting %d bits over PIO..." % nbits, flush=True)
    t0 = time.time()
    try:
        jt._pio_shift(nbits, 1 << (nbits - 1), 0)
        dt = time.time() - t0
        print("   ok  %6.2fs  %7.0f kbit/s" % (dt, nbits / dt / 1000), flush=True)
    except Exception as e:
        print("   FAILED after %.2fs: %r" % (time.time() - t0, e), flush=True)
        sys.exit(1)
print("all PIO shifts worked", flush=True)
D.quit_slave(s)
