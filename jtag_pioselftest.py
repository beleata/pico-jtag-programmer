"""Ask the slave to run the PIO self-test: does the state machine consume data?"""
import time

import serial

import jtag_do as D

s = serial.Serial(D.find_port(), 115200, timeout=0.2)
time.sleep(0.8)
s.reset_input_buffer()
print("waking the console...", flush=True)
ok, out = D.console_alive(s)
print("console:", ok, flush=True)
if not ok:
    raise SystemExit("console still dead")
print("uploading the slave...", flush=True)
D.upload_slave(s)
D.boot_slave(s)

print("running PIO self-test...", flush=True)
s.write(b"W\n")
time.sleep(3.0)
print("reply:", repr(s.read(4096)), flush=True)
D.quit_slave(s)
