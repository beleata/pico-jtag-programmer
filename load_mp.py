"""Load a MicroPython script onto the Pico and print its output."""
import os
import sys
import time

import serial
from serial.tools import list_ports

SRC = sys.argv[1] if len(sys.argv) > 1 else "pico_jtag.py"
WATCH = int(sys.argv[2]) if len(sys.argv) > 2 else 25
CODE = open(SRC, encoding="utf-8").read()


def find_port():
    for p in list_ports.comports():
        if (p.vid == 0x2E8A and p.pid == 0x0005) or "MicroPython" in (p.description or ""):
            return p.device
    return None


port = find_port()
if not port:
    raise SystemExit("no MicroPython serial port")

ser = serial.Serial(port, 115200, timeout=1)
time.sleep(1.0)
ser.reset_input_buffer()
ser.write(b"\r\x03\x03")     # Ctrl-C
time.sleep(0.4)
ser.read(8192)
ser.write(b"Q\nQ\nQ\n")      # ask the JTAG slave to quit, back to the REPL
time.sleep(0.5)
ser.read(8192)
ser.write(b"\r\x03")         # and interrupt anything else that is running
time.sleep(0.4)
ser.read(8192)
ser.write(b"\r\x02")         # friendly REPL
time.sleep(0.3)
ser.read(8192)

# save as main.py and run it
save = "f=open('main.py','w')\nf.write(" + repr(CODE) + ")\nf.close()\nprint('SAVED')\n"
ser.write(b"\r\x01")         # raw REPL
time.sleep(0.3)
ser.write(save.encode())
ser.write(b"\x04")
time.sleep(2.5)
out = ser.read(8192)
print("save:", out.decode("utf-8", "replace")[-200:])

ser.write(b"\r\x02")          # leave raw REPL
time.sleep(0.6)
ser.read(8192)
ser.write(b"\r\x04")          # Ctrl-D soft reboot -> runs main.py
t_end = time.time() + WATCH
buf = b""
while time.time() < t_end:
    chunk = ser.read(512)
    if chunk:
        buf += chunk
        sys.stdout.write(chunk.decode("utf-8", "replace"))   # stream it live
        sys.stdout.flush()
CAPTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mp_capture.txt")
cap = open(CAPTURE, "w", encoding="utf-8")
cap.write(buf.decode("utf-8", "replace"))
cap.close()
print("--- output ---")
print(buf.decode("utf-8", "replace"))
ser.close()
