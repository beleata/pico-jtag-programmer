"""Unwedge the Pico: the slave currently in main.py blocks inside
sys.stdin.buffer.read(1024), so it needs >=1024 bytes before it will answer
anything.  Push a pile of Q lines at it, which makes it quit to the REPL."""
import time

import serial

s = serial.Serial("COM6", 115200, timeout=0.5)
time.sleep(0.5)
s.write(b"Q\n" * 700)                 # 1400 bytes > 1024
time.sleep(1.5)
print("reply:", repr(s.read(4096)))
s.write(b"\x03")
time.sleep(0.4)
s.write(b"\r\x02")
time.sleep(0.6)
print("after ctrl-b:", repr(s.read(4096)))
s.write(b'print("REPL-OK", 6 * 7)\r\n')
time.sleep(0.8)
print("repl test:", repr(s.read(4096)))
s.close()
