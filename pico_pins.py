"""Pin-state fingerprint of the four JTAG wires.

Cheat sheet:
  GP11 -> board TCK, which has a pull-DOWN on the board   -> nopull should read 0
  GP15 -> board TDI, which has a pull-UP on the board     -> nopull should read 1
  GP12 -> board TMS, nothing on the board                 -> nopull floats
  GP16 -> board TDO, driven by the FPGA only while shifting
A pin that reads 0 with the internal pull-down AND 1 with the internal pull-up
is floating, i.e. nothing is connected to it.
"""
import time
from machine import Pin

print("GP  nopull    pulldown  pullup")


def sample(p, mode):
    pin = Pin(p, Pin.IN, mode)
    return "".join(str(pin.value()) for _ in range(8))


while True:
    for p in (11, 12, 15, 16):
        print("GP%02d %s  %s  %s" % (p, sample(p, None),
                                     sample(p, Pin.PULL_DOWN),
                                     sample(p, Pin.PULL_UP)))
    print("---")
    time.sleep(1)
