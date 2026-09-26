"""Drive each JTAG pin hard and read the actual pad level back.

  drive0/drive1: the pin is an output; we read SIO GPIO_IN, which reflects the
                 real pad, not our output latch
  nopull/pulldn/pullup: the pin is an input with each internal pull setting

A healthy pin on a wire with only a weak external pull reads 0 when driven 0 and
1 when driven 1.  A wire sitting on GND cannot be driven high; a wire on 3.3V
cannot be driven low.  A dead pad usually does not follow its own driver.
"""
import time

import machine
from machine import Pin

IN = 0xD0000004
OUT_SET, OUT_CLR = 0xD0000014, 0xD0000018
OE_SET, OE_CLR = 0xD0000024, 0xD0000028
PINS = (11, 12, 15, 16)

for p in PINS:
    Pin(p, Pin.IN)                       # start high-Z, no pull
time.sleep_ms(20)


def rd(p):
    return 1 if machine.mem32[IN] & (1 << p) else 0


print("pin    drive0 drive1 | nopull pulldn pullup")
for p in PINS:
    m = 1 << p
    machine.mem32[OUT_CLR] = m
    machine.mem32[OE_SET] = m            # output on, driving 0
    time.sleep_ms(10)
    d0 = rd(p)
    machine.mem32[OUT_SET] = m           # driving 1
    time.sleep_ms(10)
    d1 = rd(p)
    machine.mem32[OE_CLR] = m            # release
    machine.mem32[OUT_CLR] = m
    time.sleep_ms(5)
    n = "".join(str(Pin(p, Pin.IN).value()) for _ in range(4))
    dn = "".join(str(Pin(p, Pin.IN, Pin.PULL_DOWN).value()) for _ in range(4))
    up = "".join(str(Pin(p, Pin.IN, Pin.PULL_UP).value()) for _ in range(4))
    print("GP%02d      %d      %d   | %s %s %s" % (p, d0, d1, n, dn, up))

print("pad test done")
