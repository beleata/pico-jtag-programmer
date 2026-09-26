"""Talk to the Blaster firmware at the raw USB level.

libusbK on this machine ignores read timeouts, so every read here is non-blocking
(timeout=0) inside a poll loop.
"""
import time

import libusb_package
import usb.core
import usb.util

EP_OUT, EP_IN = 0x02, 0x81
be = libusb_package.get_libusb1_backend()
dev = usb.core.find(idVendor=0x09FB, idProduct=0x6001, backend=be)
dev.set_configuration()
usb.util.claim_interface(dev, 0)
print("claimed interface 0 of 09fb:6001")


def poll(ms=50):
    """Collect everything the device sends within ms milliseconds."""
    out = b""
    end = time.time() + ms / 1000.0
    while time.time() < end:
        try:
            b = dev.read(EP_IN, 64, timeout=0)
        except usb.core.USBError:
            time.sleep(0.002)
            continue
        if b:
            out += bytes(b)
    return out


print("stray before we say anything:", poll(200).hex())

# clear any leftover shift-mode counter, then a shift command with zero length
dev.write(EP_OUT, b"\x00" * 64, timeout=2000)
dev.write(EP_OUT, bytes([0x80]), timeout=2000)
print("after clearing leftover shift mode:", poll(50).hex())

for label, cmd in (("bitbang plain", bytes([0x00])),
                   ("bitbang RD", bytes([0x40])),
                   ("bitbang OE|NCS", bytes([0x28])),
                   ("bitbang OE|NCS|RD", bytes([0x68]))):
    dev.write(EP_OUT, cmd, timeout=2000)
    print("%-20s -> %s" % (label, poll(60).hex()))

dev.write(EP_OUT, bytes([0x80 | 0x40 | 1]) + b"\xa5", timeout=2000)
print("%-20s -> %s" % ("shift 0xa5 read", poll(60).hex()))
