"""JTAG over the Pico's Altera-USB-Blaster firmware, straight from Python.

Needs the device (VID 09FB / PID 6001) to be bound to WinUSB - done once with Zadig.

usage:
  python jtag_usb.py idcode
  python jtag_usb.py play <file.prg>
"""
import re
import sys
import time

import usb.core
import usb.util
import libusb_package

VID, PID = 0x09FB, 0x6001
EP_OUT, EP_IN = 0x02, 0x81

# bitbang byte bits (from the firmware's src/blaster.c):
#   rx 0x01 TCK | 0x02 TMS/nCONFIG | 0x04 nCE | 0x08 nCS | 0x10 TDI | 0x20 OE
#   tx 0x01 TDO/CONF_DONE | 0x02 DATAOUT/nSTATUS
# nCE must be driven LOW or the FPGA is disabled and answers nothing; nCS must be
# HIGH because the firmware picks TDO over DATAOUT based on it.
TCK = 1 << 0
TMS = 1 << 1
NCE = 1 << 2
NCS = 1 << 3
TDI = 1 << 4
OE = 1 << 5
RD = 1 << 6
SHIFT = 1 << 7

backend = libusb_package.get_libusb1_backend()


class Blaster:
    def __init__(self):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
        if self.dev is None:
            raise SystemExit("USB Blaster not found - is the Pico plugged in with the blaster firmware,\n"
                             "and is the WinUSB driver installed (Zadig)?")
        try:
            self.dev.set_configuration()
        except usb.core.USBError:
            pass
        usb.util.claim_interface(self.dev, 0)
        self.oep = EP_OUT
        self.iep = EP_IN
        self.bitbang(OE | NCS)                # outputs on, nCE low, nCS high

    def _write(self, data):
        self.dev.write(self.oep, data, timeout=5000)

    def _poll(self, ms=60):
        """Collect whatever the device sends within ms milliseconds.

        Reads are non-blocking on purpose: libusbK on this machine ignores the
        timeout argument, so a plain read() blocks forever on an empty FIFO."""
        out = b""
        end = time.time() + ms / 1000.0
        while time.time() < end:
            try:
                b = self.dev.read(self.iep, 64, timeout=0)
            except usb.core.USBError:
                time.sleep(0.002)
                continue
            if b:
                out += bytes(b)
        return out

    @staticmethod
    def _payload(data):
        """The device prefixes every flush with the 31 60 preamble (a leftover of
        the FT245 chip in the original USB Blaster) and flushes every 10 ms even
        when nothing was read, so the stream looks like 3160 3160 <payload> 3160.
        Preamble pairs are dropped, everything else is real data."""
        out = bytearray()
        i = 0
        while i < len(data):
            if data[i] == 0x31 and i + 1 < len(data) and data[i + 1] == 0x60:
                i += 2
            else:
                out.append(data[i])
                i += 1
        return bytes(out)

    def _exchange(self, cmds, nreads=0):
        """One USB write; returns exactly nreads payload bytes."""
        self._poll(20)                       # swallow pending preambles
        self._write(bytes(cmds))
        end = time.time() + 2.0
        buf = b""
        while time.time() < end:
            buf += self._poll(20)
            if len(self._payload(buf)) >= nreads:
                break
        return self._payload(buf)[:nreads]

    def bitbang(self, pins, read=False):
        """Set the pin levels; return (tdo, nstatus) when read is requested.

        The mask is 0x3F and not 0x1F on purpose: 0x20 is the output-enable bit,
        and masking it away leaves every JTAG line high-Z, which reads back as
        alternating garbage."""
        payload = self._exchange([(pins & 0x3F) | (RD if read else 0)], 1 if read else 0)
        if read:
            return (payload[0] & 1, (payload[0] >> 1) & 1)
        return (0, 0)

    def shift_bytes(self, data, read=False):
        """Shift whole bytes out (LSB first); TMS stays where it is.  Chunks are
        capped at 62 bytes so preamble plus payload still fit one 64-byte flush."""
        out = bytearray()
        i = 0
        while i < len(data):
            n = min(62, len(data) - i)
            payload = self._exchange([SHIFT | (RD if read else 0) | n] + list(data[i:i + n]),
                                     n if read else 0)
            if read:
                out += payload
            i += n
        return bytes(out)


class Jtag:
    def __init__(self, bl):
        self.bl = bl
        self.bl.bitbang(OE | NCS)              # outputs on, nCE low (device enabled)
        self.bl.bitbang(OE | NCS | TCK)        # idle, TCK high

    # --- low level -------------------------------------------------------
    def _clock_bits(self, nbits, tms_val, tdi_val, readback):
        val = 0
        for i in range(nbits):
            tms = (tms_val >> i) & 1
            tdi = (tdi_val >> i) & 1
            pins0 = OE | NCS | (TMS if tms else 0) | (TDI if tdi else 0)
            self.bl.bitbang(pins0)                    # TCK low: data valid
            if readback:
                tdo, _ = self.bl.bitbang(pins0 | TCK, read=True)   # rising edge
                if tdo:
                    val |= 1 << i
            else:
                self.bl.bitbang(pins0 | TCK)
        return val

    def shift(self, nbits, value, readback=False):
        """Shift nbits, TMS low except the last bit which is driven high."""
        tdo = 0
        if nbits <= 0:
            return 0
        whole = (nbits - 1) // 8          # bytes we can do in shift mode
        if whole:
            data = value.to_bytes(whole, "little")
            if readback:
                got = self.bl.shift_bytes(data, read=True)
                for i, b in enumerate(got):
                    tdo |= b << (8 * i)
            else:
                self.bl.shift_bytes(data)
            value >>= 8 * whole
            rest = nbits - 8 * whole
        else:
            rest = nbits
        # remaining bits, TMS low, then the very last bit with TMS=1
        if rest:
            tms = 1 << (rest - 1)
            got = self._clock_bits(rest, tms, value & ((1 << rest) - 1), readback)
            tdo |= got << (8 * whole)
        return tdo

    def exit_to_idle(self):
        """From Exit1-IR/Exit1-DR: TMS=1 -> Update, TMS=0 -> Run-Test/Idle.  The
        old 1,1,0 landed in Capture-DR instead, which breaks every instruction
        load after the first one."""
        self._clock_bits(2, 0b01, 0, False)

    def shift_ir(self, nbits, value, readback=False):
        self._clock_bits(4, 0b0011, 0, False)      # Idle->SelDR->SelIR->CapIR->ShiftIR
        r = self.shift(nbits, value, readback)
        self.exit_to_idle()
        return r

    def shift_dr(self, nbits, value, readback=False):
        self._clock_bits(3, 0b001, 0, False)       # Idle->SelDR->CapDR->ShiftDR
        r = self.shift(nbits, value, readback)
        self.exit_to_idle()
        return r

    def idle_clocks(self, n):
        self._clock_bits(n, 0, 0, False)

    def reset(self):
        self._clock_bits(8, 0xFF, 0, False)
        self._clock_bits(1, 0, 0, False)


def parse_prg(path):
    text = open(path, encoding="utf-8", errors="replace").read().replace("\\\n", " ")
    ops = []
    for line in text.splitlines():
        line = line.strip()
        m = re.match(r"^sir\s+(\d+)\s+-tdi\s+([0-9a-fA-F]+)$", line)
        if m:
            ops.append(("sir", int(m.group(1)), int(m.group(2), 16), None, None))
            continue
        m = re.match(r"^sdr\s+(\d+)\s+-tdi\s+([0-9a-fA-F]+)"
                     r"(?:\s+-tdo\s+([0-9a-fA-F]+))?(?:\s+-mask\s+([0-9a-fA-F]+))?.*$", line)
        if m:
            ops.append(("sdr", int(m.group(1)), int(m.group(2), 16),
                        int(m.group(3), 16) if m.group(3) else None,
                        int(m.group(4), 16) if m.group(4) else None))
            continue
        m = re.match(r"^runtest\s+-tck\s+(\d+)$", line)
        if m:
            ops.append(("idle", int(m.group(1)), 0, None, None))
    return ops


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "idcode"
    bl = Blaster()
    jt = Jtag(bl)
    print("blaster open, JTAG chain test:")

    jt.reset()
    idcode = jt.shift_dr(32, 0, readback=True)
    print(f"  IDCODE = 0x{idcode:08X}")
    if idcode in (0, 0xFFFFFFFF):
        print("  no JTAG answer - check wiring (TDI/TDO), pin 1 and the grounds")
    if cmd == "idcode":
        return

    path = sys.argv[2]
    ops = parse_prg(path)
    bits = sum(n for k, n, *_ in ops if k in ("sir", "sdr"))
    print(f"  playing {path}: {len(ops)} ops, {bits} shift bits")
    t0 = time.time()
    bad = 0
    for idx, (kind, n, tdi, tdo, mask) in enumerate(ops):
        if kind == "idle":
            jt.idle_clocks(n)
        elif kind == "sir":
            jt.shift_ir(n, tdi)
        else:
            got = jt.shift_dr(n, tdi, readback=tdo is not None)
            if tdo is not None and mask and (got ^ tdo) & mask:
                bad += 1
                if bad < 4:
                    print(f"    mismatch at op {idx}")
        if idx % 250 == 0:
            print(f"    {idx}/{len(ops)} ({time.time() - t0:.1f}s)", flush=True)
    print(f"  finished in {time.time() - t0:.1f}s, {bad} mismatches")


if __name__ == "__main__":
    main()
