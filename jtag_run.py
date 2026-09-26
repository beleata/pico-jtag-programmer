"""Run JTAG work through the MicroPython slave on the Pico.

Text protocol, one command per line (see pico_jtag.py).  The USB serial port is
opened ONCE and kept open for the whole run: closing and reopening the CDC port
makes the slave's stdin hit EOF and it stops answering.

usage:  python jtag_run.py idcode
        python jtag_run.py play <file.prg>
"""
import binascii
import os
import re
import sys
import time
from array import array

import serial
from serial.tools import list_ports

# must match SCAN_PINS in pico_jtag.py
SCAN_PINS = (16, 17, 13, 14, 18, 19, 20, 21, 22)


def find_port():
    for p in list_ports.comports():
        if (p.vid == 0x2E8A and p.pid == 0x0005) or "MicroPython" in (p.description or ""):
            return p.device
    return None


def is_hex(line):
    if not line or len(line) % 2:
        return False
    return all(c in b"0123456789abcdefABCDEF" for c in line)


class Jtag:
    def __init__(self, ser, verbose=True):
        self.s = ser
        self.verbose = verbose

    def _line(self, timeout=60.0):
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            c = self.s.read(1)
            if not c:
                continue
            if c == b"\n":
                return buf.strip()
            buf += c
            if len(buf) > 4_000_000:
                raise RuntimeError("runaway line from the slave")
        raise TimeoutError(f"no reply line within {timeout}s (partial {buf[:60]!r})")

    def _reply(self, timeout=60.0):
        """Next meaningful reply line; junk (boot banner, REPL noise) is skipped."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._line(timeout=max(0.5, deadline - time.time()))
            if line == b"K" or line.startswith(b"K "):
                return line
            if line.startswith(b"E "):
                raise RuntimeError(f"slave error: {line[2:].decode('utf-8', 'replace')}")
            if is_hex(line):
                return line
            if self.verbose:
                print(f"   (ignoring junk from the slave: {line[:40]!r})", flush=True)
        raise TimeoutError("slave went quiet")

    BULK_CHUNK = 32768      # bits per bulk command: the Pico has 264 kB of RAM,
                            # so a whole 2.7 Mbit shift can never be held there
    PIO_FREQ = 8000000      # state machine clock; 4 PIO cycles per bit, so this
                            # is a 2 MHz TCK and a 2 Mbit/s shift
    PIO_CHUNK = 16384      # bytes of run-length payload per PIO command; hex of it
                           # is 32 kB, which fits the Pico's RAM comfortably
    use_pio = not os.environ.get("JTAG_NO_PIO")   # PIO hardware shifting is the
                            # default now that it is proven; JTAG_NO_PIO=1 falls
                            # back to the slow but simple CPU bit-bang path

    def _rle_encode(self, buf):
        """Runs of identical 32-bit words -> (count, word) records.  The raw
        bitstream is around 90% zero bytes and one run is pure zeros, so this cuts
        the USB transfer - the real bottleneck - by several times."""
        words = array("I", buf)
        out = bytearray()
        i = 0
        n = len(words)
        while i < n:
            w = words[i]
            j = i + 1
            while j < n and words[j] == w and j - i < 255:
                j += 1
            out.append(j - i)
            out += w.to_bytes(4, "little")
            i = j
        return bytes(out)

    def _pio_shift(self, nbits, tms_bits, tdi_bits):
        """Shift through the Pico's PIO.

        The PIO loop is word granular, and padding is NOT an option: the FPGA's
        configuration logic takes the *first* bits clocked into the DR as data, so
        even one pad bit at the front shifts the whole bitstream and the device
        quietly ends up with garbage (its status reads then come back as zeros).
        Instead the run is split: the PIO clocks a whole number of 32-bit words and
        the last <=32 bits are clocked by the CPU, where the TMS=1 exit clock is
        expressible."""
        last_only = tms_bits == (1 << (nbits - 1))
        tail = ((nbits - 1) % 32) + 1 if last_only else 0     # 1..32, CPU-clocked
        pio_bits = nbits - tail
        if pio_bits % 32:
            # only for runtest-style runs: extra TMS-low clocks just idle the TAP
            pio_bits += 32 - (pio_bits % 32)
        buf = (tdi_bits & ((1 << pio_bits) - 1)).to_bytes(pio_bits >> 3, "little")
        rle = self._rle_encode(buf)

        t0 = time.time()
        off = 0
        while off < len(rle):
            take = min(self.PIO_CHUNK, len(rle) - off)
            take -= take % 5                      # whole records only
            if take == 0:
                take = 5
            payload = binascii.hexlify(rle[off:off + take])
            flags = (1 if off == 0 else 0) | (2 if off + take >= len(rle) else 0)
            self.s.write(b"P " + format(take // 5 * 32, "x").encode() + b" "
                         + format(len(payload), "x").encode() + b" "
                         + str(self.PIO_FREQ).encode() + b" "
                         + str(flags).encode() + b"\n")
            self.s.write(payload)
            ack = self._reply(timeout=300.0)
            if self.verbose and ack.startswith(b"K ") and (off // self.PIO_CHUNK) % 4 == 0:
                rd_ms, feed_ms = (int(x) for x in ack[2:].split())
                print(f"        chunk {off // self.PIO_CHUNK + 1}: payload {rd_ms} ms, "
                      f"feeding {feed_ms} ms", flush=True)
            off += take
        if tail:
            bits = (tdi_bits >> (nbits - tail)) & ((1 << tail) - 1)
            self._shift_once(tail, 1 << (tail - 1), bits, False)
        if self.verbose:
            dt = time.time() - t0
            print(f"      {nbits} bits at {self.PIO_FREQ / 4 / 1e6:.1f} MHz TCK "
                  f"in {dt:.1f}s ({nbits / dt / 1000:.0f} kbit/s)", flush=True)
        return 0

    def _bulk_shift(self, nbits, tms_bits, tdi_bits):
        """Shift a huge run in chunks.  The TAP simply stays in Shift-DR between
        chunks because no clock is generated while we are not talking."""
        # `tdi_bits >> b` on a multi-megabit int is O(nbits) for every single bit,
        # which is what made this 7 kbit/s instead of 21: pull the bits out of a
        # byte array instead, one byte index per bit
        tdi = tdi_bits.to_bytes((nbits + 7) >> 3, "little")
        last_only = tms_bits == (1 << (nbits - 1))
        off = 0
        t0 = time.time()
        while off < nbits:
            take = min(self.BULK_CHUNK, nbits - off)
            chunk = bytearray((take + 3) >> 2)
            j = 0
            for b in range(off, off + take):
                pair = ((tdi[b >> 3] >> (b & 7)) & 1) << 1
                if last_only and b == nbits - 1:
                    pair |= 1
                chunk[j >> 2] |= pair << ((j & 3) << 1)
                j += 1
            payload = binascii.hexlify(bytes(chunk))
            self.s.write(b"B " + format(take, "x").encode() + b" "
                         + format(len(payload), "x").encode() + b"\n")
            self.s.write(payload)
            self._reply(timeout=600.0)
            off += take
            if self.verbose and (off % (self.BULK_CHUNK * 8) == 0 or off >= nbits):
                rate = off / max(1e-6, time.time() - t0) / 1000.0
                print(f"      {off}/{nbits} bits  ({time.time() - t0:.1f}s, "
                      f"{rate:.0f} kbit/s)", flush=True)
        return 0

    def _shift_once(self, nbits, tms_bits, tdi_bits, readback):
        if not readback and nbits > 4096:
            if self.use_pio:
                try:
                    return self._pio_shift(nbits, tms_bits, tdi_bits)
                except RuntimeError as e:
                    print(f"   (PIO path refused: {e} - falling back to the slow "
                          f"CPU path)", flush=True)
                    self.use_pio = False
            return self._bulk_shift(nbits, tms_bits, tdi_bits)
        data = bytearray((nbits + 3) >> 2)
        for i in range(nbits):
            pair = ((tms_bits >> i) & 1) | (((tdi_bits >> i) & 1) << 1)
            data[i >> 2] |= pair << ((i & 3) << 1)
        cmd = b"D" if readback else b"S"
        payload = binascii.hexlify(bytes(data))
        # long payloads are split over several writes, the slave reads a line
        self.s.write(cmd + b" " + format(nbits, "x").encode() + b" " + payload + b"\n")
        line = self._reply(timeout=1800.0)
        if not readback:
            return 0
        raw = binascii.unhexlify(line)
        if len(raw) != (nbits + 7) >> 3:
            raise RuntimeError(f"readback was {len(raw)} bytes, wanted {(nbits + 7) >> 3}")
        val = 0
        for i, byte in enumerate(raw):
            for b in range(8):
                if i * 8 + b < nbits and (byte >> b) & 1:
                    val |= 1 << (i * 8 + b)
        return val

    def _shift(self, nbits, tms_bits, tdi_bits, readback):
        last = None
        for attempt in (1, 2, 3):
            try:
                return self._shift_once(nbits, tms_bits, tdi_bits, readback)
            except (RuntimeError, TimeoutError) as e:
                last = e
                if attempt == 3:
                    break
                print(f"   ({e} - resyncing, try {attempt + 1})", flush=True)
                self.s.reset_input_buffer()
                self.reset()
        raise RuntimeError(f"shift failed after resyncs: {last}")

    def reset(self):
        """TAP reset + resync: the slave answers every reset with SYNC, and the
        host scans for it, so leftover boot bytes can never desync the stream."""
        junk = b""
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self.s.write(b"R\n")
            try:
                line = self._line(timeout=max(0.5, deadline - time.time()))
            except TimeoutError as e:
                junk += str(e).encode()
                continue
            if line == b"SYNC":
                if junk and self.verbose:
                    print(f"   (resync: skipped {junk[:40]!r})")
                return
            junk += line + b"|"
        raise RuntimeError(f"slave never answered the reset (saw {junk[:80]!r})")

    def goto_shift_ir(self):
        self._shift(4, 0b0011, 0, False)

    def goto_shift_dr(self):
        self._shift(3, 0b001, 0, False)

    def exit_to_idle(self):
        """From Exit1-IR/Exit1-DR: TMS=1 goes to Update, TMS=0 goes to
        Run-Test/Idle.  The old 1,1,0 walked off into Capture-DR instead, which
        silently broke every instruction load after the first one."""
        self._shift(2, 0b01, 0, False)

    def shift_ir(self, nbits, value, readback=False):
        self.goto_shift_ir()
        r = self._shift(nbits, 0 if nbits == 1 else (1 << (nbits - 1)), value, readback)
        self.exit_to_idle()
        return r

    def shift_dr(self, nbits, value, readback=False):
        self.goto_shift_dr()
        r = self._shift(nbits, 0 if nbits == 1 else (1 << (nbits - 1)), value, readback)
        self.exit_to_idle()
        return r

    def idle_clocks(self, n):
        self._shift(n, 0, 0, False)

    def scan_candidates(self, nbits):
        """Shift the DR while sampling every candidate TDO pin, so a wire that
        ended up on the wrong Pico pin still gives us the data."""
        data = bytearray((nbits + 3) >> 2)
        data[(nbits - 1) >> 2] |= 1 << (((nbits - 1) & 3) << 1)   # TMS high last
        self.s.write(b"M " + format(nbits, "x").encode() + b" "
                     + binascii.hexlify(bytes(data)) + b"\n")
        self._reply()                       # K
        raw = binascii.unhexlify(self._reply())
        per = (nbits + 7) >> 3
        out = []
        for k, pin in enumerate(SCAN_PINS):
            val = 0
            for i, byte in enumerate(raw[k * per:(k + 1) * per]):
                for b in range(8):
                    if i * 8 + b < nbits and (byte >> b) & 1:
                        val |= 1 << (i * 8 + b)
            out.append((pin, val))
        return out


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


def open_slave():
    """Open the port once and reboot the Pico into main.py (the JTAG slave)."""
    port = find_port()
    if not port:
        raise SystemExit("no MicroPython port found")
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.6)
    s.reset_input_buffer()
    s.write(b"\r\x02")           # leave raw REPL, if it is in raw mode
    time.sleep(0.5)
    s.read(8192)
    s.write(b"\r\x04")           # soft reboot -> main.py starts
    time.sleep(2.5)
    banner = s.read(4096)
    print("slave said:", banner.decode("utf-8", "replace").strip()[-60:] or "(nothing)")
    s.reset_input_buffer()
    s.timeout = 0.2
    while s.read(256):           # drain until the line is quiet
        pass
    return s


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "idcode"
    s = open_slave()

    jt = Jtag(s)
    jt.reset()
    idc = jt.shift_dr(32, 0, readback=True)
    print(f"IDCODE = 0x{idc:08X}")
    if idc in (0, 0xFFFFFFFF):
        raise SystemExit("no JTAG answer")

    if cmd == "idcode":
        return

    ops = parse_prg(sys.argv[2])
    bits = sum(n for k, n, *_ in ops if k in ("sir", "sdr"))
    print(f"playing {sys.argv[2]}: {len(ops)} ops, {bits} shift bits")
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
                if bad <= 3:
                    print(f"   mismatch at op {idx}")
        if idx % 20 == 0:
            print(f"   {idx}/{len(ops)}  ({time.time() - t0:.1f}s)", flush=True)
    print(f"done in {time.time() - t0:.1f}s, {bad} readback mismatches")
    print("if the LEDs now run our blink pattern the FPGA is configured :-)")


if __name__ == "__main__":
    main()
