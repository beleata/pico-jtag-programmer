# JTAG bit-banger for the Marble Pico (RP2040) - MicroPython
#
# Wiring (same pins as the USB-Blaster firmware):
#   GP11 -> TCK      GP12 -> TMS      GP15 -> TDI      GP16 -> TDO      GND -> GND
#
# The protocol is deliberately TEXT, one command per line, because binary
# payloads are unusable on MicroPython's USB stdin: any 0x03 byte in the stream
# is swallowed by the stdio layer and raises KeyboardInterrupt, which silently
# desyncs the slave.  Hex costs bandwidth, not correctness.
#
#   host -> pico                      pico -> host
#   R\n                               SYNC\n
#   S <nbits-hex> <datahex>\n         K\n
#   D <nbits-hex> <datahex>\n         <readback hex>\n
#   Q\n                               BYE\n
#   anything broken                   E <text>\n
#
# data packs 4 clocks per byte, two bits per clock: [tms0,tdi0, tms1,tdi1, ...]
# readback packs TDO 8 bits per byte, LSB first.
import binascii
import sys
import time
from array import array

import machine
from machine import Pin

try:
    import rp2
except ImportError:
    rp2 = None

TCK, TMS, TDI, TDO = 11, 12, 15, 16
SET, CLR, IN = 0xD0000014, 0xD0000018, 0xD0000004   # RP2040 SIO registers
mem = machine.mem32

TCK_M, TMS_M, TDI_M, TDO_M = 1 << TCK, 1 << TMS, 1 << TDI, 1 << TDO

# candidate inputs for TDO: if the TDO wire sits on the wrong header pin of the
# Pico we still find the IDCODE by listening on all of them at once
SCAN_PINS = (16, 17, 13, 14, 18, 19, 20, 21, 22)
SCAN_M = tuple(1 << p for p in SCAN_PINS)

DELAY = 0        # microseconds of settle time around each TCK edge; three SIO
                 # writes in a row make a ~30 MHz clock, which is too fast for
                 # the FPGA's TDO to keep up with

for p in (TCK, TMS, TDI):
    Pin(p, Pin.OUT, value=0)
Pin(TDO, Pin.IN)
mem[CLR] = TCK_M | TMS_M | TDI_M

write = sys.stdout.buffer.write
_buf = bytearray()

# ---------------------------------------------------------------- PIO bulk engine
# The interpreter costs ~47 us per JTAG bit (every SIO store is 8 us here), which
# made the whole bitstream take seven minutes.  A PIO state machine does the same
# job in hardware: 4 PIO cycles per bit, so at 8 MHz it clocks 2 Mbit/s.
#
#   side-set bit 0 -> GP11 TCK      out pins -> GP15 TDI
#   side-set bit 1 -> GP12 TMS      in  pins -> GP16 TDO
#
# `out pins, 1` stalls while the TX FIFO is empty, and it stalls with TCK low, so
# the shift simply pauses whenever we are not feeding it.
if rp2:
    @rp2.asm_pio(
        sideset_init=(rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW),
        out_init=(rp2.PIO.OUT_LOW,),
        out_shiftdir=rp2.PIO.SHIFT_RIGHT, autopull=True, pull_thresh=32,
        in_shiftdir=rp2.PIO.SHIFT_RIGHT,
    )
    def _jtag_prog():
        wrap_target()
        set(x, 31)             .side(0)
        label("inner")
        out(pins, 1)           .side(0)      # TDI changes while TCK is low
        nop()                  .side(1)      # TCK rising: the FPGA samples TDI
        in_(pins, 1)           .side(1)      # sample TDO while TCK is high
        jmp(x_dec, "inner")    .side(0)      # TCK falling
        push(noblock)          .side(0)
        wrap()


_sm = None


def pio_start(freq):
    global _sm
    if _sm:
        try:
            _sm.active(0)
        except Exception:
            pass
    _sm = rp2.StateMachine(0, _jtag_prog, freq=freq, sideset_base=Pin(TCK),
                          out_base=Pin(TDI), in_base=Pin(TDO))
    _sm.active(1)


def pio_feed(raw):
    """Decode the run-length encoded payload straight into the FIFO.

    Two things matter here:

    * put() must not be allowed to block: if the state machine ever stops
      consuming, a blocking put() waits forever inside C where no Ctrl-C can
      reach it, which is how the Pico got wedged.  So the FIFO is polled instead.
    * The payload is runs of 32-bit words, (count, word), because the raw stream
      is mostly zeros: one of the configuration runs is 2.1 Mbit of pure zeros,
      which is 540 kB raw and 5 kB encoded.  MicroPython reads USB stdin at only
      about 110 kB/s, so the transfer, not the shifting, is the bottleneck.
    """
    sm = _sm
    i = 0
    n = len(raw)
    while i < n:
        cnt = raw[i]
        w = raw[i + 1] | (raw[i + 2] << 8) | (raw[i + 3] << 16) | (raw[i + 4] << 24)
        i += 5
        if cnt == 1:
            while sm.tx_fifo() >= 4:
                time.sleep_us(20)
            sm.put(w)
        else:
            for _ in range(cnt):
                while sm.tx_fifo() >= 4:
                    time.sleep_us(20)
                sm.put(w)


def pio_selftest(freq=8000000):
    """Feed two words and see whether they get consumed - i.e. whether the state
    machine is actually running."""
    if not rp2:
        return "no rp2 module in this build"
    try:
        pio_start(freq)
        _sm.put(0xFFFFFFFF)
        _sm.put(0xFFFFFFFF)
        time.sleep_ms(5)
        return "active=%d tx_fifo=%d rx_fifo=%d" % (_sm.active(), _sm.tx_fifo(),
                                                    _sm.rx_fifo())
    except Exception as e:
        return "exception " + repr(e)
    finally:
        pio_finish(freq)


def pio_finish(freq):
    """Let the last word clock out, then hand the pins back to the CPU."""
    global _sm
    if _sm:
        time.sleep_us(int(4 * 32 * 1000000 / freq) + 300)
        _sm.active(0)
        _sm = None
    Pin(TCK, Pin.OUT, value=0)
    Pin(TMS, Pin.OUT, value=0)
    Pin(TDI, Pin.OUT, value=0)


def pio_shift(raw, nclocks, freq):
    """Clock nclocks bits with TMS held low, taking TDI from `raw` (one bit per
    clock, LSB first).  nclocks must be a multiple of 32."""
    sm = rp2.StateMachine(0, _jtag_prog, freq=freq, sideset_base=Pin(TCK),
                          out_base=Pin(TDI), in_base=Pin(TDO))
    sm.active(1)
    for i in range(0, len(raw), 4):
        c = raw[i:i + 4]
        sm.put(c[0] | (c[1] << 8) | (c[2] << 16) | (c[3] << 24))
    # one word is 32 bits x 4 cycles; give it room to finish the last word
    time.sleep_us(int(4 * 32 * 1000000 / freq) + 300)
    sm.active(0)
    # hand the pins back to the CPU
    Pin(TCK, Pin.OUT, value=0)
    Pin(TMS, Pin.OUT, value=0)
    Pin(TDI, Pin.OUT, value=0)


def read_exact(n):
    """Exactly n raw bytes.  read(want) blocks until `want` bytes have arrived,
    which is what we want here because the host promises to send all of them, and
    it turns the byte-at-a-time stdin path into 4 kB pages.

    KeyboardInterrupt is deliberately NOT caught: it can only come from a real
    Ctrl-C (the payload is ASCII hex, which never contains 0x03), and letting it
    out drops us back to the REPL, which is how the host recovers the Pico."""
    buf = bytearray()
    rd = sys.stdin.buffer.read
    while len(buf) < n:
        want = n - len(buf)
        if want > 4096:
            want = 4096
        chunk = rd(want)
        if chunk:
            buf.extend(chunk)
    return buf


def readline():
    """One newline-framed line of stdin.

    read(1) and not read(n): on MicroPython's USB stdio a read larger than the
    data waiting BLOCKS until it is filled, so read(1024) would wedge the slave
    forever after a 2-byte command.  The buffer is a bytearray (bytes += chunk
    would be quadratic) and the search for the newline resumes where it stopped
    for the same reason.  read(1) also never over-reads into the next command."""
    global _buf
    start = 0
    while True:
        i = _buf.find(b"\n", start)
        if i >= 0:
            line = bytes(_buf[:i])
            _buf = _buf[i + 1:]          # MicroPython bytearray has no del slice
            return line.strip()
        start = len(_buf)
        chunk = sys.stdin.buffer.read(1)
        if chunk:
            _buf.extend(chunk)


def tap_reset():
    """TMS high for 8 clocks puts the TAP in Test-Logic-Reset, and one more clock
    with TMS low walks it out to Run-Test/Idle, which is the state every other
    navigation step in jtag_run.py assumes it starts from."""
    for _ in range(8):
        mem[SET] = TMS_M
        mem[CLR] = TCK_M
        mem[SET] = TCK_M
    mem[CLR] = TMS_M
    mem[CLR] = TCK_M
    mem[SET] = TCK_M          # TMS low: Test-Logic-Reset -> Run-Test/Idle
    mem[CLR] = TCK_M


def shift(nbits, data, readback):
    out = bytearray((nbits + 7) >> 3) if readback else None
    ob = nb = oi = 0
    for i in range(nbits):
        pair = (data[i >> 2] >> ((i & 3) << 1)) & 3
        if pair & 1:
            mem[SET] = TMS_M
        else:
            mem[CLR] = TMS_M
        if pair & 2:
            mem[SET] = TDI_M
        else:
            mem[CLR] = TDI_M
        mem[CLR] = TCK_M          # data changes on the falling edge
        if DELAY:
            time.sleep_us(DELAY)
        mem[SET] = TCK_M          # device samples on the rising edge
        if readback:
            if DELAY:
                time.sleep_us(DELAY)
            if mem[IN] & TDO_M:
                ob |= 1 << nb
            nb += 1
            if nb == 8:
                out[oi] = ob
                oi += 1
                ob = nb = 0
    if readback and nb:
        out[oi] = ob
    return out


def scan_many(nbits, data):
    """Like shift(), but samples every SCAN_PINS input on every clock."""
    nb = (nbits + 7) >> 3
    outs = [bytearray(nb) for _ in SCAN_PINS]
    obs = [0] * len(SCAN_PINS)
    cnt = [0] * len(SCAN_PINS)
    idx = [0] * len(SCAN_PINS)
    for i in range(nbits):
        pair = (data[i >> 2] >> ((i & 3) << 1)) & 3
        if pair & 1:
            mem[SET] = TMS_M
        else:
            mem[CLR] = TMS_M
        if pair & 2:
            mem[SET] = TDI_M
        else:
            mem[CLR] = TDI_M
        mem[CLR] = TCK_M
        mem[SET] = TCK_M
        v = mem[IN]
        for k in range(len(SCAN_PINS)):
            if v & SCAN_M[k]:
                obs[k] |= 1 << cnt[k]
            cnt[k] += 1
            if cnt[k] == 8:
                outs[k][idx[k]] = obs[k]
                idx[k] += 1
                obs[k] = 0
                cnt[k] = 0
    for k in range(len(SCAN_PINS)):
        if cnt[k]:
            outs[k][idx[k]] = obs[k]
    return outs


write(b"JTAG-READY\n")

while True:
    try:
        line = readline()
        if not line:
            continue
        parts = line.split()
        cmd = parts[0]
        if cmd == b"R":
            tap_reset()
            write(b"SYNC\n")
        elif cmd == b"Q":
            write(b"BYE\n")
            break
        elif cmd == b"U":
            # 0 = no pull, 1 = pull-down, 2 = pull-up on the TDO input, so the
            # host can tell "the FPGA drives TDO" from "TDO just floats".
            # (This used to be P, which then shadowed the PIO command below.)
            pull = int(parts[1])
            Pin(TDO, Pin.IN, None if pull == 0
                else (Pin.PULL_DOWN if pull == 1 else Pin.PULL_UP))
            write(b"K\n")
        elif cmd == b"T":
            DELAY = int(parts[1])         # TCK settle time in microseconds
            write(b"K\n")
        elif cmd == b"M":
            nbits = int(parts[1], 16)
            data = binascii.unhexlify(parts[2])
            write(b"K\n")
            write(binascii.hexlify(b"".join(scan_many(nbits, data))) + b"\n")
        elif cmd == b"P" and rp2:
            # hardware-shifted bulk run, fed in chunks so both the payload and the
            # decoded data always fit in the Pico's 264 kB of RAM:
            #   P <clocks-hex> <hexlen-hex> <freq> <flags>
            # flags bit0 = create the state machine (first chunk)
            # flags bit1 = finish, wait for the last word, release the pins
            # The payload is hex of (count, word) run records, see pio_feed.
            # The TAP simply stays in Shift-DR between chunks, because no clock is
            # generated while we are not feeding the FIFO.
            nclocks = int(parts[1], 16)
            hexlen = int(parts[2], 16)
            freq = int(parts[3])
            flags = int(parts[4])
            t0 = time.ticks_ms()
            raw = binascii.unhexlify(read_exact(hexlen))
            t1 = time.ticks_ms()
            if flags & 1:
                pio_start(freq)
            pio_feed(raw)
            t2 = time.ticks_ms()
            if flags & 2:
                pio_finish(freq)
            write(b"K %d %d\n" % (time.ticks_diff(t1, t0), time.ticks_diff(t2, t1)))
        elif cmd == b"W":
            write(b"W " + pio_selftest().encode() + b"\n")
        elif cmd == b"B":
            # bulk shift: B <nbits-hex> <hexlen-hex> then that many ASCII hex
            # characters, four clocks per packed byte.  Used for the multi-megabit
            # configuration shifts, where hex over the byte-at-a-time stdin path
            # is hopeless.  It is hex and not raw bytes because a raw 0x03 would
            # be swallowed by MicroPython's stdio layer as Ctrl-C.
            nbits = int(parts[1], 16)
            hexlen = int(parts[2], 16)
            data = binascii.unhexlify(read_exact(hexlen))
            shift(nbits, data, False)
            write(b"K\n")
        elif cmd in (b"S", b"D"):
            nbits = int(parts[1], 16)
            data = binascii.unhexlify(parts[2])
            res = shift(nbits, data, cmd == b"D")
            write(b"K\n" if res is None else binascii.hexlify(res) + b"\n")
        else:
            write(b"E unknown command\n")
    except KeyboardInterrupt:
        continue
    except Exception as exc:                 # never die on a bad line
        try:
            write(b"E " + repr(exc).encode() + b"\n")
        except Exception:
            pass
