"""One serial session for the whole JTAG job: upload the slave, reboot into it,
then talk JTAG - without ever closing the port in between.

Closing the CDC port while the slave is running kills its stdin for good, so the
old two-process dance (load_mp.py then jtag_run.py) could never work.

usage:  python jtag_do.py idcode
        python jtag_do.py play F:\\FPGA\\proj\\blink\\blink_sram.prg
        python jtag_do.py check          # is the Pico's console reachable?
"""
import os
import sys
import time

import serial
from serial.tools import list_ports

import jtag_run
from jtag_run import Jtag, open_slave, parse_prg

SLAVE = r"F:\FPGA\pico_jtag.py"


def find_port():
    for p in list_ports.comports():
        if (p.vid == 0x2E8A and p.pid == 0x0005) or "MicroPython" in (p.description or ""):
            return p.device
    return None


def drain(s, secs=0.5, limit=65536):
    t0 = time.time()
    got = b""
    while time.time() - t0 < secs:
        c = s.read(512)
        if c:
            got += c
    return got[:limit]


def slave_ping(s, timeout=3.0):
    """Is the JTAG slave already running and listening?  That is the normal case
    right after a boot, because main.py *is* the slave."""
    s.reset_input_buffer()
    s.write(b"R\n")
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout:
        c = s.read(1)
        if c:
            buf += c
            if b"SYNC" in buf:
                return True
    return False


def console_alive(s):
    """Interrupt whatever is running and ask the REPL to prove it is there."""
    s.dtr = False                       # drop DTR/RTS, then raise them again: some
    s.rts = False                       # USB stdio stacks only resume reading
    time.sleep(0.3)                     # after seeing a fresh line state
    s.dtr = True
    s.rts = True
    time.sleep(0.5)
    for _ in range(6):
        s.write(b"\r\x03")
        time.sleep(0.15)
    drain(s, 0.4)
    s.write(b"Q\nQ\nQ\n")               # any slave revision quits on Q
    time.sleep(0.5)
    drain(s, 0.4)
    s.write(b"\r\x02")                  # friendly REPL
    time.sleep(0.4)
    drain(s, 0.4)
    s.write(b"\r\x01")                  # raw REPL
    time.sleep(0.4)
    out = drain(s, 0.4)
    s.write(b"print('PICO-ALIVE')\x04")  # raw REPL: run, Ctrl-D executes
    time.sleep(0.8)
    out += drain(s, 0.6)
    return b"PICO-ALIVE" in out, out


def upload_slave(s):
    code = open(SLAVE, encoding="utf-8").read()
    save = ("f=open('main.py','w')\nf.write(" + repr(code) + ")\nf.close()\n"
            "print('SAVED', len(open('main.py').read()))\n")
    s.write(b"\r\x01")
    time.sleep(0.3)
    drain(s, 0.3)
    s.write(save.encode())
    s.write(b"\x04")
    time.sleep(2.5)
    out = drain(s, 1.0)
    print("upload:", out.decode("utf-8", "replace").strip()[-120:])
    return b"SAVED" in out


def boot_slave(s):
    s.write(b"\r\x02")                  # leave raw REPL
    time.sleep(0.5)
    drain(s, 0.4)
    s.write(b"\r\x04")                  # soft reboot -> main.py runs, port stays open
    time.sleep(2.0)
    out = drain(s, 0.6)
    print("slave said:", out.decode("utf-8", "replace").strip()[-60:] or "(nothing)")
    return b"JTAG-READY" in out


def quit_slave(s):
    """Leave the Pico at the REPL instead of running the slave with a soon-to-be
    closed port, which is how it got wedged before."""
    try:
        s.write(b"Q\n")
        time.sleep(0.4)
        s.read(4096)
    except Exception:
        pass
    s.close()


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "idcode"
    port = find_port()
    if not port:
        raise SystemExit("no MicroPython port found")
    print("port:", port)
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.8)
    s.reset_input_buffer()

    if cmd == "check":
        print("slave running:", slave_ping(s))
        quit_slave(s)
        return

    # always reload the slave: a slave from an earlier revision answers the ping
    # too, and now that the slave no longer swallows Ctrl-C, the interrupt gets
    # us back to the REPL reliably
    ok, out = console_alive(s)
    print("console:", "reachable" if ok else "NO ANSWER",
          "|", out.decode("utf-8", "replace").strip()[-60:].replace("\n", " "))
    if not ok:
        quit_slave(s)
        raise SystemExit("the Pico's console does not answer - unplug and replug "
                         "the Pico's USB cable")
    if not upload_slave(s):
        quit_slave(s)
        raise SystemExit("upload of main.py failed")
    if not boot_slave(s):
        quit_slave(s)
        raise SystemExit("the slave did not start (no JTAG-READY)")

    s.reset_input_buffer()
    jt = Jtag(s)
    delay = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    s.write(b"T %d\n" % delay)          # microseconds of TCK settle time
    jt._reply()
    print(f"TCK settle time: {delay} us")

    jt.reset()
    jt.shift_ir(10, 0x6)                # IDCODE instruction, IR is 10 bits wide
    idc = 0
    for _ in range(3):
        jt.reset()
        jt.shift_ir(10, 0x6)
        idc = jt.shift_dr(32, 0, readback=True)
        if idc == 0x01000001:
            break
    print(f"IDCODE = 0x{idc:08X}")
    if idc in (0, 0xFFFFFFFF):
        print("no JTAG answer - check the four wires and the board's power")
        quit_slave(s)
        return
    if idc == 0x01000011:
        print('   chip answers 0x01000011 = the AG10KL144H variant, so projects must be '
              'built for it (af_run.tcl: set DEVICE "AG10KL144H").  A bitstream built '
              'for a plain AG10KL144 expects 0x01000001 and this chip silently rejects it.')
    elif idc != 0x01000001:
        print("   note: unexpected IDCODE for both known variants")
    if cmd == "idcode":
        quit_slave(s)
        return

    ops = parse_prg(sys.argv[2])
    bits = sum(n for k, n, *_ in ops if k in ("sir", "sdr"))
    # JTAG_MAX_OPS=N plays only the first N operations.  Useful to dry-run a flash
    # loader up to its first status read, without writing anything to the flash.
    max_ops = int(os.environ.get("JTAG_MAX_OPS", "0"))
    if max_ops:
        ops = ops[:max_ops]
        print(f"dry run: only the first {len(ops)} of the file's operations")
    print(f"playing {sys.argv[2]}: {len(ops)} ops, {bits} shift bits")
    t0 = time.time()
    bad = 0
    for idx, (kind, n, tdi, tdo, mask) in enumerate(ops):
        t1 = time.time()
        if kind == "idle":
            # `runtest -tck N` is a wait, and the .prg was written for a roughly
            # 21 kHz cable: 100 clocks means "give the FPGA a few milliseconds to
            # think".  Clocking those 100 bits at 2 MHz would be 50 us, and then
            # the status reads come back as zeros because the device is not ready,
            # so the wall-clock wait is honoured as well.
            jt.idle_clocks(n)
            time.sleep(n / 21000.0)
        elif kind == "sir":
            jt.shift_ir(n, tdi)
        else:
            # a zero mask means "do not bother checking", so do not pay for the
            # readback of a multi-megabit shift either; and a huge readback would
            # not fit in the Pico's 264 kB of RAM anyway (157 kB of TDO packed
            # plus the hex of it), so skip the host-side check and say so
            want_rb = bool(tdo is not None and mask)
            if want_rb and n > 65536:
                print(f"   op {idx + 1}: {n} bits with a TDO check - skipping the "
                      f"check, the Pico cannot hold that readback", flush=True)
                want_rb = False
            got = jt.shift_dr(n, tdi, readback=want_rb)
            if want_rb and mask and (got ^ tdo) & mask:
                bad += 1
                print(f"   mismatch at op {idx} ({kind} {n}): got 0x{got:X} "
                      f"wanted 0x{tdo:X} mask 0x{mask:X}")
        print(f"   op {idx + 1}/{len(ops)} {kind} {n} bits  "
              f"{time.time() - t1:.1f}s  (total {time.time() - t0:.1f}s)", flush=True)
    print(f"done in {time.time() - t0:.1f}s, {bad} readback mismatches")
    print("if the LEDs run our blink pattern now, the FPGA is configured :-)")
    quit_slave(s)


if __name__ == "__main__":
    main()
