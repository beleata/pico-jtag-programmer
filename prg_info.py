"""Inspect an AGM / Altera .prg file before you spend time playing it.

    python prg_info.py path\\to\\design_sram.prg

It prints the instruction widths, how many shift bits the file contains, the
IDCODE it expects, and the biggest runs.  The expected IDCODE is the important
one: a bitstream built for the wrong device variant is silently rejected by the
FPGA, so compare this number with `python jtag_do.py idcode`.
"""
import sys
from collections import Counter

from jtag_run import parse_prg


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    path = sys.argv[1]
    ops = parse_prg(path)
    if not ops:
        raise SystemExit(f"{path}: no sir/sdr/runtest operations found - not a .prg?")

    kinds = Counter(k for k, *_ in ops)
    shift_bits = sum(n for k, n, *_ in ops if k in ("sir", "sdr"))
    ir_widths = sorted({n for k, n, *_ in ops if k == "sir"})
    idcodes = [o for o in ops if o[0] == "sdr" and o[3] is not None and o[4] == 0xFFFFFFFF]
    drift = sum(n for k, n, *_ in ops if k == "idle")
    big = sorted(((n, k) for k, n, *_ in ops if k in ("sir", "sdr")), reverse=True)[:5]

    print(f"file            : {path}")
    print(f"operations      : {len(ops)}  {dict(kinds)}")
    print(f"shift bits      : {shift_bits}")
    print(f"idle clocks     : {drift}")
    print(f"IR widths       : {ir_widths or 'none'}")
    if idcodes:
        print(f"expected IDCODE : 0x{idcodes[0][3]:08X}   "
              f"(from the first fully-masked DR check)")
    else:
        print("expected IDCODE : none - this file has no full-mask IDCODE check")
    print(f"largest shifts  : {[f'{n} {k}' for n, k in big]}")

    print()
    print("compare the IDCODE with the chip:")
    print("  python jtag_do.py idcode")
    print("AG10KL144H answers 0x01000011, a plain AG10KL144 expects 0x01000001.")


if __name__ == "__main__":
    main()
