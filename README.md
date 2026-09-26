# Pico JTAG programmer for AGM / Altera FPGAs

Program an **AGM AG10KL144H** (or any Altera-compatible FPGA, if you have its bitstream
as an SVF-like `.prg` file) from a plain **Raspberry Pi Pico** — no USB Blaster, no FTDI
cable, no Quartus.

Verified on an **AGM TCX** board (AG10KL144H, LQFP-144): the FPGA is configured in
**~13 seconds** for an 8.8 Mbit design, and it survives nothing — this is the volatile
SRAM configuration path (`*_sram.prg`).

---

## Why this exists

The vendor toolchain (Supra, AGM's Quartus fork) can synthesise a bitstream for the
AG10K family entirely offline, but its **Downloader only talks to an FTDI-based
USB-Blaster cable**. A Pico can *emulate* the USB-Blaster protocol (see the bonus
section at the end — it works at protocol level), but both Supra and OpenOCD refuse
non-FTDI transports, so the standard tools are out unless you buy a real cable.

This project replaces that last step: the Pico bit-bangs JTAG from MicroPython, using
the RP2040's **PIO** hardware so the shifting is fast, and plays the `.prg` files Supra
already generates.

---

## Hardware

* Raspberry Pi Pico (RP2040, any variant) running **MicroPython**
* 4 jumper wires plus a ground

| Pico GPIO | physical pin | FPGA / JTAG connector | signal |
|---|---|---|---|
| GP11 | 15 | TCK — JTAG pin 1 (the square pad) | clock |
| GP12 | 16 | TMS — JTAG pin 5 | mode select |
| GP15 | 20 | TDI — JTAG pin 9 | data into the FPGA |
| GP16 | 21 | TDO — JTAG pin 3 | data out of the FPGA |
| GND | 3 / 8 / 13 / 18 / … | JTAG pin 2 or 10 | ground |

Notes:

* The RP2040 is a 3.3 V part and the AG10K JTAG pins are 3.3 V, so no level shifter is
  needed. Do **not** copy the pinout of another USB-Blaster firmware: the popular
  [pico-usb-blaster](https://github.com/thisiseth/pico-usb-blaster) uses the same pins
  for TCK/TMS but drives TDI on GP15/TDO on GP16 **only in its default build** — its
  "prettier" variant uses pins 8..14 and will look like a wiring fault.
* Measure, do not assume: the square pad on the header is pin 1, and on some boards the
  silkscreen marking is misleading. `pico_pins.py` and `pico_padtest.py` (below) verify
  every wire electrically before you blame the FPGA.

---

## Quick start

1. Flash **MicroPython** onto the Pico: hold BOOTSEL, plug in USB, drop
   `micropython-rp2040.uf2` onto the `RPI-RP2` drive.
2. `pip install pyserial`
3. Read the chip's IDCODE:

   ```
   python jtag_do.py idcode
   ```

   Expected for an AG10KL144H: `IDCODE = 0x01000011`
4. Configure the FPGA (this is the volatile SRAM load):

   ```
   python jtag_do.py play path\to\design_sram.prg
   ```

   Add a trailing number to tune the TCK settle time for the slow CPU path
   (`... 2` = 2 µs), and set `JTAG_NO_PIO=1` to force that slow path.

`jtag_do.py` does everything in **one USB serial session**: it wakes the Pico's REPL,
uploads `pico_jtag.py` as `main.py`, soft-reboots into it, then speaks the JTAG
protocol. Closing the port in between kills the slave's stdin for good, which is why
there is no separate "upload" step to run by hand.

---

## The whole chain: from Verilog to a running FPGA

This is the complete path, using `examples/blink` (four LEDs blinking together at 1 Hz
on an AGM TCX board). Quartus is not involved anywhere.

### 0. What you need

* The AGM SDK package (`AgRV_pio`, which contains Supra): `bin/af.exe`,
  `map/bin/yosys.exe`. `%SDK%` below means that package's `tool-agrv_logic` folder.
* Python 3 with `pyserial` on the PC, MicroPython on the Pico.
* The board powered, four wires as in the table above.

### 1. Write the design — `blink.v`

```verilog
module blink(
  input  clk,          // 50 MHz oscillator, pin 23 on this board
  input  resetn,       // active low
  output reg [3:0] led // D0..D3, pins 141..144, active low
);
  reg [25:0] cnt;
  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      cnt <= 26'd0;
      led <= 4'b0000;
    end else if (cnt == 26'd24_999_999) begin
      cnt <= 26'd0;
      led <= ~led;     // all four toggle together, 1 Hz
    end else begin
      cnt <= cnt + 26'd1;
    end
  end
endmodule
```

### 2. Pin constraints — `blink.ve`

One line per port; the numbers come from the board schematic (the AGM TCX board has a
50 MHz oscillator on pin 23 and the LEDs on 141–144):

```
clk PIN_23
resetn PIN_25
led[0] PIN_141
led[1] PIN_142
led[2] PIN_143
led[3] PIN_144
```

The LEDs are **active low**, which is why the design drives them with `0` to light them
and why an un-driven or un-configured FPGA shows them dark.

### 3. Timing constraints — `blink.sdc`

```
create_clock -name clk -period 20.000 [get_ports {clk}]
```

Keep this file to real constraints only. Putting `read_sdc "blink.sdc"` inside it makes
the tool read itself recursively and hang.

### 4. Create the project (once per design)

```
cd examples\blink
"%SDK%\bin\af.exe" --setup --design blink --top_module blink ^
        --device AG10KL144H --verilog blink.v --ve blink.ve
```

This writes `af_run.tcl`, `af_map.tcl` and friends, with the device baked into
`af_run.tcl` as `set DEVICE "AG10KL144H"`. **That string has to match the silicon** —
see the warning further down, it is the single most expensive mistake in this flow.

### 5. Synthesise, place, route, generate the bitstream

```
"%SDK%\map\bin\yosys.exe" -c af_map.tcl
"%SDK%\bin\af.exe" --batch --mode NATIVE
```

Afterwards the folder contains:

| file | what it is | used by |
|---|---|---|
| `blink_sram.prg` | volatile configuration over JTAG (SRAM) | **this programmer** |
| `blink_master.prg` | writes the design into the serial configuration flash, over JTAG | this programmer (untested here), or a real cable |
| `blink_master_as.prg` | AS-mode programming: needs a cable that drives nCS/DCLK/ASDI | a real cable |
| `blink_slave.rbf` | raw bitstream for passive-serial (PS) loading | another controller |
| `blink.bin` | raw configuration data | your own loader |

### 6. Check the file before you play it

```
python prg_info.py examples\blink\blink_sram.prg
python jtag_do.py idcode
```

The expected IDCODE printed by `prg_info.py` must equal the chip's answer. For the AGM
TCX board a correct `_sram.prg` is **81 operations / 8 786 472 shift bits / expected
IDCODE 0x01000011** — identical in structure to the vendor's own demo file. A file with
63 operations and 5 312 316 bits was built for a plain AG10KL144 and is rejected without
a single error message by this chip.

### 7. Load it into the FPGA

```
python jtag_do.py play examples\blink\blink_sram.prg
```

The LEDs stop showing the factory pattern and start blinking together: that is your
design running. The whole run takes about 13 seconds and reports `0 readback
mismatches`, which is the `.prg`'s own TDO verification talking to you.

### 8. Iterate

Edit the Verilog, then repeat steps 5 and 7: roughly one minute of build and 13 seconds
of programming. **Nothing about the programmer is design specific** — it plays whatever
`.prg` you give it, so a 20 000-LUT design needs no changes at all, only more seconds.

### 9. Make it permanent

The SRAM configuration above is lost when the board loses power. `blink_master.prg`
writes the same design into the EPCS4 configuration flash over the same JTAG path and
is played exactly the same way:

```
python jtag_do.py play examples\blink\blink_master.prg
```

We tried this on an AGM TCX board and **it does not work yet**. The run takes ~10
minutes (14971 operations, none of them large enough for the PIO path) and then reports
805 readback mismatches, starting at operation 28: a 40-bit status read of the FPGA's
flash loader returns `0xC000000000` where the file expects `0x0`, and the mask on that
operation only checks the top bit — the loader is raising an error or busy flag.

The decisive test: dry-running the **vendor's own** `HY601_master.prg` up to that same
operation gives the identical answer,

```
mismatch at op 28 (sdr 40): got 0xC000000000 wanted 0x0 mask 0x8000000000
```

so the bitstream is not at fault — the loader handshake itself does not work through
this programmer. It fails identically with the FPGA left unconfigured at power-up (the
flash already empty), so the device state is not the reason either.

AGM document the rule themselves: their downloader (a DAP-Link / CMSIS-DAP probe,
described as the equivalent of an Altera USB Blaster) is the hardware that supports
writing the SPI configuration flash. Four JTAG wires and DR/IR sequences are not enough,
which is exactly what that top status bit is telling us.

After a power cycle the board came up with no configuration at all, so
the flash no longer holds a valid design and the factory demo that shipped in it is
gone. SRAM programming is unaffected and works every time.

Practical advice: if you value the vendor design in your board's flash, dump it first
(or leave the flash alone) until this path is proven, and use a real USB Blaster with
the official Downloader for flash writes. Reproducing the check is one command:

```
set JTAG_MAX_OPS=30
python jtag_do.py play HY601_master.prg
```

---

## How it works

### 1. The transport is text, on purpose

Commands are newline-framed ASCII hex (`S <bits> <hex>`, `D …` with TDO readback,
`B …` bulk, `P …` PIO bulk, `R` TAP reset, `Q` quit). That is not elegance, it is
survival instinct — MicroPython's USB stdio has two traps:

* `sys.stdin.buffer.read(n)` **blocks until n bytes have arrived**, so a single
  `read(1024)` wedges the interpreter forever when a 2-byte command shows up. The slave
  therefore reads lines one byte at a time (`read(1)`) and only uses `read(4096)` when
  the host has promised that many bytes.
* Byte `0x03` is swallowed by the stdio layer and raises `KeyboardInterrupt`. Raw binary
  payloads silently lose every 0x03 they contain, so payloads are hex (or, for the PIO
  path, hex in 4 kB pages).

### 2. Shifting is done in hardware (PIO)

A MicroPython bit-bang costs ~47 µs per JTAG bit here — every `machine.mem32` store is
about 8 µs on this chip — which made a full bitstream take **7 minutes**. One PIO state
machine does the same job in 4 PIO cycles per bit:

```
.program jtag_shift
.side_set 2                      ; side-set bit0 -> TCK (GP11), bit1 -> TMS (GP12)
    set(x, 31)          .side(0)
  inner:
    out(pins, 1)        .side(0) ; TDI (GP15) changes while TCK is low
    nop()               .side(1) ; rising edge: the FPGA samples TDI
    in_(pins, 1)        .side(1) ; sample TDO (GP16) while TCK is high
    jmp(x_dec, "inner") .side(0) ; falling edge
    push(noblock)       .side(0)
```

`out pins, 1` stalls while the TX FIFO is empty **and stalls with TCK low**, so the TAP
simply stays in Shift-DR between chunks and the shift can be fed in pieces.

### 3. Two rules that are easy to get wrong

* **No padding in a DR run.** The PIO loop is word granular, so the obvious trick is to
  pad the front with zeros. Do not: the FPGA's configuration logic takes the *first*
  bits clocked into the DR as data, so a single pad bit shifts the whole bitstream and
  the device quietly ends up with garbage (symptom: its status reads come back `0x0`).
  Instead the PIO clocks a whole number of 32-bit words and the final ≤32 bits are
  clocked by the CPU, where the TMS=1 exit clock is expressible.
* **`runtest -tck N` is a delay, not a clock count.** A `.prg` is written for a roughly
  21 kHz cable, so `idle 100` means "give the FPGA a few milliseconds". Clocking those
  100 bits at 2 MHz takes 50 µs and the following status reads return zeros, so the
  wall-clock wait is honoured as well.

### 4. The transfer is the bottleneck, not the shifting

MicroPython reads USB stdin at about **110 kB/s**, and the raw stream is one bit per
clock. The payload is therefore RLE-compressed as runs of 32-bit words `(count, word)`:
the main configuration run is 89 % zero bytes and two 2.1 Mbit runs are 100 % zeros
(540 kB → 5 kB each). Everything is chunked to fit the RP2040's 264 kB of RAM — the
whole bitstream never fits, and neither does a 157 kB TDO readback, which is why
readback checks for runs above 65536 bits are skipped.

### 5. TAP navigation

```
reset:            8 clocks with TMS high, then 1 clock with TMS low  -> Run-Test/Idle
goto Shift-DR:    1,0,0     goto Shift-IR: 1,1,0,0
leave Shift:      1,0       (Exit1 -> Update -> Run-Test/Idle)
```

Both of the naive variants cost us hours: leaving a reset without walking out to
Run-Test/Idle makes every later navigation land in the wrong state, and exiting a shift
with `1,1,0` lands in Capture-DR instead of Idle. The symptom is nasty: the IDCODE still
reads (after a reset the default DR *is* the IDCODE), but **no instruction ever loads**,
so every readback returns the IDCODE no matter how long the shift was. If you see that,
check the navigation first.

---

## Performance

| implementation | 8.79 Mbit configuration |
|---|---|
| MicroPython bit-bang, interpreter | 415 s |
| PIO hardware shifting, hex payload | 31.6 s |
| PIO + RLE payload (default) | **12.8 s** |

Comfortably faster than the workflow needs: a Supra rebuild takes ~1 minute.

---

## The device string is the one thing that must be right

Everything else in this flow fails loudly; this fails silently. The commands are in the
walkthrough above — what matters here is the check.

A bitstream built for a plain `AG10KL144` expects IDCODE `0x01000001`. An
AG10KL144**H** answers `0x01000011` and **rejects the file without any error**: no
message from the programmer, no failure from the FPGA — it simply stays blank and its
LEDs go dark. Two cheap checks before you blame anything else:

* `python prg_info.py your_sram.prg` — the expected IDCODE it prints must equal what
  `python jtag_do.py idcode` reports from the chip;
* compare the structure with the vendor's own demo file for the same device. On the AGM
  TCX board both a correct build and the vendor's `HY601_sram.prg` are 81 operations /
  8 786 472 shift bits, while a build for the wrong variant is 63 operations /
  5 312 316 bits.

---

## Diagnostics

| script | what it does |
|---|---|
| `jtag_do.py idcode` | full sequence: upload slave, TAP reset, IDCODE |
| `jtag_irtest.py` | proves the IR loads: IDCODE -> `0x01000011`, BYPASS -> `0x00000000` |
| `jtag_findtdo.py` | samples 9 candidate pins at once, for a TDO wire on the wrong Pico pin |
| `jtag_pulltest.py` | internal pull-down/pull-up on TDO: is the FPGA driving the line at all |
| `jtag_pioselftest.py` | does the PIO state machine actually consume FIFO data |
| `jtag_piohost.py` | throughput of the PIO path, size by size |
| `pico_pins.py` | electrical fingerprint of the four wires from their external pulls |
| `pico_padtest.py` | drives each pin and reads the pad back: catches a wire on GND or 3.3 V |
| `pico_unwedge.py` | pushes 1024+ bytes to free a slave stuck in a blocking read |
| `load_mp.py <file> <secs>` | uploads any MicroPython script as `main.py`, prints its output live |

### Troubleshooting

| symptom | cause | fix |
|---|---|---|
| `IDCODE = 0xFFFFFFFF` | TDO floats: wire off, wrong pin, or the FPGA is not enabled | `pico_pins.py`, `pico_padtest.py`, `jtag_pulltest.py` |
| IDCODE is alternating garbage (`0x5555…`, `0xAA…`) | outputs never enabled, or the bitstream was rejected | check the driver's OE handling / rebuild for the right device |
| every readback equals the IDCODE | the IR never loads: broken TAP navigation | see section 5 |
| status reads return `0x0` after a big write | padding in the DR stream, or `runtest` not honoured in time | see sections 3 |
| the Pico stops answering, no REPL | a blocking `read()` or a blocking PIO `put()` | `pico_unwedge.py`, else replug USB; keep FIFO feeding non-blocking |
| only some bytes of a payload arrive | raw binary with a `0x03` in it | send hex |

---

## Bonus: is a Pico a real USB Blaster?

Partly. With [pico-usb-blaster](https://github.com/thisiseth/pico-usb-blaster) flashed
the Pico enumerates as `09FB:6001` ("Altera USB-Blaster") and speaks the real protocol;
`tools/jtag_usb.py` drives it over libusb and reads the FPGA's IDCODE in both bit-bang
and shift modes. But Supra's `af.exe --prg` still answers *"Couldn't connect to suitable
USB device"* (also after switching the driver to WinUSB, which is what AGM's own Zadig
instruction installs), and the OpenOCD that ships with the SDK is compiled with only
`usb_blaster lowlevel_driver ftdi`. Both expect the **FTDI chip** of a real cable, so a
~€5 clone remains the answer if you want Quartus/Supra/SignalTap, AS/PS modes or flash
programming. For configuring SRAM this tool is faster anyway.

Protocol notes we reverse-engineered while testing, in case they help someone:

```
bit-bang byte: TCK 0x01 | TMS 0x02 | nCE 0x04 | nCS 0x08 | TDI 0x10 | OE 0x20 | read 0x40
shift mode:    0x80 | read(0x40) | byte count (<=63), followed by the data bytes
response:      ALWAYS two preamble bytes 0x31 0x60 before any payload
flushing:      every 10 ms, or when 64 bytes have piled up
```

Watch out: the output-enable bit is `0x20` (mask with `0x3F`, never `0x1F`), nCE must be
driven **low** or the FPGA is disabled, and libusbK on Windows ignores read timeouts —
poll with `read(..., timeout=0)` instead of trusting the timeout argument.

---

## License

MIT — see `LICENSE`.

## Credits

* [thisiseth/pico-usb-blaster](https://github.com/thisiseth/pico-usb-blaster) — the
  USB-Blaster protocol reference and the firmware used in the bonus experiment.
* AGM's Supra documentation and the AGM TCX demo project, which supplied the `.prg`
  format and the cross-check bitstream.
* Built and verified on an AGM TCX board with an AG10KL144H, programmed by a Raspberry
  Pi Pico over four jumper wires.
