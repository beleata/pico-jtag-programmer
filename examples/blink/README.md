# Example: four LEDs blinking on an AGM TCX board (AG10KL144H)

A minimal design that proves the whole chain works: Verilog -> Supra native flow ->
`.prg` -> programmed over JTAG by a Raspberry Pi Pico.

Files:

| file | role |
|---|---|
| `blink.v` | the design: all four LEDs toggle together at 1 Hz |
| `blink.ve` | pin constraints: `clk` on pin 23 (the 50 MHz oscillator), LEDs on 141–144, `resetn` on 25 |
| `blink.sdc` | timing: a single 50 MHz `create_clock` |

## Build

```bat
set SDK=C:\path\to\AgRV_pio\packages\tool-agrv_logic

"%SDK%\bin\af.exe" --setup --design blink --top_module blink ^
        --device AG10KL144H --verilog blink.v --ve blink.ve
"%SDK%\map\bin\yosys.exe" -c af_map.tcl
"%SDK%\bin\af.exe" --batch --mode NATIVE
```

`--device` must name the actual chip. A bitstream for a plain `AG10KL144` expects IDCODE
`0x01000001`, while an AG10KL144**H** answers `0x01000011` and silently refuses the wrong
file — the LEDs simply stay dark. Check before programming:

```bat
python ..\..\prg_info.py blink_sram.prg
python ..\..\jtag_do.py idcode
```

On this board a good `blink_sram.prg` is 81 operations, 8 786 472 shift bits, expecting
`0x01000011`.

## Program

```bat
python ..\..\jtag_do.py play blink_sram.prg
```

Expected result: the four LEDs blink **together**, 0.5 s on, 0.5 s off. If they never
light at all, the configuration was rejected (wrong device variant) or the FPGA is not
enabled; if they stay on permanently, the design is running but `resetn` is held low.

## Board hardware used here

| signal | FPGA pin | note |
|---|---|---|
| 50 MHz oscillator | 23 | drives `clk` |
| D0..D3 | 141, 142, 143, 144 | active low, so `0` lights them |
| reset button / `resetn` | 25 | on this board pin 25 may be left floating, which also works because FPGA inputs have weak pull-ups |

The LED and pin numbers come from the board schematic; always confirm them against your
own board before trusting a `.ve` file.
