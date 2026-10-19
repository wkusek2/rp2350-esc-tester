# rp2350-esc-tester

Bench firmware for a Raspberry Pi RP2350 (Pico 2 / Waveshare RP2350) that drives
two 4-in-1 BLHeli_32 ESCs (8 motors) with **bidirectional DShot**, reads ESC
telemetry, and lets **BLHeliSuite32 configure the ESCs directly through the Pico**
(no flight controller needed). Built for an ROV thruster test stand.

## What it does

- 8x bidirectional DShot150/300/600/1200 on PIO (one state machine per motor),
  1 kHz frame rate, GCR decode of the eRPM reply with CRC check, EDT support.
- KISS telemetry (temperature, voltage, current, mAh, eRPM) from both ESC
  telemetry wires on UART0 and UART1, round-robin per motor.
- ESC current-sensor outputs on ADC0/ADC1.
- USB CDC console: throttle in %, PWM-style 1000..2000 (3D mode), DShot
  commands (3D mode, direction, save, beep, EDT), live status, diagnostics.
- **BLHeliSuite32 passthrough**: a port of Betaflight's 4-way interface plus a
  minimal MSP responder, so BLHeliSuite32 "BLHeli32 Bootloader
  (Betaflight/Cleanflight)" can read and write ESC settings over the Pico's USB
  port. The bootloader bit-bang runs on core 1 with no interrupts.
- Failsafe: all motors stop when USB disconnects.

Tested with 2x Diatone MAMBA F45 128K (BLHeli_32 Rev 32.8, layout
`MAMBA_F45_128K_F4201`).

## Wiring

| RP2350 | Signal |
|---|---|
| GPIO2..5 | ESC #1 motor 1..4 (DShot, bidirectional) |
| GPIO10..13 | ESC #2 motor 1..4 (DShot, bidirectional) |
| GPIO1 | ESC #1 TLM (UART0 RX, 115200 8N1) |
| GPIO9 | ESC #2 TLM (UART1 RX) |
| GPIO26 | ESC #1 CUR (ADC0) |
| GPIO27 | ESC #2 CUR (ADC1) |
| VSYS | 5 V from BEC |
| GND | common ground with both ESCs |

stdio over UART is disabled because UART0 is used for telemetry. Pins are
defined at the top of `main.c`.

## Build

Requires the Pico SDK 2.x toolchain (the layout installed by the VS Code Pico
extension in `~/.pico-sdk` works out of the box).

```sh
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2
ninja -C build            # -> build/esc_test.uf2
```

Flash by copying the UF2 to the BOOTSEL drive or with `picotool load -f`.
If the board's flash is unusable, build a RAM-only image and run it with
picotool (it is lost on reset):

```sh
cmake -S . -B build_ram -G Ninja -DPICO_BOARD=pico2 -DRAM_BUILD=ON
ninja -C build_ram
picotool load -x build_ram/esc_test.uf2
```

## Console

Open the USB serial port (any baud rate) and type `help`. The main commands:

```
t <m|all|e1|e2> <pct>    throttle in % (0 = stop; -100..100 in 3D mode)
pwm <m|all> <1000..2000> PWM-style value, 1500 = stop, below = reverse, above = forward (3D)
s                        stop all motors
cmd <m> <0..47> [n]      send a DShot command n times (motors must be stopped)
3d on|off / dir <m> n|r / save / beep / edt
poles <n>                motor pole count used for RPM
rate 150|300|600|1200    DShot rate
stat / rpm / log <ms>    status, one-line eRPM + current, periodic status
pins / info <m> / bl <m> diagnostics: TLM lines, ESC info frame, bootloader probe
4way                     enter BLHeliSuite32 mode manually (exit: Disconnect in Suite or type exit)
```

Motors are numbered 1..8 (1..4 = ESC #1, 5..8 = ESC #2).

## BLHeliSuite32 through the Pico

1. ESCs powered, Pico on USB, nothing else holding the COM port.
2. BLHeliSuite32 -> interface **BLHeli32 Bootloader (Betaflight/Cleanflight)**
   -> COM port -> Connect -> Read Setup.
3. Write Setup, Disconnect. The ESCs go back to DShot about 2 s later.

How it works: the Suite talks MSP first (the firmware answers as a Betaflight
4.4.3 board), then switches to the 4-way protocol. The firmware stops DShot as
soon as the first MSP frame arrives, because a BLHeli_32 ESC only enters its
bootloader after the signal line has been idle high for a few seconds (about
4-5 s on the tested ESCs). The Suite itself waits 8 s after 4-way init, which is
enough. Short bootloader traffic (DeviceReset) does not reset that timer.

The 1200-baud reset to BOOTSEL of the Pico SDK's USB stdio is disabled, because
the Suite opens the port at 1200 baud when it "reboots the flight controller".

## Files

- `main.c` - console, UART telemetry, ADC, MSP responder, core 0 loop
- `dshot.c/.h`, `dshot_bidir.pio` - bidirectional DShot on PIO + DMA, GCR decode
- `kiss.c/.h` - KISS telemetry frame parser
- `fourway.c/.h` - 4-way interface and BLHeli bootloader bit-bang (core 1)
- `USTAWIENIA_ESC.txt` - bench notes and ESC settings (Polish)

## License

GPL-3.0. `fourway.c` is a port of Betaflight's `serial_4way.c` and
`serial_4way_avrootloader.c` (author 4712), which are GPL-3.0.
