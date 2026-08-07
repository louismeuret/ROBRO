# ROBRO — STM32G474 Motor Controller

Open-loop sinusoidal (V/f) 3-phase motor drive on an STM32G474, with live voltage telemetry (VDC/VU/VV/VW) streamed to a PC web dashboard.

## What it does

- Drives a 3-phase PMSM/BLDC via TIM1 complementary PWM (PA8/PA9/PA10 + PB13/PB14/PB15) at **25 kHz**, center-aligned, with dead time.
- Open-loop **sinusoidal V/f** startup: 1 s rotor-alignment hold, then ramps electrical frequency (default **5 Hz**, ~75 rpm for a 4-pole-pair motor) and modulation index together (0.25 → 0.70), using SVPWM (min-max 3rd-harmonic injection) for max voltage.
- Measures **VDC (PB12), VU (PB1), VV (PB2), VW (PA6)** on ADC1 and streams them out.

## Data path

```
STM32 (PB3, hardware USART2 @ 57600)
   → DCduino (Arduino Uno clone) bridge sketch
      → USB (fake CH340 chip) → /dev/ttyUSB0 → PC web dashboard
```

The STM32 sends ASCII lines: `VDC,<raw>,VU,<raw>,VV,<raw>,VW,<raw>,IU,<raw>,IV,<raw>,IW,<raw>\r\n` (~80 lines/s at 57600, hardware USART2 on PB3, no sprintf).

## Prerequisites (Arch Linux)

```bash
sudo pacman -S gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
  dfu-util arduino-cli python-pyserial python-matplotlib python-numpy
```

## Build & flash the STM32

```bash
make                     # builds build/Repository.bin
```

Flash via the built-in USB DFU bootloader:
1. Set **BOOT0 = HIGH**, press reset → board shows as `0483:df11`.
2. Flash:
   ```bash
   dfu-util -a 0 -d 0483:df11 --dfuse-address 0x08000000 -D build/Repository.bin
   ```
3. Set **BOOT0 = LOW**, press reset → app runs. (BOOT0 high = bootloader/DFU, low = app.)

## The DCduino & the fake-CH340 fix (important)

The DCduino's CH340 USB chip is a **counterfeit** clone that the kernel `ch341` driver cannot handle (control/bulk endpoint stalls → `Input/output error`, uploads fail). It is fixed by using the **WCH official driver (`ch34x`)** instead:

```bash
sudo bash ~/setup_dcduino.sh     # blacklists ch341, installs ch34x, reloads it
```

- The patched `ch34x` auto-clears the chip's halted endpoints at every open.
- `/home/anolys/ch34x.ko` is the built module; `~/setup_dcduino.sh` installs it.

### Uploading the bridge sketch to the DCduino

Uploads are flaky (the fake chip) — the reliable recipe:
1. Power-cycle the DCduino (unplug ~10 s).
2. Run avrdude and **press the DCduino reset button** right when it prints `Setting baud rate`:
   ```bash
   sudo /home/anolys/.arduino15/packages/arduino/tools/avrdude/8.0.0-arduino1/bin/avrdude \
     -C /home/anolys/.arduino15/packages/arduino/tools/avrdude/8.0.0-arduino1/etc/avrdude.conf \
     -v -patmega328p -carduino -P /dev/ttyUSB0 -b 115200 \
     -U flash:w:/home/anolys/.cache/arduino/sketches/FB50A01A50B25FCF0F581748146EDEEB/arduino_bridge.ino.hex:i
   ```
3. Look for `Writing ... verified` + `Avrdude done. Thank you.` (ignore the earlier "not in sync" retries).

The bridge sketch (`arduino_bridge/arduino_bridge.ino`) forwards **57600** on SoftwareSerial pin 2 → USB Serial.

## Wiring

| STM32 | DCduino |
|-------|---------|
| PB3 (USART2 TX) | pin 2 (SoftwareSerial RX) |
| GND | GND |

The DCduino is powered by its own USB cable.

Analog sense inputs:

| Signal | Pin | ADC channel |
|--------|-----|-------------|
| VDC (bus voltage) | PB12 | ADC1_IN11 |
| VU | PB1 | ADC1_IN12 |
| VV | PB2 | ADC2_IN12 |
| VW | PA6 | ADC2_IN3 |
| IU (U current) | PA0 | ADC1_IN1 |
| IV (V current) | PA4 | ADC2_IN17 |
| IW (W current) | PB0 | ADC1_IN15 |

> **Current reads 0?** The current channels (IU/IV/IW) only show a value if the
> current-sense shunt/op-amp outputs are actually wired to PA0/PA4/PB0. An
> unloaded motor also draws very little current, so the raw counts can be near
> zero. The voltage channels read your divider values once wired.

## Web dashboard

```bash
sudo python3 ~/Projects/ROBRO/compilation_version_B/web_voltages.py        # port 8080
sudo python3 ~/Projects/ROBRO/compilation_version_B/web_voltages.py /dev/ttyUSB1 8000
```

Then open `http://localhost:8080/` in a browser: live VDC/VU/VV/VW + IU/IV/IW values and a history chart. The `VSCALE`/`ISCALE` dicts in the script map raw ADC counts to volts/amps (full-scale value at 4095 counts); set them to your divider/sense ratios.

## Plotting (terminal alternative)

```bash
sudo python3 ~/Projects/ROBRO/compilation_version_B/plot_currents.py
```

Shows VDC, VU, VV, VW, IU, IV, IW with auto-zoomed y-axis. Values are raw 12-bit ADC counts (0–4095).

## Files

- `Core/Src/main.c` — motor drive, ADC reads, USART2 serial TX.
- `Core/Src/stm32g4xx_it.c` — interrupts.
- `arduino_bridge/arduino_bridge.ino` — DCduino forwarder.
- `web_voltages.py` — live web dashboard (stdlib + pyserial, SSE).
- `plot_currents.py` — matplotlib live plotter.
- `setup_dcduino.sh`, `ch34x.ko`, `ch340_usb_test.c` — fake-CH340 tooling.

## Notes / limits

- Open-loop only — no encoder/Hall feedback yet, no current limiting. Use a current-limited bench supply.
