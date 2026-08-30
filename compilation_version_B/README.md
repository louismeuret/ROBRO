# ROBRO — STM32G474 Motor Controller

Open-loop sinusoidal (V/f) 3-phase motor drive on an STM32G474, with live voltage telemetry (VDC/VU/VV/VW) streamed to a PC web dashboard.

## What it does

- Drives a 3-phase PMSM/BLDC via TIM1 complementary PWM (PA8/PA9/PA10 + PB13/PB14/PB15) at **25 kHz**, center-aligned, with dead time.
- Open-loop **sinusoidal V/f** startup: 1 s rotor-alignment hold, then ramps electrical frequency (default **5 Hz**, ~75 rpm for a 4-pole-pair motor) and modulation index together (0.25 → 0.70), using SVPWM (min-max 3rd-harmonic injection) for max voltage.
- Measures **VDC (PB12), VU (PB1), VV (PB2), VW (PA6)** on ADC1 and streams them out.

## Data path

```
                 SPI1, STM32 is master, 1 MHz, 32-byte full-duplex frames
STM32 PB3 SCK  ──────────────▶ C3 GPIO4              ┌── WiFi ──▶ browser dashboard
STM32 PA7 MOSI ──telemetry───▶ C3 GPIO5    ESP32-C3 ──┤
STM32 PB4 MISO ◀──commands─── C3 GPIO6              └── USB ───▶ /dev/ttyACM0 ──▶ web_voltages.py
STM32 PB9 CS   ──────────────▶ C3 GPIO7
STM32 GND      ─────────────── C3 GND
```

Every 50 ms the STM32 clocks one 32-byte transfer. The telemetry frame goes
out on MOSI while the command frame the ESP32 staged comes back on MISO —
one transfer, both directions.

> **The two RJ45 jacks on the board are NOT Ethernet.** They carry the RS485
> daisy-chain bus between boards. Never plug an Ethernet adapter or an ESP32
> into them — same connector, unrelated signals.

### Why SPI and not a UART

SPI is the only full-duplex link available on the pins that are physically
reachable on the header:

| Header | Pin | UART option | Verdict |
|--------|-----|-------------|---------|
| GPIO2 SWDIO | PA13 | `USART3_CTS`/`NSS` only | no TX/RX |
| GPIO3 SWCLK | PA14 | `USART2_TX` | USART2 is the RS485 chain |
| GPIO4 SPI_SCK | PB3 | `USART2_TX` | same conflict |
| GPIO5 SPI_MISO | PB4 | `USART2_RX` | same conflict |
| GPIO6 SPI_MOSI | PA7 | none | — |
| free | PB9 | `USART3_TX` | TX only; its RX partner is PB8-**BOOT0** |

Driving PB8 would hold BOOT0 high (a UART line idles high) and drop the board
into the DFU bootloader at every reset. SPI1 avoids all of it: it was already
configured by CubeMX as a full-duplex master with soft NSS and never used, and
all four signals are on the header — nothing has to be soldered to the LED
pads. PB6/PB7 keep their status LEDs.

### Telemetry frame (STM32 → ESP32), 72 bytes, little-endian

| Bytes | Field |
|-------|-------|
| 0–1 | sync `0xA5 0x5A` |
| 2–15 | VDC, VU, VV, VW, IU, IV, IW — `uint16` raw ADC counts (0–4095) |
| 16 | flags: b0 = 15V fault, b1 = outputs enabled, b2 = reverse, b3 = PB10 level |
| 17 | number of daisy-chain peers in the table |
| 18–21 | total RS485 frames from other boards (`uint32`) |
| 22–23 | electrical frequency setpoint, centi-Hz |
| 24–25 | modulation index target, milli-units |
| 26–29 | this board's id (`uint32`) |
| 30–31 | modulation index at ramp start (low-speed boost), milli-units |
| 32–33 | ramp time, milliseconds |
| 34–35 | live ramp progress, milli (0–1000) |
| 36–37 | `PWM_ARR` — carrier = 16 MHz / (2 × ARR) |
| 38, 39 | LED1 / LED2 override mode (0 auto, 1 on, 2 off) |
| 40–63 | peer table: 4 × { `uint32` id, `uint16` age in 10 ms units } |
| 70 | XOR checksum of bytes 0–69 |

### Command frame (ESP32 → STM32), 72 bytes

| Bytes | Field |
|-------|-------|
| 0–1 | sync `0xC3 0x3C` |
| 2 | command id (below) |
| 3 | sequence number — the STM32 acts only when it **changes** |
| 4–5 | argument (`uint16`) |
| 71 | XOR checksum of bytes 0–70 |

| Id | Command | Argument |
|----|---------|----------|
| 0x01 | SET_FREQ | centi-Hz, clamped 0–100 Hz |
| 0x02 | SET_MOD | milli-units, clamped 0–0.95 |
| 0x03 | START | — (restarts the ramp from zero, enables outputs) |
| 0x04 | STOP | — (clears `BDTR.MOE`, tri-states all six IPM inputs) |
| 0x05 | PING | — (sends an RS485 ping onto the chain) |
| 0x06 | SET_MOD_START | milli-units — the low-speed boost the ramp starts from |
| 0x07 | SET_RAMP_MS | milliseconds, floored at 100 |
| 0x08 | SET_DIR | 0 = forward, 1 = reverse (swaps V/W in the sine LUT) |
| 0x09 | SET_LED | low byte = LED index (0/1), high byte = mode (0 auto, 1 on, 2 off) |

The ESP32 stages its reply before a transfer begins, so a command lands one
frame after it is posted. The sequence number is what makes each command
execute exactly once despite being re-sent every 50 ms.

> **VV is not measured.** `PB2` is a GPIO output in both `Repository.ioc` and
> `Repository (copy).ioc`, so the "VV = PB2 = ADC2_IN12" row that earlier
> revisions of this README claimed was never true for this pinout. VV is sent
> as `0`. To get a real VV, reassign PB2 to `ADC2_IN12` in CubeMX and read
> `ADC_CHANNEL_12` on `hadc2` in `Esp_BuildTelemetry()`.

## The dashboard

Plain, unstyled HTML served by the ESP32-C3 at its IP address. Polls `/api`
twice a second. Four sections:

- **Sensors** — all six live ADC channels, raw counts and scaled units.
- **Digital inputs** — the 15V rail status (PA15) and the PB10 EXTI input.
- **Waveform / motor settings** — electrical frequency, modulation index
  target, the low-speed boost the ramp starts from, ramp time, direction, live
  ramp progress, PWM carrier (read-only), and START/STOP.
- **LEDs** — one button each for LED1 (PB6) and LED2 (PB7), cycling
  AUTO → ON → OFF → AUTO. In AUTO the firmware owns the LED as before.
- **Daisy chain** — this board's id, the running count of frames from other
  boards, a PING button, and a table of every board heard on the RS485 bus
  with how long ago each was last seen.

Endpoints: `/` the page, `/api` a JSON snapshot, `/cmd?c=<name>&v=<value>`
(plus `&i=<index>` for `led`) to post a command.

## The ESP32 bridge

Board: **ESP32-C3 SuperMini**.

| STM32 | Header | ESP32-C3 |
|-------|--------|----------|
| PB3 | GPIO4 SPI_SCK | GPIO4 SCLK |
| PA7 | GPIO6 SPI_MOSI | GPIO5 MOSI |
| PB4 | GPIO5 SPI_MISO | GPIO6 MISO |
| PB9 | free header pin | GPIO7 CS |
| GND | GND | GND |

Why GPIO4/5/6/7 specifically — most of the C3's small pin count is spoken for:

| C3 pins | Why they are unusable here |
|---------|---------------------------|
| GPIO2, GPIO8, GPIO9 | Strapping pins. Driven at boot they can stop the board starting or force download mode. |
| GPIO18, GPIO19 | Native USB D-/D+ — the SuperMini's USB port. |
| GPIO20, GPIO21 | UART0 (serial console). |

That leaves GPIO4–7 as the only clean contiguous block. They route through the
GPIO matrix rather than IO_MUX, which is irrelevant at 1 MHz (the matrix adds
tens of nanoseconds against a 500 ns half-period). If you ever push the SPI
clock much higher, the C3's IO_MUX pins for FSPI are CLK=6, MOSI=7, MISO=2,
CS=10 — but MISO on GPIO2 collides with a strapping pin, so 1 MHz on the
matrix is the better trade.

The C3 has a single user-available SPI controller, `SPI2_HOST`; the classic
ESP32's `VSPI_HOST` / `SPI3_HOST` do not exist on it.

Flash `esp32_bridge/esp32_bridge.ino` with the ESP32 Arduino core, board
"ESP32C3 Dev Module". **Set "USB CDC On Boot: Enabled"** so `Serial` goes to
the native USB port. Set `WIFI_SSID` / `WIFI_PASS` at the top for the
dashboard, or leave `WIFI_SSID` empty to run as a USB-only bridge. The board
prints its IP on USB serial at boot.

The C3 also mirrors every frame to USB serial in the original ASCII format, so
`web_voltages.py` and `plot_currents.py` work unchanged — but the C3
enumerates as **`/dev/ttyACM0`**, not `/dev/ttyUSB0`, so pass the path:

```bash
sudo python3 web_voltages.py /dev/ttyACM0
```

`VSCALE_FS` / `ISCALE_FS` in the sketch convert raw counts to volts/amps (the
value at 4095 counts) — set them to your divider and shunt ratios.

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

## Wiring

ESP32 link: see "The ESP32 bridge" above. The ESP32 is powered by its own USB cable.

Analog sense inputs:

| Signal | Pin | ADC channel |
|--------|-----|-------------|
| VDC (bus voltage) | PB12 | ADC1_IN11 |
| VU | PB1 | ADC1_IN12 |
| VV | — | **not available** — PB2 is a GPIO output in the .ioc |
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
# the ESP32-C3 enumerates as /dev/ttyACM0 (native USB), not /dev/ttyUSB0
sudo python3 ~/Projects/ROBRO/compilation_version_B/web_voltages.py /dev/ttyACM0
sudo python3 ~/Projects/ROBRO/compilation_version_B/web_voltages.py /dev/ttyACM0 8000
```

Then open `http://localhost:8080/` in a browser: live VDC/VU/VV/VW + IU/IV/IW values and a history chart. The `VSCALE`/`ISCALE` dicts in the script map raw ADC counts to volts/amps (full-scale value at 4095 counts); set them to your divider/sense ratios.

## Plotting (terminal alternative)

```bash
sudo python3 ~/Projects/ROBRO/compilation_version_B/plot_currents.py /dev/ttyACM0
```

Shows VDC, VU, VV, VW, IU, IV, IW with auto-zoomed y-axis. Values are raw 12-bit ADC counts (0–4095).

## Files

- `Core/Src/main.c` — motor drive, RS485 daisy chain (USART2), ADC reads and the ESP32 telemetry/command link (SPI1).
- `Core/Src/stm32g4xx_it.c` — interrupts.
- `esp32_bridge/esp32_bridge.ino` — ESP32 SPI slave: WiFi dashboard + USB mirror + command staging.
- `arduino_bridge/arduino_bridge.ino` — superseded DCduino forwarder, kept for reference.
- `web_voltages.py` — live web dashboard on the PC (stdlib + pyserial, SSE).
- `plot_currents.py` — matplotlib live plotter.

## Notes / limits

- Open-loop only — no encoder/Hall feedback yet, no current limiting. Use a current-limited bench supply.
