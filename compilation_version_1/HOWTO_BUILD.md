# How to build / flash

All commands run from the `compilation/` folder:

```sh
cd compilation
```

There are three firmware versions. You pick one with the `APP=` option.
Each build overwrites `build/Repository.{elf,hex,bin}`.

## The three versions

| `APP=` value                     | What it does                                                        |
|----------------------------------|--------------------------------------------------------------------|
| `Core/Src/main.c` (default)      | Original motor / PWM firmware. Unchanged.                          |
| `Core/Src/encoder_read.c`        | Reads the encoder only. Value lives in `enc_*` globals (watch over SWD) + PC13 blinks. |
| `Core/Src/encoder_read_arduino.c`| Reads the encoder AND streams the position out PB3 to an Arduino/ESP32. |

## Commands

```sh
make clean                                          # delete old build output

make                                                # build the DEFAULT (motor) firmware
make APP=Core/Src/encoder_read.c                    # build the encoder-only reader
make APP=Core/Src/encoder_read_arduino.c            # build the encoder -> Arduino version
```

Always `make clean` when you switch versions, e.g.:

```sh
make clean && make APP=Core/Src/encoder_read_arduino.c
```

Result is `build/Repository.bin` (and `.hex`). Flash it the same way you
already flash this board (ST-Link / dfu / etc.).

## Wiring quick reference

Encoder (both encoder versions), via an external MAX485:

| STM32 pin | MAX485 |
|-----------|--------|
| PB6       | DI     |
| PB7       | RO     |
| PB9       | DE + RE (tied together) |
| —         | A / B  -> encoder pair |

Companion board (only `encoder_read_arduino.c`):

| STM32 pin | Arduino / ESP32 |
|-----------|-----------------|
| PB3 (TX)  | RX              |
| GND       | GND             |

The STM32 sends one line per reading: `full_mdeg,single_mdeg` (millidegrees,
integers). Example: `123456,-45000` = 123.456° absolute, -45.000° within the turn.

The Arduino/ESP32 sketch that forwards this to the PC is in
`arduino_relay/arduino_relay.ino`.
