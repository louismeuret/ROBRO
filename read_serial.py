import serial
import csv
import time
from datetime import datetime
import math

PORT      = "/dev/ttyUSB0"
BAUD      = 115200
OUT_FILE  = "encoder_log.csv"

def decode_position(bytes_dec):
    """
    Decode encoder position from raw bytes (0x32 command response).

    Based on original_code.c protocol:
    - Byte 0: command echo (0x32)
    - Byte 1: unknown/status
    - Byte 2: bit 7 is LSB of 17-bit position
    - Byte 3: mid 8 bits of position
    - Byte 4: high 8 bits of position
    - Byte 5: low byte of turn count
    - Byte 6: high byte of turn count (signed 16-bit)
    - Byte 7: unknown
    - Byte 8: checksum (XOR of bytes 0-7 should equal byte 8)
    """
    if len(bytes_dec) < 7:
        return None, None, None, None

    # Check if this is a valid 0x32 response
    if bytes_dec[0] != 0x32:
        return None, None, None, f"cmd_mismatch(0x{bytes_dec[0]:02X})"

    b2, b3, b4 = bytes_dec[2], bytes_dec[3], bytes_dec[4]
    b5, b6 = bytes_dec[5], bytes_dec[6]

    # Single-turn position (17-bit, per original_code.c line 211)
    # ipos = (b2 & 0x80) + (b3 << 8) + (b4 << 16)
    ipos = (b2 & 0x80) + (b3 << 8) + (b4 << 16)

    # Convert to degrees: maps [0, 2^24) to [-180, 180)
    single_turn_deg = (ipos * 360.0 / 16777216.0) - 180.0

    # Multi-turn count (signed 16-bit little-endian)
    turns_raw = b5 + (b6 << 8)
    if turns_raw >= 32768:  # Handle signed
        turns_raw = turns_raw - 65536

    # Encoder's internal full position (before gear ratio correction)
    encoder_full_deg = single_turn_deg + (turns_raw * 360.0)

    # The encoder appears to have 256:1 internal gear ratio
    # (one physical turn = 256 encoder turns), so divide by 256
    GEAR_RATIO = 256
    full_deg = encoder_full_deg / GEAR_RATIO
    turns = turns_raw / GEAR_RATIO  # For display purposes

    return single_turn_deg, turns, full_deg, None

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Listening on {PORT} @ {BAUD} baud — logging to {OUT_FILE}")
    print(f"{'Time':<12} {'Single°':>10} {'Turns':>8} {'Full°':>12} {'Raw'}")

    with open(OUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)

        # Header: timestamp + one column per byte (up to 9 for 0x32 response)
        writer.writerow([
            "timestamp",
            "unix_time",
            "b0","b1","b2","b3","b4","b5","b6","b7","b8",
            "single_turn_deg", "turns", "full_deg",
            "raw_line"
        ])

        try:
            while True:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if not line or line in ("READY", "NO_RESPONSE"):
                    if line == "NO_RESPONSE":
                        print(f"[{datetime.now().isoformat()}] No response from encoder")
                    continue

                now      = datetime.now()
                unix_now = time.time()

                # Expect comma-separated hex bytes, e.g. "32,01,AB,..."
                parts = line.split(",")[1:]
                try:
                    bytes_dec = [int(b, 16) for b in parts]
                except ValueError:
                    print(f"Unrecognised line: {line}")
                    continue

                # Pad to 9 columns so the CSV stays aligned
                padded = bytes_dec + [""] * (9 - len(bytes_dec))

                # Decode position
                single_deg, turns, full_deg, err = decode_position(bytes_dec)

                row = [now.isoformat(), f"{unix_now:.6f}"] + padded[:9]
                if err:
                    row += ["", "", err]
                else:
                    row += [f"{single_deg:.3f}" if single_deg is not None else "",
                            f"{turns:.3f}" if turns is not None else "",
                            f"{full_deg:.3f}" if full_deg is not None else ""]
                row += [line]
                writer.writerow(row)
                f.flush()   # write immediately so you can tail -f the file

                # Print decoded values
                if single_deg is not None:
                    print(f"{now.strftime('%H:%M:%S.%f')[:12]} {single_deg:>10.2f} {turns:>8.2f} {full_deg:>12.2f}  {line}")
                else:
                    print(f"{now.strftime('%H:%M:%S.%f')[:12]} {'--':>10} {'--':>6} {'--':>12}  {line} ({err})")

        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            ser.close()

if __name__ == "__main__":
    main()
