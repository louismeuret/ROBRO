#!/usr/bin/env python3
"""Live plot of the STM32 telemetry: VDC/VU/VV/VW + IU/IV/IW.

Usage:
    sudo python3 plot_currents.py            # /dev/ttyUSB0
    sudo python3 plot_currents.py /dev/ttyUSB1

Requires: python-pyserial python-matplotlib
"""
import re
import sys
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 57600
WINDOW = 200  # number of samples shown

pat = re.compile(r"VDC,(\d+),VU,(\d+),VV,(\d+),VW,(\d+),IU,(\d+),IV,(\d+),IW,(\d+)")

names = ["VDC", "VU", "VV", "VW", "IU", "IV", "IW"]
colors = ["r", "g", "b", "y", "m", "c", "#8f8"]
dqs = [deque([0.0] * WINDOW, maxlen=WINDOW) for _ in names]

ser = serial.Serial(PORT, BAUD, timeout=0.2)
ser.reset_input_buffer()
print("streaming from", PORT, "at", BAUD, "baud")

fig, ax = plt.subplots()
lines = [ax.plot([], [], c, lw=1.5, label=n)[0] for n, c in zip(names, colors)]
ax.legend(loc="upper right")
ax.set_xlim(0, WINDOW)
ax.set_xlabel("sample")
ax.set_ylabel("ADC raw")
ax.grid(True, alpha=0.3)


def update(_frame):
    try:
        line = ser.readline().decode(errors="ignore")
        m = pat.search(line)
        if m:
            for i in range(7):
                dqs[i].append(int(m.group(i + 1)))
    except serial.SerialException as e:
        print("serial error:", e)
        plt.close(fig)
        return lines

    x = range(len(dqs[0]))
    for ln, dq in zip(lines, dqs):
        ln.set_data(x, dq)
    ax.relim()
    ax.autoscale_view()
    return lines


ani = FuncAnimation(fig, update, interval=50, blit=False)
try:
    plt.show()
finally:
    ser.close()
