import smbus2
import time
import matplotlib.pyplot as plt
from collections import deque

# I2C address and register constants
ADXL345_ADDR = 0x53
REG_DATA_FORMAT = 0x31
REG_POWER_CTL = 0x2D
REG_DATAX0 = 0x32

# Initialize I2C (bus 1 on Raspberry Pi)
bus = smbus2.SMBus(1)

# Setup ADXL345
bus.write_byte_data(ADXL345_ADDR, REG_POWER_CTL, 0x08)     # Measurement mode
bus.write_byte_data(ADXL345_ADDR, REG_DATA_FORMAT, 0x08)   # Full resolution, ±2g

# Helper to read 2 bytes and convert to signed int
def read_axis(reg):
    low = bus.read_byte_data(ADXL345_ADDR, reg)
    high = bus.read_byte_data(ADXL345_ADDR, reg + 1)
    val = (high << 8) | low
    if val > 32767:
        val -= 65536
    return val * 0.004  # 4 mg/LSB

# Plot setup
plt.ion()
fig, ax = plt.subplots()
window = 100
x_vals, y_vals, z_vals = deque([0]*window, maxlen=window), deque([0]*window, maxlen=window), deque([0]*window, maxlen=window)
line1, = ax.plot(x_vals, label='X')
line2, = ax.plot(y_vals, label='Y')
line3, = ax.plot(z_vals, label='Z')
ax.legend()
ax.set_ylim(-2, 2)  # ±2g

# Real-time loop
while True:
    x = read_axis(REG_DATAX0)
    y = read_axis(REG_DATAX0 + 2)
    z = read_axis(REG_DATAX0 + 4)

    x_vals.append(x)
    y_vals.append(y)
    z_vals.append(z)

    line1.set_ydata(x_vals)
    line2.set_ydata(y_vals)
    line3.set_ydata(z_vals)

    line1.set_xdata(range(len(x_vals)))
    line2.set_xdata(range(len(y_vals)))
    line3.set_xdata(range(len(z_vals)))

    plt.draw()
    plt.pause(0.01)