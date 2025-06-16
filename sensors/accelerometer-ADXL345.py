import smbus
import time

# ADXL345 constants
ADXL345_ADDR = 0x53
REG_DATA_FORMAT = 0x31
REG_POWER_CTL = 0x2D
REG_DATAX0 = 0x32

# Initialize I2C bus (bus 1 on Raspberry Pi)
bus = smbus.SMBus(1)

# Power on and configure ADXL345
bus.write_byte_data(ADXL345_ADDR, REG_POWER_CTL, 0x08)     # Measurement mode
bus.write_byte_data(ADXL345_ADDR, REG_DATA_FORMAT, 0x08)   # Full resolution ±2g

# Function to read 2 bytes and convert to g
def read_axis(reg):
    low = bus.read_byte_data(ADXL345_ADDR, reg)
    high = bus.read_byte_data(ADXL345_ADDR, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value * 0.004  # Convert to g (4 mg/LSB)

# Print header
print("X\tY\tZ")

# Loop to print acceleration data
while True:
    x = read_axis(REG_DATAX0)
    y = read_axis(REG_DATAX0 + 2)
    z = read_axis(REG_DATAX0 + 4)

    print(f"{x:.3f}\t{y:.3f}\t{z:.3f}")  # Tab-separated for plotter

    time.sleep(0.05)  # ~20Hz sampling