# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096 | 135
servo_max = 600  # Max pulse length out of 4096 | 620
servo_min = 115  # Min pulse length out of 4096 | 135
servo_max = 535  # Max pulse length out of 4096 | 620

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(50)

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def set_angle(channel, angle):
    pulse = int((angle / 180.0) * (servo_max - servo_min) + servo_min)
    pwm.set_pwm(channel, 0, pulse)


while True:
    angle = int(input('enter angle: '))
    # pwm.set_pwm(0,0,angle)
    set_angle(0, angle)
    time.sleep(0.1)


# for i in range(0, 181, 10):
#     set_angle(0, i)
#     time.sleep(1)
#     set_angle(0, i-5 if i > 5 else 0)
#     time.sleep(0.5)

# print('Moving servo on channel 0, press Ctrl-C to quit...')
# while True:
#     # Move servo on channel O between extremes.
#     pwm.set_pwm(0, 0, servo_min)
#     # pwm.set_pwm(4, 0, servo_min)
#     # pwm.set_pwm(8, 0, servo_min)
#     # pwm.set_pwm(12, 0, servo_min)
#     time.sleep(1)
#     pwm.set_pwm(0, 0, servo_max)
#     # pwm.set_pwm(4, 0, servo_max)
#     # pwm.set_pwm(8, 0, servo_max)
#     # pwm.set_pwm(12, 0, servo_max)
#     time.sleep(1)


########## 155 horizontal 60 kg-cm
## 