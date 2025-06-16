# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

f = open('forward_data2.csv')
d=f.read()
d = [[int(float(j)) for j in i.split(',')[:-2]] for i in d.split('\n')][:-2]

g = []
for i in range(len(d)):
    g.append(d[i][0:2] + d[i][4:6])

print(g)
# exit()

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(50)

# Helper function to make setting a servo pulse width simpler.
# def set_servo_pulse(channel, pulse):
#     pulse_length = 1000000    # 1,000,000 us per second
#     pulse_length //= 60       # 60 Hz
#     print('{0}us per period'.format(pulse_length))
#     pulse_length //= 4096     # 12 bits of resolution
#     print('{0}us per bit'.format(pulse_length))
#     pulse *= 1000
#     pulse //= pulse_length

#     pwm.set_pwm(channel, 0, pulse)

# pulse_50 = [300,300,300,300]
# pulse_105 = [300,300,300,300]

servo_min = [115, 100]*2
servo_max = [535, 530]*2

def set_angle(channel, angle):
    pulse = int((angle / 180.0) * (servo_max[channel] - servo_min[channel]) + servo_min[channel])
    pwm.set_pwm(channel, 0, pulse)


set_angle(0, 270-g[0][0])
set_angle(1, g[0][1])

set_angle(2, 180-g[0][2])
set_angle(3, g[0][3])

time.sleep(1)

for i in range(500):
    set_angle(0, 270- g[i][0])
    set_angle(1, g[i][1])

    set_angle(2,180- g[i][2])
    set_angle(3, g[i][3])

    time.sleep(0.01)



# while True:
#     angle = int(input('enter angle: '))
#     pwm.set_pwm(0,0,angle)
#     time.sleep(0.1)


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

# for FR do 180 - big motor