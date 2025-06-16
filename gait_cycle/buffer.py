import time
import math
import Adafruit_PCA9685

# ========= Workaround for Windows =========
# Override the default bus detection to always return bus number 1.
# import Adafruit_GPIO.I2C as I2C
# I2C.get_default_bus = lambda: 1

# ------------------------------------------------------------
# SERVO MAPPING: Each leg has three servo channels.
# ------------------------------------------------------------
# Here we assume that every leg has:
#   - A hip servo in the YZ plane (to achieve 0° here),
#   - A hip servo in the XZ plane (to be set to 140°),
#   - A knee servo (set to 90°).
servo_channels = {
    'LF': {'hip_yz': 0,  'hip_xz': 1,  'knee': 2},
    'RF': {'hip_yz': 3,  'hip_xz': 4,  'knee': 5},
    'LH': {'hip_yz': 6,  'hip_xz': 7,  'knee': 8},
    'RH': {'hip_yz': 9,  'hip_xz': 10, 'knee': 11}
}

servo_min = 115  # Min pulse length out of 4096 | 135
servo_max = 535  # Max pulse length out of 4096 | 620

# ------------------------------------------------------------
# INITIALIZE THE PCA9685 BOARD
# ------------------------------------------------------------
# With the override in place, this should now work on Windows.
pwm = Adafruit_PCA9685.PCA9685()  # Alternatively, you could try PCA9685(busnum=1)
pwm.set_pwm_freq(50)  # Typical servo frequency (50 Hz)

# ------------------------------------------------------------
# UTILITY FUNCTIONS
# ------------------------------------------------------------
def angle_to_pulse(angle_deg, min_angle=0, max_angle=180, min_pulse=150, max_pulse=600):
    """
    Convert an angle (in degrees) to a servo pulse width (ticks).
    Adjust min_pulse and max_pulse to suit your servos.
    """
    angle_deg = max(min_angle, min(max_angle, angle_deg))
    pulse_range = max_pulse - min_pulse
    angle_range = max_angle - min_angle
    pulse = int(min_pulse + (angle_deg - min_angle) * pulse_range / angle_range)
    return pulse

def set_angle(channel, angle):
    pulse = int((angle / 180.0) * (servo_max - servo_min) + servo_min)
    pwm.set_pwm(channel, 0, pulse)

def move_servo(channel, angle_deg):
    """Send the servo command on the specified channel."""
    pulse = angle_to_pulse(angle_deg)
    print(angle_deg)
    # pwm.set_pwm(channel, 0, pulse)

# ------------------------------------------------------------
# DESIRED NEUTRAL POSTURE (all angles are in degrees)
# ------------------------------------------------------------
HIP_YZ_TARGET = 90    # Hip angle in YZ plane = 0°
HIP_XZ_TARGET = 140  # Hip angle in XZ plane = 140°
KNEE_TARGET   = 90   # Knee joint (thigh/shank) angle = 90°

# ------------------------------------------------------------
# FUNCTION: Command All Legs to the Neutral Posture
# ------------------------------------------------------------
def return_to_neutral():
    """
    Command every leg to move to the desired (neutral) configuration.
    This forces the robot to return to the state where:
      - The knee angle is 90°,
      - The hip angle in the YZ plane is 0°, and
      - The hip angle in the XZ plane is 140°.
    """
    for leg, channels in servo_channels.items():
        move_servo(channels['hip_yz'], HIP_YZ_TARGET)
        move_servo(channels['hip_xz'], HIP_XZ_TARGET)
        move_servo(channels['knee'], KNEE_TARGET)
    print("Commanded all legs to neutral configuration.")

# ------------------------------------------------------------
# MAIN CONTROL LOOP
# ------------------------------------------------------------
print("Starting neutral posture control. Press Ctrl+C to exit.")

# try:
#     while True:
#         return_to_neutral()
#         time.sleep(1)  # Update at 1 Hz (adjust as necessary)

# except KeyboardInterrupt:
#     print("Exiting control loop.")
