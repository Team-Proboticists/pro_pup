import RPi.GPIO as GPIO
from simple_pid import PID
import time
import pandas as pd

SERVO_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(7.5)

pid = PID(1.0, 0.1, 0.05, setpoint=90)
pid.output_limits = (2.5, 12.5)

log = []
current_pos = 0

def get_position():
    global current_pos
    current_pos += (pid.setpoint - current_pos) * 0.05
    return current_pos

try:
    start = time.time()
    while time.time() - start < 20:
        pos = get_position()
        pwm_val = pid(pos)
        error = pid.setpoint - pos
        pwm.ChangeDutyCycle(pwm_val)
        log.append([time.time(), pid.setpoint, pos, pwm_val, error])
        time.sleep(0.05)

finally:
    pwm.stop()
    GPIO.cleanup()
    df = pd.DataFrame(log, columns=["Time", "Setpoint", "Position", "PWM", "Error"])
    df.to_csv("servo_log.csv", index=False)
    print("Saved log to servo_log.csv")

