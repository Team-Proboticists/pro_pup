


############## LOOPING

import time
import numpy as np
import Adafruit_PCA9685

# === Servo Driver Setup ===
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

def set_angle(channel, angle):
    pulse = int((angle / 180.0) * (servo_max - servo_min) + servo_min)
    pwm.set_pwm(channel, 0, pulse)

def safe_arccos(x):
    return np.arccos(np.clip(x, -1.0, 1.0))

def inverse_kinematics_leg_3d(x, y, z, L1, L2):
    D = np.sqrt(x**2 + z**2)
    D = max(1e-6, min(D, L1 + L2))
    try:
        cos_a = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
        a = safe_arccos(cos_a)
        b = np.arctan2(z, x)
        theta2 = b - a
        cos_theta3 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
        theta3 = safe_arccos(cos_theta3)
        theta1 = 0
        return theta1, theta2, theta3
    except:
        return 0, 0, 0

# === Gait Parameters ===
cycle_time = 1.0
dt = 0.02
time_steps = np.arange(0, cycle_time, dt)

L1, L2 = 12, 14  # Thigh and shank lengths
step_length = 15
step_height = step_length / 2
swing_phase_ratio = 0.5
stance_phase_ratio = 0.5

def generate_foot_trajectory(time_shift=0.0):
    traj = []
    for t in time_steps:
        shifted_t = (t + time_shift) % cycle_time
        phase = shifted_t / cycle_time
        if phase < swing_phase_ratio:
            tau = phase / swing_phase_ratio
            x = -step_length / 2 + step_length * tau
            y = 0
            z = step_height * np.sin(np.pi * tau)
        else:
            tau = (phase - swing_phase_ratio) / stance_phase_ratio
            x = step_length / 2 - step_length * tau
            y = 0
            z = 0
        traj.append((x, y, z))
    return np.array(traj)

print("Starting biped gait loop...")

while True:
    # === Get trajectories for both legs ===
    left_traj = generate_foot_trajectory(time_shift=0.0)
    right_traj = generate_foot_trajectory(time_shift=0.5 * cycle_time)

    # === Compute angles for both legs ===
    left_hip_abd, left_hip_flex, left_knee = [], [], []
    right_hip_abd, right_hip_flex, right_knee = [], [], []

    for (lx, ly, lz), (rx, ry, rz) in zip(left_traj, right_traj):
        t1, t2, t3 = inverse_kinematics_leg_3d(lx, ly, lz, L1, L2)
        left_hip_abd.append(t1)
        left_hip_flex.append(t2)
        left_knee.append(t3)

        t1, t2, t3 = inverse_kinematics_leg_3d(rx, ry, rz, L1, L2)
        right_hip_abd.append(t1)
        right_hip_flex.append(t2)
        right_knee.append(t3)

    # === Convert and clean up ===
    def clean(angles):
        angles = np.nan_to_num(np.degrees(angles), nan=0.0)
        return np.clip(angles, 0, 180)

    left_hip_abd = clean(left_hip_abd)
    left_hip_flex = clean(left_hip_flex)
    left_knee = clean(left_knee)

    right_hip_abd = clean(right_hip_abd)
    right_hip_flex = clean(right_hip_flex)
    right_knee = clean(right_knee)

    # === Drive both legs ===
    for i in range(len(time_steps)):
        # Left leg
        set_angle(0, left_hip_abd[i])
        set_angle(1, left_hip_flex[i])
        set_angle(2, left_knee[i])
        # Right leg
        set_angle(4, right_hip_abd[i])
        set_angle(5, right_hip_flex[i])
        set_angle(6, right_knee[i])
        time.sleep(dt)

