import time
import numpy as np
import math
import Adafruit_PCA9685

# ========================
# LEG AND GAIT PARAMETERS
# ========================
L1 = 12.0      # Thigh length (cm)
L2 = 15.5      # Shank length (cm)
hip_z = 20.0   # Fixed hip height (cm)

# Leg identifiers and horizontal offsets for the hip (in cm)
gait_order = ['RF', 'LH', 'RH', 'LF']
legs = ['LF', 'RF', 'LH', 'RH']
offsets = {'LF': -10, 'RF': 10, 'LH': -10, 'RH': 10}

# Trajectory parameters
step_length = 8.0    # Horizontal displacement for swing phase (cm)
step_height = 5.0    # Vertical lift during swing (cm)
swing_steps = 20     # Number of discrete steps during swing phase
stance_z = 0.0       # Foot height in stance (cm)

# ========================
# SERVO SETUP & MAPPING
# ========================
servo_channels = {
    'LF': {'hip': 0, 'knee': 1},
    'RF': {'hip': 2, 'knee': 3},
    'LH': {'hip': 4, 'knee': 5},
    'RH': {'hip': 6, 'knee': 7}
}

# pwm = Adafruit_PCA9685.PCA9685()
# pwm.set_pwm_freq(50)

# def angle_to_pulse(angle_deg, min_angle=0, max_angle=180, min_pulse=150, max_pulse=600):
#     angle_deg = max(min_angle, min(max_angle, angle_deg))
#     pulse_range = max_pulse - min_pulse
#     angle_range = max_angle - min_angle
#     pulse = int(min_pulse + (angle_deg - min_angle) * pulse_range / angle_range)
#     return pulse

# def move_servo(channel, angle_rad):
#     angle_deg = math.degrees(angle_rad)
#     pulse = angle_to_pulse(angle_deg)
#     pwm.set_pwm(channel, 0, pulse)

# =================================
# GAIT CYCLE KINEMATICS FUNCTIONS
# =================================
def inverse_kinematics_2d(x, z, L1, L2):
    r = np.sqrt(x**2 + z**2)
    D = min(r, L1 + L2)
    cos_theta2 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.pi - np.arccos(cos_theta2)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)
    gamma = np.arctan2(z, x)
    theta1 = gamma + beta
    return theta1, theta2

def generate_trajectory():
    traj = []
    for i in range(swing_steps):
        t = i / (swing_steps - 1)
        x = -step_length / 2 + step_length * t
        z = step_height * np.sin(np.pi * t)
        traj.append((x, z))
    return traj

swing_trajectory = generate_trajectory()

def leg_ik_for_servo(x_offset, x, z):
    x_rel = x
    z_rel = hip_z - z
    theta_hip, theta_knee = inverse_kinematics_2d(x_rel, z_rel, L1, L2)
    return theta_hip, theta_knee

# ========================
# FUNCTION TO MOVE FORWARD ONE CYCLE
# ========================
def execute_single_gait_cycle():
    gait_phase = [0]
    swing_timer = [0]
    foot_pos = {leg: (0.0, stance_z) for leg in legs}
    drag_counters = {leg: 0 for leg in legs}
    drag_total_frames = 60

    def get_sensor_values():
        sensor = {leg: 1 for leg in legs}
        swing_leg = gait_order[gait_phase[0] % 4]
        if swing_timer[0] < 10:
            sensor[swing_leg] = 0
        return sensor

    total_phases = 4
    while gait_phase[0] < total_phases:
        swing_leg = gait_order[gait_phase[0] % 4]
        sensors = get_sensor_values()

        if swing_timer[0] < swing_steps:
            foot_pos[swing_leg] = swing_trajectory[swing_timer[0]]
            swing_timer[0] += 1
        else:
            if sensors[swing_leg] != 0:
                foot_pos[swing_leg] = (step_length / 2, stance_z)
                drag_counters[swing_leg] = 0
                gait_phase[0] += 1
                swing_timer[0] = 0

        for leg in legs:
            if leg != swing_leg:
                x, z = foot_pos[leg]
                if drag_counters[leg] < drag_total_frames:
                    x -= step_length / drag_total_frames
                    drag_counters[leg] += 1
                foot_pos[leg] = (x, z)

        for leg in legs:
            x_offset = offsets[leg]
            x, z = foot_pos[leg]
            theta_hip, theta_knee = leg_ik_for_servo(x_offset, x, z)
            move_servo(servo_channels[leg]['hip'], theta_hip)
            move_servo(servo_channels[leg]['knee'], theta_knee)

        time.sleep(0.1)

# Example usage:
execute_single_gait_cycle()
