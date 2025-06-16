import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# === LEG PARAMETERS ===
L1 = 12  # Thigh length (cm)
L2 = 15.5  # Shank length (cm)
hip_z = 20  # Fixed hip height (cm)

# === GAIT CONFIGURATION ===
gait_order = ['RF', 'LH', 'RH', 'LF']
legs = ['LF', 'RF', 'LH', 'RH']
offsets = {'LF': -10, 'RF': 10, 'LH': -10, 'RH': 10}

# === TRAJECTORY PARAMETERS ===
step_length = 8
step_height = 5
swing_steps = 20
stance_z = 0

# === SIMULATED SENSOR FUNCTION ===
def get_sensor_values():
    sensor = {leg: 1 for leg in legs}
    swing_leg = gait_order[gait_phase[0] % 4]
    if swing_timer[0] < 10:
        sensor[swing_leg] = 0
    return sensor

# === INVERSE KINEMATICS ===
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

# === TRAJECTORY GENERATOR ===
def generate_trajectory():
    traj = []
    for i in range(swing_steps):
        t = i / (swing_steps - 1)
        x = -step_length / 2 + step_length * t
        z = step_height * np.sin(np.pi * t)
        traj.append((x, z))
    return traj

swing_trajectory = generate_trajectory()

# === FORWARD KINEMATICS ===
def leg_fk(x_offset, x, z):
    x_rel, z_rel = x, hip_z - z
    theta1, theta2 = inverse_kinematics_2d(x_rel, z_rel, L1, L2)
    kx = x_offset + L1 * np.cos(theta1)
    kz = hip_z - L1 * np.sin(theta1)
    fx = kx + L2 * np.cos(theta1 - theta2)
    fz = kz - L2 * np.sin(theta1 - theta2)
    return (x_offset, hip_z), (kx, kz), (fx, fz)

# === PLOT SETUP ===
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-30, 30)
ax.set_ylim(-10, 60)
ax.set_aspect('equal')
ax.set_title("4-Leg Gait Synchronized to Pressure Sensors")

lines = {}
for leg in legs:
    thigh, = ax.plot([], [], 'ro-', lw=3)
    shank, = ax.plot([], [], 'bo-', lw=3)
    lines[leg] = (thigh, shank)

# === STATE VARIABLES ===
gait_phase = [0]  # 0 to 3
swing_timer = [0]
foot_pos = {leg: (0, stance_z) for leg in legs}  # Relative foot pos per leg
drag_counters = {leg: 0 for leg in legs}
drag_total_frames = 60

# Initialize with default stance foot positions
for leg in legs:
    foot_pos[leg] = (0, stance_z)

# === ANIMATION UPDATE ===
def update(_):
    swing_leg = gait_order[gait_phase[0] % 4]
    sensors = get_sensor_values()

    # Advance swing trajectory
    if swing_timer[0] < swing_steps:
        foot_pos[swing_leg] = swing_trajectory[swing_timer[0]]
        swing_timer[0] += 1
    else:
        if sensors[swing_leg] != 0:
            foot_pos[swing_leg] = (step_length / 2, stance_z)
            drag_counters[swing_leg] = 0
            gait_phase[0] += 1
            swing_timer[0] = 0

    # Update stance legs
    for leg in legs:
        if leg != swing_leg:
            x, z = foot_pos[leg]
            if drag_counters[leg] < drag_total_frames:
                x -= step_length / drag_total_frames
                drag_counters[leg] += 1
            foot_pos[leg] = (x, z)

    # Draw legs
    for leg in legs:
        x_offset = offsets[leg]
        x, z = foot_pos[leg]
        hip, knee, foot = leg_fk(x_offset, x, z)
        lines[leg][0].set_data([hip[0], knee[0]], [hip[1], knee[1]])
        lines[leg][1].set_data([knee[0], foot[0]], [knee[1], foot[1]])

    return [l for pair in lines.values() for l in pair]

ani = FuncAnimation(fig, update, interval=100, blit=True)
plt.tight_layout()
plt.show()  