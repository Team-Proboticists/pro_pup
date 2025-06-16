import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Leg dimensions (in cm)
L1 = 12.0    # Thigh
L2 = 15.5    # Shin

# Standing angles (degrees)
thigh_angle_deg = 40
knee_angle_deg = 100

# Convert to radians
thigh_angle_rad = np.radians(thigh_angle_deg)
knee_angle_rad = np.radians(knee_angle_deg)

# Compute standing foot position from hip
def compute_standing_foot():
    knee_x = L1 * np.cos(thigh_angle_rad)
    knee_z = -L1 * np.sin(thigh_angle_rad)

    total_angle = thigh_angle_rad + np.pi - knee_angle_rad
    foot_x = knee_x + L2 * np.cos(total_angle)
    foot_z = knee_z - L2 * np.sin(total_angle)
    return foot_x, foot_z

stand_x, stand_z = compute_standing_foot()

# Gait phases: LF, RF, LR, RR
phases = [0.25, 0.75, 0.0, 0.5]
leg_offsets = [(-8, 10), (8, 10), (-8, -10), (8, -10)]

# Gait parameters
step_range = 6  # swing range around standing x
step_height = 3
cycle_time = 2.0
fps = 30
frames = int(cycle_time * fps)

# Foot path (gait) relative to standing position
def foot_trajectory(phase, t):
    t_phase = (t + phase) % 1.0
    if t_phase < 0.25:
        ratio = t_phase / 0.25
        x = stand_x - step_range / 2 + step_range * ratio
        z = stand_z + step_height * np.sin(np.pi * ratio)
    else:
        ratio = (t_phase - 0.25) / 0.75
        x = stand_x + step_range / 2 - step_range * ratio
        z = stand_z
    return x, z

# Inverse kinematics from hip to foot
def inverse_kinematics_2d(x, z):
    dx, dz = x, -z
    dist = np.hypot(dx, dz)
    dist = min(dist, L1 + L2 - 0.001)
    a = np.arccos(np.clip((L1**2 + dist**2 - L2**2) / (2 * L1 * dist), -1, 1))
    b = np.arctan2(dz, dx)
    theta1 = b - a
    knee_x = L1 * np.cos(theta1)
    knee_z = L1 * np.sin(theta1)
    return (knee_x, knee_z)

# Set up 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(-35, 5)
ax.view_init(elev=20, azim=-60)
lines = [ax.plot([], [], [], 'o-', lw=2)[0] for _ in range(4)]

def init():
    for line in lines:
        line.set_data([], [])
        line.set_3d_properties([])
    return lines

def update(frame):
    t = frame / frames
    body_offset_x = 10 * t  # forward motion
    for i in range(4):
        phase = phases[i]
        (foot_x_local, foot_z) = foot_trajectory(phase, t)
        hip_x, hip_y = leg_offsets[i]
        hip_x += body_offset_x
        hip_z = 0

        (knee_dx, knee_dz) = inverse_kinematics_2d(foot_x_local, foot_z)

        knee_x = hip_x + knee_dx
        knee_y = hip_y
        knee_z = hip_z - knee_dz

        foot_x = hip_x + foot_x_local
        foot_y = hip_y
        foot_z = foot_z

        lines[i].set_data([hip_x, knee_x, foot_x], [hip_y, knee_y, foot_y])
        lines[i].set_3d_properties([hip_z, knee_z, foot_z])
    return lines

ani = FuncAnimation(fig, update, frames=frames, init_func=init, blit=False, interval=1000/fps)
plt.title("3D Robot Dog Walk Gait (Natural Standing Posture)")
plt.show()
