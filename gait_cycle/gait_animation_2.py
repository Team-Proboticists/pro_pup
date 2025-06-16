import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def inverse_kinematics_leg_3d(x, y, z, L1, L2):
    """
    Inverse kinematics for a 3-DOF leg:
      - θ1: Hip abduction/adduction (rotation about vertical axis)
      - θ2: Hip flexion/extension (pitch angle in the sagittal plane)
      - θ3: Knee flexion/extension
    All dimensions (x, y, z, L1, L2) are in centimeters.
    """
    # θ1: Hip abduction/adduction angle
    theta1 = np.arctan2(y, x)  # If y=0, this is either 0 or π, depending on x sign
    
    # Project foot position onto the sagittal plane (r, z)
    r = np.sqrt(x**2 + y**2)  # Corrected formula
    D = np.sqrt(r**2 + z**2)  # Corrected formula
    max_reach = L1 + L2
    if D > max_reach:
        # Scale (r,z) to maximum reachable distance
        scale = max_reach / D
        r *= scale
        z *= scale
        D = max_reach

    # Knee angle (θ3) via law of cosines
    cos_theta3 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
    theta3 = np.pi - np.arccos(cos_theta3)

    # Hip flexion (θ2)
    gamma = np.arctan2(z, r)  # angle from horizontal to the foot in sagittal plane
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)
    theta2 = gamma + beta

    return theta1, theta2, theta3

# === Gait Cycle Parameters (Time in seconds, lengths in cm) ===
cycle_time = 1.0      # seconds per gait cycle
dt = 0.02             # time step
time_steps = np.arange(0, cycle_time, dt)

# Leg geometry (cm)
L1 = 12  # Thigh length
L2 = 14  # Shank length

# Gait parameters (in cm)
step_length = 15      # forward displacement
step_height = step_length / 2  # foot lift

swing_phase_ratio = 0.5
stance_phase_ratio = 0.5

# === Generate Foot Trajectory in the XZ Plane (y=0) ===
foot_traj = []
for t in time_steps:
    phase = t / cycle_time
    if phase < swing_phase_ratio:
        # Swing phase
        tau = phase / swing_phase_ratio
        x = -step_length / 2 + step_length * tau
        y = 0  # force the trajectory to stay in XZ plane
        z = -step_height * np.sin(np.pi * tau)  # sinusoidal vertical lift
    else:
        # Stance phase
        tau = (phase - swing_phase_ratio) / stance_phase_ratio
        x = step_length / 2 - step_length * tau
        y = 0
        z = 0
    foot_traj.append((x, y, z))
foot_traj = np.array(foot_traj)

# === Compute Joint Angles ===
hip_abduction = []
hip_flexion = []
knee_flexion = []

for (x, y, z) in foot_traj:
    theta1, theta2, theta3 = inverse_kinematics_leg_3d(x, y, z, L1, L2)
    hip_abduction.append(theta1)
    hip_flexion.append(theta2)
    knee_flexion.append(theta3)

hip_abduction = np.array(hip_abduction)
hip_flexion = np.array(hip_flexion)
knee_flexion = np.array(knee_flexion)

# === Plot Results ===
fig = plt.figure(figsize=(12, 6))

# 3D Foot Trajectory
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot(foot_traj[:, 0], foot_traj[:, 1], foot_traj[:, 2], 'b.-')
ax1.set_xlabel('X (cm)')
ax1.set_ylabel('Y (cm)')
ax1.set_zlabel('Z (cm)')
ax1.set_title("3D Foot Trajectory in XZ Plane (y=0)")

# Make the view such that X is horizontal and Z is vertical
# You can tweak elev and azim to get the view you prefer
ax1.view_init(elev=0, azim=-90)

# Joint Angles Over Time
ax2 = fig.add_subplot(122)
ax2.plot(time_steps, np.degrees(hip_abduction), label='Hip Abduction (θ1)')
ax2.plot(time_steps, np.degrees(hip_flexion), label='Hip Flexion (θ2)')
ax2.plot(time_steps, np.degrees(knee_flexion), label='Knee (θ3)')
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Angle (degrees)")
ax2.set_title("Joint Angles Over Gait Cycle")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()