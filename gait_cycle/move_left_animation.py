import time
import numpy as np
import math
import Adafruit_PCA9685
# For optional visualization:

# ========================
# LEG AND GAIT PARAMETERS
# ========================
L1 = 12.0      # Thigh length (cm)
L2 = 15.5      # Shank length (cm)
hip_z = 20.0   # Fixed hip height (cm)

# Leg identifiers and horizontal offsets for the hip (in cm)
# (x-offsets for IK may be used for fine adjustments in the servo transforms)
gait_order = ['RF', 'LH', 'RH', 'LF']
legs = ['FR', 'FL', 'RR', 'RL']
offsets = {'FL': -10, 'FR': 10, 'RR': 10, 'RL': -10}

# ========================
# STEP TRAJECTORY PARAMETERS (SIDEWAYS GAIT)
# ========================
cycle_time = 1.0      # Total gait cycle time (seconds)
dt = 0.02             # Time step (seconds)
time_steps = np.arange(0, cycle_time, dt)
swing_duration = 0.5  # Duration (seconds) of swing phase for each leg

# Step parameters for sideways (lateral) displacement:
step_width = 2.0    # Lateral displacement (cm) during swing
step_height = 2.0   # Vertical (z) lift (cm) during swing

# Direction multiplier: set -1 for leftward motion, +1 for rightward.
direction = -1

# ------------------------------------------------------
# Define Base Positions and Phase Offsets for Each Leg
# ------------------------------------------------------
# For this quadruped layout:
#   - x (lateral) coordinates: right-side legs are positive, left-side negative.
#   - y (fore/aft) coordinates: front and rear are different.
#   - z is taken as 0 for the foot on the ground.
legs_info = {
    'FR': {'base': np.array([5.0,  10.0, 0.0]), 'phase_offset': 0.5},  # Front Right
    'FL': {'base': np.array([-5.0, 10.0, 0.0]), 'phase_offset': 0.0},   # Front Left
    'RR': {'base': np.array([5.0,  -10.0, 0.0]), 'phase_offset': 0.0},   # Rear Right
    'RL': {'base': np.array([-5.0, -10.0, 0.0]), 'phase_offset': 0.5}    # Rear Left
}
# Phase offsets ensure that FL and RR begin their swing at t_eff = 0,
# while FR and RL have a half-cycle offset (swing starts later).

# ------------------------------------------------------
# Generate Leg Trajectories for the Sideways Gait
# ------------------------------------------------------
# Each leg’s foot trajectory is computed based on its base position and phase offset.
leg_trajs = {name: [] for name in legs_info}  # Stores (x, y, z) positions for each leg
for t in time_steps:
    for leg, info in legs_info.items():
        base = info['base']
        offset = info['phase_offset']
        # Compute effective time in cycle (wrap-around with phase offset)
        t_eff = (t + offset * cycle_time) % cycle_time
        
        if t_eff < swing_duration:
            # SWING PHASE: foot moves laterally from base and lifts vertically
            tau = t_eff / swing_duration  # normalized time in swing phase
            # Lateral displacement (x coordinate): move gradually by step_width (with direction multiplier)
            x = base[0] + direction * step_width * tau
            # Vertical displacement (z coordinate): sinusoidal lift (zero at beginning and end)
            z = step_height * np.sin(np.pi * tau)
        else:
            # STANCE PHASE: foot remains at the new lateral position with z=0
            x = base[0] + direction * step_width
            z = 0.0
        # y (fore/aft) remains constant at the base value.
        y = base[1]
        pos = np.array([x, y, z])
        leg_trajs[leg].append(pos)
        
# Convert trajectory lists to numpy arrays for easier indexing.
for leg in leg_trajs:
    leg_trajs[leg] = np.array(leg_trajs[leg])

# ========================
# SERVO SETUP & MAPPING
# ========================
# Mapping for each leg: assign two channels per leg (hip and knee)
servo_channels = {
    'FL': {'hip': 0, 'knee': 1},
    'FR': {'hip': 2, 'knee': 3},
    'RR': {'hip': 4, 'knee': 5},
    'RL': {'hip': 6, 'knee': 7}
}

# Initialize the PWM controller (PCA9685) and set frequency to 50Hz.
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

# Define a conversion function mapping an angle (in degrees) to a servo pulse (ticks).
def angle_to_pulse(angle_deg, min_angle=0, max_angle=180, min_pulse=150, max_pulse=600):
    angle_deg = max(min_angle, min(max_angle, angle_deg))
    pulse_range = max_pulse - min_pulse
    angle_range = max_angle - min_angle
    pulse = int(min_pulse + (angle_deg - min_angle) * pulse_range / angle_range)
    return pulse

def move_servo(channel, angle_rad):
    angle_deg = math.degrees(angle_rad)
    pulse = angle_to_pulse(angle_deg)
    # Uncomment the next line for debug printing if needed.
    # print("Channel:", channel, "Angle (deg):", angle_deg, "Pulse:", pulse)
    pwm.set_pwm(channel, 0, pulse)

# =================================
# GAIT CYCLE KINEMATICS FUNCTIONS
# =================================
def inverse_kinematics_2d(x, z, L1, L2):
    """
    Compute the two joint angles (hip and knee) in the sagittal plane.
    x: horizontal displacement (in leg's plane)
    z: vertical displacement (hip_z - foot_z)
    """
    r = np.sqrt(x**2 + z**2)
    D = min(r, L1 + L2)  # Ensure the target is reachable.
    cos_theta2 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.pi - np.arccos(cos_theta2)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)
    gamma = np.arctan2(z, x)
    theta1 = gamma + beta
    return theta1, theta2

def leg_ik_for_servo(x_offset, x, z):
    """
    Compute the IK for a leg given:
      - x_offset: Horizontal hip offset (for positioning in body frame)
      - x: Relative swing displacement (cm) in the leg's plane
      - z: Desired foot height (cm)
    Returns:
      hip_angle, knee_angle (both in radians)
    """
    # In the leg’s sagittal plane, x_rel is the horizontal displacement, and
    # z_rel is the vertical difference between hip and foot.
    # (Adjust x_rel by offset if needed; here we assume offset is applied elsewhere.)
    x_rel = x
    z_rel = hip_z - z
    theta_hip, theta_knee = inverse_kinematics_2d(x_rel, z_rel, L1, L2)
    return theta_hip, theta_knee

# ========================
# MAIN CONTROL LOOP (SIDEWAYS GAIT)
# ========================
print("Starting sideways gait cycle... (Press Ctrl+C to quit)")

# A counter to iterate through the pre-computed trajectory
frame_idx = 0
n_frames = len(time_steps)

try:
    while True:
        # Cycle through the gait cycle repeatedly.
        for frame_idx in range(n_frames):
            # For each leg, get the current (x, y, z) position from the trajectory.
            for leg in legs_info:
                pos = leg_trajs[leg][frame_idx]
                # x here is lateral displacement; use it in IK.
                # The x_offset (from 'offsets') may be used to adjust the hip's lateral position.
                # Here we simply add the x_offset to the computed x if needed.
                x_offset = offsets[leg]  # This may be incorporated into the kinematics as required.
                x_pos = pos[0]  # lateral
                z_pos = pos[2]  # vertical (lift)
                # Compute inverse kinematics for the leg (in the leg’s sagittal plane).
                theta_hip, theta_knee = leg_ik_for_servo(x_offset, x_pos, z_pos)
                # Send computed angles to the corresponding servos.
                move_servo(servo_channels[leg]['hip'], theta_hip)
                move_servo(servo_channels[leg]['knee'], theta_knee)

            time.sleep(dt)  # Wait for the next time step

except KeyboardInterrupt:
    print("Exiting gait control program.")

# ========================
# OPTIONAL: Visualization of the Sideways Gait Cycle
# ========================
# Uncomment the following block if you wish to see an animation of the gait cycle
# instead of (or in addition to) driving servos.

"""
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
colors = {'FR': 'r', 'FL': 'g', 'RR': 'b', 'RL': 'm'}
markers = {}
lines = {}
trace_data = {leg: [] for leg in legs_info}

for leg in legs_info:
    pos0 = leg_trajs[leg][0]
    markers[leg], = ax.plot([pos0[0]], [pos0[1]], [pos0[2]], marker='o',
                            color=colors[leg], markersize=8, label=leg)
    lines[leg], = ax.plot([], [], [], color=colors[leg], linestyle='--', linewidth=1)

ax.set_xlim(-10, 10)
ax.set_ylim(-15, 15)
ax.set_zlim(0, step_height + 1)
ax.set_xlabel('X (cm) - Lateral')
ax.set_ylabel('Y (cm) - Fore/Aft')
ax.set_zlabel('Z (cm) - Vertical')
ax.set_title("Quadruped Sideways Gait (Moving to the LEFT)")
ax.legend()

def update(frame):
    for leg in legs_info:
        pos = leg_trajs[leg][frame]
        markers[leg].set_data([pos[0]], [pos[1]])
        markers[leg].set_3d_properties([pos[2]])
        trace_data[leg].append(pos)
        trace = np.array(trace_data[leg])
        lines[leg].set_data(trace[:, 0], trace[:, 1])
        lines[leg].set_3d_properties(trace[:, 2])
    return list(markers.values()) + list(lines.values())

ani = FuncAnimation(fig, update, frames=n_frames, interval=dt*1000, blit=True)
plt.show()
"""
