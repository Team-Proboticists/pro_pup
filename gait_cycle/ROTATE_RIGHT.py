import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ===========================
# Gait and Robot Parameters
# ===========================

cycle_time = 5.0       # seconds per gait cycle
dt = 0.02              # time step in seconds remains the same
time_steps = np.arange(0, cycle_time, dt)

# Step parameters (in centimeters)
base_step_length = 2.0     # nominal forward displacement (cm) for a straight gait
step_height = 2.0          # maximum vertical lift (cm)

# Differential multipliers for turning in the opposite direction (clockwise):
# Swap the multipliers:
left_multiplier = 1.25     # now left legs take longer steps
right_multiplier = 0.75    # right legs take shorter steps

# Hip positions for each leg (in robot body frame, in cm)
# (x: lateral offset, y: forward offset, z: hip height)
hip_positions = {
    'left_front':  np.array([ 5.0,  10.0, 0]),
    'right_front': np.array([-5.0,  10.0, 0]),
    'left_rear':   np.array([ 5.0, -10.0, 0]),
    'right_rear':  np.array([-5.0, -10.0, 0])
}

# Phase offsets for each leg (to model gait timing)
# Diagonal trot: front left & rear right swing together, and the other pair is offset by 0.5 cycle.
phase_offsets = {
    'left_front': 0.0,
    'right_rear': 0.0,
    'right_front': 0.5,
    'left_rear':   0.5
}

# Leg side assignment (to select multiplier)
leg_side = {
    'left_front': 'left',
    'right_front': 'right',
    'left_rear': 'left',
    'right_rear': 'right'
}

# ===========================
# Trajectory Generation Function
# ===========================
def generate_leg_trajectory(phase, base_step, height, multiplier, swing_ratio=0.3):
    """
    Compute the leg's local foot (contact point) trajectory given a normalized phase.
    
    For phase < swing_ratio (swing phase): 
      - forward_disp ramps linearly from 0 to (base_step * multiplier) and
      - z_disp follows a sine profile from 0 up to 'height' (and back to 0 at tau = 1).
    
    For phase >= swing_ratio (stance phase):
      - The foot is held constant at the touchdown point.
    
    Parameters:
        phase: normalized time in the gait cycle [0, 1)
        base_step: nominal step length for straight motion (cm)
        height: maximum vertical lift (cm)
        multiplier: differential multiplier for turning (affects step length)
        swing_ratio: fraction of the cycle spent in swing (foot in air)
    
    Returns:
        forward_disp: forward displacement (cm) relative to the hip.
        z_disp: vertical displacement (cm) relative to the hip.
    """
    if phase < swing_ratio:
        # Swing phase: moving through the air quickly.
        tau = phase / swing_ratio
        forward_disp = base_step * multiplier * tau
        z_disp = height * np.sin(np.pi * tau)
    else:
        # Stance phase: the foot stays on the ground (dwell state).
        forward_disp = base_step * multiplier
        z_disp = 0.0
    return forward_disp, z_disp

# ===========================
# Compute Contact Trajectories for Each Leg
# ===========================
# Store each leg’s trajectory in a dictionary.
# Each trajectory is an array of [x, y, z] coordinates.
trajectories = {leg: [] for leg in hip_positions.keys()}

for t in time_steps:
    for leg in hip_positions.keys():
        # Compute the cyclic phase [0, 1) for the leg (including its phase offset)
        phase = ((t / cycle_time) + phase_offsets[leg]) % 1.0
        
        # Determine the multiplier based on leg side (left vs. right)
        multiplier = left_multiplier if leg_side[leg] == 'left' else right_multiplier
        
        # Generate the local trajectory (relative to the hip)
        forward_disp, z_disp = generate_leg_trajectory(phase, base_step_length, step_height, multiplier, swing_ratio=0.3)
        
        # Local contact point relative to the hip (forward_disp along y)
        local_contact = np.array([0.0, forward_disp, z_disp])
        
        # Global contact point by adding the fixed hip position.
        global_contact = hip_positions[leg] + local_contact
        
        trajectories[leg].append(global_contact)

# Convert trajectory lists into numpy arrays
for leg in trajectories:
    trajectories[leg] = np.array(trajectories[leg])

# ===========================
# Animation Setup
# ===========================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the hip positions as fixed markers.
for leg, hip in hip_positions.items():
    ax.plot([hip[0]], [hip[1]], [hip[2]], 'ko', markersize=8, label=f'Hip: {leg}')

# Create and store markers for each leg's foot (contact point).
foot_markers = {}
colors = {'left_front': 'r', 'right_front': 'b', 'left_rear': 'g', 'right_rear': 'm'}
for leg in hip_positions.keys():
    foot_markers[leg], = ax.plot([], [], [], 'o', color=colors[leg], markersize=10, label=f'Foot: {leg}')

ax.set_xlim(-15, 15)
ax.set_ylim(-20, 20)
ax.set_zlim(-1, 8)
ax.set_xlabel('X (Lateral, cm)')
ax.set_ylabel('Y (Forward, cm)')
ax.set_zlabel('Z (Vertical, cm)')
ax.set_title("Turning Gait for Robodog with Slow Stance Phase (Clockwise Turn)")
ax.legend(loc='upper left')

# ===========================
# Animation Update Function
# ===========================
def update(frame):
    # Update marker positions for each leg's contact point.
    for leg in trajectories.keys():
        pos = trajectories[leg][frame]
        foot_markers[leg].set_data([pos[0]], [pos[1]])
        foot_markers[leg].set_3d_properties([pos[2]])
    return list(foot_markers.values())

ani = FuncAnimation(fig, update, frames=len(time_steps), interval=dt*1000, blit=True)
plt.show()
