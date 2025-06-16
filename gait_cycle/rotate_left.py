import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ===========================
# Gait and Robot Parameters
# ===========================

cycle_time = 2.0       # seconds per gait cycle (change as needed)
dt = 0.02              # time step in seconds
time_steps = np.arange(0, cycle_time, dt)

# Step parameters (in centimeters)
base_step_length = 2.0     # nominal forward displacement (cm)
step_height = 2.0          # maximum vertical lift (cm)

# Differential multipliers for turning anti-clockwise (left turn):
left_multiplier = 0.75     # left legs take shorter steps
right_multiplier = 1.25    # right legs take longer steps

# Hip positions for each leg (in robot body frame, in cm)
# (x: lateral offset, y: forward offset, z: hip height)
hip_positions = {
    'left_front':  np.array([ 5.0,  10.0, 0]),
    'right_front': np.array([-5.0,  10.0, 0]),
    'left_rear':   np.array([ 5.0, -10.0, 0]),
    'right_rear':  np.array([-5.0, -10.0, 0])
}

# Phase offsets for each leg to simulate a diagonal trot:
# Front left and rear right swing together, and front right and rear left are offset by 0.5 cycle.
phase_offsets = {
    'left_front': 0.0,
    'right_rear': 0.0,
    'right_front': 0.5,
    'left_rear':   0.5
}

# Leg side assignment (to choose multipliers)
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
    Generate the local (relative to the hip) trajectory for one leg's contact point.
    For phase < swing_ratio (swing phase), the foot moves forward and upward (using a sine profile).
    For phase >= swing_ratio (stance phase), the foot remains at its touchdown point.
    
    Parameters:
        phase: normalized phase of the gait cycle [0, 1)
        base_step: nominal step length (cm)
        height: maximum vertical lift (cm)
        multiplier: differential multiplier affecting the step length
        swing_ratio: fraction of the cycle during which the leg is in swing
        
    Returns:
        forward_disp: forward displacement (cm) relative to the hip.
        z_disp: vertical displacement (cm) relative to the hip.
    """
    if phase < swing_ratio:
        tau = phase / swing_ratio
        forward_disp = base_step * multiplier * tau
        z_disp = height * np.sin(np.pi * tau)
    else:
        forward_disp = base_step * multiplier
        z_disp = 0.0
    return forward_disp, z_disp

# ===========================
# Compute Contact Trajectories for Each Leg
# ===========================
# Store trajectories in a dictionary; each will be an array of [x, y, z] coordinates.
trajectories = {leg: [] for leg in hip_positions.keys()}

for t in time_steps:
    for leg in hip_positions.keys():
        # Compute normalized phase in [0,1) for each leg (accounting for its phase offset)
        phase = ((t / cycle_time) + phase_offsets[leg]) % 1.0
        
        # Select the proper multiplier based on the leg side for anti-clockwise turning.
        multiplier = left_multiplier if leg_side[leg] == 'left' else right_multiplier
        
        # Generate the local trajectory (relative to the hip)
        forward_disp, z_disp = generate_leg_trajectory(phase, base_step_length, step_height, multiplier, swing_ratio=0.3)
        
        # Local contact point: forward_disp along y (foot's motion relative to the hip)
        local_contact = np.array([0.0, forward_disp, z_disp])
        
        # Global contact point by adding the hip position.
        global_contact = hip_positions[leg] + local_contact
        
        trajectories[leg].append(global_contact)

# Convert lists to numpy arrays for easier access.
for leg in trajectories:
    trajectories[leg] = np.array(trajectories[leg])

# ===========================
# Animation Setup
# ===========================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot fixed hip positions as markers.
for leg, hip in hip_positions.items():
    ax.plot([hip[0]], [hip[1]], [hip[2]], 'ko', markersize=8, label=f'Hip: {leg}')

# Create markers to visualize each leg’s foot (contact point)
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
ax.set_title("Anti‑Clockwise Turning Gait for Robodog")
ax.legend(loc='upper left')

# ===========================
# Animation Update Function
# ===========================
def update(frame):
    for leg in trajectories.keys():
        pos = trajectories[leg][frame]
        foot_markers[leg].set_data([pos[0]], [pos[1]])
        foot_markers[leg].set_3d_properties([pos[2]])
    return list(foot_markers.values())

ani = FuncAnimation(fig, update, frames=len(time_steps), interval=dt*1000, blit=True)
plt.show()
