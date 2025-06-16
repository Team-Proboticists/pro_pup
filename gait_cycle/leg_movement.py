import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox  # Not used here, just for example
import serial

# --- Set up the serial port for the pressure sensor ---
# Change 'COM3' to your port name (e.g., '/dev/ttyUSB0' on Linux) and adjust baud rate if necessary.
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
except Exception as e:
    print("Error opening serial port: ", e)
    ser = None  # In testing, you might want to simulate sensor values

# === Parameters ===
L1 = 12
L2 = 15.5
cycle_time = 1.0
dt = 0.02
time_steps = np.arange(0, cycle_time, dt)

step_length = 15
step_height = 15
stride_speed = 25  # cm/s
hip_z = 25

body_x = stride_speed * time_steps

# --- Generate Foot Trajectory (swing + stance) ---
foot_global = []
for t in time_steps:
    phase = t / cycle_time
    if phase < 0.5:
        tau = phase / 0.5
        x = -step_length / 2 + step_length * tau
        z = step_height * np.sin(np.pi * tau)
    else:
        tau = (phase - 0.5) / 0.5
        x = step_length / 2 - step_length * tau
        z = 0
    foot_global.append((x, z))
foot_global = np.array(foot_global)

# --- Inverse Kinematics Function ---
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

# --- Compute Positions and Joint Angles ---
joint1, foot_computed, hip = [], [], []
theta1_list, theta2_list = [], []

for i, (xf, zf) in enumerate(foot_global):
    hx = body_x[i]
    hip.append((hx, hip_z))
    x_rel = xf
    z_rel = hip_z - zf
    theta1, theta2 = inverse_kinematics_2d(x_rel, z_rel, L1, L2)
    theta1_list.append(np.degrees(theta1))
    theta2_list.append(np.degrees(theta2))
    
    kx = hx + L1 * np.cos(theta1)
    kz = hip_z - L1 * np.sin(theta1)
    fx = kx + L2 * np.cos(theta1 - theta2)
    fz = kz - L2 * np.sin(theta1 - theta2)
    
    joint1.append((kx, kz))
    foot_computed.append((fx, fz))
    
hip = np.array(hip)
joint1 = np.array(joint1)
foot_computed = np.array(foot_computed)

# --- Transform to Hip-Fixed View (Hip fixed at (0,0)) ---
hip_view = []
knee_view = []
foot_view = []
foot_des_view = []

for i in range(len(time_steps)):
    hx, hz = hip[i]
    kx, kz = joint1[i]
    fx, fz = foot_computed[i]
    # foot desired: computed as body_x plus foot offset, then
    # adjust the vertical from hip_z minus foot_global z.
    dx, dz = (body_x[i] + foot_global[i][0], hip_z - foot_global[i][1])
    
    hip_view.append((0, 0))
    knee_view.append((kx - hx, kz - hz))
    foot_view.append((fx - hx, fz - hz))
    foot_des_view.append((dx - hx, dz - hz))
    
hip_view = np.array(hip_view)
knee_view = np.array(knee_view)
foot_view = np.array(foot_view)
foot_des_view = np.array(foot_des_view)

# --- Real Pressure Sensor Reading Function ---
def get_pressure_sensor_value(frame):
    """
    Read a line from the sensor via serial.
    If the sensor value is 0, return 0; otherwise, return a non-zero value.
    In this example, we expect the sensor to send ASCII text representing a number.
    """
    if ser is not None:
        try:
            line = ser.readline().strip()
            if line:
                # Decode and convert to float
                value = float(line.decode('utf-8'))
                return value
            else:
                return 1  # No reading; assume non-zero
        except Exception as e:
            print("Error reading sensor:", e)
            return 1
    else:
        # If serial not available, simulate:
        # For example: pause between frames 10 and 20
        if 10 <= frame < 20:
            return 0
        else:
            return 1

# --- Plot Setup ---
fig, (ax_leg, ax_angles) = plt.subplots(2, 1, figsize=(8, 8), gridspec_kw={'height_ratios': [2, 1]})
plt.subplots_adjust(bottom=0.25)

# Top: Leg Animation Plot
ax_leg.set_xlim(-30, 30)
ax_leg.set_ylim(-10, 60)
ax_leg.set_aspect('equal')
ax_leg.set_title("1-Leg Walking with Pressure Sensor Control")
ax_leg.set_xlabel("X (cm)")
ax_leg.set_ylabel("Z (cm)")

thigh_line, = ax_leg.plot([], [], 'ro-', lw=4, label='Thigh')
shank_line, = ax_leg.plot([], [], 'bo-', lw=4, label='Shank')
ground_line, = ax_leg.plot([], [], 'k--', lw=1)
ax_leg.legend()

# Bottom: Joint Angle Plot (θ₁ and θ₂)
time_sec = time_steps
theta1_arr = np.array(theta1_list)
theta2_arr = np.array(theta2_list)

theta1_line, = ax_angles.plot(time_sec, theta1_arr, 'r-', label='θ₁ (Hip)')
theta2_line, = ax_angles.plot(time_sec, theta2_arr, 'b-', label='θ₂ (Knee)')
# Fix the marker: must pass list to set_xdata
frame_marker = ax_angles.axvline([0], color='k', linestyle='--', label='Current Frame')

ax_angles.set_xlim(0, cycle_time)
ax_angles.set_ylim(0, 180)
ax_angles.set_xlabel("Time (s)")
ax_angles.set_ylabel("Angle (deg)")
ax_angles.legend()
ax_angles.grid(True)

# --- Animation State ---
is_running = True
current_frame = 0

def init():
    thigh_line.set_data([], [])
    shank_line.set_data([], [])
    ground_line.set_data([], [])
    frame_marker.set_xdata([0])
    return thigh_line, shank_line, ground_line, frame_marker

def manual_update(_):
    global current_frame
    sensor_value = get_pressure_sensor_value(current_frame)
    # If sensor_value is non-zero, resume; if 0, pause (hold frame)
    if sensor_value != 0:
        draw_frame(current_frame)
        current_frame = (current_frame + 1) % len(time_steps)
    # Else: sensor_value == 0, do nothing (hold current frame)
    return thigh_line, shank_line, ground_line, frame_marker

def draw_frame(frame):
    x_shift = hip[frame, 0]
    thigh_line.set_data(
        [0, knee_view[frame, 0]],
        [0, knee_view[frame, 1]]
    )
    shank_line.set_data(
        [knee_view[frame, 0], foot_view[frame, 0]],
        [knee_view[frame, 1], foot_view[frame, 1]]
    )
    ground_line.set_data([-30, 30], [0, 0])
    frame_marker.set_xdata([time_steps[frame]])

# --- Animation Loop ---
ani = FuncAnimation(fig, manual_update, frames=len(time_steps), init_func=init,
                    interval=40, blit=True)

plt.show()
