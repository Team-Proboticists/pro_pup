import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# ─── ROBOT & GAIT PARAMETERS ────────────────────────────────────────────────

# Leg segment lengths (cm)
L1 = 12.0    # thigh
L2 = 15.5    # shin

# Gait timing & geometry
STEP_LENGTH   = 5.0   # cm
STEP_HEIGHT   = 3.0   # cm
CYCLE_TIME    = 2.0   # seconds per gait cycle
FPS           = 30
FRAMES_PER_CYCLE = int(CYCLE_TIME * FPS)

# Leg order: LF, RF, LR, RR
PHASES      = [0.25, 0.75, 0.0, 0.5]
LEG_OFFSETS = [(-8, 10), (8, 10), (-8, -10), (8, -10)]  # (x,y) of each hip

# Initial “standing” servo angles (channels 0–11) — tweak these manually
INITIAL_SERVO_ANGLES = [
    90, 40, 100,   # LF: yaw, thigh, knee
    90, 40, 100,   # RF
    90, 40, 100,   # LR
    90, 40, 100    # RR
]

# Mirror flags per channel: if True, final_angle = 180 - angle
SERVO_MIRROR = [
    False, True, True,   # LF
    False, True, True,   # RF
    False, True, True,   # LR
    False, True, True    # RR
]

# ─── STANDING FOOT POSITION ─────────────────────────────────────────────────

# Thigh 40° down from horizontal, knee 100° between thigh & shin
THIGH_ANGLE_RAD = np.radians(40)
KNEE_ANGLE_RAD  = np.radians(100)

def compute_standing_foot():
    knee_x = L1 * np.cos(THIGH_ANGLE_RAD)
    knee_z = -L1 * np.sin(THIGH_ANGLE_RAD)
    total  = THIGH_ANGLE_RAD + np.pi - KNEE_ANGLE_RAD
    foot_x = knee_x + L2 * np.cos(total)
    foot_z = knee_z - L2 * np.sin(total)
    return foot_x, foot_z

STAND_X, STAND_Z = compute_standing_foot()

# ─── FOOT TRAJECTORY & I.K. ───────────────────────────────────────────────────

def foot_trajectory(phase, t):
    """
    Returns (x, z) of the foot relative to hip in the gait cycle.
    Swing moves foot backward under body; stance carries body forward.
    """
    t_phase = (t + phase) % 1.0
    if t_phase < 0.25:
        r = t_phase / 0.25
        x = STAND_X + STEP_LENGTH/2 - STEP_LENGTH * r
        z = STAND_Z + STEP_HEIGHT * np.sin(np.pi * r)
    else:
        r = (t_phase - 0.25) / 0.75
        x = STAND_X - STEP_LENGTH/2 + STEP_LENGTH * r
        z = STAND_Z
    return x, z

def inverse_kinematics_angles(x, z):
    """
    Given foot (x forward, z up-negative), returns:
      hip_pitch (deg), knee_bend (deg)
    """
    dx, dz = x, -z
    dist = np.hypot(dx, dz)
    dist = min(dist, L1 + L2 - 1e-6)

    # Knee angle
    cos_k = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
    theta_knee = np.degrees(np.arccos(np.clip(cos_k, -1, 1)))

    # Hip pitch
    cos_a = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
    alpha = np.arccos(np.clip(cos_a, -1, 1))
    beta  = np.arctan2(dz, dx)
    theta_hip = np.degrees(beta - alpha)

    return theta_hip, theta_knee

def inverse_kinematics_2d(x, z):
    """
    Given foot (x, z), returns knee offset (dx, dz) from hip.
    Used for plotting 3D animation.
    """
    dx, dz = x, -z
    dist = np.hypot(dx, dz)
    dist = min(dist, L1 + L2 - 1e-6)
    a = np.arccos(np.clip((L1**2 + dist**2 - L2**2) / (2 * L1 * dist), -1, 1))
    b = np.arctan2(dz, dx)
    theta1 = b - a
    return L1 * np.cos(theta1), L1 * np.sin(theta1)

# ─── SERVO ANGLE EXTRACTION ───────────────────────────────────────────────────

def get_servo_angles(frame):
    """
    Returns {channel: angle_deg} for channels 0–11.
    Combines INITIAL offsets, gait offsets, and optional mirroring.
    """
    t = (frame % FRAMES_PER_CYCLE) / FRAMES_PER_CYCLE
    out = {}
    for leg_idx, phase in enumerate(PHASES):
        x_loc, z_loc = foot_trajectory(phase, t)
        hip_pitch, knee_bend = inverse_kinematics_angles(x_loc, z_loc)

        raw_angles = [0.0, hip_pitch, knee_bend]  # yaw=0 for straight walk
        ch_base = leg_idx * 3

        for i, offset in enumerate(raw_angles):
            ch = ch_base + i
            angle = INITIAL_SERVO_ANGLES[ch] + offset
            if SERVO_MIRROR[ch]:
                angle = 180.0 - angle
            out[ch] = angle

    return out

# ─── 3D WALK CYCLE ANIMATION ─────────────────────────────────────────────────

def walk_cycle(n_cycles=1):
    fig = plt.figure()
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-20, 20); ax.set_ylim(-20, 20); ax.set_zlim(-35, 5)
    ax.view_init(elev=20, azim=-60)

    lines = [ax.plot([], [], [], 'o-', lw=2)[0] for _ in range(4)]

    def update(frame):
        t = (frame % FRAMES_PER_CYCLE) / FRAMES_PER_CYCLE
        body_x = 10 * t

        for i, phase in enumerate(PHASES):
            # foot & hip
            x_loc, z_loc = foot_trajectory(phase, t)
            hip_x, hip_y = LEG_OFFSETS[i]
            hip_x += body_x
            hip_z  = 0

            # knee pos
            kdx, kdz = inverse_kinematics_2d(x_loc, z_loc)
            knee_x = hip_x + kdx
            knee_y = hip_y
            knee_z = hip_z - kdz

            # foot pos
            foot_x, foot_y, foot_z = hip_x + x_loc, hip_y, z_loc

            lines[i].set_data([hip_x, knee_x, foot_x],
                              [hip_y, knee_y, foot_y])
            lines[i].set_3d_properties([hip_z, knee_z, foot_z])

        return lines

    total_frames = FRAMES_PER_CYCLE * n_cycles
    FuncAnimation(fig, update, frames=total_frames,
                  blit=False, interval=1000/FPS, repeat=False)
    plt.show()

# ─── EXAMPLE USAGE ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Print servo angles at a few frames
    for f in (0, 30, 60, 90):
        print(f"Frame {f} →", get_servo_angles(f))
    # Play 2 gait cycles in 3D
    walk_cycle(n_cycles=2)
