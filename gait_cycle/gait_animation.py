import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Parameters
step_length = 30
step_height = 15
cycle_time = 2.0  # longer cycle for slow walking
fps = 30
frames = int(cycle_time * fps)

# LEG ORDER: [LF, RF, LR, RR]
# Walk gait phase offsets (each leg moves one at a time)
phases = [0.25, 0.75, 0.0, 0.5]

# Leg positions relative to body center
leg_offsets = [(-1,  1), (1,  1), (-1, -1), (1, -1)]  # LF, RF, LR, RR

fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-100, 100)
ax.set_ylim(-60, 60)
lines = [ax.plot([], [], 'o-', lw=2)[0] for _ in range(4)]
body, = ax.plot([], [], 'ks-', lw=3)

def leg_trajectory(phase, t):
    """Foot trajectory for walking."""
    t_phase = (t + phase) % 1.0
    if t_phase < 0.25:
        # swing phase
        ratio = t_phase / 0.25
        x = -step_length / 2 + step_length * ratio
        y = step_height * np.sin(np.pi * ratio)
    else:
        # stance phase
        ratio = (t_phase - 0.25) / 0.75
        x = step_length / 2 - step_length * ratio
        y = 0
    return x, y

def init():
    for line in lines:
        line.set_data([], [])
    body.set_data([], [])
    return lines + [body]

def update(frame):
    t = frame / frames
    body_x, body_y = 0, 0
    foot_positions = []

    for i in range(4):
        foot_x, foot_y = leg_trajectory(phases[i], t)
        offset = leg_offsets[i]
        hip_x = body_x + offset[0] * 30
        hip_y = body_y + offset[1] * 20
        world_x = hip_x + foot_x
        world_y = hip_y - foot_y
        lines[i].set_data([hip_x, world_x], [hip_y, world_y])
        foot_positions.append((world_x, world_y))

    # Draw a simple body box connecting hip points (approximation)
    body.set_data(
        [foot_positions[0][0], foot_positions[1][0], foot_positions[3][0], foot_positions[2][0], foot_positions[0][0]],
        [foot_positions[0][1]+30, foot_positions[1][1]+30, foot_positions[3][1]+30, foot_positions[2][1]+30, foot_positions[0][1]+30]
    )
    return lines + [body]

ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=1000/fps)
plt.title("Robot Dog Walk Gait Simulation (3 legs down)")
plt.show()
