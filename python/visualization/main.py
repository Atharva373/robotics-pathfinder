import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# ---- disable annoying default keybinds ----
mpl.rcParams['keymap.save'] = []
mpl.rcParams['keymap.quit'] = []
mpl.rcParams['keymap.fullscreen'] = []
mpl.rcParams['keymap.zoom'] = []
mpl.rcParams['keymap.pan'] = []

SIZE = 70
SENSOR_RANGE = 20

# ===== REAL MAP (ground truth hidden from robot) =====
real_map = np.zeros((SIZE, SIZE))

real_map[20:40, 25:30] = 1
real_map[10:15, 5:20] = 1
real_map[45:50, 40:55] = 1
real_map[30:35, 40:50] = 1

# ===== ROBOT MEMORY MAP =====
known_map = -np.ones_like(real_map)

robot = [5, 5]


# ===== SENSOR MODEL =====
def update_visibility():

    rx, ry = robot

    num_rays = 180   # resolution
    angles = np.linspace(0, 2*np.pi, num_rays)

    for theta in angles:

        for r in np.linspace(0, SENSOR_RANGE, SENSOR_RANGE*3):

            x = int(round(rx + r * np.cos(theta)))
            y = int(round(ry + r * np.sin(theta)))

            if not (0 <= x < SIZE and 0 <= y < SIZE):
                break

            known_map[y, x] = real_map[y, x]

            if real_map[y, x] == 1:
                break

# ===== CREATE FIGURE / SCENE ONCE =====
fig, ax = plt.subplots()

display = known_map.copy()
display[display == -1] = 0.5

img = ax.imshow(display,
                origin="lower",
                cmap="gray_r",
                extent=[0, SIZE, 0, SIZE],
                vmin=0,
                vmax=1)

robot_dot = ax.scatter(robot[0], robot[1],
                       c='red', s=120)

sensor_circle = plt.Circle(robot,
                           SENSOR_RANGE,
                           color='blue',
                           fill=False,
                           linewidth=2)

ax.add_patch(sensor_circle)

ax.set_xlim(0, SIZE)
ax.set_ylim(0, SIZE)
ax.set_aspect('equal')
ax.grid(True, color='lightgray', linewidth=0.5)

plt.title("Exploration Simulator")


# ===== UPDATE SCENE (NO CLA) =====
def update_scene():
    display = known_map.copy()
    display[display == -1] = 0.5

    img.set_data(display)

    robot_dot.set_offsets([robot[0], robot[1]])

    sensor_circle.center = robot

    fig.canvas.draw_idle()


# ===== COLLISION SAFE MOVE =====
def try_move(dx, dy):
    nx = robot[0] + dx
    ny = robot[1] + dy

    if 0 <= nx < SIZE and 0 <= ny < SIZE:
        if real_map[ny, nx] == 0:
            robot[0] = nx
            robot[1] = ny


# ===== KEY HANDLER =====
def on_key(event):
    k = event.key.lower()

    if k == 'w':
        try_move(0, 1)
    elif k == 's':
        try_move(0, -1)
    elif k == 'a':
        try_move(-1, 0)
    elif k == 'd':
        try_move(1, 0)
    else:
        return

    update_visibility()
    update_scene()


fig.canvas.mpl_connect('key_press_event', on_key)

# initial sensing
update_visibility()
update_scene()

plt.show()