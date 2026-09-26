import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PAPER = "#f5f2ec"
INK = "#1f2937"
RAY = "#e4572e"
ACCENT = "#f2b134"
GHOST = "#9aa5b1"

ghost_a = []
ghost_b = []
frames = []
path = []
current = None

with open("pose.txt", encoding="utf-8") as f:
    for line in f:
        parts = line.split()
        if not parts or parts[0].startswith("#"):
            continue
        tag = parts[0]
        if tag == "ghostA":
            ghost_a.append([float(v) for v in parts[1:7]])
        elif tag == "ghostB":
            ghost_b.append([float(v) for v in parts[1:7]])
        elif tag == "frame":
            current = []
            frames.append(current)
        elif tag == "segment" and current is not None:
            current.append([float(v) for v in parts[1:7]])
        elif tag == "path":
            path.append([float(v) for v in parts[1:4]])

print(f"Leidos {len(frames)} cuadros")
print(f"Trayectoria del efector: {len(path)} puntos")


def seg_arrays(segs):
    xs = []
    ys = []
    zs = []
    for s in segs:
        xs.append(s[0])
        xs.append(s[3])
        xs.append(np.nan)
        ys.append(s[1])
        ys.append(s[4])
        ys.append(np.nan)
        zs.append(s[2])
        zs.append(s[5])
        zs.append(np.nan)
    return np.array(xs), np.array(ys), np.array(zs)


fig = plt.figure(facecolor=PAPER)
ax = fig.add_subplot(111, projection="3d", facecolor=PAPER)

ga = seg_arrays(ghost_a)
ax.plot(ga[0], ga[1], ga[2], color=GHOST, linewidth=2, linestyle="--", label="pose A")
gb = seg_arrays(ghost_b)
ax.plot(gb[0], gb[1], gb[2], color=GHOST, linewidth=2, linestyle=":", label="pose B")

line0, = ax.plot([], [], [], color=RAY, linewidth=3, label="brazo")
joints, = ax.plot([], [], [], marker="o", linestyle="none", color=ACCENT,
                  markeredgecolor=INK, markersize=8)
trail, = ax.plot([], [], [], color=ACCENT, linewidth=2, alpha=0.8, label="trayectoria")

ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_zlim(-0.5, 3.5)
ax.set_xlabel("X", color=INK)
ax.set_ylabel("Y", color=INK)
ax.set_zlabel("Z", color=INK)
ax.set_title("geoalg: interpolacion suave entre dos poses", color=INK)
ax.legend(loc="upper left")
ax.grid(True, alpha=0.25)


def update(i):
    segs = frames[i]
    xs, ys, zs = seg_arrays(segs)
    line0.set_data(xs, ys)
    line0.set_3d_properties(zs)

    pts = np.array([segs[0][0:3]] + [s[3:6] for s in segs], dtype=float)
    joints.set_data(pts[:, 0], pts[:, 1])
    joints.set_3d_properties(pts[:, 2])

    p = np.array(path[: i + 1], dtype=float)
    trail.set_data(p[:, 0], p[:, 1])
    trail.set_3d_properties(p[:, 2])
    return [line0, joints, trail]


anim = FuncAnimation(fig, update, frames=len(frames), interval=50, blit=False)
plt.tight_layout()
plt.show()