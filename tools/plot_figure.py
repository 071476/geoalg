import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PAPER = "#f5f2ec"
INK = "#1f2937"
RAY = "#e4572e"
ACCENT = "#f2b134"

frames = []
current = None

with open("figure.txt", encoding="utf-8") as f:
    for line in f:
        parts = line.split()
        if not parts or parts[0].startswith("#"):
            continue
        tag = parts[0]
        if tag == "frame":
            current = {"segments": [], "points": []}
            frames.append(current)
        elif tag == "segment" and current is not None:
            current["segments"].append([float(v) for v in parts[1:7]])
        elif tag == "point" and current is not None:
            current["points"].append([float(v) for v in parts[1:4]])

print(f"Leidos {len(frames)} cuadros")
print(f"Primer cuadro: {len(frames[0]['segments'])} segmentos, {len(frames[0]['points'])} puntos")


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

line0, = ax.plot([], [], [], color=RAY, linewidth=3)
joints, = ax.plot([], [], [], marker="o", linestyle="none", color=ACCENT,
                  markeredgecolor=INK, markersize=8)

ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_zlim(0, 2.5)
ax.set_xlabel("X", color=INK)
ax.set_ylabel("Y", color=INK)
ax.set_zlabel("Z", color=INK)
ax.set_title("geoalg: personaje animado con motores", color=INK)
ax.grid(True, alpha=0.25)


def update(i):
    fr = frames[i]
    xs, ys, zs = seg_arrays(fr["segments"])
    line0.set_data(xs, ys)
    line0.set_3d_properties(zs)

    pts = np.array(fr["points"], dtype=float)
    joints.set_data(pts[:, 0], pts[:, 1])
    joints.set_3d_properties(pts[:, 2])
    return [line0, joints]


anim = FuncAnimation(fig, update, frames=len(frames), interval=50, blit=False)
plt.tight_layout()
plt.show()