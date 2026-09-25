import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PAPER = "#f5f2ec"
INK = "#1f2937"
RAY = "#e4572e"
ACCENT = "#f2b134"

frames = []
current = None

with open("arm.txt", encoding="utf-8") as f:
    for line in f:
        parts = line.split()
        if not parts or parts[0].startswith("#"):
            continue
        if parts[0] == "frame":
            current = {"points": [], "segments": []}
            frames.append(current)
        elif parts[0] == "point" and current is not None:
            current["points"].append([float(v) for v in parts[1:4]])
        elif parts[0] == "segment" and current is not None:
            current["segments"].append([float(v) for v in parts[1:7]])

print(f"Leidos {len(frames)} cuadros")
print(f"Primer cuadro: {len(frames[0]['points'])} puntos, {len(frames[0]['segments'])} segmentos")

fig = plt.figure(facecolor=PAPER)
ax = fig.add_subplot(111, projection="3d", facecolor=PAPER)

line0, = ax.plot([], [], [], color=RAY, linewidth=3)

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 3)
ax.set_xlabel("X", color=INK)
ax.set_ylabel("Y", color=INK)
ax.set_zlabel("Z", color=INK)
ax.set_title("geoalg: brazo robotico de 3 articulaciones", color=INK)
ax.grid(True, alpha=0.25)


def update(i):
    fr = frames[i]
    pts = np.array(fr["points"], dtype=float)
    xs = []
    ys = []
    zs = []
    for s in fr["segments"]:
        xs.append(s[0])
        xs.append(s[3])
        xs.append(np.nan)
        ys.append(s[1])
        ys.append(s[4])
        ys.append(np.nan)
        zs.append(s[2])
        zs.append(s[5])
        zs.append(np.nan)
    line0.set_data(np.array(xs), np.array(ys))
    line0.set_3d_properties(np.array(zs))
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color=ACCENT, s=50, edgecolors=INK, linewidths=1.0)
    return [line0]


anim = FuncAnimation(fig, update, frames=len(frames), interval=50, blit=False)
plt.tight_layout()
plt.show()