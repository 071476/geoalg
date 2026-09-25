import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

PAPER = "#f5f2ec"
INK = "#1f2937"
GROUND = "#8fa8c8"
RAY = "#e4572e"
ACCENT = "#f2b134"

segments = []
points = []
plane = None

with open("scene.txt", encoding="utf-8") as f:
    for line in f:
        parts = line.split()
        if not parts or parts[0].startswith("#"):
            continue
        if parts[0] == "plane":
            plane = [float(v) for v in parts[1:5]]
        elif parts[0] == "segment":
            coords = [float(v) for v in parts[2:8]]
            segments.append((parts[1], coords))
        elif parts[0] == "point":
            coords = [float(v) for v in parts[2:5]]
            points.append((parts[1], coords))

fig = plt.figure(facecolor=PAPER)
ax = fig.add_subplot(111, projection="3d", facecolor=PAPER)

if plane is not None:
    a, b, c, d = plane
    n = np.array([a, b, c], dtype=float)
    n = n / np.linalg.norm(n)
    helper = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(n, helper)) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)
    center = -d * n
    size = 2.0
    corners = [
        center + size * (sx * u + sy * v)
        for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    ]
    quad = Poly3DCollection([corners], alpha=0.35, facecolor=GROUND, edgecolor=INK, linewidths=0.6)
    ax.add_collection3d(quad)

for name, coords in segments:
    x1, y1, z1, x2, y2, z2 = coords
    ax.plot([x1, x2], [y1, y2], [z1, z2], color=RAY, linewidth=3, label=name)
    ax.scatter([x1, x2], [y1, y2], [z1, z2], color=RAY, s=25)

for name, coords in points:
    x, y, z = coords
    ax.scatter([x], [y], [z], color=ACCENT, s=140, edgecolors=INK, linewidths=1.5, zorder=10)
    ax.text(x, y, z + 0.15, name, color=INK, fontsize=11)

ax.set_xlabel("X", color=INK)
ax.set_ylabel("Y", color=INK)
ax.set_zlabel("Z", color=INK)
ax.set_title("geoalg: rayo contra el suelo", color=INK, fontsize=14)
ax.set_xlim(-1, 2)
ax.set_ylim(-1, 2)
ax.set_zlim(-1, 2)
ax.grid(True, alpha=0.25)

seen = set()
handles, labels = ax.get_legend_handles_labels()
uniq = [(h, l) for h, l in zip(handles, labels) if not (l in seen or seen.add(l))]
if uniq:
    ax.legend(*zip(*uniq), loc="upper left")

plt.tight_layout()
plt.show()