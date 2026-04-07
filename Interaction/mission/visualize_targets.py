import yaml
import matplotlib
matplotlib.use('MacOSX')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

yaml_path = os.path.join(os.path.dirname(__file__), 'acm_interaction_peer.yaml')

with open(yaml_path) as f:
    data = yaml.safe_load(f)

fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection='3d')

colors = ['#4C9BE8', '#E87B4C', '#4CE87B', '#E84C9B', '#9BE84C', '#E8D44C', '#4CE8D4']
for i, (name, d) in enumerate(data['drones'].items()):
    t = d['target']
    x, y, z = t[0], t[1], t[2]
    c = colors[i % len(colors)]
    ax.scatter(x, y, z, s=120, color=c, edgecolors='black', linewidths=0.5, zorder=5)
    ax.text(x + 0.02, y + 0.02, z + 0.025, name, fontsize=9, fontweight='bold', color='black')

ax.set_xlabel('X (m)', labelpad=8)
ax.set_ylabel('Y (m)', labelpad=8)
ax.set_zlabel('Z (m)', labelpad=8)
ax.set_title('ACM Drone Target Positions', pad=12)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()
