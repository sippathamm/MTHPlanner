import numpy as np
import matplotlib.pyplot as plt


def schwefel_function(x, y):
    return 418.9829 * 2 - (x * np.sin(np.sqrt(np.abs(x)))) - (y * np.sin(np.sqrt(np.abs(y))))


x = np.linspace(-500, 500, 100)
y = np.linspace(-500, 500, 100)
x, y = np.meshgrid(x, y)
z = schwefel_function(x, y)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(x, y, z, cmap='viridis')

ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False
ax.xaxis._axinfo['grid'].update(color='white', linestyle='-', linewidth=2)
ax.yaxis._axinfo['grid'].update(color='white', linestyle='-', linewidth=2)
ax.zaxis._axinfo['grid'].update(color='white', linestyle='-', linewidth=2)

ax.title.set_color('white')
ax.tick_params(axis='x', labelsize=12, colors='white')
ax.tick_params(axis='y', labelsize=12, colors='white')
ax.tick_params(axis='z', labelsize=12, colors='white')
ax.xaxis.label.set_color('white')
ax.yaxis.label.set_color('white')
ax.zaxis.label.set_color('white')

ax.set_xlabel('X', fontsize=18, labelpad=10)
ax.set_ylabel('Y', fontsize=18, labelpad=10)
ax.set_zlabel('Z', fontsize=18, labelpad=10)

ax.xaxis.line.set_color('white')
ax.yaxis.line.set_color('white')
ax.zaxis.line.set_color('white')

plt.savefig('../Schwefel\'s Functinon 2.26.png', dpi=600, bbox_inches='tight', pad_inches=0.3, transparent=True)
plt.show()
