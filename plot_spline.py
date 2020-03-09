from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
spline = []
with open('spline.txt', 'r') as f:
    for _ in range(500):
        spline.append([float(i) for i in f.readline().split(',')])
spline = np.array(spline)

print(spline)
fig = plt.figure()
ax = plt.axes(projection="3d")

# z_line = np.linspace(0, 15, 1000)
# x_line = np.cos(z_line)
# y_line = np.sin(z_line)
ax.scatter3D(spline[:, 0], spline[:, 1], spline[:, 2], 'gray')

# z_points = 15 * np.random.random(100)
# x_points = np.cos(z_points) + 0.1 * np.random.randn(100)
# y_points = np.sin(z_points) + 0.1 * np.random.randn(100)
# ax.scatter3D(x_points, y_points, z_points, c=z_points, cmap='hsv');

plt.show()
plt.plot(spline[:, 0], spline[:, 3])
plt.show()
