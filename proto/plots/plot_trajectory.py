import matplotlib.pyplot as plt
import numpy as np

points = np.loadtxt('csv_dat/fb3.csv', delimiter=',')
x, y = points.T
plt.scatter(x[1:], y[1:], label="Trajectory")
plt.scatter(x[0], y[0], label="Start")

plt.ylim(ymin=-5, ymax=5)
plt.xlim(xmin=-5, xmax=5)
plt.scatter(4, 0, label="Goal")
plt.grid()
plt.legend()
plt.show()