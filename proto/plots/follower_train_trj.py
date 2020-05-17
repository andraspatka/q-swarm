import matplotlib.pyplot as plt
import numpy as np

from utils import readAndClean




follower = readAndClean('logs/follow-train.csv')
leader1 = readAndClean('logs/follow-leader1.csv')
leader2 = readAndClean('logs/follow-leader2.csv')
xf, yf = follower.T
xl1, yl1 = follower.T
xl2, yl2 = follower.T
plt.scatter(xf[1:], yf[1:], label="Trajectory")
plt.scatter(xf[0], yf[0], label="Start")

plt.ylim(ymin=-5, ymax=5)
plt.xlim(xmin=-5, xmax=5)
plt.scatter(-0.8, -0.7, label="Goal")
plt.grid()
plt.legend()
plt.show()