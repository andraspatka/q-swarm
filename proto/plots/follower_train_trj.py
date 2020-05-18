import matplotlib.pyplot as plt
import numpy as np

from utils import readAndClean, createPathToLogs


follower = readAndClean(createPathToLogs('follow-train.csv'))
follower = follower[0:800]
leader2 = readAndClean(createPathToLogs('follow-leader2.csv'))
xf, yf = follower.T
xl2, yl2 = leader2.T
area = 6
plt.scatter(xf[1:], yf[1:], s=area, label="Trajectory")
plt.scatter(xf[0], yf[0], s=area * 2, label="Start")
plt.scatter(xf[-1], yf[-1], s=200)

plt.scatter(xl2, yl2, s=area, label="Leader 2")

plt.ylim(ymin=-5, ymax=5)
plt.xlim(xmin=-5, xmax=5)
plt.scatter(-0.8, -0.7, label="Goal")
plt.grid()
plt.legend()
plt.show()