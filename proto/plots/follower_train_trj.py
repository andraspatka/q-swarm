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
plt.scatter(xf[0], yf[0], s=100, label="Start")
plt.scatter(xf[-1], yf[-1], s=100, label="End")

plt.scatter(xl2, yl2, s=area, label="Leader 2")
plt.title('Training the follower agent')
plt.ylim(ymin=-5, ymax=5)
plt.xlim(xmin=-5, xmax=5)
plt.scatter(-0.8, -0.7, label="Goal", s=100)
plt.grid()
plt.legend()
plt.savefig('follow_train_trj.png', transparent=False, dpi=300)
plt.show()