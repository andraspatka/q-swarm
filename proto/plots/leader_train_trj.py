import matplotlib.pyplot as plt
import numpy as np

from utils import readAndClean, createPathToLogs

leader = readAndClean(createPathToLogs('leader-train.csv'))
xl, yl = leader.T
area = 6
plt.scatter(xl[1:], yl[1:], s=area, label="Trajectory")
plt.scatter(xl[0], yl[0], s=100, label="Start")
plt.scatter(xl[-1], yl[-1], s=100)

plt.title('Training the leader agent')
plt.ylim(ymin=-5, ymax=5)
plt.xlim(xmin=-5, xmax=5)
plt.scatter(2, 2.3,s=100, label="Goal")
plt.grid()
plt.legend()
plt.savefig('leader_train_trj.png', transparent=False, dpi=300)
plt.show()