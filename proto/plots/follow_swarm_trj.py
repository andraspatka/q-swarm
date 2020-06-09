import matplotlib.pyplot as plt
import numpy as np

from utils import readAndClean, createPathToLogs



num_followers = 9
followersColor = 'gray'
followerStartColor = 'blue'
followerEndColor = 'blue'
leaderColor = 'red'
leaderStartColor = 'blue'
goalColor = 'black'
area = 4
for i in range(0,num_followers):
    follower = readAndClean(createPathToLogs(str(i) + '.csv'))
    xf, yf =follower.T
    plt.scatter(xf[1], yf[1],c=followerStartColor, s=area * 10)
    plt.scatter(xf, yf, s=area, c=followersColor, label="follower " + str(i))
    plt.scatter(xf[-1], yf[-1], c=followerEndColor, s=area * 10)

leader = readAndClean(createPathToLogs('leaderA.csv'))
xl, yl = leader.T
plt.scatter(xl[1], yl[1], c=leaderStartColor, s=area * 10)
plt.scatter(xl, yl, s=area,c=leaderColor, label="Leader")
plt.title('1 leader, 9 followers')
plt.ylim(ymin=-5, ymax=5)
plt.xlim(xmin=-5, xmax=5)
plt.scatter(2, -3, s=200,c=goalColor, label="Goal")
plt.grid()
plt.legend()

plt.savefig('png/follow_swarm.png', transparent=False, dpi=300)
plt.show()