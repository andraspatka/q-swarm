import matplotlib.pyplot as plt
import numpy as np
from utils import removeStateAndAction

FOLDER_ROOTS = [
    ('../../../experiments/results/flock/flock/logs/', 'first version'),
    ('../../../experiments/results/snake/flock/logs/', 'second version')
]

followersColor = 'gray'
followerStartColor = 'blue'
followerEndColor = 'blue'
leaderColor = 'red'
leaderStartColor = 'blue'
goalColor = 'black'
wallColor = 'black'
area = 0.4
NUM_FOLLOWERS = 6


def draw_obstacles():
    wall_04 = np.linspace(0, 0.4, 400)
    y = np.zeros(400)
    plt.scatter(wall_04 + 1.8, y + 1.8,     s=0.1, c=wallColor)
    plt.scatter(wall_04 + 1.8, y + 2.2,     s=0.1, c=wallColor)
    plt.scatter(y + 1.8, wall_04 + 1.8,     s=0.1, c=wallColor)
    plt.scatter(y + 2.2, wall_04 + 1.8,     s=0.1, c=wallColor)

    plt.scatter(wall_04 + 3.8, y + 0.3,     s=0.1, c=wallColor)
    plt.scatter(wall_04 + 3.8, y + 0.7,     s=0.1, c=wallColor)
    plt.scatter(y + 3.8, wall_04 + 0.3,     s=0.1, c=wallColor)
    plt.scatter(y + 4.2, wall_04 + 0.3,     s=0.1, c=wallColor)




for (root, name) in FOLDER_ROOTS:
    plt.figure()
    draw_obstacles()
    leader = removeStateAndAction(np.genfromtxt(root + 'leader.csv', delimiter=','))
    xl, yl = leader.T
    plt.scatter(xl[1], yl[1], c=leaderStartColor, s=area * 10)
    plt.scatter(xl, yl, s=area, c=leaderColor, label="Leader")
    for i in range(0, NUM_FOLLOWERS):
        follower = removeStateAndAction(np.genfromtxt(root + str(i) + '.csv', delimiter=','))
        xf, yf = follower.T
        plt.scatter(xf[1], yf[1], c=followerStartColor, s=area * 10)
        plt.scatter(xf, yf, s=area, c=followersColor, label="follower " + str(i))
        plt.scatter(xf[-1], yf[-1], c=followerEndColor, s=area * 10)
    plt.xlim(xmin=-3, xmax=8)
    plt.ylim(ymin=-4, ymax=6)
    plt.scatter(7, 0, label="Goal", s = 20)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid()
    plt.title('Flocking, ' + name)
    plt.legend()
    plt.savefig('png/flock_' + name + '.png', transparent=False, dpi=300)
    plt.show()



