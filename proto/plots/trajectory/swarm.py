import matplotlib.pyplot as plt
import numpy as np
from utils import removeStateAndAction

FOLDER_ROOTS = [
    ('../../../experiments/results/flock/swarm/logs/', 'first version'),
    ('../../../experiments/results/snake/swarm/logs/', 'second version')
]

followersColor = 'gray'
followerStartColor = 'blue'
followerEndColor = 'blue'
leaderColor = 'red'
leaderStartColor = 'blue'
goalColor = 'purple'
wallColor = 'black'
area = 0.4
NUM_FOLLOWERS = 30


def draw_obstacles():
    wall_20 = np.linspace(0, 20, 1000)
    y = np.zeros(1000)
    plt.scatter(wall_20 - 10, y - 10,     s=0.1, c=wallColor)
    plt.scatter(wall_20 - 10, y + 10,     s=0.1, c=wallColor)
    plt.scatter(y + 10, wall_20 - 10,     s=0.1, c=wallColor)
    plt.scatter(y - 10, wall_20 - 10,     s=0.1, c=wallColor)

for (root, name) in FOLDER_ROOTS:
    name_no_space = name.replace(' ', '_')
    plt.figure()
    draw_obstacles()
    plot_for_legend = True
    for i in range(0, NUM_FOLLOWERS):
        follower = removeStateAndAction(np.genfromtxt(root + str(i) + '.csv', delimiter=','))
        xf, yf = follower.T
        if plot_for_legend:
            plt.scatter(xf, yf, s=area, c=followersColor, label='follower 1-30')
            plot_for_legend = False
        else:
            plt.scatter(xf, yf, s=area, c=followersColor)
    for i in range(0, NUM_FOLLOWERS):
        follower = removeStateAndAction(np.genfromtxt(root + str(i) + '.csv', delimiter=','))
        xf, yf = follower.T
        plt.scatter(xf[-1], yf[-1], c=followerEndColor, s=area * 20)
    for i in range(0, NUM_FOLLOWERS):
        follower = removeStateAndAction(np.genfromtxt(root + str(i) + '.csv', delimiter=','))
        xf, yf = follower.T
        plt.scatter(xf[1], yf[1], c=followerStartColor, s=area * 10)

    leader = removeStateAndAction(np.genfromtxt(root + 'leader.csv', delimiter=','))
    xl, yl = leader.T
    plt.scatter(xl[1], yl[1], c=leaderStartColor, s=area)
    plt.scatter(xl, yl, s=area / 20, c=leaderColor, label="Leader")

    plt.xlim(xmin=-10, xmax=10)
    plt.ylim(ymin=-10, ymax=10)
    plt.scatter(9.5, 9, label="Goal", s = 40)
    plt.xlabel("x")
    plt.ylabel("y")
    # plt.grid()
    plt.title('1 leader 30 followers: ' + name)
    plt.legend()
    plt.savefig('png/swarm_' + name_no_space + '.png', transparent=False, dpi=300, bbox_inches="tight")
    plt.show()



