import matplotlib.pyplot as plt
import numpy as np
from utils import removeStateAndAction

FOLDER_ROOTS = [
    ('../../../experiments/results/flock/labyrinth/logs/', 'first version'),
    ('../../../experiments/results/snake/labyrinth/logs/', 'second version')
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


def draw_labyrinth():
    wall_2 = np.linspace(0, 2, 400)
    wall_3 = np.linspace(0, 3, 400)
    y = np.zeros(400)
    plt.scatter(wall_2 - 1, y,     s=0.1, c=wallColor)
    plt.scatter(wall_2 + 2, y,     s=0.1, c=wallColor)
    plt.scatter(wall_3 - 1, y - 1, s=0.1, c=wallColor)
    plt.scatter(wall_3 + 1, y - 2, s=0.1, c=wallColor)
    plt.scatter(y - 1, wall_3 - 3, s=0.1, c=wallColor)
    plt.scatter(y + 4, wall_2 - 2, s=0.1, c=wallColor)




for (root, name) in FOLDER_ROOTS:
    plt.figure()
    draw_labyrinth()
    name_no_space = name.replace(' ', '_')
    plot_for_legend = True
    for i in range(1, NUM_FOLLOWERS + 1):
        follower = removeStateAndAction(np.genfromtxt(root + str(i) + '.csv', delimiter=','))
        xf, yf = follower.T
        if plot_for_legend:
            plt.scatter(xf, yf, s=area, c=followersColor, label='follower 1-30')
            plot_for_legend = False
        else:
            plt.scatter(xf, yf, s=area, c=followersColor)
    for i in range(1, NUM_FOLLOWERS + 1):
        follower = removeStateAndAction(np.genfromtxt(root + str(i) + '.csv', delimiter=','))
        xf, yf = follower.T
        plt.scatter(xf[-1], yf[-1], c=followerEndColor, s=area * 20)
    for i in range(1, NUM_FOLLOWERS + 1):
        follower = removeStateAndAction(np.genfromtxt(root + str(i) + '.csv', delimiter=','))
        xf, yf = follower.T
        plt.scatter(xf[1], yf[1], c=followerStartColor, s=area * 10)

    leader = removeStateAndAction(np.genfromtxt(root + 'leader.csv', delimiter=','))
    xl, yl = leader.T
    plt.scatter(xl[1], yl[1], c=leaderStartColor, s=area)
    plt.scatter(xl, yl, s=area / 20, c=leaderColor, label="Leader")
    plt.xlim(xmin=-3, xmax=6)
    plt.ylim(ymin=-6, ymax=4)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title('Labyrinth, 1 leader 6 followers: ' + name)
    # plt.grid()
    plt.legend()
    plt.savefig('png/labyrinth_' + name_no_space + '.png', transparent=False, dpi=300, bbox_inches="tight")
    plt.show()



