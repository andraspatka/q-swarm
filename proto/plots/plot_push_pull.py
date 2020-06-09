import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

WHITE = 'white'
BLACK = 'black'
mpl.rcParams['text.color'] = BLACK
mpl.rcParams['font.size'] = 16
mpl.rcParams['axes.labelcolor'] = BLACK
mpl.rcParams['xtick.color'] = BLACK
mpl.rcParams['ytick.color'] = BLACK
mpl.rcParams['lines.linewidth'] = 4


B_PUSH = 0  # The robot's centre
B_PULL = 3
C_PUSH = 0.5  # width of the gauss curve
C_PULL = 0.65
cut_off = 0.05
A = 1  # height of the gauss curve
x_plt = np.linspace(0, B_PULL, num=100)

num_push = - (x_plt - B_PUSH) ** 2
denum_push = 2 * C_PUSH ** 2
y_push = A * np.exp(num_push / denum_push)
y_push[y_push <= cut_off] = 0
y_push[x_plt > 1] = 0
push_plot = plt.plot(x_plt, y_push, label="Push")

num_pull = - (x_plt - B_PULL) ** 2
denum_pull = 2 * C_PULL ** 2
y_pull = A * np.exp(num_pull / denum_pull)
y_pull[y_pull <= cut_off] = 0
pull_plot = plt.plot(x_plt, y_pull, label="Pull")
plt.xlabel("Distance")
plt.ylabel("Magnitude")
plt.ylim(ymin=0, ymax=1)
plt.xlim(xmin=0, xmax=B_PULL)
plt.title('Push-pull forces for the follower agent')

plt.grid()
plt.legend()

plt.savefig('png/pushpull_follow.png', transparent=False, dpi=300)
plt.show()

