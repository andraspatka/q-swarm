import matplotlib.pyplot as plt
import numpy as np

x_plt = np.linspace(0, 3, num=100)
B_PUSH = 0  # The robot's centre
B_PULL = 3  # The prox sensor's coverage limit
C_PUSH = 0.5  # width of the gauss curve
C_PULL = 0.8
A = 1  # height of the gauss curve

num_push = - (x_plt - B_PUSH) ** 2
denum_push = 2 * C_PUSH ** 2
y_push = A * np.exp(num_push / denum_push)
y_push[y_push <= 0.15] = 0
y_push[x_plt > 1] = 0
push_plot = plt.plot(x_plt, y_push, label="Push")

num_pull = - (x_plt - B_PULL) ** 2
denum_pull = 2 * C_PULL ** 2
y_pull = A * np.exp(num_pull / denum_pull)
y_pull[y_pull <= 0.15] = 0
pull_plot = plt.plot(x_plt, y_pull, label="Pull")

plt.xlabel("Distance")
plt.ylabel("Magnitude")
plt.ylim(ymin=0, ymax=1)
plt.xlim(xmin=0, xmax=3)

plt.grid()
plt.legend()

plt.show()
