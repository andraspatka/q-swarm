import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 1, num=100)  # measured distance
B_PUSH = 0  # The robot's centre
B_PULL = 1  # The prox sensor's coverage limit
C_PUSH = 0.15  # width of the gauss curve
C_PULL = 0.2
A = 1  # height of the gauss curve

num_push = - (x - B_PUSH) ** 2
denum_push = 2 * C_PUSH ** 2
y_push = A * np.exp(num_push / denum_push)
y_push[y_push < 0.1] = 0
push_plot = plt.plot(x, y_push, label="Push")

num_pull = - (x - B_PULL) ** 2
denum_pull = 2 * C_PULL ** 2
y_pull = A * np.exp(num_pull / denum_pull)
y_pull[y_pull < 0.1] = 0
pull_plot = plt.plot(x, y_pull, label="Pull")

plt.xlabel("Distance")
plt.ylabel("Magnitude")
plt.ylim(ymin=0, ymax=1)
plt.xlim(xmin=0, xmax=1)

plt.grid()
plt.legend()

plt.show()
