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

X_LIM = 10
CUT_OFF = 0.05
A = 1  # height of the gauss curve

B_PUSH = 0  # proximity sensor
C_PUSH = 0.5  # width of the gauss curve

B_PUSH_CAMERA = 0
C_PUSH_CAMERA = 0.6

C_PULL_LIGHT = 3.4
B_PULL_LIGHT = 10

x_plt = np.linspace(0, X_LIM, num=100)

# Proximity sensor
num_push = - (x_plt - B_PUSH) ** 2
denum_push = 2 * C_PUSH ** 2
y_push = A * np.exp(num_push / denum_push)
y_push[y_push <= CUT_OFF] = 0
y_push[x_plt > 1] = 0

# Camera
num_pull = - (x_plt - B_PUSH_CAMERA) ** 2
denum_pull = 2 * C_PUSH_CAMERA ** 2
y_push_camera = A * np.exp(num_pull / denum_pull)
y_push_camera[y_push_camera <= CUT_OFF] = 0

# Light sensor
num_pull = - (x_plt - B_PULL_LIGHT) ** 2
denum_pull = 2 * C_PULL_LIGHT ** 2
y_pull_light = A * np.exp(num_pull / denum_pull)
y_pull_light[y_pull_light <= CUT_OFF] = 0

plt.plot(x_plt, y_push, label="Push: obstacle avoidance")
plt.plot(x_plt, y_push_camera, label="Push: social distancing ")
plt.plot(x_plt, y_pull_light, label="Pull: towards isolation")

plt.xlabel("Distance")
plt.ylabel("Magnitude")
plt.ylim(ymin=0, ymax=1)
plt.xlim(xmin=0, xmax=X_LIM)
plt.title('Push-pull forces for the isolation\nand social distancing agents')

plt.legend(bbox_to_anchor=(1.05, 0), loc="lower right")

plt.grid()

plt.savefig('png/covid_forces_isolation_social_distance.png', transparent=False, dpi=300)
plt.show()

