import matplotlib.pyplot as plt
import numpy as np

rewards = np.loadtxt('../../experiments/logs/rewards.csv', delimiter=',')
N = len(rewards)
epochs = np.linspace(0, N, N)

for r in rewards:
    epochs

plt.plot(epochs, rewards)
plt.savefig('png/leader_rewards.png', transparent=False, dpi=300)
plt.show()

