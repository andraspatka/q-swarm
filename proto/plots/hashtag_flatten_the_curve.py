import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from utils import AGENT_TYPE

aggData = pd.DataFrame()

for i in range(0, 30):
    data = pd.read_csv('../../measurements/disease/' + str(i) + '.csv')
    data = data[data.columns[4:5]] # only take the column containing infection info
    aggData = pd.concat((aggData, data),axis=1, ignore_index=True)

n = aggData.shape[0]
epochs = np.arange(n)
infected = np.zeros(n)
susceptible = np.zeros(n)
removed = np.zeros(n)

i = 0
for index, row in aggData.iterrows():

    for d in row:
        if d == AGENT_TYPE.INFECTIOUS:
            infected[i] = infected[i] + 1
        if d == AGENT_TYPE.SUSCEPTIBLE:
            susceptible[i] = susceptible[i] + 1
        if d == AGENT_TYPE.REMOVED:
            removed[i] = removed[i] + 1
    i = i + 1

plt.title('30 agents, 3 infected, 10% chance of infection')
plt.plot(epochs, infected, c='red', label='Infectious')
plt.plot(epochs, susceptible, c='cyan', label='Susceptible')
plt.plot(epochs, removed, c='grey', label='Removed')
plt.legend()
plt.savefig('infect_random_walk.png', transparent=False, dpi=300)
plt.show()