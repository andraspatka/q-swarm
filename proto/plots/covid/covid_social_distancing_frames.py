import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from utils import AGENT_TYPE

NUMBER_OF_AGENTS = 100

infected_chance_to_infect = [
    (25, 1, 60)
]

for rk in infected_chance_to_infect:
    aggData = pd.DataFrame()
    INFECTED = rk[1]
    PERCENTAGE = rk[1]
    CHANCE_TO_INFECT = rk[0]
    CONFORM_TO_POLICY = rk[2]
    FOLDER_ROOT = '../../../experiments/results/covid/presi/social_distancing/'
    EXPERIMENT = f"r_{CHANCE_TO_INFECT}_k_{PERCENTAGE}_c_{CONFORM_TO_POLICY}"

    PATH = FOLDER_ROOT + EXPERIMENT + '/logs/'

    for a in range(0, NUMBER_OF_AGENTS):
        data = pd.read_csv(PATH + str(a) + '.csv')
        data = data[data.columns[4:5]] # only take the column containing infection info
        aggData = pd.concat((aggData, data),axis=1, ignore_index=True)

    n = aggData.shape[0]
    epochs = np.arange(n)
    infected = np.zeros(n)
    susceptible = np.zeros(n)
    deceased = np.zeros(n)
    recovered = np.zeros(n)

    t = 0
    for index, row in aggData.iterrows():

        for d in row:
            if d == AGENT_TYPE.INFECTIOUS:
                infected[t] = infected[t] + 1
            if d == AGENT_TYPE.SUSCEPTIBLE:
                susceptible[t] = susceptible[t] + 1
            if d == AGENT_TYPE.DECEASED:
                deceased[t] = deceased[t] + 1
            if d == AGENT_TYPE.RECOVERED:
                recovered[t] = recovered[t] + 1

        t = t + 1

    for f in range(0, n):
        plt.figure()
        plt.title(f"Social distancing: {NUMBER_OF_AGENTS} agents, {INFECTED} infected \n{CHANCE_TO_INFECT}% chance of infection, {CONFORM_TO_POLICY}% conforms to social distancing")
        plt.plot(epochs[0:f], infected[0:f], c='red', label='Infectious')
        plt.plot(epochs[0:f], susceptible[0:f], c='cyan', label='Susceptible')
        plt.plot(epochs[0:f], deceased[0:f], c='grey', label='Deceased')
        plt.plot(epochs[0:f], recovered[0:f], c='green', label='Recovered')
        plt.xlabel("Epochs")
        plt.ylabel("Number of agents")
        plt.xlim(0, n)
        plt.ylim(0, 100)
        plt.legend()
        plt.grid()
        frame_num = str(f).zfill(5)
        plt.savefig(f"frames/frame_{frame_num}.png", transparent=False, dpi=300, bbox_inches="tight")
        plt.close()
        # plt.show()