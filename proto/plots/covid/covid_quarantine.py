import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from utils import AGENT_TYPE

NUMBER_OF_AGENTS = 100

infected_chance_to_infect = [
    (5, 1), (5, 5), (5, 10),
    (25, 1), (25, 5), (25, 10),
    (50, 1), (50, 5), (50, 10)
]

for rk in infected_chance_to_infect:
    aggData = pd.DataFrame()
    INFECTED = rk[1]
    PERCENTAGE = rk[1]
    CHANCE_TO_INFECT = rk[0]
    FOLDER_ROOT = '../../../experiments/results/covid/quarantine/'
    EXPERIMENT = f"r_{CHANCE_TO_INFECT}_k_{PERCENTAGE}"

    PATH = FOLDER_ROOT + EXPERIMENT + '/logs/'

    for i in range(0, NUMBER_OF_AGENTS):
        data = pd.read_csv(PATH + str(i) + '.csv')
        data = data[data.columns[4:5]] # only take the column containing infection info
        aggData = pd.concat((aggData, data),axis=1, ignore_index=True)

    n = aggData.shape[0]
    epochs = np.arange(n)
    infected = np.zeros(n)
    susceptible = np.zeros(n)
    deceased = np.zeros(n)
    recovered = np.zeros(n)

    i = 0
    for index, row in aggData.iterrows():

        for d in row:
            if d == AGENT_TYPE.INFECTIOUS:
                infected[i] = infected[i] + 1
            if d == AGENT_TYPE.SUSCEPTIBLE:
                susceptible[i] = susceptible[i] + 1
            if d == AGENT_TYPE.DECEASED:
                deceased[i] = deceased[i] + 1
            if d == AGENT_TYPE.RECOVERED:
                recovered[i] = recovered[i] + 1

        i = i + 1

    plt.title(f"Quarantine: {NUMBER_OF_AGENTS} agents, {INFECTED} infected, {CHANCE_TO_INFECT}% chance of infection")
    plt.plot(epochs, infected, c='red', label='Infectious')
    plt.plot(epochs, susceptible, c='cyan', label='Susceptible')
    plt.plot(epochs, deceased, c='grey', label='Deceased')
    plt.plot(epochs, recovered, c='green', label='Recovered')
    plt.legend()
    plt.grid()
    plt.savefig(f"png/quarantine/covid_quarantine_{EXPERIMENT}.png", transparent=False, dpi=300)
    plt.show()