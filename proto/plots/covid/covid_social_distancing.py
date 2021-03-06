import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from utils import AGENT_TYPE

aggData = pd.DataFrame()

NUMBER_OF_AGENTS = 100

infected_chance_to_infect_conform = [
    (5, 5, 20), (5, 5, 60), (5, 5, 80),
    (25, 1, 20), (25, 1, 60), (25, 1, 80),
    (50, 5, 20), (50, 5, 60), (50, 5, 80),
    (50, 10, 20), (50, 10, 60), (50, 10, 80)
]

for rkc in infected_chance_to_infect_conform:

    aggData = pd.DataFrame()
    INFECTED = rkc[1]
    PERCENTAGE = rkc[1]
    CHANCE_TO_INFECT = rkc[0]
    CONFORM_TO_POLICY = rkc[2]
    FOLDER_ROOT = '../../../experiments/results/covid/social_distancing/'
    EXPERIMENT = f"r_{CHANCE_TO_INFECT}_k_{PERCENTAGE}_c_{CONFORM_TO_POLICY}"

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

    plt.title(f"Isolation: {NUMBER_OF_AGENTS} agents, {INFECTED} infected, {CHANCE_TO_INFECT}% chance of infection, \n{CONFORM_TO_POLICY}% conforms to social distancing")
    plt.plot(epochs, infected, c='red', label='Infectious')
    plt.plot(epochs, susceptible, c='cyan', label='Susceptible')
    plt.plot(epochs, deceased, c='grey', label='Deceased')
    plt.plot(epochs, recovered, c='green', label='Recovered')
    plt.xlabel("Epochs")
    plt.ylabel("Number of agents")
    plt.legend()
    plt.grid()
    plt.savefig(f"png/social_distancing/covid_social_distance_{EXPERIMENT}.png", transparent=False, dpi=300, bbox_inches="tight")
    plt.show()