import numpy as np


interactions = np.genfromtxt('csv_dat/interactions.txt', delimiter=',')

print(interactions.max())
print(interactions.mean())