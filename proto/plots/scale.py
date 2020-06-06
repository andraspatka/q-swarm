import matplotlib.pyplot as plt
import numpy as np

N = 1000
x = np.linspace(-2.14, 4.14, N)
x2 = np.linspace(-4.14, 2.14, N)

y = 9.55 * x - 9.53

y2 = 9.55 * x2 + 9.57

plt.subplot(211)
plt.plot(x, y)
plt.grid()
plt.subplot(212)
plt.grid()
plt.plot(x2, y2)
plt.show()