import matplotlib.pyplot as plt
import numpy as np

theta = np.linspace(0,2*np.pi, 100)
r = 3
x = r*np.cos(theta)
y = r*np.sin(theta)
plt.plot(x,y)
plt.show()