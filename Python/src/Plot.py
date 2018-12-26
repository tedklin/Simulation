import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('output/pd.csv', delimiter=',', skip_header=1, names=['time', 'position', 'velocity', 'voltage'])
plt.plot(data['time'], data['position'])
plt.show()
