from scipy.interpolate import splprep, splev, interp1d
import numpy as np
import matplotlib.pyplot as plt

pts = np.array([[1,0], [0,1], [-1, 0], [0, -1], [1,0]])

x, y = pts.T
i = np.arange(len(pts))

interp_i = np.linspace(0, i.max(), 10000*i.max())

xi = interp1d(i, x, kind = 'cubic')(interp_i)
yi = interp1d(i, y, kind = 'cubic')(interp_i)

plt.plot(x, y, 'ko')
plt.plot(xi, yi)
plt.show()