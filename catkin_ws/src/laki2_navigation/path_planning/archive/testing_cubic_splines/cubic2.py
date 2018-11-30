from scipy.interpolate import splprep, splev, interp1d
import numpy as np
import matplotlib.pyplot as plt

pts = np.array([[1,0], [0,1], [-1, 0], [0, -1], [0, -1]])

tck, u = splprep(pts.T, u = None, s = 0.0, per=1)
u_new = np.linspace(u.min(), u.max(), 1000)
x_new, y_new = splev(u_new, tck, der=0)

plt.plot(pts[:, 0], pts[:, 1], 'ro')
plt.plot(x_new, y_new, 'b--')
plt.show()