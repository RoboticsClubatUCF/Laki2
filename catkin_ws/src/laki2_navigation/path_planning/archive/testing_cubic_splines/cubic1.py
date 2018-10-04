from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt

def f(x):
    x_points = [1, 0, -1, 0, 1]

    y_points = [0, 1, 0, -1, 0]

    t_points = [0, 0.1, 0.2, 0.8, 1]
    #np.linspace(0, 1, len(x_points))

    #print t_points

    tck_x = interpolate.splrep(t_points, x_points, s = 0)
    tck_y = interpolate.splrep(t_points, y_points, s = 0)

    superT_points = np.linspace(0, 1, 100000)

    plt.plot(x_points, y_points, 'o', interpolate.splev(superT_points, tck_x, der=0), interpolate.splev(superT_points, tck_y, der=0), 'r-')

    return 1



print f(0.5)
plt.show()