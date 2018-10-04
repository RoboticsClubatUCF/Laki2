from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt

def f(x):
    t = np.arange(0, 1.1, .1)

    x = np.sin(2*np.pi*t)
    y = np.cos(2*np.pi*t)

    tck, u = interpolate.splprep([x, y], s = 0)
    unew = np.arange(0, 1.01, 0.01)
    out = interpolate.splev(unew, tck)


    plt.figure()
    plt.plot(x, y, 'x', out[0], out[1], np.sin(2*np.pi*unew), np.cos(2*np.pi*unew), x, y, 'b')
    return 1



print f(0.5)
plt.show()