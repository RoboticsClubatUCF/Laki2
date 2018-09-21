from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np

t = np.arange(3)
x = np.arange(3)
y = np.sin(x)
csx = CubicSpline(t, x)
csy = CubicSpline(t, y)

print csx.x
print csx.c

print
print csy.x
print csy.c

print
print csx.c[0], csx.c[1], csx.c[2], csx.c[3]

plt.figure(figsize=(6.5, 4))
plt.plot(x, y, 'o', label ='data')

tSpace = np.arange(-0.5, 2.5, 0.1)
plt.plot(csx(tSpace), csy(tSpace), label="S")
plt.show()