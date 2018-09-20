from __future__ import division
from scipy import interpolate
import numpy as np

poly = [(0, 0), (1, 0), (2, 1), (0, 1)]

x = [poly[0][0], poly[-1][0]]
y = [poly[0][1], poly[-1][1]]

if ((x[0] - x[1]) == 0):
    m = float("inf")
else:
    m = (y[0] - y[1]) / (x[0] - x[1])

# Calculate the (possibly theoretical) y intercept of both line segments
b = y[0] - (m * x[0])

print m, b