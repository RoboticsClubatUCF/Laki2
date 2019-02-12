from gps_converter import *
import numpy as np
import matplotlib.pyplot as plt

latOrig = 38.1404633
lonOrig = 76.4212051

gs_lat = 38.14627
gs_lon = 76.42816

latPos, lonPos = (38.14586, 76.42639)
x, y = gps_to_xy(latOrig, lonOrig, latPos, lonPos)
plt.plot(x, y, 'x', label = 'data', color = (0,0,0,1))


latPos, lonPos = (38.14617, 76.42642)
x, y = gps_to_xy(latOrig, lonOrig, latPos, lonPos)
plt.plot(x, y, 'o', label = 'data', color = (0,0,0,1))


UGV_boundaries = [(38.14617, 76.42667),
                (38.14636, 76.42617),
                (38.14558, 76.42608),
                (38.14542, 76.42661)]

x_orig, y_orig = 525, 860
plt.plot(x_orig, y_orig, 'x', label = 'data', color = (0,0,0,1))

poly = [(609.9915651830946, 644.454456932276),
            (655.4769561099155, 1238.9138134970472),
            (899.4365305847842, 1268.1819761471047),
            (1240.810387266854, 1124.454562201312),
            (976.1887502964521, 788.4094397923109),
            (1029.310174576658, 466.5050901843899),
            (1188.824911231627, 309.8511306983688),
            (1002.0957697243854, 0.0036295812471155995),
            (421.55939798675325, 28.420887104681732),
            (0.03993415704199533, 366.0542881303085),
            (175.83977103766549, 764.1079686968526),
            (477.5319123645307, 629.0467535497892)]

# Plot the competition boundary
xVals = []
yVals = []

for pt in poly:
    x, y = pt

    xVals.append(x)
    yVals.append(y)

x, y = poly[0]
xVals.append(x)
yVals.append(y)
plt.plot(xVals, yVals)

xVals = []
yVals = []

for pos in UGV_boundaries:
    latPos, lonPos = pos
    x, y = gps_to_xy(latOrig, lonOrig, latPos, lonPos)

    xVals.append(x)
    yVals.append(y)

latPos, lonPos = UGV_boundaries[0]
x, y = gps_to_xy(latOrig, lonOrig, latPos, lonPos)

xVals.append(x)
yVals.append(y)
plt.plot(xVals, yVals)

dist = -np.inf
for pos in UGV_boundaries:
    latPos, lonPos = pos
    x, y = gps_to_xy(latOrig, lonOrig, latPos, lonPos)

    print x, y

    new_dist = np.sqrt((x - x_orig)**2 + (y-y_orig)**2)
    if (new_dist > dist):
        dist = new_dist

print dist

plt.show()
