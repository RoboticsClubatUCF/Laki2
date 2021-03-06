import os
import csv
import operator
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

csvf = open('thrust_poly_data.csv', 'r')
rf = csv.reader(csvf)

# key is brand, value is prop dictionary
propData = dict()

# key is diameter, pitch, value is (J, Cp)
prop = dict()

fig = plt.figure()
ax = Axes3D(fig)

for element in rf:
    if (propData.has_key(element[0])):
        prop = propData[element[0]]
    else:
        prop = dict()
    
    if (prop.has_key((element[1], element[2]))):
        data = prop[(element[1], element[2])]
    else:
        data = []

    data.append((element[3], element[4], element[5]))

    prop[(element[1], element[2])] = data
    propData[element[0]] = prop

for key1 in propData:
    print key1

    fig = plt.figure()
    ax = Axes3D(fig)

    prop = propData[key1]
    for key2 in prop:
        data = prop[key2][0]
        data = [float(i) for i in data]

        poly = np.poly1d(data)
        
        t = np.linspace(0, 1, 100)
        y = poly(t)
        p = np.full(len(y), float(key2[1]))
        ax.plot(p, t, y)
            
    ax.set_xlabel('Pitch')
    ax.set_ylabel('Advance')
    ax.set_zlabel('Cp')        
    plt.show()