import os
import csv
import operator
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from time import sleep

csvf = open('propData.csv', 'r')
rf = csv.reader(csvf)

# key is brand, value is prop dictionary
propData = dict()

# key is diameter, pitch, value is (J, Ct)
prop = dict()

fig = plt.figure()
ax = Axes3D(fig)

for element in rf:
    if ((float(element[2]) < 4) or (float(element[2]) > 7)):
        continue

    if (propData.has_key(element[0])):
        prop = propData[element[0]]
    else:
        prop = dict()

    if (prop.has_key((element[1], element[2]))):
        data = prop[(element[1], element[2])]
    else:
        data = []

    data.append((element[4], element[5]))

    prop[(element[1], element[2])] = data
    propData[element[0]] = prop

for key1 in propData:
    prop = propData[key1]
    for key2 in prop:
        data = prop[key2]
        
        for datum in data:
            ax.scatter(float(key2[1]), float(datum[0]), float(datum[1]))
            
ax.set_xlabel('Pitch')
ax.set_ylabel('Advance')
ax.set_zlabel('Ct')        
plt.show()