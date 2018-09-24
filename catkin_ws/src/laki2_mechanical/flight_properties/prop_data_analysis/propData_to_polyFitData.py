import os
import csv
import operator
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

csvf = open('propData.csv', 'r')
rf = csv.reader(csvf)

csvR = open('polyData.csv', 'w')
wf = csv.writer(csvR)

propData = dict()
data = []

# Populate all data into a hashSet
for element in rf:
    hashVal = (element[0], element[1], element[2])

    if (propData.has_key(hashVal)):
        data = propData[hashVal]
    else:
        data = []

    data.append(element[3:8])
    propData[hashVal] = data

for key in propData:
    data = propData[key]
    xData = []
    yData = []

    for datum in data:
        xData.append(float(datum[1]))
        yData.append(float(datum[2]))

    xData = np.array(xData)
    yData = np.array(yData)

    pOpt = np.polyfit(xData, yData, 2)
    
    data = [key[0], key[1], key[2], pOpt[0], pOpt[1], pOpt[2]]

    wf.writerow(data)