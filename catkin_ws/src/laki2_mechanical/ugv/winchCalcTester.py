import matplotlib.pyplot as plt
import numpy as np
from winchCalc import *

# max Elec properties
maxElec = [4*4.2, 38.13, 610]

# Motor Kv in RPM / V
Kv = 2600
Kv *= 2 * np.pi / 60

# Motor resistance (armature resistance) in omhs
Ra = 0.056

# Gear Ratio (Winch : Motor)
gear = 1

hub = minimizeTime(Kv, Ra, gear, maxElec)

# 0.009524999999999999  47.1595
# 0.014287499999999998  -1
# 0.011906249999999998  -1
# 0.010715625000000000  -1
# 0.010120312499999999  51.4465
# 0.010417968750000000  -1
# 0.010269140625000000  52.68
# 0.010343554687500000  53.7595
# 0.010380761718750000  55.71
# 0.010399365234375000  54.96
# 0.010408666992187500  -1
# 0.010404016113281250  57.04
# 0.010406341552734375  -1
# 0.010405178833007813 56.965
# 0.010405760192871094  -1


# 0.004318000000000000  24.137
# 0.006477000000000000  36.186
# 0.007556500000000000  40.045
# 0.008096250000000000  41.877
# 0.008366124999999999  
# 0.008501062500000000  
# 0.008568531250000000  
# 0.008602265625000001  
# 0.008619132812500000  
# 0.008627566406249999  43.700

res = resCalc(Kv, Ra, gear, hub, maxElec)

print ('res: ', res)

maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)
speedHeight = speedHeightCalc(maxSpeed)

print (speedHeight, brakeHeight)

print (speedHeight - brakeHeight)

(tVals, yVals, yPrimeVals, tensionVals) = drop(brakeHeight, maxSpeed, Kv, Ra, gear, hub, res)

f, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey=False)
ax1.plot(tVals, yVals)
ax1.set_title('Height vs Time')

ax2.plot(tVals, yPrimeVals)
ax2.set_title('Speed vs Time')

ax3.plot(tVals, tensionVals)
ax3.set_title('Tension vs Time')

plt.show()

"""
hub = minimizeTime(Kv, Ra, gear, maxElec)
res = resCalc(Kv, Ra, gear, hub, maxElec)
maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)

(tVals, yVals, yPrimeVals, tensionVals) = drop(brakeHeight, maxSpeed, Kv, Ra, gear, hub, res)

print (np.round(tVals[-1], 3))



print ("hub: ", hub, "res: ", res)
"""