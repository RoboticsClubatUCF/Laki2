from __future__ import division
import matplotlib.pyplot as plt
import numpy as np
from winchCalc import *
from scipy import stats

maxSpeed = 7
brakeHeight = 7

Kv = 1800
Kv *= 2 * np.pi / 60

hub = 0.45 / 2
hub *= 0.0254

spoolWidth = 0.55 * 0.0254

print effHub(hub, 28.48, spoolWidth)/0.0254

retVals = binaryDrop(6, Kv, hub, spoolWidth, 0.107, 0.01, 1000)

tVals, yVals, yPrimeVals, tensionVals, peakPhaseVolt, peakRes1Volt, peakRes2Volt, peakRes1Pow, peakRes2Pow, peakMotorPow, peakCurrent, spoolRadius = retVals

print "Time: ", tVals[-1]
print "Pos: ", yVals[-1]
print "Speed: ", yPrimeVals[-1]
print "Spool Size: ", spoolRadius[-1]
print "Max res1Pow: ", max(peakRes1Pow)
print "Max res2Pow: ", max(peakRes2Pow)
print "Max voltage: ", max(abs(min(peakPhaseVolt)), abs(max(peakPhaseVolt)))
print "Max current: ", max(abs(min(peakCurrent)), abs(max(peakCurrent)))
print "Max Tension: ", max(tensionVals)

f, ((ax1, ax2, ax3, ax4), (ax5, ax6, ax7, ax8)) = plt.subplots(2, 4, sharey=False)

ax1.plot(tVals, yVals)
ax1.set_title("Height")

ax2.plot(tVals, yPrimeVals)
ax2.set_title("Speed")

ax3.plot(tVals, peakCurrent)
ax3.set_title("Current")

ax4.plot(tVals, peakPhaseVolt)
ax4.set_title("peakPhaseVolt")

ax6.plot(tVals, peakRes1Pow)
ax6.set_title("res1Power")

ax5.plot(tVals, peakMotorPow)
ax5.set_title("motorPower")

ax7.plot(tVals, spoolRadius)
ax7.set_title("spoolRadius")

ax8.plot(tVals, tensionVals)
ax8.set_title("tensionVals")

plt.show()

"""
tVals, yVals, yPrimeVals, motorVolt, power, tensionVals = newDrop(brakeHeight, 
        maxSpeed, Kv, hub, spoolWidth) 



f, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, sharey=False)

print "Pos: ", yVals[-1]
print "Speed: ", yPrimeVals[-1]
print "Tension: ", tensionVals[-1]
print "Voltage: ", motorVolt[-1]

ax1.plot(tVals, yVals)
ax1.set_title("Height")

ax2.plot(tVals, yPrimeVals)
ax2.set_title("Speed")

ax3.plot(tVals, tensionVals)
ax3.set_title("Tension")

ax4.plot(tVals, motorVolt)
ax4.set_title("Voltage")

ax5.plot(tVals, power)
ax5.set_title("Power")

plt.show()
"""

"""
# max Elec properties
maxElec = [[4*4.2*0.9, 38.13*0.9, 610*0.9], [1000, 1000, 100*0.8]]

# Motor Kv in RPM / V
Kv = 2600
Kv *= 2 * np.pi / 60

# Motor resistance (armature resistance) in omhs
Ra = 0.056

# Gear Ratio (Motor : Winch)
gear = 2

resistor = 0.2
mosfet = 9/1000
res = resistor + mosfet

# hub, res, t = motorEvaluator(Kv, Ra, gear, maxElec)

#print (hub, res)
"""

"""
totalRes = Ra + res

hub = largestHub(Kv, gear, totalRes, maxElec)

print ("hub", hub)

hub = 0.3 * 0.0254
spoolWidth = 0.1524 / 2

(tVals, yVals, yPrimeVals, resVolt, motorVolt, sysCurr, rmsPower, resPower, motorPower, tensionVals) = drop(5, 10, Kv, Ra, gear, hub, spoolWidth, res)

f, ((ax1, ax2, ax3), (ax4, ax5, ax6), (ax7, ax8, ax9)) = plt.subplots(3, 3, sharey=False)

print ('time: ', tVals[-4])
print ('speed: ', yPrimeVals[-7] - 4.056322225893055)
# print ('max motor voltage: ', max(motorVolt) - 1.793939356362689)
# print ('max res voltage: ', max(resVolt)*resistor/res - 6.40692627272389)
# print ('max mosfet voltage: ', max(resVolt)*mosfet/res - 0.28831168227257503)
print ('max current: ', max(sysCurr) - 32.03463136361945)
# print ('max rms power: ', max(rmsPower) - 135.9738328748963)
print ('max res power: ', max(resPower)*resistor/res - 68.41450710686605)
print ('max mosfet power: ', max(resPower)*mosfet/res - 3.078652819808972)
# print ('max motorPower: ', max(motorPower) - 45.974548775813986)
print ('max tension: ', max(tensionVals))

ax1.set_title('Height')
ax1.plot(tVals, yVals)

ax2.set_title('Speed')
ax2.plot(tVals, yPrimeVals)

ax3.set_title('tension')
ax3.plot(tVals, tensionVals)

ax4.set_title('motorVolt')
ax4.plot(tVals, motorVolt)

ax5.set_title('resVolt')
ax5.plot(tVals, resVolt)

ax6.set_title('sysCurr')
ax6.plot(tVals, sysCurr)

ax7.set_title('rmsPower')
ax7.plot(tVals, rmsPower)

ax8.set_title('resPower')
ax8.plot(tVals, resPower)

ax9.set_title('motorPower')
ax9.plot(tVals, motorPower)

plt.show()

maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, 0.00458410190779233, 0.2+res, maxElec)
print (brakeHeight)

print (speedHeightCalc(7.92))


tVals, yVals, yPrimeVals = analyticalDrop(-1, 1000, Kv, gear, 0.00458410190779233, 0.2+res+Ra, flag = 1)

print (tVals[-1])

plt.plot(tVals, yVals)
plt.show()

externRes = np.linspace(0, 0.35, 1000)
hub = []
xVals = []
power = []
speed = []

for extern in externRes:
    y = largestHub(Kv, gear, Ra+res+extern, maxElec)

    if (y != -1):
        xVals.append(extern)
        hub.append(y)
        speed.append(brakeVals(Kv, Ra, gear, hub[-1], res+extern, maxElec)[0])
        
        if (speed[-1] > 8):
            print (hub[-1])

        power.append(brakeElecProps(Kv, Ra, gear, hub[-1], res+extern, speed[-1])[2][1])

f, (ax1, ax2) = plt.subplots(1, 2, sharey=False)
ax1.plot(xVals, hub)
ax2.plot(xVals, speed)
plt.show()

hub = largestHub(Kv, gear, Ra+res, maxElec)
print ("largest hub", hub)

maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)
print ("maxSpeed, brakeHeight", maxSpeed, brakeHeight)

print ("maxSpeed props", brakeElecProps(Kv, Ra, gear, hub, res, maxSpeed))
print ("landing props", brakeElecProps(Kv, Ra, gear, hub, res, 6))

print (analyticalDrop(brakeHeight, maxSpeed, Kv, gear, hub, Ra + res))

hub = minimizeTime(Kv, Ra, gear, res, maxElec)
print ("\noptimal hub: ", hub)

maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)
print ("maxSpeed, brakeHeight", maxSpeed, brakeHeight)

print ("maxSpeed props", brakeElecProps(Kv, Ra, gear, hub, res, maxSpeed))
print ("landing props", brakeElecProps(Kv, Ra, gear, hub, res, 6))

print (analyticalDrop(brakeHeight, maxSpeed, Kv, gear, hub, Ra + res))


# print (motorEvaluator(Kv, Ra, gear, res, maxElec))
"""

"""
hub = 0.716/2 * 0.0254
res = 0.005

totalRes = res + Ra
maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)

#print (brakeElecProps(Kv, Ra, gear, hub, res, maxSpeed))


#print (maxSpeed, brakeHeight)
#print (analyticalDrop(brakeHeight, maxSpeed, Kv, gear, hub, totalRes, 'landingSpeed'))

tVals, yVals, yPrimeVals = analyticalDrop(brakeHeight, maxSpeed, Kv, gear, hub, totalRes, 1)
print (tVals[-1])

f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharey=False)
ax1.plot(tVals, yVals)
ax1.set_title('Analytical Height vs Time')

ax2.plot(tVals, yPrimeVals)
ax2.set_title('Analytical Speed vs Time')


(tVals, yVals, yPrimeVals, tensionVals) = drop(brakeHeight, maxSpeed, Kv, Ra, gear, hub, res)
print (tVals[-1])

ax3.plot(tVals, yVals)
ax3.set_title('Numerical Height vs Time')

ax4.plot(tVals, yPrimeVals)
ax4.set_title('Numerical Speed vs Time')

plt.show()
"""

"""
print ('res: ', res)

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

"""
hub = minimizeTime(Kv, Ra, gear, maxElec)
res = resCalc(Kv, Ra, gear, hub, maxElec)
maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)

(tVals, yVals, yPrimeVals, tensionVals) = drop(brakeHeight, maxSpeed, Kv, Ra, gear, hub, res)

print (np.round(tVals[-1], 3))



print ("hub: ", hub, "res: ", res)
"""