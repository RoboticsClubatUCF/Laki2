import numpy as np

# Takes omega in rad/s
# r in inches
# pitch in inches / Rev
def numericalTorque(omega, r, pitch):
    r *= 0.0254

    rSpace = np.linspace(0.05*r, r, 10000)
    rSpace = rSpace.tolist()

    exitVelocity = omega * pitch * 0.0254 / (2 * np.pi)
    torqueTotal = 0

    for i in range(len(rSpace) - 1):       
        # Calculate mass flow thru the annulus
        annulusArea = np.pi * (rSpace[i+1]**2 - rSpace[i]**2)

        # Vp = 0.5(exitVelocity + vehicleVelocity)
        # m_dot = rho * area * Vp
        m_dot = 0.5 * 1.225 * exitVelocity * annulusArea

        # Assume the entire annulus is at the same radius
        rAvg = (rSpace[i] + rSpace[i+1])/2

        # Assume the entire annulus moves at the same speed
        annulusDeltaV = (omega * rAvg) - exitVelocity/np.sqrt(5)

        # Specific energy lost: 0.5 * exitVelocity**2
        # Specific energy of flow: 0.5 * (omega * r)**2
        # Energy Before Blade:
        #   0.5 * (omega * r)**2
        # Energy After Blade:
        #   E = (0.5 * (omega * r)**2 - 0.5 * exitVelocity**2)
        # Energy After Blade:
        #   E = 0.5 * (V_aft ** 2)
        # Velocity After Blade:
        #   sqrt((omega*r)**2 - exitVelocity**2)
        # DeltaV = (omega*r) - sqrt((omega*r)**2 - exitVelocity**2)

        annulusDeltaV = (omega*rAvg) if (((omega*rAvg)**2 - exitVelocity**2) < 0) else (omega*rAvg) - np.sqrt((omega*rAvg)**2 - exitVelocity**2)

        # Calculate the torque due to that annulus
        annulusForce = m_dot * annulusDeltaV
        annulusTorque = annulusForce * rAvg

        # Sum the total torque
        torqueTotal += annulusTorque

    return torqueTotal

def thrust(motorRPM, propDiameter, propPitch, speed, alpha, isStacked):
    # Assume that two motors on the same arm each operate at n% efficiency compared 
    #   to that of a flat frame
    stackedCoeff = 0

    # exitVelocity is the speed in m/s of the air after is has gone thru the prop
    # This is based on the definition of propeller pitch
    # Multiplied by constants for unit conversions
    # Assume 4% propeller slip
    exitVelocity = 0.96 * motorRPM * propPitch * 0.0254 / 60
    
    # Assume slip velocity is 20% of maximum velocity
    # TO DO: Find a more accurate way to calculate propeller slip
    exitVelocity *= 0.8

    if isStacked:
        exitVelocity *= stackedCoeff
    
    # diskArea is the area of the circle created by the spinning propeller
    # Multiplied by constant for unit conversion
    diskArea = np.pi * (0.0254 * propDiameter / 2) ** 2
    
    # mass flow rate = density * area * propeller velocity
    massFlowRate = (1.225 * diskArea) * (0.5 * 
            (exitVelocity + np.sin(alpha) * speed))
          
    thrustX = massFlowRate * (exitVelocity * np.sin(alpha) - speed)
    thrustY = massFlowRate * exitVelocity * np.cos(alpha)

    return (thrustX, thrustY, exitVelocity)


omega = 3299*2*np.pi/60
print "omega: ", omega, "rad/s"
torqueTotal = numericalTorque(omega, 9, 4.7)

# Power:
power = torqueTotal * omega

print "current: ", power/21.18, "A"

# Velocity Constant
Kv = 320 * 2  * np.pi / 60

# Torque Constant
Kt = 1 / Kv

print "torque: ", torqueTotal, "N*m"
print "current: ", torqueTotal / Kt / 0.80, "A"

#print "thrust: ", thrust(5125, 18, 4.7, 0, 0, 0)
