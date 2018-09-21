from __future__ import division
import math
import copy
import numpy as np
import scipy
from scipy.optimize import minimize

# Global constants
# Assumed drag to be 1.6 for all configurations
dragCoef = 1.6
# Assume air density to be that of air at sea level
airDensity = 1.225

# MotorSpec:
#   0 - resistance
#   1 - Kv
#   2 - Kt
#   3 - no load power
#   4 - max current
#   5 - weight (g)

# PropSpec
#   0 - diameter
#   1 - pitch
#   2 - Pconst

#----------------------------------------------------------------------------------#

# Lookup table for lateral cross sectional area of requested drone in m^2
# Estimate values based on quick frame designs
# Returns (topArea, frontArea)
def lookupLateralArea(numArms):
    if (numArms == 3):
        topArea, frontArea = (0.037739937423, 0.02793626671)

    elif (numArms == 4):
        topArea, frontArea = (0.0323465160, 0.006939136)

    elif (numArms == 6):
        topArea, frontArea = (0.0648640896264, 0.0163591208301)

    elif (numArms == 8):
        topArea, frontArea = (0.0694350353232, 0.0153517009094)

    else:
        # Return a ridiculous number to denote an error
        return (1e99, 1e99)

    # Estimate for lateral area of payload
    frontArea += 0.01

    return (topArea, frontArea)

#----------------------------------------------------------------------------------#

# Returns the air drag on a vehicle with the given properties in N
# Inputs:
#   speed in m/s
#   alpha in rad
#   numArms is an integer for the number of arms on the vehicle
def airDrag(speed, alpha, numArms):
    # Populate lateral areas of vehicle
    topArea, frontArea = lookupLateralArea(numArms)
    
    # Calculate lateral areas of the vehicle to oncoming air
    areaEff = topArea * np.sin(alpha) + frontArea * np.cos(alpha)

    # Drag Equation
    airDrag = 0.5 * dragCoef * airDensity * speed**2 * areaEff

    return airDrag

#----------------------------------------------------------------------------------#

# Returns the x and y components thrust of specified motor/prop combination in N
# Returns the exit velocity of the air in m/s

# This assumes that the exit velocity over the propeller is constant.
# Break this assumption using WB Garner Model Airplane Propellers write up

# NOTES: 
#   Will return a negative number if the speed is creater than the exit velocity 
#       of the propeller. This is not an error
#   Assumes vehicleSpeed is only in horizontal direction. 
#   Thrust does not necessarily act perpendicular to frame
#   Will return a value for a negative vehicleSpeed

# Inputs:
#   propDiameter and prop_pitch in inches
#   vehicleSpeed in m/s
#   alpha is the pitch angle of the vehicle in rad
#   isStacked is boolean, 0 for flat frame, 1 for stacked

# TO DO: Validate this with empirical data

def thrust(motorRPM, propSpec, speed, alpha, isStacked):
    # Propeller Information
    propDiameter, propPitch = propSpec[0:2]

    # Assume that two motors on the same arm each operate at n% efficiency compared 
    #   to that of a flat frame
    stackedCoeff = 0

    # exitVelocity is the speed in m/s of the air after is has gone thru the prop
    # This is based on the definition of propeller pitch
    # Multiplied by constants for unit conversions
    exitVelocity = np.sqrt(2) * motorRPM * propPitch * 0.0254 / 60

    if isStacked:
        exitVelocity *= stackedCoeff
    
    # diskArea is the area of the circle created by the spinning propeller
    # Multiplied by constant for unit conversion
    diskArea = np.pi * (0.0254 * propDiameter / 2) ** 2
    
    propVelocity = 0.5 * (exitVelocity + np.sin(alpha) * speed)

    # Prop Slip
    propVelocity *= 0.95

    # mass flow rate = density * area * propeller velocity
    massFlowRate = (airDensity * diskArea) * propVelocity
          
    thrustX = massFlowRate * (exitVelocity * np.sin(alpha) - speed)
    thrustY = massFlowRate * exitVelocity * np.cos(alpha)

    return (thrustX, thrustY, exitVelocity, propVelocity)

#----------------------------------------------------------------------------------#

# Calculates the max speed of the vehicle in m/s
# Returns all -1's is vehicle cannot sustain lift
# Returns all -1's for inputs that don't not converge after 1000 iterations

# Calculates the steady state, steady level flight, speed and pitch angle of the 
#   vehicle with the given given properties, with air resistance

# This analysis assumes that at max speed all motors are operating at the same RPM. 

# Inputs:
#   weight is the weight of the vehicle in N 
#   topArea and frontArea are the lateral areas of the vehicle from the top and 
#       front respectively, in m^2
#   numMotors is an integer representing the number of motors 
#   motorRPM and RPM of all motors
#   propDiameter and propPitch are diameter and pitch of propeller respectively, 
#       both in inches
#   isStacked is 0 for flat designs and 1 otherwise

# Returns: (speed, alpha)
#   speed is the final speed of the vehicle in m/s
#   alpha is the pitch angle of the vehicle in rad

def speed(weight, numMotors, numArms, motorRPM, propSpec):
    # A boolean representing if the vehicle is of a stacked configuration
    isStacked = not (numMotors == numArms)

    propDiameter, propPitch = propSpec[0:2]

    # Calculate the thrust at rest and no pitch to determine if the drone can hover
    thrustX, thrustY, exitVelocity = thrust(motorRPM, propSpec, 0, 0, isStacked)[0:3]

    # Find the total thrust of the vehicle to hover
    totalThrust = thrustY * numMotors

    # If the totalThrust is less than the weight, it cannot hover
    if(totalThrust < weight):
        return (-1, -1)

    # Function to be minimized
    def speedEvaluator(x):
        speed, alpha = x
        # Find thrust at this speed
        thrustX, thrustY = thrust(motorRPM, propSpec, speed, alpha, isStacked)[0:2]

        # Forces in the y:
        verticalThrust = thrustY * numMotors

        # Assume some thrust to weight ratio that increases linear with speed
        # Assumed because controlling vehicle requires ratio greater than 1.0
        #   Assumed 1.4 at hover and an additional 1.0 for every 10 m/s of speed
        thrustToWeight = (1.4 + 1.0 * speed / 10)

        # netVerticalThrust is (weight - verticalThrust)
        #   increased, 'artifical',  weight is necessary for control of vehicle
        netVerticalThrust =  thrustToWeight * weight - verticalThrust
    
        # Forces in the x:
        horizontalThrust = thrustX * numMotors
        drag = airDrag(speed, alpha, numArms)
        
        netHorizontalThrust = horizontalThrust - drag

        return np.linalg.norm([netVerticalThrust, netHorizontalThrust])

    # Initial guesses
    speed0 = 0.5 * exitVelocity
    alpha0 = np.pi / 4

    # Physical constraints on the system:
    #   0 < speed
    #   0 < alpha < pi / 2    
    bnds = ((0, None), (0, np.pi/2),)

    speed, alpha = minimize(speedEvaluator, [speed0, alpha0], method = 'SLSQP', 
            bounds = bnds, tol = 1e-8).x

    return (speed, alpha)

#----------------------------------------------------------------------------------#

# Returns the weight of specified vehicle in N

# numMotors and numArms are the number of motors and arms respectively
# numBatteries and numBatteryCells are the number of batteries and cells in those 
#   batteries respectively
# propDiameter is the diameter of each prop in inches
# isPayloadAttached is a boolean, 1 when payload is attached, 0 if not attached

# This estimate assumes that the weight of the batteries scales linearly with the 
#   number of battery cells and the number of batteries. This is reasonable, and 
#   allows the analysis to test different voltages and capacities

# All constants for item weights are in grams. Converted to N in last step

def weight(numMotors, numArms, motorSpec, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached):   
    # Values obtained from gram scale at robotics
    esc = 10
    jetson = 144.0
    gps = 18.0
    ardupilot = 28.1
    remoteReceiver = 6.0
    powerDistribution = 39.0
    
    # Estimated Values
    payloadDropMechanism = 226.8
    usbHub = 30.34
    electronicsHardware = 68.0
    radioTransceiver = 23.0
    landingLegs = 200.0
    cameras = 3 * 19.5
    oneTB_SSD_Harddrive = 81.65
    
    # Weights of various frames. Assumes frames weight approximately the same and 
    #   only the arms increase with size
    # Does not include weight of clamps for arms, that is included in weight of arm
    if (numArms == 3):
        centerFrame = 288.5
    
    elif (numArms == 4):
        centerFrame = 264.4

    elif (numArms == 6):
        centerFrame = 250.2
    
    elif (numArms == 8):
        centerFrame = 248.2
        
    else:
        # If the frame is not that of a known desing, return an absurd number
        centerFrame = 1e99
    
    # Center of Frame, includes all electronics
    torso = (jetson + gps + ardupilot + remoteReceiver + powerDistribution + 
            payloadDropMechanism + usbHub + electronicsHardware + radioTransceiver + 
            landingLegs + cameras + oneTB_SSD_Harddrive + centerFrame)
    
    # Payload Weight
    if isPayloadAttached:
        waterBottle = 226.796
        UGV = 907.185
        extraPayload = 453.592
    else:
        waterBottle = 0
        UGV = 0
        extraPayload = 0

    # Total payload weight
    payload = waterBottle + UGV + extraPayload
    
    # Property specific to each motor
    motor = motorSpec[5]

    # Estimate weight of 25mm carbon tube based on creo model
    armPerIn = 17.004 / 16 * 25 / 8.66142

    # Estimate from Creo Model (includes weight of clamps on both sides of arm
    genericArm = 53.4 + 16
    armWeight = genericArm + (8 + armPerIn * ((propDiameter / 2) - 0.75))

    # Estimate Battery Weights
    # Based off ncr batteries (ncr more energy dense that lipo)
    batteryPerCellPerMilliAmpHr = 0.0153
    batteryWeight = numBatteryCells * batteryCapacity * batteryPerCellPerMilliAmpHr

    # XOAR 18in prop weighs 31g
    propWeight = 31/18 * propDiameter

    # totalWeight in lb
    totalWeight = armWeight * numArms 
    totalWeight += (esc + motor) * numMotors 
    totalWeight += torso + payload + batteryWeight
    totalWeiht += propWeight * numMotors
    
    # Convert from g to N
    totalWeight *= 9.81 / 1000

    return totalWeight

#----------------------------------------------------------------------------------#

# Calculates the Required Number of Batteries and the Time of Flight

# Returns: (batteryCapacity, currents, speeds, canHover, isFeasible)
#   batteryCapacity is the required battery capacity to finish the competition
#   speeds, and currents are 2 x 1 vectors with the pre and post speeds and currents
#   canHover is a 2 x 1 bool vector denoting if it is possible to hover pre and post 
#   isFeasible is a boolean for whether or not this drone can finish the total dist

# Inputs:
#   voltages is a list with the pre and post drop voltage (in that order)
#   motorSpec contains the relevant data of the motor. See start of script for info
#   numMotors, numArms, numBatteryCells, are the number of motors, number of arms, 
#       and number of battery cells respectively
#   propDiameter, and propPitch are the propeller diameter and pitch respectively, 
#       in inches
#   dist is a list with the pre and post drop distances needed to travel by the 
#       vehicle (in that order), in meters

def batteryCapacity(motorRPMs, motorSpec, numMotors, numArms, numBatteryCells, propSpec, dists):
    # isStacked is a constant based off the given number of motors and arms
    isStacked = not (numMotors == numArms)
    
    # Find the battery voltage in order to know the total energy in the battery
    batteryVoltage = 3.7 * numBatteryCells

    # Propeller Information
    propDiameter, propPitch = propSpec[0:2]

    # Initialize values
    minCapacity = 0
    # Ridiculous number of batteries as an upper limit to the optomization
    # Set max at 100 Ah capacity
    maxCapacity = 100000
    
    convError = 1
    absError = 1
    remainingEnergy = 1
    capacity = 0
    
    currents = [-1, -1]
    speeds = [0, 0]
    alphas = [-1, -1]
    canHovers = [-1, -1]
    powers = [-1, -1]
    currents = [-1, -1]
    iter = 0
    
    # Uses a bisection method instead of Newton or Gradient descent because the 
    #   speed calculation is itself a numerical solver, and thus has no derivatives
    # Cannot use scipy solver of unary requirements conditions
    while (convError > 1e-10 and absError > 1e-8):
        # Usually takes about 50 iterations to converge, if it takes more something 
        #   is wrong
        iter += 1
        print "iter: ", iter
        if (iter > 1e3):
            return (-1, [-1, -1], [-1, -1], [-1, -1])
        
        # Calculate updated values and errors
        capacity_old = capacity
        capacity = (minCapacity + maxCapacity) / 2

        print "capacity: ", capacity

        convError = abs(capacity_old - capacity)
        absError = abs(remainingEnergy)

        # Theoretical Battery Energy
        batteryEnergy = batteryVoltage * (capacity / 1000 * 60 * 60)
        
        # Battery Energy
        #   Assumed landing with 16% of battery energy (SF 1.20)
        #   Assumed 95% depth of discharge
        #   Assumed 90% battery efficiency (actual charge / theoretical charge)
        batteryEnergy *= (1.00 - 0.16) * 0.95 * 0.9

        # Weight is a function of the number of batteries and payload attachment
        weights = [
            weight(numMotors, numArms, motorSpec, capacity, numBatteryCells, 
                    propDiameter, 1),

            weight(numMotors, numArms, motorSpec, capacity, numBatteryCells, 
                    propDiameter, 0)]

        # Find speed and angle at which the drone will fly
        speeds[0], alphas[0] = speed(weights[0], numMotors, numArms, motorRPMs[0], 
                propSpec)

        speeds[1], alphas[1] = speed(weights[1], numMotors, numArms, motorRPMs[1], 
                propSpec)

        print "speeds: ", speeds
        
        # speed() returns -1's if it cannot hover
        # Cannot hover if too heavy, the only way this function can make reduce 
        #   weight is to reduce the number of batteries 
        canHovers = [0, 0]
        
        # Check the pre-drop hovering
        if (speeds[0] == -1):
            maxCapacity = capacity;
            canHovers[0] = 0
            continue
        else:
            canHovers[0] = 1
        
        # Check the post-drop hovering
        if (speeds[1] == -1):
            maxCapacity = capacity;
            canHovers[1] = 0
            continue
        else:
            canHovers[1] = 1

        # Calculate energy required to finish this leg of the competition
        powers[0], currents[0] = powerConsumption(motorRPMs[0], motorSpec, 
                batteryVoltage, numMotors, propSpec, speeds[0], alphas[0], 
                isStacked)

        powers[1], currents[1] = powerConsumption(motorRPMs[1], motorSpec, 
                batteryVoltage, numMotors, propSpec, speeds[1], alphas[1], 
                isStacked)

        timesOfFlight = [dists[0] / speeds[0], dists[1] / speeds[1]]

        energyForFlights = [timesOfFlight[0] * powers[0], 
                            timesOfFlight[1] * powers[1]]

        # Find the total energy used in the competition
        totalEnergy = energyForFlights[0] + energyForFlights[1]

        # Value to optimize to 0
        remainingEnergy = batteryEnergy - totalEnergy

        print "remainingEnergy: ", remainingEnergy 

        # if there is energy remaining, reduce the number of batteries,
        # else increase the number of batteries
        if (remainingEnergy > 0):
            maxCapacity = capacity
        else:
            minCapacity = capacity
    
    # The loop can finish because it converged on a garbage value
    # Check here to see if the result is garbage, and make the last item of the 
    #   return 0 or 1 accordingly
    if (abs(remainingEnergy) > 10.0 or not canHover[0] or not canHover[1]):
        return (capacity, currents, speeds, canHovers, 0)
    else:
        return (capacity, currents, speeds, canHovers, 1)

#----------------------------------------------------------------------------------#    

# Calculates the total power and current consumption per motor of the given vehicle

# Inputs:
# motorRPM and numMotors are the motor RPM and number of motors respectively
# propDiameter and propPitch are the propeller diameter and pitch repectively, both 
#   in inches
# speed is the vehicle speed in m/s
# alpha is the angle of the vehicle in radians
# isStacked is a boolean, 0 for flat frames, 1 else
# voltage is the voltage being supplied to the motors

# returns (totalPower, motorCurrent)

def powerConsumption(motorRPM, motorSpec, voltage, numMotors, propSpec, speed, alpha, isStacked):
    
    # Propeller Information
    propDiameter, propPitch = propSpec[0:2]

    # Mechanical work done by the prop
    aeroWork = aerodynamicWork(motorRPM, propSpec, speed, alpha, isStacked)

    mechWork = aeroWork[0] * propSpec[2]
    
    # Motor Current is torque / Kt
    # This is the current required with no losses
    motorCurrent = aeroWork[1] / motorSpec[2]

    # Voltage is regulated by the ESC
    motorVoltage = mechWork / motorCurrent

    # Motor resistance is a property specific to each motor
    motorResistance = motorSpec[0]

    # No load power is the power pulled to do no work
    noLoadPower = motorSpec[3]

    # Calculate the motor current after motor losses
    # P_in = P_out + I^2 * R + V_motor * I_0
    # noLoadPower = V_motor * I_0
    # P_in = V_motor * I
    # P_out = propPower
    # Algebraic Manipulation:
    #   0 = I^2 * R - V_motor * I + (propPower + noLoadPower)  
    motorCurrents = np.roots([motorResistance, -motorVoltage, 
            (mechWork + noLoadPower)])

    # Calculate the actual power pulled from the roots: --------------------------#
    # If by some miracle the parabola is perfectly placed on the x-axis, that is the 
    #   current draw
    if (len(motorCurrents) == 1):
        motorCurrent = motorCurrents[0]
    
    # If the solution is complex, the current draw is infinite
    elif (not np.isreal(motorCurrents).any()):
        motorCurrent = float('inf')
    
    # If both currents are valid, the smaller one is valid
    elif (motorCurrents[0] > 0 and motorCurrents[1] > 0):
        motorCurrent = np.min(motorCurrents)
        
    # If both currents are negative, something is wrong
    elif (motorCurrents[0] < 0 and motorCurrents[1] < 0):
        return (float('inf'), float('inf'))
      
    # To reach this point, the roots must be real and exactly one is non-negative
    else:
        motorCurrent = max(motorCurrents)

    # Power = V * I
    motorPower = motorVoltage * motorCurrent

    # Current into the esc    
    escCurrent = motorPower / voltage

    # Resistive losses of the esc
    escLosses = escCurrent**2 * 0.005

    # The total power is the sum of all motors
    totalPower = motorPower * numMotors + escLosses

    # Add liberal estimate power required by all electronics
    totalPower += 50

    return (totalPower, motorCurrent)

#----------------------------------------------------------------------------------#

# Calculates the mechanical work, torque required, and rotational speed in rad/s of 
#   the done/required by propeller

# May have to break constant exit velocity assumption

# TO DO: Include the linear viscous effects of the propeller

# TO DO: Include tip losses of prop

def aerodynamicWork(motorRPM, propSpec, speed, alpha, isStacked):
    # Propeller Information
    propDiameter, propPitch = propSpec[0:2]

    # Calculate thrust of that vehicle configuration at that speed and alpha
    thrustX, thrustY, exitVelocity, propVelocity = thrust(motorRPM, propSpec, speed, 
            alpha, isStacked)

    # Calculate the total thrust from one motor 
    totalThrust = np.linalg.norm([thrustX, thrustY])
 
    # Total work done on air by propeller
    power = totalThrust * propVelocity

    # Rotational speed in rad/s
    omega = motorRPM * 2 * np.pi / 60

    # Assume 80% efficiency
    power /= 0.8

    torque = power / omega

    return (power, torque, omega)

#----------------------------------------------------------------------------------#

# Calculates the max torque of a motor at a given RPM.
# returns torque

def maxMotorTorque(motorRPM, motorSpec, maxVoltage):
    # Relevant Motor Properties
    resistance = motorSpec[0]
    Kt = motorSpec[2]
    ke = Kt

    # Rotational speed of motor in rad/s
    omega = motorRPM * 2 * np.pi / 60

    # BLDC motors follow the same curve as a DC motor, and vary V with PWM signals
    #   T = Kt * V / R - Ke * Kt * omega / R
    torque = (Kt * maxVoltage / resistance) - (Ke * Kt * omega / resistance)

    return torque

#----------------------------------------------------------------------------------#

def optimalMotorRPM(weight, numMotors, numArms, motorSpec, propSpec, maxVoltage):
    (maxMotorTorue > aerodynamicWork[1])
    maximize (speed[0] / powerConsumption[0])
    powerConsumption[1] < motorSpec[4]

#----------------------------------------------------------------------------------#

def main():
    voltage = 21.14

    # Motor Data
    Kv = 320 * 2 * np.pi / 60
    Kt = 1/Kv
    motorSpec = [0.116, Kv, Kt, 7, 50, 165]    
    
    numMotors = 8
    isStacked = 0
    numArms = 8
    numBatteryCells = 6

    # Propeller Data
    propDiameter = 18
    propPitch = 5.5
    propConst = 1.11
    propSpec = [propDiameter, propPitch, propConst]

    dists = [6437.38, 4828.03]

    motorRPM = 5084

    heavy = weight(8, 8, motorSpec, 0, 6, 18, 1)
    print "weight: ", heavy

    print "thrust: ", thrust(motorRPM, propSpec, 0, 0, isStacked)

    speedster, alpha = speed(heavy, numMotors, numArms, motorRPM, propSpec)
    print "(", speedster, ", ", alpha, ")"

    print aerodynamicWork(motorRPM, propSpec, speedster, alpha, isStacked)

    # Thrust, and mechanicalWork need some polishing to be complete
    # Need some better way to estimate thrust and power required of the prop at a 
    #   given RPM


if (__name__ == "__main__"):
    main()