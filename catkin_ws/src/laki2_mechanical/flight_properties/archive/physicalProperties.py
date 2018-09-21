from __future__ import division
import math
import copy
import numpy as np


# Global constants
# Assumed drag to be 0.9 for all configurations
dragCoef = 0.9
# Assume air density to be that of air at sea level
airDensity = 1.225

# MotorSpec:
#   0 - resistance
#   1 - Kv
#   2 - no load power
#   3 - max current
#   4 - weight

#----------------------------------------------------------------------------------#

# Lookup table for lateral cross sectional area of requested drone in m^2
# Estimate values based on quick frame designs
# To Do: have lateral area take propeller diameter as an input and scale the drone 
#   based on some parametric models, with a minimum size of that listed below
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
    massFlowRate = (airDensity * diskArea) * (0.5 * 
            (exitVelocity + np.sin(alpha) * speed))
          
    thrustX = massFlowRate * (exitVelocity * np.sin(alpha) - speed)
    thrustY = massFlowRate * exitVelocity * np.cos(alpha)

    return (thrustX, thrustY, exitVelocity)


#----------------------------------------------------------------------------------#

# Calculates the max speed of the vehicle in m/s
# Returns all -1's is vehicle cannot sustain lift
# Returns all -1's for inputs that don't not converge after 1000 iterations

# Calculates the steady state, steady level flight, speed and pitch angle of the 
#   vehicle with the given given properties, with air resistance

# This analysis assumes that at max speed all motors are operating at the same RPM. 

# Uses a Newton's Method solver to find the speed and pitch angle

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

def speed(weight, numMotors, numArms, motorRPM, propDiameter, propPitch):
    # A boolean representing if the vehicle is of a stacked configuration
    isStacked = not (numMotors == numArms)

    # Calculate the thrust at rest and no pitch to determine if the drone can hover
    thrustX, thrustY, exitVelocity = thrust(motorRPM, propDiameter, propPitch, 0, 0, 
            isStacked)

    # Find the total thrust of the vehicle to hover
    totalThrust = thrustY * numMotors

    # If the totalThrust is less than the weight, it cannot hover
    if(totalThrust < weight):
        return (-1, -1)


    # Cannot use scipy optomizer because of special restrictions on variables
    #   Namely:
    #       0 < speed
    #       0 < alpha < pi / 2

    # Initialize values & initial guesses
    iter = 0
    speed = exitVelocity
    alpha = 0.9 * np.pi / 2
    convError = 1
    absError = 1

    # Use Newton's Method to solve for alpha and speed
    while(absError > 1e-8 or convError > 1e-10):
        # Test runs converged within about 9 iterations. 
        # If it takes more than 1000 something is wrong
        iter += 1;        
        if (iter > 1000):
            return (-1, -1)

        # Find thrust at this speed
        thrustX, thrustY = thrust(motorRPM, propDiameter, propPitch, speed, alpha, 
                isStacked)[0:2]

        # Forces in the y:
        verticalThrust = thrustY * numMotors
        netVerticalThrust = weight - verticalThrust
        
        # Forces in the x:
        horizontalThrust = thrustX * numMotors
        drag = airDrag(speed, alpha, numArms)
        
        netHorizontalThrust = horizontalThrust - drag

        # Function Vector
        F = np.array([[netHorizontalThrust], [netVerticalThrust]])

        # Calculate the Jacobian
        speed_fd = speed + 1e-4
        alpha_fd = alpha + 1e-4
       
        # Finite Difference w/ respect to speed
        thrustX_speed, thrustY_speed = thrust(motorRPM, propDiameter, propPitch, 
                speed_fd, alpha, isStacked)[0:2]
        
        verticalThrust_speed = thrustY_speed * numMotors
        horizontalThrust_speed = thrustX_speed * numMotors
        
        netVerticalThrust_speed = weight - verticalThrust_speed
        
        # Since speed has changed air drag also changes
        drag_speed = airDrag(speed_fd, alpha, numArms)
        netHorizontalThrust_speed = horizontalThrust_speed - drag_speed
        
        # Finite difference values w/ respect to speed
        fd_netVerticalThrust_speed = (netVerticalThrust - 
                netVerticalThrust_speed) / (speed - speed_fd)
        fd_netHorizontalThrust_speed = (netHorizontalThrust - 
                netHorizontalThrust_speed) / (speed - speed_fd) 
        
        # Finite Difference w/ respect to alpha
        thrustX_alpha, thrustY_alpha = thrust(motorRPM, propDiameter, propPitch, 
                speed, alpha_fd, isStacked)[0:2]

        verticalThrust_alpha = thrustY_alpha * numMotors
        horizontalThrust_alpha = thrustX_alpha * numMotors
        
        netVerticalThrust_alpha = weight - verticalThrust_alpha
        
        drag_alpha = airDrag(speed, alpha_fd, numArms)
        netHorizontalThrust_alpha = horizontalThrust_alpha - drag_alpha
        
        fd_netVerticalThrust_alpha = (netVerticalThrust - 
                netVerticalThrust_alpha) / (alpha - alpha_fd)
        fd_netHorizontalThrust_alpha = (netHorizontalThrust - 
                netHorizontalThrust_alpha) / (alpha - alpha_fd) 
        
        J = np.array(
            [[fd_netHorizontalThrust_speed, fd_netHorizontalThrust_alpha], 
            [fd_netVerticalThrust_speed, fd_netVerticalThrust_alpha]])

        # Calculate Error
        # Equivalent to X = inv(J) * F
        #   but significantly faster and more accurate
        # Also equivalent to X = J \ F
        X = np.linalg.solve(J.T.dot(J), J.T.dot(F))

        alpha_old = alpha
        speed_old = speed

        # Update Speed and Alpha
        speed -= X[0][0]
        alpha -= X[1][0]
        
        # Restrict speed to be a positive number
        speed = abs(speed)
    
        # Restrict alpha to be between 0 and pi/2
        alpha = abs(alpha % (np.pi/2))

        # Update Error Values
        convError = np.sqrt((alpha_old - alpha) ** 2 + (speed_old - speed) ** 2)
        absError  = np.sqrt((netVerticalThrust) ** 2 + (netHorizontalThrust) ** 2)

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

# All constants for item weights are in lb. Converted to N in last step

# TO DO: fix this so it takes motorSpec as an input and uses the weight specific to 
#   each motor
def weight(numMotors, numArms, batteryCapacity, numBatteryCells, 
        propDiameter, isPayloadAttached):
    
    # Weight of 16mm carbon tube based on creo model
    armPerIn = 0.037486529988/8.66142

    # Estimate from Creo Model (includes weight of clamps on both sides of arm
    genericArm = 0.06986696
    armWeight = genericArm + (armPerIn * ((propDiameter / 2) - 0.75))
    
    # Values obtained from gram scale at robotics
    #motor = 0.1000899
    #odroid = 0.132277
    odroid = 0
    motor = 0
    gps = 0.0396832
    pix = 0.0749572
    remoteReceiver = 0.0132277
    powerDistribution = 0.0859803
    camera = 3 * 0.0220462
    
    # Estimated Values
    waterBottleMechanism = 0.5
    electronicsHardware = 0.15
    radioTransceiver = 0.0507063
    landingLegs = 0.202825
    oneTB_SSD_Harddrive = 0.18
    
    # Weights of various frames
    # Does not include weight of clamps for arms, that is included in weight of arm
    # To Do: make this a parametric design by scaling based on packing of circle of
    #   diameter 'propDiameter' around a the smallest circle possible
    if (numArms == 3):
        centerFrame = 0.636163313
    
    elif (numArms == 4):
        centerFrame = 0.472710296

    elif (numArms == 6):
        centerFrame = 0.710648931
    
    elif (numArms == 8):
        centerFrame = 0.699119006
        
    else:
        # If the frame is not that of a known desing, return an absurd number
        centerFrame = 1e99
    
    # Center of Frame, includes all electronics
    torso = (odroid + gps + pix + remoteReceiver + radioTransceiver + 
            powerDistribution + oneTB_SSD_Harddrive + waterBottleMechanism + 
            landingLegs + electronicsHardware + camera + centerFrame)
    
    # Payload Weight
    if isPayloadAttached:
        waterBottle = 0.51
        payload = 0.49
    else:
        waterBottle = 0
        payload = 0
    
    # Estimate Battery Weights
    weightPerCellPermAh = 0.988 / 3 / 6000
    batteryWeight = numBatteryCells * batteryCapacity * weightPerCellPermAh
    
    # totalWeight in lb
    totalWeight = (armWeight * numArms + motor * numMotors + torso + waterBottle + 
            payload + batteryWeight)
    
    # Convert from lb to N
    totalWeight *= 4.45

    return totalWeight

#----------------------------------------------------------------------------------#

# Calculates the Required Number of Batteries and the Time of Flight

# Returns: (batteryCapacity, currents, speeds, canHover)
#   batteryCapacity is the required battery capacity to finish the competition
#   speeds, and currents are 2 x 1 vectors with the pre and post speeds and currents
#   canHover is a 2 x 1 bool vector denoting if it is possible to hover pre and post 

# Inputs:
#   voltages is a list with the pre and post drop voltage (in that order)
#   motorSpec contains the relevant data of the motor. See start of script for info
#   numMotors, numArms, numBatteryCells, are the number of motors, number of arms, 
#       and number of battery cells respectively
#   propDiameter, and propPitch are the propeller diameter and pitch respectively, 
#       in inches
#   dist is a list with the pre and post drop distances needed to travel by the 
#       vehicle (in that order), in meters

# TO DO: fix this so speed is a function of voltage instead of RPM

def batteryCapacity(voltages, motorSpec, numMotors, numArms, numBatteryCells, 
        propDiameter, propPitch, dist):

    # Constants:
    # isStacked is based off the given number of motors and arms
    isStacked = not (numMotors == numArms)
    
    # Find the battery voltage in order to know the total energy in the battery
    batteryVoltage = 3.7 * numBatteryCells

    # Calculate the motor RPM for the given voltage
    motorRPMs = [
        motorRPM(voltages[0], motorSpec, propDiameter, propPitch, 0, alpha)[0], 
        motorRPM(voltages[1], motorSpec, propDiameter, propPitch, 0, alpha)[0]]

    # Optomize number of batteries for the given vehicle configuration

    # Initialize values
    minBatteryCapacity = 0
    # Ridiculous number of batteries as an upper limit to the optomization
    # Set max at 100 Ah capacity
    maxBatteryCapacity = 100000
    convError = 1
    absError = 1
    remainingEnergy = 1
    batteryCapacity = 0
    
    currents = [-1, -1]
    speeds = [0, 0]
    alphas = [-1, -1]
    canHovers = [-1, -1]
    powers = [-1, -1]
    currents = [-1, -1]
    iter = 0
    
    # Uses a bisection method instead of Newton or Gradient descent because the 
    #   speed calculation is itself a numerical solver, and thus has no derivatives
    # Cannot use scipy solver because too many binary conditions
    while (convError > 1e-10 and absError > 1e-8):
        # Usually takes about 50 iterations to converge, if it takes more something 
        #   is wrong
        iter += 1;
        if (iter > 1e3):
            return (-1, [-1, -1], [-1, -1], [-1, -1])
        
        # Calculate updated values and errors
        batteryCapacity_old = batteryCapacity
        batteryCapacity = (minBatteryCapacity + maxBatteryCapacity) / 2
        
        motorRPMs_old = copy.deepcopy(motorRPMs)

        motorRPMs = [
            motorRPM(voltages[0], motorSpec, propDiameter, propPitch, speeds[0], 
                alpha)[0], 
            motorRPM(voltages[1], motorSpec, propDiameter, propPitch, speeds[1], 
                alpha)[0]]

        convErrorList = [batteryCapacity_old - batteryCapacity, 
                        motorRPMs_old[0] - motorRPMs[0], 
                        motorRPMs_old[0] - motorRPMs[1]]

        convError = np.norm(convErrorList)
        absError = abs(remainingEnergy)

        # Theoretical Battery Energy
        batteryEnergy = batteryVoltage * (batteryCapacity / 1000 * 60 * 60)
        
        # Battery Energy
        #   Assumed landing with 20% of battery energy (SF 1.25)
        #   Assumed 95% depth of discharge
        #   Assumed 90% battery efficiency (actual charge / theoretical charge)
        batteryEnergy *= (1.00 - 0.20) * 0.95 * 0.9

        # Weight is a function of the number of batteries and payload attachment
        weights = [
            weight(numMotors, numArms, batteryCapacity, numBatteryCells, 
                propDiameter, 1), 
            weight(numMotors, numArms, batteryCapacity, numBatteryCells, 
                propDiameter, 0)]


        # Find speed and angle at which the drone will fly
        speeds[0], alphas[0] = speed(weight[0], numMotors, numArms, 
                motorRPMs[0], propDiameter, propPitch)

        speeds[1], alphas[1] = speed(weight[1], numMotors, numArms, 
                motorRPMs[1], propDiameter, propPitch)
        
        # speed() returns -1's if it cannot hover
        # Cannot hover if too heavy, the only way this function can make reduce 
        #   weight is to reduce the number of batteries 
        canHovers = [0, 0]
        
        # Check the pre-drop hovering
        if (speeds[0] == -1):
            maxBatteryCapacity = batteryCapacity;
            canHovers[0] = 0
            continue
        else:
            canHovers[0] = 1
        
        # Check the post-drop hovering
        if (speeds[1] == -1):
            maxBatteryCapacity = batteryCapacity;
            canHovers[1] = 0
            continue
        else:
            canHovers[1] = 1

        # Calculate energy required to finish this leg of the competition
        powers[0], currents[0] = powerConsumption(motorRPMs[0], numMotors, 
                propDiameter, propPitch, speeds[0], alphas[0], isStacked, 
                batteryVoltage)

        powers[0], currents[1] = powerConsumption(motorRPMs[1], numMotors, 
                propDiameter, propPitch, speeds[1], alphas[1], isStacked, 
                batteryVoltage)

        # TO DO: Need to check if those are -1's for errors

        timesOfFlight = [dists[0] / speeds[0], dists[1] / speed[1]]

        energiesForFlight = [timesOfFlight[0] * powers[0], 
                            timesOfFlight[1] * powers[1]]

        # Find the total energy used in the competition
        totalEnergy = energiesForFlight[0] + energiesForFlight[1]

        # Value to optimize to 0
        remainingEnergy = batteryEnergy - totalEnergy
        
        # if there is energy remaining, reduce the number of batteries,
        # else increase the number of batteries
        if (remainingEnergy > 0):
            maxBatteryCapacity = batteryCapacity
        else:
            minBatteryCapacity = batteryCapacity
    
    return (batteryCapacity, currents, speeds, canHovers)

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

# TO DO: Include the linear viscous effects of the propeller

def powerConsumption(motorRPM, numMotors, propDiameter, propPitch, speed, alpha, 
        batteryVoltage):    
    # Motor Velocity Constant in (rad/s) / V
    Kv = motorSpec[1] * 2  * np.pi / 60

    # Torque Constant
    Kt = 1 / Kv

    # Torque required by the 
    torque = 1 / 3 * 1.225 * exitVelocity * np.pi * r**2 * exitVelocity * r

    # Current of motor 
    #current = (torque / Kt) + 

    # Rotation Speed in rad / s
    omega = motorRPM * 2 * np.pi / 60

    Mechpower = torque * omega

    return
    """
    print
    print "current: ", current
    print "propPower: ", propPower, "rad/s:", omega, "voltage: ", voltage

    # Motor resistance is a property specific to each motor
    motorResistance = motorSpec[0]
    
    # Assumed controller resistance
    # TO DO: verify this
    controllerResistance = 0.0008
    
    # Assumed battery resistance
    batteryResistance = 0.0006 / numMotors
    
    # Total internal resistance of components from battery to motor
    totalResistance = motorResistance + controllerResistance + batteryResistance
    
    # The no load power is the consumption of the motor eaten by magnetic effects in 
    #   the motor
    # It is a property specific to each motor
    noLoadPower = motorSpec[2]
    
    # Solving the quadratic equation from above, substituting P_in for V * I, and
    #   P_out for propPower:
    motorCurrents = np.roots([totalResistance, -voltage, 
            (propPower + noLoadPower)])
    
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
        return (-1, -1)
      
    # To reach this point, the roots must be real and exactly one is non-negative
    else:
        motorCurrent = max(motorCurrents)

    # Power = V * I
    motorPower = voltage * motorCurrent
    
    # The total power is the sum of all motors
    totalPower = motorPower * numMotors

    # Add a conservative estimate power of all electronics
    totalPower += 50

    return (totalPower, motorCurrent)
    """

#----------------------------------------------------------------------------------#

def main():
    voltage = 22.04
    motorSpec = [0.102, 320, 7]
    propDiameter = 18
    propPitch = 4.7
    speed = 0
    alpha = 0
    numMotors = 8

    print weight(8, 8, 0, 0, 18, 0)

    #print motorRPM(voltage, motorSpec, propDiameter, propPitch, speed, 1.18682)
    print 
    print powerConsumption(voltage, motorSpec, numMotors, propDiameter, propPitch, 
            speed, alpha)


if (__name__ == "__main__"):
    main()