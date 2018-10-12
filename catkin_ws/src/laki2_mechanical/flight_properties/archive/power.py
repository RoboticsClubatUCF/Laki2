# Calculates the total power and current consumption per motor of the given vehicle

# Inputs:
# motorRPM and numMotors are the motor RPM and number of motors respectively
# propDiameter and propPitch are the propeller diameter and pitch repectively, both 
#   in inches
# speed is the vehicle speed in m/s
# alpha is the angle of the vehicle in radians
# isStacked is a boolean, 0 for flat frames, 1 else
# voltage is the voltage being supplied to the motors

# TO DO: Include air resistance of the vehicle as a vehicle loss
# TO DO: Include the linear viscous effects of the propeller
# TO DO: Figure out how to incoperate counter torque as a loss?

def powerConsumption(voltage, motorSpec, motorRPM, numMotors, numArms, propDiameter, 
            propPitch, speed, alpha, isStacked):

    # The power balance equation for the vehicle is of the following form:
    # P_in = P_out + I^2 * R_o + I_o * V
    
    # P_out is the power out, primarily power pulled by the propellers to
    #   a) do work on the air
    #   b) counteract air drag
    #   c) counteract propeller losses

    # I^2 * R_m are the "copper losses" due to the no load resistance of the 
    #   motor, controller, and battery
    
    # I_o * V are the "iron losses" are due to the no load current 
    #   required by the motor to produce 0 work
    
    # P_in is the power in, equal to V * I
    # Where V is the RMS voltage going to the motors (voltage from the controller)

    # Calculate the power required from work being done on the air
    thrustX, thrustY, exitVelocity = thrust(motorRPM, propDiameter, propPitch, 
            speed, alpha, isStacked)

    totalThrust = np.sqrt(thrustX ** 2 + thrustY ** 2)
    
    # If the x thrust is negative, it is actually generating power
    # Don't use sign because if thrustX is 0, Y direction can still be using power
    if (totalThrustX < 0):
        totalThrust *= -1
    
    # Work being done on the air by the propeller
    workOnAir = totalThrust * exitVelocity

    # TO DO: populate this with some propeller loss model
    propPowerLoss = 0

    # Total power consumed by the propeller
    propPower = workOnAir + propPowerLoss

    # TO DO: populate this with vehicle losses. air drag + others?
    vehicleLosses = airDrag(speed, alpha, numArms) * speed

    # Total power out to each motor
    pOut = propPower + vehicleLosses # + others?

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
    motorCurrents = numpy.roots([totalResistance, -batteryVoltage, 
            (Pout + noLoadPower)])
    
    # If by some miracle the parabola is perfectly placed on the x-axis, that is the 
    #   current draw
    if (len(motorCurrents) == 1):
        motorCurrent = motorCurrents[0]
    
    # If the solution is complex, the current draw is infinite
    elif (not numpy.isreal(motorCurrents).any()):
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