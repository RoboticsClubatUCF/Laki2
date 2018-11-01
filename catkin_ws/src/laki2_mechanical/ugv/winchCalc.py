import numpy as np
from scipy import integrate
from scipy.optimize import minimize

# Define Constants:

# Mass in kg
mass = 1.36078

# Drop Height in m
dropHeight = 60.96

# Gravity in m/s^2
g = 9.81

# Landing speed in m/s
landingSpeed = 6

# Max motor hub radius
maxAllowableHub =  3 * 0.0254

# This is basically useless
# maxElec is a list of [maxVoltage, maxCurrent, maxPower]
def resCalc(Kv, Ra, gear, hub, maxElec):
    # Force of gravity
    Fg = mass * g

    # Power of gravity
    Pg = Fg * landingSpeed

    # Motor speed at the given linear speed
    omega = gear * landingSpeed / hub

    # Voltage produced by the motor
    voltage = omega / Kv

    # Returns the net power into the UGV with the given external resistance and
    #   motor hub size
    def resEval(externRes):
        # Resistors in series
        totalRes = Ra + externRes

        # Power across equivalent resistance
        Pr = voltage**2 / totalRes
        
        return abs(Pr - Pg)

    bnds = ((0, None),)
    opt = minimize(resEval, Ra, method = 'SLSQP', bounds = bnds, tol = 1e-8)

    # If the minimizer optimized to a garbage value don't return it
    if (resEval(opt.x[0]) > 1e-5):
        return -1

    # If the optimizer was sucessful return the resistance value
    if (opt.success):
        return opt.x[0]
    
    # Otherwise return a non-physical value
    else:
        return -1

# Returns lists for time, y, y', and tension in the tether
#   of a vehicle dropping in 0 air friction from the drop height to the ground
# Solve numerically
def drop(brakeHeight, maxSpeed, Kv, Ra, gear, hub, res):
    # Calculates the voltage, current, power, and force from the motors
    # Inputs speed in m/s
    # Outputs [voltage, current, power, force]
    def motorVals(speed, height):
        if (speed == 0):
            return [0, 0, 0, 0]

        if ((height > brakeHeight) and (abs(speed) < maxSpeed)):
            return [0, 0, 0, 0]

        # Motor speed in rad/s: gear-ratio * linear-velocity / hub-radius
        omega = gear * abs(speed) / hub

        # Voltage produced by the motor
        voltage = omega / Kv

        # Total resistance of the system
        totalRes = Ra + res

        # Current throught the motor and resistor: V = IR
        current = voltage / totalRes

        # Power produced by the motor: V^2 / R
        power = voltage**2 / totalRes

        # Force from motor: power = force * linear-velocity
        # TO DO: Double check that this lines up with 
        #   power = omega * torque
        #   torque = hub-radius * force
        force = power / abs(speed)

        return [voltage, current, power, force]

    # Update function for IVP solver
    # my" = Fm - Fg
    # Ground is 0, up is +
    # Inputs:
    #   U = [y, y']
    #   t is unused
    # Outputs:
    #   [y', y"]
    def dU_dt(t, U):
        # Force of gravity
        Fg = mass * g

        # Pull force from the motor from the helper function
        motorForce = motorVals(U[1], U[0])[3]

        # Net Force acting on the motor
        netForce = motorForce - Fg

        # Net acceleration on the UGV (y double prime)
        yDub = netForce / mass

        # Return [y', y"]
        return [U[1], yDub]

    # Dropped from the drop height with 0 initial velocity
    U = [dropHeight, 0]
    ts = [0, 0]

    # Make a list to populate with (t, y)
    tVals = []
    yVals = []
    yPrimeVals = []
    tensionVals = []

    # While the vehicle is not on the ground, continue to iterate
    while (U[0] > 0):
        # Take another time step
        ts = [ts[1], ts[1] + 1e-2]

        # Solve for the next time step
        intermediate = integrate.solve_ivp(dU_dt, ts, U)

        tensionVals.append((dU_dt(0, U)[1] + g) / mass)
        U = [intermediate.y[0][-1], intermediate.y[1][-1]]

        tVals.append(ts[1])
        yVals.append(U[0])
        yPrimeVals.append(U[1])

    return (tVals, yVals, yPrimeVals, tensionVals)

# Returns lists for time, y, y', and tension in the tether
#   of a vehicle dropping in 0 air friction from the drop height to the ground
# Solve analytically
def analyticalDrop(brakeHeight, speedHeight, maxSpeed, Kv, Ra, gear, hub, res):
    # TO DO:
    #   Walk thru this peicewise:
    #   step 1: complete free fall from drop height to speedHeight, use kinematics
    #   step 2: perfect PWM from speedHeight to brakeHeight, constant speed of maxSpeed
    #   step 3: decay into maxSpeed, diff Eq:
    #       c^2 * y(t) = a * c^2 + b * c * m * (1-e^(-c*t/m)) + d * m * (e^(-c*t/m) - 1) + c*d*t
    #       c = (gear * hub / Kv)^2 / (totRes)
    #       d = mass * g
    #       m = mass
    #       a = brakeHeight
    #       b = maxSpeed

# Calculates electric properties
def brakeElecProps(Kv, Ra, gear, hub, res, speed):
    # Resistance of motor and external resistor act in series
    totalRes = Ra + res

    # Motor rotational speed at the control height
    omega = gear * speed / hub

    # Voltage generated at the control height
    volt = omega / Kv

    # Power generated at the control height
    power = volt**2 / totalRes

    # Current generated at the control height
    curr = volt / totalRes

    return [volt, curr, power]    

# Calculates the fastest speed allowable and the lowest height to deploy the brake
def brakeVals(Kv, Ra, gear, hub, res, maxElec):
    speed = 0
    minSpeed = 0
    # Twice the max speed achievable during the fall
    maxSpeed = 2 * np.sqrt(2 * dropHeight * g)
    convError = 1

    # Calculate the min height that the brake can be applied in order to not
    #   exceed the elctrical limits
    while (convError > 1e-6):
        speed_old = speed
        speed = (minSpeed + maxSpeed)/2
        convError = abs(speed_old - speed)

        [volt, curr, power] = brakeElecProps(Kv, Ra, gear, hub, res, speed)

        # If it is over the electrical limits, the speed is too fast
        if (volt > maxElec[0] or curr > maxElec[1] or power > maxElec[2]):
            maxSpeed = speed

        else:
            minSpeed = speed

    height = 0
    minHeight = 0
    maxHeight = 2 * dropHeight
    convError = 1

    # Calculate the min height that the brake can be applied to reach the desired
    #   speed
    while (convError > 1e-6):
        # Improve the hub guess
        height_old = height
        height = (minHeight + maxHeight)/2
        
        convError = abs(height_old - height)

        finalSpeed = -drop(height, speed, Kv, Ra, gear, hub, res)[2][-1]

        # If the drop height slows down the UGV to within 0.5% of the desired speed by 
        #   the end, the height can be raised
        if (abs(finalSpeed - landingSpeed) < (0.005 * landingSpeed)):
            maxHeight = height
        
        # Otherwise it is not enough braking distance
        else:
            minHeight = height

    return [speed, height]

# Returns the height that the given speed with be achieved
# Will return a negative number
def speedHeightCalc(speed):
    dropDist = (speed**2) / (2*g)
    speedHeight = dropHeight - dropDist

    return speedHeight

def minimizeTime(Kv, Ra, gear, maxElec):
    minHub = 0
    maxHub = 0.17*0.0254*2
    hub = 0
    convError = 1

    while (convError > 1e-5):
        hub_old = hub
        hub = (minHub + maxHub)/2

        print (hub)

        convError = abs(hub_old - hub)

        res = resCalc(Kv, Ra, gear, hub, maxElec)
        
        print (res)

        if (res == -1):
            # If the resistance is non-physical, reduce the size of the hub
            maxHub = hub

        maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)

        speedHeight = speedHeightCalc(maxSpeed)

        # brakeHeight is the height that is required to continually brake from
        # speedHeight is the height that it reaches the max falling speed and must
        #   start the PWM pulses until the brakeHeight
        # Shortest time will minimize the distance between these two

        if (speedHeight > brakeHeight):
            maxHub = hub
        else:
            minHub = hub

    return hub