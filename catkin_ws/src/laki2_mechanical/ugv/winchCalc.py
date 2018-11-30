from __future__ import division
import numpy as np
from scipy import integrate
from scipy.optimize import minimize
import matplotlib.pyplot as plt

#----------------------------------------------------------------------------------#

# BE WARNED: currently does not work for spool sizes other than what is listed as a
#   constant here

# Define Constants:

# Mass in kg
mass = 1.36078

# Drop Height in m
dropHeight = 30.48

# Gravity in m/s^2
g = 9.81

# Landing speed in m/s
landingSpeed = 5.5

# Max motor hub radius
maxHub =  3 * 0.0254

#----------------------------------------------------------------------------------#

def newDrop(brakeHeight, maxSpeed, Kv, hub, spoolWidth, gear = 1):
    # Calculates the voltage, current, power, and force from the motors
    # Inputs speed in m/s
    # Outputs [voltage, current, power, force]
    def motorVals(speed, height):
        if (speed == 0):
            return [0, 0, 0]

        if ((height > brakeHeight) and (abs(speed) < maxSpeed)) or (abs(speed) < 3):
            return [0, 0, 0]

        # Motor speed in rad/s: gear-ratio * linear-velocity / hub-radius
        omega = gear * abs(speed) / effHub(hub, height, spoolWidth)

        # Voltage produced by the motor
        voltage = omega / Kv

        # Power produced by the motor: V^2 / R
        power = 3 * (20.88 * voltage - 42.1)

        # Max power the motor can handle
        if (power > 113.4*0.9):
            power = 113.4*0.9

        # Force from motor: power = force * linear-velocity
        force = power / abs(speed)

        return [voltage, power, force]

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
        motorForce = motorVals(U[1], U[0])[2]

        print U[1], motorForce

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
    motorVolt = []
    power = []
    tensionVals = []

    # While the vehicle is not on the ground, continue to iterate
    while (U[0] > 0):
        # Take another time step
        ts = [ts[1], ts[1] + 1e-2]

        # Solve for the next time step
        intermediate = integrate.solve_ivp(dU_dt, ts, U)

        volt, powerVal, force = motorVals(U[1], U[0])

        print "power: ", powerVal

        tensionVals.append((dU_dt(0, U)[1] + g) / mass)
        U = [intermediate.y[0][-1], intermediate.y[1][-1]]

        tVals.append(ts[1])
        yVals.append(U[0])
        yPrimeVals.append(-U[1])

        motorVolt.append(volt)

        power.append(powerVal)

    return (tVals, yVals, yPrimeVals, motorVolt, power, tensionVals)


# Estimates the size of a given hub at a given height due to the wrapping of the 
#   tether
def effHub(hub, height, spoolWidth):
    # The fishing line can be thought of as cylinders wrapped around a circle
    # It can be modeled ad cylinders with some packing efficiency

    # cross sectional area is assumed to be that of 30lb fishing line (Dia = 0.022")
    tetherCrossArea = np.pi * (0.022 * 0.0254 / 2)**2

    # Height is the length of wire remaining
    # Volume (with 100% packing efficiency) is height * cross sectional area
    volume = height * tetherCrossArea

    # Assume it has 90% of the theoretical cylinder packing efficiency of 91%:
    volume /= 0.91 / 0.9

    # Find the cross sectional area of the spool
    spoolCrossArea = volume / spoolWidth

    # Area of an annulus is pi(R^2 -r^2)
    effHub = np.sqrt((spoolCrossArea / np.pi) + hub**2)

    return effHub

#----------------------------------------------------------------------------------#

# Returns lists for time, y, y', and tension in the tether
#   of a vehicle dropping in 0 air friction from the drop height to the ground
# Solve numerically
def drop(brakeHeight, maxSpeed, Kv, hub, spoolWidth, res):
    # Calculates the voltage, current, power, and force from the motors
    # Inputs speed in m/s
    # Outputs [voltage, current, power, force]
    def motorVals(speed, height):
        if (speed == 0):
            return [0, 0, 0, 0]

        if ((height > brakeHeight) and (abs(speed) < maxSpeed)):
            return [0, 0, 0, 0]

        # Motor speed in rad/s: gear-ratio * linear-velocity / hub-radius
        omega = gear * abs(speed) / effHub(hub, height, spoolWidth)

        # Voltage produced by the motor
        voltage = omega / Kv

        # Total resistance of the system
        totalRes = Ra + res

        # Current throught the motor and resistor: V = IR
        current = voltage / totalRes

        # Power produced by the motor: V^2 / R
        power = voltage**2 / totalRes

        # Force from motor: power = force * linear-velocity
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
    resVolt = []
    motorVolt = []
    sysCurr = []
    rmsPower = []
    resPower = []
    motorPower = []

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
        yPrimeVals.append(-U[1])

        volts, curr, powers = brakeElecProps(Kv, Ra, gear, 
            effHub(hub, U[0], spoolWidth), res, U[1])

        motorVolt.append(-volts[0])
        resVolt.append(-volts[1])
        
        sysCurr.append(-curr)
        
        resPower.append(powers[1])
        motorPower.append(powers[0])

        power = motorVals(U[1], U[0])[2]
        rmsPower.append(power)

    return (tVals, yVals, yPrimeVals, resVolt, motorVolt, sysCurr, rmsPower, 
            resPower, motorPower, tensionVals)

#----------------------------------------------------------------------------------#

# Returns lists for time, and y
#   of a vehicle dropping in 0 air friction from the drop height to the ground
# Solve analytically (at least mostly analytically)
def analyticalDrop(brakeHeight, maxSpeed, Kv, gear, hub, totalRes, flag = None):
    # Returns the height of the UGV for a given time after the brakeHeight
    def decayedDescentPos(t):
        # Height as a function of time (solved by wolfram symbolic diff eq solver):        
        #   y(t) = brakeHeight + (g * m^2 / c^2) * (exp(r) - 1) + (g * m / c) * t
        #       + (maxSpeed * m / c) * (1 - exp(r))

        #   Where:
        #       g = -abs(gravity)
        #       m = mass
        #       c = (gear * hub / Kv)^2 / (totRes)
        #       r = -c * t / m

        c = (gear /(hub * Kv))**2 / (totalRes)        
        r = -c * t / mass
        gravity = -g
        speed = -maxSpeed

        height = (brakeHeight + (gravity * mass**2 / c**2) * (np.exp(r) - 1)
            + (gravity * mass / c) * t - (maxSpeed * mass / c) * (1 - np.exp(r)))

        return height

    # Returns the speed of the UGV for a given time after the brakeHeight
    def decayedDescentVel(t):
        # Velocity as a function of time (solved by wolfram symbolic diff eq solver):        
        #   y'(t) = (g * m / c) * exp(r) + (g * m / c) + maxSpeed * exp(r)

        #   Where:
        #       g = -abs(gravity)
        #       m = mass
        #       c = (gear * hub / Kv)^2 / (totRes)
        #       r = -c * t / m

        c = (gear /(hub * Kv))**2 / (totalRes)        
        r = -c * t / mass
        gravity = -g
        speed = -maxSpeed

        speed = ((-gravity * mass / c) * np.exp(r) + (gravity * mass / c) 
            + speed * np.exp(r))

        return speed

    # Height at which the max speed is achieved
    speedHeight = speedHeightCalc(maxSpeed)

    # If the brakeHeight is less than the speedHeight, the system follows 3 phases:
    #   Phase 1: Free Fall
    #   Phase 2: Perfect PWM to maintain max speed
    #   Phase 3: Full Braked Descent to Ground
    if (brakeHeight < speedHeight):
        # Phase 1: Free Fall:
        #   y = y0 - 0.5*g*t^2
        tSpeed = np.sqrt(2*(dropHeight - speedHeight) / g)

        # Phase 2: Perfect PWM to maintain the maxSpeed
        #   y = y0 + maxSpeed * (t - tSpeed)
        tBrake = tSpeed + (speedHeight - brakeHeight) / maxSpeed

    # Otherwise, the vehicle skips combines phases 1 and 2 to reach the brake height 
    #   before it reaches max speed:
    else:
        # Phase 1 & 2: Free Fall to BrakeHeight:
        #   y = y0 - g*t^2
        tBrake = np.sqrt(2*(dropHeight - brakeHeight) / g)

        # For plotter to plot correctly
        tSpeed = tBrake

        # In this case, the speed that starts the exponential decay is no the max
        #   speed of the vehicle but rather the speed achieved at the drop height
        maxSpeed = g * tBrake

    # Phase 3: Full Braked Descent to Ground (Exponential Decay of Speed)
    # See decayedDescent for eqn
        
    # Quick helper function to return error of the decayed descent
    def evalTime(t):
        return abs(decayedDescentPos(t))

    bnds = ((0, None),)
    opt = minimize(evalTime, 1e-6, method = 'SLSQP', bounds = bnds, tol = 1e-8)
    
    tLand = tBrake + opt.x[0]

    # If the input didn't ask for anything, output the time to hit the ground
    if (flag == None):
        return tLand        

    # The input requested the landing speed
    if (flag == 'landingSpeed'):
        return decayedDescentVel(opt.x[0])

    # The input requested the piecewise time values
    if (flag == 'times'):
        return (tSpeed, tBrake, tLand[0])

    else:
        tVals = np.linspace(0, tLand, 1000)
        yVals = []
        yPrimeVals = []

        for t in tVals:
            # Phase 1:
            if (t < tSpeed):
                yVals.append(dropHeight - 0.5*g*t**2)
                yPrimeVals.append(-g*t)

            # Phase 2:
            elif (t < tBrake):
                yVals.append(speedHeight - maxSpeed * (t-tSpeed))
                yPrimeVals.append(-maxSpeed)

            # Phase 3:
            else:
                yVals.append(decayedDescentPos(t - tBrake))
                yPrimeVals.append(decayedDescentVel(t - tBrake))

        return (tVals, yVals, yPrimeVals)

#----------------------------------------------------------------------------------#

# Calculates peak to peak electric properties of the motor and resistor
def brakeElecProps(Kv, Ra, gear, hub, res, speed):
    # Resistance of motor and external resistor act in series
    totalRes = Ra + res

    # Motor rotational speed at the given linear speed
    omega = gear * speed / hub

    # Total voltage generated at the motor rotational speed
    volt = omega / Kv

    # volts is [motor voltage, resistor voltage]
    volts = [volt * (Ra / (Ra + res)) * np.sqrt(2), 
            volt * (res / (Ra+res)) * np.sqrt(2)]

    # Current generated at the control height
    curr = volt / totalRes * np.sqrt(2)

    # Power generated at the control height
    powers = [volts[0] * curr * 0.8, volts[1] * curr / 3]

    return [volts, curr, powers]    

#----------------------------------------------------------------------------------#

# Calculates the fastest speed allowable and the lowest height to deploy the brake
# TO DO: Make this work with the new variable hub model
def brakeVals(Kv, Ra, gear, hub, res, maxElec):
    speed = 0
    minSpeed = 0
    # Twice the max speed achievable during the fall
    maxSpeed = 2 * np.sqrt(2 * dropHeight * g)
    convError = 1
    lastValid = -1

    # Calculate the min height that the brake can be applied in order to not
    #   exceed the elctrical limits
    while (convError > 1e-6):
        speed_old = speed
        speed = (minSpeed + maxSpeed)/2
        convError = abs(speed_old - speed)

        [volts, curr, powers] = brakeElecProps(Kv, Ra, gear, hub, res, speed)

        # If it is over the motor electrical limits, the speed is too fast
        # Voltage multiplied by sqrt(2) for peak to peak voltage
        # Current multiplied by sqrt(2) for peak to peak current
        # Power Multiplied by 2 for peak to peak power
        if (volts[0] > maxElec[0][0] or curr > maxElec[0][1] 
                or powers[0] > maxElec[0][2]):
            maxSpeed = speed

        # If it is over the MOSFET electrical limits, the speed is too fast
        # Voltage multiplied by sqrt(2) for peak to peak voltage
        # Current multiplied by sqrt(2) for peak to peak current
        # Power Multiplied by 2 for peak to peak power
        elif (volts[1] > maxElec[1][0] or curr > maxElec[1][1] 
                or powers[1] > maxElec[1][2]):
            lastValid = speed
            maxSpeed = speed

        else:
            minSpeed = speed

    speed = lastValid

    if (speed < landingSpeed):
        return [speed, -1]

    height = 0
    minHeight = 0
    maxHeight = dropHeight
    convError = 1
    lastValid = -1

    totalRes = Ra + res

    # Calculate the min height that the brake can be applied to reach the desired
    #   speed
    while (convError > 1e-6):
        # Improve the hub guess
        height_old = height
        height = (minHeight + maxHeight)/2
        
        convError = abs(height_old - height)

        finalSpeed = -analyticalDrop(height, speed, Kv, gear, hub, totalRes, 
                    flag = 'landingSpeed')

        finalSpeed = drop(height, speed, Kv, Ra, gear, hub, spoolWidth, res)[2][-1]

        # If the drop height slows down the UGV to within 0.5% of the desired speed by 
        #   the end, the height can be raised
        if (abs(finalSpeed - landingSpeed) < (0.005 * landingSpeed)):
            lastValid = height
            maxHeight = height
        
        # Otherwise it is not enough braking distance
        else:
            minHeight = height

    finalSpeed = -analyticalDrop(lastValid, speed, Kv, gear, hub, totalRes, 
        flag = 'landingSpeed')

    finalSpeed = drop(lastValid, speed, Kv, Ra, gear, hub, spoolWidth, res)[2][-1]

    if (abs(finalSpeed - landingSpeed) > (0.005 * landingSpeed)):
        return [speed, -1]

    else:
        return [speed, lastValid]

#----------------------------------------------------------------------------------#

# Returns the height that the given speed with be achieved
# Will return a negative number
def speedHeightCalc(speed):
    dropDist = (speed**2) / (2*g)
    speedHeight = dropHeight - dropDist

    return speedHeight

#----------------------------------------------------------------------------------#

# Returns the largest hub possible with the given configuration
# TO DO: make this work with the variable hub model
def largestHub(Kv, gear, totalRes, maxElec):
    def evalHub(hub):
       # Force of gravity
        Fg = mass * g

        # Power of gravity
        Pg = Fg * landingSpeed

        # Motor speed at the given linear speed
        omega = gear * landingSpeed / hub

        # Voltage produced by the motor
        voltage = omega / Kv

        # Power across equivalent resistance
        Pr = voltage**2 / totalRes
            
        return abs(Pr - Pg)

    bnds = ((1e-6, None),)
    opt = minimize(evalHub, maxHub/2, method = 'L-BFGS-B', bounds = bnds, tol = 1e-6)

    # If converged, return the result
    if (opt.success):
        return opt.x[0]

    opt = minimize(evalHub, maxHub/2, method = 'TNC', bounds = bnds, tol = 1e-6)

    # If converged, return the result
    if (opt.success):
        return opt.x[0]

    opt = minimize(evalHub, maxHub/2, method = 'SLSQP', bounds = bnds, tol = 1e-6)

    # If converged, return the result
    if (opt.success):
        return opt.x[0]
    
    # At this point all of the methods that allow for boundaries have been used.
    #   The solution was not found
    return -1

#----------------------------------------------------------------------------------#

# Returns the hub radius that minimizes the descent time within the given constraints
#   This assumes that the hub is sized approriately such that at steady state with
#       full break the speed equals the max allowable landing speed. This does not
#       have to be the case as the system could be PWM'ed into disapating less power
# TO DO: at the very least helper functions are broken due to the new variable hub
#   model. Fix those first
def minimizeTime(Kv, Ra, gear, res, maxElec):
    maxHub = largestHub(Kv, gear, Ra + res, maxElec)
    
    # TO DO: Calculate a minimum hub size to stay within bounds

    # A spool with a 0.125" radius is the minimum we can tolerate
    if (maxHub < 0.125 * 0.0254):
        return -1

    bnds = ((0.125 * 0.0254, maxHub),)

    def evalHub(hub):
        maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)
        t = analyticalDrop(brakeHeight, maxSpeed, Kv, gear, hub, Ra + res)
        return t

    opt = minimize(evalHub, maxHub/2, method = 'L-BFGS-B', bounds = bnds, tol = 1e-6)

    # If converged, return the result
    if (opt.success):
        return opt.x[0]

    opt = minimize(evalHub, maxHub/2, method = 'TNC', bounds = bnds, tol = 1e-6)

    # If converged, return the result
    if (opt.success):
        return opt.x[0]

    opt = minimize(evalHub, maxHub/2, method = 'SLSQP', bounds = bnds, tol = 1e-6)

    # If converged, return the result
    if (opt.success):
        return opt.x[0]
    
    # At this point all of the methods that allow for boundaries have been used.
    #   The solution was not found
    return -1

#----------------------------------------------------------------------------------#

# Returns the minimum time to descent for the given motor
# This is basically useless
def motorEvaluator(Kv, Ra, gear, res, maxElec):
    hub = minimizeTime(Kv, Ra, gear, res, maxElec)

    if (hub == -1):
        print 'hub error'
        return (-1, -1, np.inf)

    maxSpeed, brakeHeight = brakeVals(Kv, Ra, gear, hub, res, maxElec)

    print maxSpeed, brakeHeight

    if (maxSpeed == -1 or brakeHeight == -1):
        print 'brakeVal error'
        return (-1, -1, np.inf)

    t = analyticalDrop(brakeHeight, maxSpeed, Kv, gear, hub, Ra + res)

    if (t == -1):
        print 'drop error'
        return (-1, -1, np.inf)
    else:
        return (hub, t)
