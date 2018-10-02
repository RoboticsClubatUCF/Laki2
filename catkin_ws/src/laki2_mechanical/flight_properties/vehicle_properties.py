from __future__ import division
import csv
import math
import copy
import numpy as np
import scipy
from scipy.optimize import minimize
from scipy.interpolate import interp1d


# MotorSpec:
#   0 - resistance
#   1 - Kv ((rads / s) / V)
#   2 - Kt (N-m / A)
#   3 - no load power
#   4 - max current
#   5 - weight (g)

# PropSpec
#   0 - diameter
#   1 - pitch


class vehicle:
    # Initialize by populating the thrust and power databases from xoar data
    def __init__(self):
        # Assumed drag to be 1.6 for all configurations
        self.dragCoef = 1.6
        
        # Assume air density to be that of air at sea level
        self.airDensity = 1.225
        
        self.thrustDatabase = dict()
        self.powerDatabase = dict()

        # (diameter, pitch, (Ct vals), (Cp vals), (RPMs))
        xoarData = [(11, 5.5, (0.0964, 0.1025), (0.0596, 0.0569), (4114, 6903)), 
                    (12, 4.5, (0.0727, 0.0784), (0.0335, 0.0223), (3420, 8050)), 
                    (13, 4.5, (0.0666, 0.0699), (0.0415, 0.0261), (2614, 7607)),
                    (14, 5.0, (0.0750, 0.0823), (0.0301, 0.0237), (3010, 7212)),
                    (15, 5.0, (0.0874, 0.1009, 0.0928, 0.0862, 0.091),
                            (0.0561, 0.0422, 0.0401, 0.0365, 0.0358), 
                            (2010, 2919, 3225, 4830, 6615)), 
                    (15, 5.5, (0.0848, 0.0892), (0.0345, 0.0307), (3518, 6029)),
                    (16, 6.0, (0.0830, 0.0847), (0.0399, 0.0422), (2641, 6020)),
                    (17, 6.0, (0.0872, 0.0929), (0.0409, 0.0561), (2811, 5618)), 
                    (18, 6.5, (0.0879, 0.0936), (0.0413, 0.0412), (2002, 4759)),
                    (20, 6.0, (0.0631, 0.0635), (0.0266, 0.0220), (2625, 4860)), 
                    (21, 6.0, (0.0589, 0.0628), (0.0229, 0.0210), (2457, 5700)), 
                    (21, 12., (0.1023, 0.1091), (0.0747, 0.0506), (2009, 5272)), 
                    (22, 8.0, (0.0756, 0.0767), (0.0325, 0.0302), (2004, 5300)),
                    (24, 9.0, (0.0706, 0.0751), (0.0274, 0.0294), (2412, 6167))]

        for prop in xoarData:
            thrustFunc = interp1d(prop[4], prop[2], 
                    bounds_error = False, fill_value = "extrapolate")
            self.thrustDatabase[(prop[0], prop[1])] = thrustFunc

            powerFunc = interp1d(prop[4], prop[3], 
                    bounds_error = False, fill_value = "extrapolate") 
            self.powerDatabase[(prop[0], prop[1])] = powerFunc

    #------------------------------------------------------------------------------#

    # Lookup table for lateral cross sectional area of requested drone in m^2
    # Estimate values based on quick frame designs
    # Values from quick Creo Parametric model
    # Returns (topArea, frontArea)
    def lookupLateralArea(self, numArms):
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

    #------------------------------------------------------------------------------#

    # Returns the air drag on a vehicle with the given properties in N
    # Inputs:
    #   speed in m/s
    #   alpha in rad
    #   numArms is an integer for the number of arms on the vehicle
    def airDrag(self, speed, alpha, numArms):
        # Populate lateral areas of vehicle
        topArea, frontArea = self.lookupLateralArea(numArms)
        
        # Calculate lateral areas of the vehicle to oncoming air
        areaEff = topArea * np.sin(alpha) + frontArea * np.cos(alpha)

        # Drag Equation
        airDrag = 0.5 * self.dragCoef * self.airDensity * speed**2 * areaEff

        return airDrag

    #------------------------------------------------------------------------------#

    # Returns the weight of specified vehicle in N

    # numMotors and numArms are the number of motors and arms respectively
    # numBatteries and numBatteryCells are the number of batteries and cells in 
    #   those batteries respectively
    # propDiameter is the diameter of each prop in inches
    # isPayloadAttached is a boolean, 1 when payload is attached, 0 if not attached

    # This estimate assumes that the weight of the batteries scales linearly with 
    #   the number of battery cells and the number of batteries. This is reasonable, 
    #   and allows the analysis to test different voltages and capacities

    # All constants for item weights are in grams. Converted to N in last step

    def weight(self, numMotors, numArms, motorSpec, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached):   
        # Values obtained from gram scale at robotics
        esc = 10
        jetson = 144.0
        gps = 18.0
        beaglebone = 36.0
        remoteReceiver = 6.0
        powerDistribution = 39.0
        
        # Estimated Values
        payloadDropMechanism = 226.8
        usbHub = 30.34
        electronicsHardware = 68.0
        radioTransceiver = 23.0
        landingLegs = 200.0
        camera = 19.5 + 2.4
        oneTB_SSD_Harddrive = 81.65
        networkDevices = 40.0
        cables = 50.0
        
        # Weights of various frames. Assumes frames weight approximately the same 
        #   and only the arms increase with size.
        # Values from weight of plastic frame, carbon frames ~2.5 times less dense
        # Doesn't count weight of clamps for arms, that is included in weight of arm
        if (numArms == 3):
            centerFrame = 288.5 / 2.5
        
        elif (numArms == 4):
            centerFrame = 264.4 / 2.5

        elif (numArms == 6):
            centerFrame = 250.2 / 2.5
        
        elif (numArms == 8):
            centerFrame = 248.2 / 2.5
            
        else:
            # If the frame is not that of a known desing, return an absurd number
            centerFrame = 1e99
        
        # Center of Frame, includes all electronics
        torso = (jetson + 2 * gps + beaglebone + remoteReceiver + powerDistribution 
                + payloadDropMechanism + usbHub + electronicsHardware + 
                radioTransceiver + landingLegs + 3 * camera + oneTB_SSD_Harddrive + 
                centerFrame + networkDevices + cables)
        
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

        # Estimate from Creo Model (includes weight of clamps on both sides of arm)
        clampWeight = 53.4 + 16
        armWeight = clampWeight + (8 + armPerIn * ((propDiameter / 2) - 0.75))

        # Estimate Battery Weights
        # Based off ncr batteries (ncr more energy dense that lipo)
        battPerCellPerMilliAmpHr = 0.01575
        batteryWeight = numBatteryCells * batteryCapacity * battPerCellPerMilliAmpHr

        # XOAR 18in prop weighs 31g
        propWeight = 31/18 * propDiameter

        # totalWeight in lb
        totalWeight = armWeight * numArms 
        totalWeight += (esc + motor) * numMotors 
        totalWeight += torso + payload + batteryWeight
        totalWeight += propWeight * numMotors
        
        # Convert from g to N
        totalWeight *= 9.81 / 1000

        return totalWeight

    #------------------------------------------------------------------------------#

    # Calculates the Required Number of Batteries and the Time of Flight

    # Returns: (batteryCapacity, motorCurrents, escCurrents, 
    #           speeds, canHover, isFeasible)
    #   batteryCapacity is the required battery capacity to finish the competition
    #   motorCurrents and escCurrents are the currents seen by the motors and esc's
    #   speeds, and currents are tuple with the pre and post speeds and currents
    #   canHover is a tuple denoting if it is possible to hover pre and post 
    #   isFeasible is a boolean for if drone can fly the total dist

    # Inputs:
    #   motorRPMs is a list of the pre and post drop motor RPMs
    #   motorSpec is a list of relevant data of the motor
    #       See start of class for info
    #   numMotors, numArms, numBatteryCells, are the number of motors, number of 
    #       arms, and number of battery cells respectively
    #   propSpec is a list of relevant data of the propeller.
    #       See start of document for info
    #   dist is a list with the pre and post drop distances needed to travel by the 
    #       vehicle (in that order), in meters

    def batteryCapacity(self, motorRPMs, motorSpec, numMotors, numArms, numBatteryCells, propSpec, dists):
        # Find the battery voltage in order to know the total energy in the battery
        batteryVoltage = 3.7 * numBatteryCells

        # Propeller Information
        propDiameter, propPitch = propSpec[0:2]

        # Initialize values
        minCapacity = 0
        # Ridiculous number of batteries as an upper limit to the optomization
        # Set max at 100 Ah capacity
        maxCapacity = 120000
        
        convError = 1
        absError = 1
        remainingEnergy = 1
        capacity = 0
        
        motorCurrents = [-1, -1]
        escCurrents = [-1, -1]
        speeds = [0, 0]
        alphas = [-1, -1]
        canHovers = [-1, -1]
        powers = [-1, -1]
        currents = [-1, -1]
        iter = 0
        
        # Did not use scipy solver due to number of unary conditions
        while (convError > 1e-10 and absError > 1e-8):
            # Usually takes about 50 iterations to converge, if it takes more 
            #   something is wrong
            if (iter > 1e3):
                return (-1, [-1, -1], [-1, -1], [-1, -1])
            
            # Calculate updated values and errors
            capacity_old = capacity
            capacity = (minCapacity + maxCapacity) / 2

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
                self.weight(numMotors, numArms, motorSpec, capacity, 
                        numBatteryCells, propDiameter, 1),

                self.weight(numMotors, numArms, motorSpec, capacity, 
                        numBatteryCells, propDiameter, 0)]

            # Find speed and angle at which the drone will fly
            speeds[0], alphas[0] = self.speed(weights[0], numMotors, numArms, 
                    motorRPMs[0], propSpec)

            speeds[1], alphas[1] = self.speed(weights[1], numMotors, numArms, 
                    motorRPMs[1], propSpec)
            
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
            powers[0] = self.propProperties(motorRPMs[0], propSpec, speeds[0], alphas[0])[3]
            powers[0] *= numMotors

            powers[1] = self.propProperties(motorRPMs[1], propSpec, speeds[1], alphas[1])[3]
            powers[1] *= numMotors

            timesOfFlight = [dists[0] / speeds[0], dists[1] / speeds[1]]

            energyForFlights = [timesOfFlight[0] * powers[0], 
                                timesOfFlight[1] * powers[1]]

            # Find the total energy used in the competition
            energyUsed = energyForFlights[0] + energyForFlights[1]

            # Value to optimize to 0
            remainingEnergy = batteryEnergy - energyUsed

            print "\nenergyUsed: \t", energyUsed
            print "batteryEnergy: \t", batteryEnergy
            print "capacity: \t", capacity

            # if there is energy remaining, reduce the number of batteries,
            # else increase the number of batteries
            if (remainingEnergy > 0):
                maxCapacity = capacity
            else:
                minCapacity = capacity

        # Estimate current pull to be prop power / battery voltage        
        currents = [powers[0]/batteryVoltage/numMotors, powers[1]/batteryVoltage/numMotors]

        # The loop can finish because it converged on a garbage value
        # Check here to see if the result is garbage, and make the last item of the 
        #   return 0 or 1 accordingly
        if (abs(remainingEnergy) > 10.0 or not canHovers[0] or not canHovers[1]):
            return (capacity, currents, speeds, canHovers, 0)
        else:
            return (capacity, currents, speeds, canHovers, 1)

    #------------------------------------------------------------------------------#    

    # Calculates the max torque of a motor at a given RPM.
    # returns torque

    def maxMotorTorque(self, motorRPM, motorSpec, maxVoltage):
        # Relevant Motor Properties
        resistance = motorSpec[0]
        Kt = motorSpec[2]
        ke = Kt

        # Rotational speed of motor in rad/s
        omega = motorRPM * 2 * np.pi / 60

        # BLDC motors follow same curve as a DC motor, and vary V with PWM signals
        #   T = Kt * V / R - Ke * Kt * omega / R
        maxTorque = (Kt * maxVoltage / resistance) - (Ke * Kt * omega / resistance)

        return maxTorque

    #------------------------------------------------------------------------------#

    # returns (thrustX, thrustY, torque, power)
    def propProperties(self, motorRPM, propSpec, speed, alpha):
        # The diameter and pitch are from the given prop
        diameter, pitch = propSpec

        # Advance Ratio (in x direction)
        # Must convert RPM to rev / s and diameter from in to m
        J = np.cos(alpha) * speed / (motorRPM / 60 * diameter * 0.0254)

        thrustFunc = self.thrustDatabase[propSpec]
        Ct_static = thrustFunc(motorRPM)

        # Roughly Linear for J < 0.2
        Ct = Ct_static * (1 - J)

        powerFunc = self.powerDatabase[propSpec]
        # Roughly constant for J < 0.2 
        Cp = powerFunc(motorRPM) * (1 - 0.5*J)
        Cq = Cp / (2 * np.pi)

        # Calculate Thrust
        thrust = Ct * self.airDensity * (motorRPM / 60)**2 * (diameter * 0.0254)**4
        thrustX = np.sin(alpha) * thrust
        thrustY = np.cos(alpha) * thrust

        # Calculate back torque
        torque = Cq * self.airDensity * (motorRPM / 60)**2 * (diameter * 0.0254)**5

        # Calculate power required
        power = Cp * self.airDensity * (motorRPM / 60)**3 * (diameter * 0.0254)**5

        # Package everything nicely and return it
        return (thrustX, thrustY, torque, power)

    #------------------------------------------------------------------------------#

    # Calculates the required thrust to weight ratio for a given speed

    def thrustToWeight(self, speed):
        # Assume some thrust to weight ratio that increases linear with speed
        # Assumed because controlling vehicle requires ratio greater than 1.0
        #   Assumed 1.1 at hover and an additional 1.4 for every 10 m/s of speed
        # Ratio required due to fighting wind, balancing, etc
        # See references for scaling values
        return (1.1 + 0.7 * speed / 10)

    #------------------------------------------------------------------------------#

    def alpha(self, speed, weight, numMotors, motorRPM, propSpec):
        def alphaEval(x):
            alpha = x[0]
            thrustY = self.propProperties(motorRPM, propSpec, speed, alpha)[1]

            # Find the total thrust of the vehicle to hover
            verticalThrust = thrustY * numMotors   

            thrustToWeight = self.thrustToWeight(speed)
            artificialWeight = thrustToWeight * weight

            # netVerticalThrust is (weight - verticalThrust)
            #   increased, 'artifical',  weight is necessary for control of vehicle
            netVerticalThrust = artificialWeight - verticalThrust

            return abs(netVerticalThrust)

        alpha0 = np.pi/4
        bnds = ((0, np.pi/2),)

        opt = minimize(alphaEval, alpha0, method = 'SLSQP', 
                bounds = bnds, tol = 1e-10)

        return opt.x[0]

    #------------------------------------------------------------------------------#

    def eulerSpeed(self, weight, numMotors, numArms, motorRPM, propSpec):
        speed = 0
        mass = weight / 9.81
        timeStep = 0.01
        t = 0

        while (t < 0.5):
            alpha = self.alpha(speed, weight, numMotors, motorRPM, propSpec)

            drag = self.airDrag(speed, alpha, numArms)
            thrustX = self.propProperties(motorRPM, propSpec, speed, alpha)[0] 
            totalThrust = thrustX * numMotors

            netThrust = totalThrust - thrustX
            acceleration = netThrust / mass    
            speed += acceleration * timeStep

            t += timeStep

            if ((acceleration * timeStep) < 1e-6):
                break

        return speed



    #------------------------------------------------------------------------------#

    # Calculates the max speed of the vehicle in m/s
    # Returns all -1's is vehicle cannot sustain lift
    #   DO NOT FORGET THRUST:WEIGHT AT HOVER IS NOT 0
    # Returns all -1's for inputs that don't not converge after 1000 iterations

    # Calculates the steady state, steady level flight, speed and pitch angle of the 
    #   vehicle with the given given properties, with air resistance

    # Analysis assumes that at max speed all motors are operating at the same RPM. 

    # Inputs:
    #   weight is the weight of the vehicle in N 
    #   topArea and frontArea are the lateral areas of the vehicle from the top and 
    #       front respectively, in m^2
    #   numMotors is an integer representing the number of motors 
    #   motorRPM of all motors
    #   propSpec is a list of relevant data of the propeller.
    #       See start of document for info

    # Returns: (speed, alpha)
    #   speed is the final speed of the vehicle in m/s
    #   alpha is the pitch angle of the vehicle in rad

    def speed(self, weight, numMotors, numArms, motorRPM, propSpec):
        propDiameter, propPitch = propSpec[0:2]

        # Calculate the thrust at rest and no pitch angle to determine if the drone 
        #   can hover
        thrustX, thrustY = self.propProperties(motorRPM, propSpec, 0, 0)[0:2]

        # Find the total thrust of the vehicle to hover
        totalThrust = thrustY * numMotors

        # If the totalThrust is less than (thrust:weight * weight), it cannot hover
        if(totalThrust < (self.thrustToWeight(0) * weight)):
            return (-1, -1)

        # Function to be minimized
        def speedEvaluator(x):
            speed, alpha = x
            # Find thrust at this speed
            thrustX, thrustY = self.propProperties(motorRPM, propSpec, speed, 
                    alpha)[0:2]

            # Forces in the y:
            verticalThrust = thrustY * numMotors

            thrustToWeight = self.thrustToWeight(speed)
            artificialWeight = thrustToWeight * weight

            # netVerticalThrust is (weight - verticalThrust)
            #   increased, 'artifical',  weight is necessary for control of vehicle
            netVerticalThrust = artificialWeight - verticalThrust
        
            # Forces in the x:
            horizontalThrust = thrustX * numMotors
            drag = self.airDrag(speed, alpha, numArms)

            # Additional forces are from the drone fighting the wind, balancing, etc
            # See references for scaling values
            netHorizontalThrust = horizontalThrust - 0.05 * weight - drag

            return np.linalg.norm([netVerticalThrust, netHorizontalThrust])

        # Initial guesses
        speed0 = 10
        alpha0 = np.pi / 4

        # Physical constraints on the system:
        #   0 < speed
        #   0 < alpha < pi / 2    
        bnds = ((0, None), (0, np.pi/2),)

        opt = minimize(speedEvaluator, [speed0, alpha0], method = 'SLSQP', 
                bounds = bnds, tol = 1e-8)

        if (opt.success):
            speed, alpha = opt.x
            return (speed, alpha)
        
        else:
            return (-1, -1)

    #------------------------------------------------------------------------------#

#----------------------------------------------------------------------------------#

def optimalMotorRPM(weight, numMotors, numArms, motorSpec, propSpec, maxVoltage):
    (maxMotorTorue > aerodynamicWork[1])
    maximize (speed[0] / powerConsumption[0])
    powerConsumption[1] < motorSpec[4]





    