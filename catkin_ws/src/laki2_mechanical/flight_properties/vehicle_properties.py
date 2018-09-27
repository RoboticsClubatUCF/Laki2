from __future__ import division
import csv
import math
import copy
import numpy as np
import scipy
from scipy.optimize import minimize

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
    # Initialize by populating the thrust and power databases
    def __init__(self):
        # Assumed drag to be 1.6 for all configurations
        self.dragCoef = 1.6
        
        # Assume air density to be that of air at sea level
        self.airDensity = 1.225
        
        # Type of analysis ('lo' or 'hi' for lowest or highest thrust data)
        self.analysis = 'lo'

        # Create the thrust database (based off UIUC data)
        csvT = open('polyDataThrust.csv', 'r')
        rfT = csv.reader(csvT)

        # key is pitch, value is prop dictionary
        # each prop dictionary has 'lo' and 'hi' as keys and the respective 
        #   quadratic coefficients as the value
        self.thrustDatabase = dict()

        # Populate the thrust database
        for element in rfT:
            if (self.thrustDatabase.has_key(float(element[2]))):
                propData = self.thrustDatabase[float(element[2])]
            else:
                propData = dict()
            
            if (propData.has_key(element[6])):
                prop = propData[element[6]]
            else:
                prop = []

            # p is a numpy polynomial object
            # input is J, output is Ct
            p = np.poly1d((float(element[3]), float(element[4]), float(element[5])))

            # Append an tuple of (polynomial, (brand, diameter))
            # The (brand, diameter object is used by other methods to know what prop 
            #   from the database is being used
            prop.append((p, (element[0], element[1])))

            # Re-insert the data back into the database (possible not necessary the 
            #   way python does aliasing
            propData[element[6]] = prop
            self.thrustDatabase[float(element[2])] = propData

        # Create the power database (based off UIUC data)
        csvP = open('polyDataPower.csv', 'r')
        rfP = csv.reader(csvP)

        # key is pitch, then the value is a set of cubic functions for Cp
        self.powerDatabase = dict()

        # Populate the power database
        for element in rfP:
            pitch = float(element[2])

            if (self.powerDatabase.has_key(pitch)):
                powers = self.powerDatabase[pitch]
            else:
                powers = []

            # p is a numpy polynomial object
            # input is J, output is cubic function for Cp
            p = np.poly1d((float(element[3]), float(element[4]), 
                    float(element[5]), float(element[6])))

            # Insert set of power functions back into dictionary
            powers.append(p)
            self.powerDatabase[pitch] = powers

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

    def weight(self, numMotors, numArms, motorSpec, batteryCapacity, 
            numBatteryCells, propDiameter, isPayloadAttached):   
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

    def batteryCapacity(self, motorRPMs, motorSpec, numMotors, numArms, 
            numBatteryCells, propSpec, dists):
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
            powers[0], motorCurrents[0], escCurrents[0] = self.powerConsumption(
                motorRPMs[0], motorSpec, batteryVoltage, numMotors, propSpec, 
                speeds[0], alphas[0])

            powers[1], motorCurrents[1], escCurrents[1] = self.powerConsumption(
                motorRPMs[0], motorSpec, batteryVoltage, numMotors, propSpec, 
                speeds[0], alphas[0])

            timesOfFlight = [dists[0] / speeds[0], dists[1] / speeds[1]]

            energyForFlights = [timesOfFlight[0] * powers[0], 
                                timesOfFlight[1] * powers[1]]

            # Find the total energy used in the competition
            totalEnergy = energyForFlights[0] + energyForFlights[1]

            # Value to optimize to 0
            remainingEnergy = batteryEnergy - totalEnergy

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
            return (capacity, motorCurrents, escCurrents, speeds, canHovers, 0)
        else:
            return (capacity, motorCurrents, escCurrents, speeds, canHovers, 1)

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
        print J

        # Initialize coefficient of thrust to inf or -inf based on the type of 
        #   analysis. (lowest thrust or highest thrust)
        # Index 0 is the Ct in the X direction, index 1 is in the Y Direction
        if (self.analysis == 'lo'):
            Ct = np.inf
        else:
            Ct = -np.inf

        # PropKey is the (brand, UIUC_diameter) where the data originated
        propKey = []

        # There was one data point for 4.5" and 5.5", and no data for 6.5"
        # Relationship w/ pitch is linear, average +/- pitch of 0.5 above and below
        if ((pitch == 4.5) or (pitch == 5.5) or (pitch == 6.5)):
            # For those pitch values, use the average of the pitch above and below
            lower = pitch - 0.5
            higher = pitch + 0.5

            # Ct1 is Ct of the pitch slightly lower, Ct2 is the Ct slightly above
            if (self.analysis == 'lo'):
                Ct1 = np.inf
                Ct2 = np.inf
            else:
                Ct1 = -np.inf
                Ct2 = -np.inf

            # Take the data from the props below the current pitch
            prop = self.thrustDatabase[lower]
            data = prop[self.analysis]

            # Loop thru all props of lower pitch to find the lowest or highest val
            #   (Depending on if the analysis wants 'lo' or 'hi')
            for datum in data:
                newCt = datum[0](J)

                if (self.analysis == 'lo'):
                    if (newCt < Ct1): 
                        Ct1 = newCt
                        propKey = [datum[1], None]
                    
                else:
                    if (newCt > Ct1): 
                        Ct1 = newCt
                        propKey = [datum[1], None]

            # Take the data from the props above the current pitch
            prop = self.thrustDatabase[higher]
            data = prop[self.analysis]

            # Loop thru all props of higher pitch to find the lowest or highest val
            #   (Depending on if the analysis wants 'lo' or 'hi')
            for datum in data:
                newCt = datum[0](J)

                if (self.analysis == 'lo'):
                    if (newCt < Ct2): 
                        Ct2 = newCt
                        propKey[1] = datum[1]
                    
                else:
                    if (newCt > Ct2): 
                        Ct2 = newCt
                        propKey[1] = datum[1]

            # One possible value of Ct is average of the pitch just below and just 
            #   above the current pitch
            Ct = (Ct1 + Ct2) / 2

        # There was no data for a prop with pitch 6.5"
        if (pitch != 6.5):
            # Take the data from the props below the current pitch
            prop = self.thrustDatabase[pitch]
            data = prop[self.analysis]

            # Loop thru all props of pitch to find the lowest or highest value
            #   (Depending on if the analysis wants 'lo' or 'hi')
            for datum in data:
                newCt = datum[0](J)

                if (self.analysis == 'lo'):
                    if (newCt < Ct): 
                        Ct = newCt
                        propKey = [datum[1]]
                    
                else:
                    if (newCt > Ct): 
                        Ct = newCt
                        propKey = [datum[1]]

        # Thrust from the standard propeller model
        thrustTotal = (Ct * self.airDensity * (motorRPM / 60)**2 
                        * (diameter * 0.0254)**4)

        # Thrust in the X and Y directions
        # Return Values:
        thrustX = thrustTotal * np.sin(alpha)
        thrustY = thrustTotal * np.cos(alpha)

        if (self.analysis == 'lo'):
            Cp = np.inf
            Cp1 = np.inf
            Cp2 = np.inf
        else:
            Cp = -np.inf
            Cp1 = -np.inf
            Cp2 = -np.inf
        
        if ((pitch == 4.5) or (pitch == 5.5) or (pitch == 6.5)):
            lower = pitch - 0.5
            higher = pitch + 0.5

            powers = self.powerDatabase[lower]
            for CpFunc in powers:
                if (self.analysis == 'lo'):
                    Cp1 = min(CpFunc(J), Cp1)                
                        
                else:
                    Cp1 = max(CpFunc(J), Cp1)

            powers = self.powerDatabase[higher]
            for CpFunc in powers:
                if (self.analysis == 'lo'):
                    Cp2 = min(CpFunc(J), Cp2)                
                        
                else:
                    Cp2 = max(CpFunc(J), Cp2)

            Cp = (Cp1 + Cp2) / 2

        else:
            powers = self.powerDatabase[pitch]
            for CpFunc in powers:
                if (self.analysis == 'lo'):
                    Cp = min(CpFunc(J), Cp)                
                        
                else:
                    Cp = max(CpFunc(J), Cp)

        # Power from the standard propeller model
        power = Cp * self.airDensity * (motorRPM / 60)**3 * (diameter * 0.0254)**5

        # Torque is just Cp / (2*pi) 
        Cq = Cp / (2 * np.pi)

        # Torque from the standard propeller model
        torque = Cq * self.airDensity * (motorRPM / 60)**2 * (diameter * 0.0254)**5

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
        return (1.1 + 1.4 * speed / 10)

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

            # netVerticalThrust is (weight - verticalThrust)
            #   increased, 'artifical',  weight is necessary for control of vehicle
            netVerticalThrust =  thrustToWeight * weight - verticalThrust
        
            # Forces in the x:
            horizontalThrust = thrustX * numMotors
            drag = self.airDrag(speed, alpha, numArms)

            # Additional forces are from the drone fighting the wind, balancing, etc
            # See references for scaling values
            netHorizontalThrust = horizontalThrust - 8.14 * thrustToWeight * drag

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

    # Calculates the total power and current consumption per motor of the vehicle

    # Inputs:
    #   motorRPM and numMotors are the motor RPM and number of motors respectively
    #   propSpec is a list of relevant data of the propeller.
    #       See start of document for info    
    #   voltage is the voltage being supplied to the motors
    #   speed is the vehicle speed in m/s
    #   alpha is the angle of the vehicle in radians

    # returns (totalPower, motorCurrent, escCurrent)

    def powerConsumption(self, motorRPM, motorSpec, voltage, numMotors, propSpec, 
            speed, alpha):
        # Propeller Information
        propDiameter, propPitch = propSpec[0:2]

        # Mechanical work done by the prop
        aeroTorque, aeroWork = self.propProperties(motorRPM, propSpec, speed, alpha)[2:4]
        print aeroTorque, aeroWork


        # Motor Current is torque / Kt
        # This is the current required with no losses
        motorCurrent = aeroTorque / motorSpec[2]

        # Voltage is regulated by the ESC
        motorVoltage = aeroWork / motorCurrent

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
                (aeroWork + noLoadPower)])

        # Calculate the actual power pulled from the roots:
        # If by some miracle the parabola is perfectly placed on the x-axis, that is 
        #   the current draw
        if (len(motorCurrents) == 1):
            motorCurrent = motorCurrents[0]
        
        # If the solution is complex, the current draw is infinite
        elif (not np.isreal(motorCurrents).any()):
            motorCurrent = np.inf
        
        # If both currents are valid, the smaller one is valid
        elif (motorCurrents[0] > 0 and motorCurrents[1] > 0):
            motorCurrent = np.min(motorCurrents)
            
        # If both currents are negative, something is wrong
        elif (motorCurrents[0] < 0 and motorCurrents[1] < 0):
            return (np.inf, np.inf)
          
        # To reach this point, the roots must be real and exactly one is 
        #   non-negative
        else:
            motorCurrent = max(motorCurrents)

        # Power = V * I
        motorPower = motorVoltage * motorCurrent

        # Current into the esc    
        escCurrent = motorPower / voltage

        # Resistive losses of the esc (approximate)
        escLosses = escCurrent**2 * 0.005

        # The total power is the sum of all motors
        totalPower = motorPower * numMotors + escLosses

        # Add liberal estimate power required by all electronics
        totalPower += 50

        return (totalPower, motorCurrent, escCurrent)

#----------------------------------------------------------------------------------#

def optimalMotorRPM(weight, numMotors, numArms, motorSpec, propSpec, maxVoltage):
    (maxMotorTorue > aerodynamicWork[1])
    maximize (speed[0] / powerConsumption[0])
    powerConsumption[1] < motorSpec[4]





    