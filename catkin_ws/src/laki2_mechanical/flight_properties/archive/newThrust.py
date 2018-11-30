import csv
import numpy as np

analysis = 'lo'
airDensity = 1.225

csvT = open('polyDataThrust.csv', 'r')
rfT = csv.reader(csvT)

# key is pitch, value is prop dictionary
# each prop dictionary has 'lo' and 'hi' as keys and the respective quadratic 
#   coefficients as the value
thrustDatabase = dict()

# Populate the thrust database
for element in rfT:
    if (thrustDatabase.has_key(float(element[2]))):
        propData = thrustDatabase[float(element[2])]
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
    # The (brand, diameter object is used by other methods to know what prop from
    #   the database is being used
    prop.append((p, (element[0], element[1])))

    # Re-insert the data back into the database (possible not necessary the way
    #   python does aliasing
    propData[element[6]] = prop
    thrustDatabase[float(element[2])] = propData

# Reset propData and prop for debugging purposes
propData = None
prop = None

csvP = open('polyDataPower.csv', 'r')
rfP = csv.reader(csvP)

# key is (brand, pitch,  UIUC_diameter), then the value is cubic function for Cp
powerDatabase = dict()

# Populate the power database
for element in rfP:
    # p is a numpy polynomial object
    # input is J, output is cubic function for Cp
    p = np.poly1d((float(element[3]), float(element[4]), 
            float(element[5]), float(element[6])))

    # Each key is 1-1 from the spreadsheet
    # Insert p for the (brand, pitch, UIUC_diameter)
    powerDatabase[(element[0], float(element[1]), float(element[2]))] = p

# Reset propData and prop for debugging purposes
propData = None
prop = None

#----------------------------------------------------------------------------------#


def propProperties(motorRPM, propSpec, speed, alpha):
    # The diameter and pitch are from the given prop
    diameter, pitch = propSpec

    # Advance Ratio (in x direction)
    # Must convert RPM to rev / s and diameter from in to m
    J = np.cos(alpha) * speed / (motorRPM / 60 * diameter * 0.0254)

    # Initialize coefficient of thrust to inf or -inf based on the type of analysis
    # Index 0 is the Ct in the X direction, index 1 is in the Y Direction
    if (analysis == 'lo'):
        Ct = np.inf
    else:
        Ct = -np.inf

    # PropKey is the (brand, UIUC_diameter) where the data originated
    propKey = []

    # There was one data point for 4.5" and 5.5", and no data for 6.5"
    # Relationship with pitch is linear, average +/- pitch of 0.5 above and below
    if ((pitch == 4.5) or (pitch == 5.5) or (pitch == 6.5)):
        # For those pitch values, use the average of the pitch above and below
        lower = pitch - 0.5
        higher = pitch + 0.5

        # Ct1 is the Ct of the pitch slightly lower, Ct2 is the Ct slightly above
        if (analysis == 'lo'):
            Ct1 = np.inf
            Ct2 = np.inf
        else:
            Ct1 = -np.inf
            Ct2 = -np.inf

        # Take the data from the props below the current pitch
        prop = thrustDatabase[lower]
        data = prop[analysis]

        # Loop thru all props of the lower pitch to find the lowest or highest value
        #   (Depending on if the analysis wants 'lo' or 'hi')
        for datum in data:
            newCt = datum[0](J)

            if (analysis == 'lo'):
                if (newCt < Ct1): 
                    Ct1 = newCt
                    propKey = [datum[1], None]
                
            else:
                if (newCt > Ct1): 
                    Ct1 = newCt
                    propKey = [datum[1], None]

        # Take the data from the props above the current pitch
        prop = thrustDatabase[higher]
        data = prop[analysis]

        # Loop thru all props of the lower pitch to find the lowest or highest value
        #   (Depending on if the analysis wants 'lo' or 'hi')
        for datum in data:
            newCt = datum[0](J)

            if (analysis == 'lo'):
                if (newCt < Ct2): 
                    Ct2 = newCt
                    propKey[1] = datum[1]
                
            else:
                if (newCt > Ct2): 
                    Ct2 = newCt
                    propKey[1] = datum[1]

        # One possible value of val is average of the pitch just below and just 
        #   above the current pitch
        Ct = (Ct1 + Ct2) / 2

    # There was no data for a prop with pitch 6.5"
    if (pitch != 6.5):
        # Take the data from the props below the current pitch
        prop = thrustDatabase[pitch]
        data = prop[analysis[0]]

        # Loop thru all props of the lower pitch to find the lowest or highest value
        #   (Depending on if the analysis wants 'lo' or 'hi')
        for datum in data:
            newCt = datum[0](J)

            if (analysis == 'lo'):
                if (newCt < Ct): 
                    Ct = newCt
                    propKey = [datum[1]]
                
            else:
                if (newCt > Ct): 
                    Ct = newCt
                    propKey = [datum[1]]

    # Thrust from the standard propeller model
    thrustTotal = Ct * airDensity * (motorRPM / 60)**2 * (diameter * 0.0254)**4

    # Thrust in the X and Y directions
    thrustX = thrustTotal * np.sin(alpha)
    thrustY = thrustTotal * np.cos(alpha)

    # Coefficient of power is based off the propeller used
    CpFunc = powerDatabase[(propKey[0], pitch, propKey[1])]
    Cp = CpFunc(J)
    
    # Torque is just Cp / (2*pi) 
    Cq = Cp / (2 * np.pi)

    # Torque from the standard propeller model
    torque = Cq * airDensity * (motorRPM / 60)**2 * (diameter * 0.0254)**5

    # Power from the standard propeller model
    power = aeroTorque * (motorRPM / 60)

    # Package everything nicely and return it
    return (thrustX, thrustY, torque, power)


print thrust(3000, [18, 6.5], 0, 0, 0)