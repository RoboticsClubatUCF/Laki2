import csv
import numpy as np

analysis = ['lo']
airDensity = 1.225

csvf = open('polyDataThrust.csv', 'r')
rf = csv.reader(csvf)

# key is pitch, value is prop dictionary
# each prop dictionary has lo/hi as the key and the quadratic coefs as the value
thrustDatabase = dict()

for element in rf:
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

    prop.append(p)

    propData[element[6]] = prop
    thrustDatabase[float(element[2])] = propData

propData = None
prop = None

def thrust(motorRPM, propSpec, speed, alpha, isStacked):
    diameter, pitch = propSpec

    # Advance Ratio (in x direction)
    # Must convert RPM to rev / s and diameter from in to m
    J = np.cos(alpha) * speed / (motorRPM / 60 * diameter * 0.0254)

    # Initialize coefficient of thrust to inf or -inf based on the type of analysis
    # Index 0 is the Ct in the X direction, index 1 is in the Y Direction
    if (analysis[0] == 'lo'):
        Ct = [np.inf, np.inf]
    else:
        Ct = [-np.inf, -np.inf]

    # There was one data point for 4.5" and 5.5", and no data for 6.5"
    # Relationship with pitch is linear, average +/- pitch of 0.5 above and below
    if ((pitch == 4.5) or (pitch == 5.5) or (pitch == 6.5)):
        lower = pitch - 0.5
        higher = pitch + 0.5

        # Ct1 is the Ct of the pitch slightly lower, Ct2 is the Ct slightly above
        if (analysis[0] == 'lo'):
            Ct1 = [np.inf, np.inf]
            Ct2 = [np.inf, np.inf]
        else:
            Ct1 = [-np.inf, -np.inf]
            Ct2 = [-np.inf, -np.inf]

        prop = thrustDatabase[lower]
        data = prop[analysis[0]]

        for datum in data:
            if (analysis[0] == 'lo'):
                Ct1[0] = min(datum(J), Ct1[0])
                Ct1[1] = min(datum(0), Ct1[1])
            else:
                Ct1[0] = max(datum(J), Ct1[0])
                Ct1[1] = max(datum(0), Ct1[1])

        prop = thrustDatabase[higher]
        data = prop[analysis[0]]

        for datum in data:
            if (analysis[0] == 'lo'):
                Ct2[0] = min(datum(J), Ct2[0])
                Ct2[1] = min(datum(0), Ct2[1])
            else:
                Ct2[0] = max(datum(J), Ct2[0])
                Ct2[1] = max(datum(0), Ct2[1])

        # One possible value of val is average of the pitch just below and just 
        #   above the current pitch
        Ct[0] = (Ct1[0] + Ct2[0]) / 2
        Ct[1] = (Ct1[1] + Ct2[1]) / 2

    if (pitch != 6.5):
        prop = thrustDatabase[pitch]
        data = prop[analysis[0]]

        for datum in data:
            if (analysis[0] == 'lo'):
                Ct[0] = min(datum(J), Ct[0])
                Ct[1] = min(datum(0), Ct[1])
            else:
                Ct[0] = max(datum(J), Ct[0])
                Ct[1] = max(datum(0), Ct[1])

    # Oblique angle applied to the standard propeller model
    thrustX = Ct[0] * airDensity * (motorRPM / 60)**2 * (diameter * 0.0254)**4
    thrustX *= np.sin(alpha)

    thrustY = Ct[1] * airDensity * (motorRPM / 60)**2 * (diameter * 0.0254)**4
    thrustY *= np.cos(alpha)

    return (thrustX, thrustY)

print thrust(3000, [18, 6.5], 0, 0, 0)