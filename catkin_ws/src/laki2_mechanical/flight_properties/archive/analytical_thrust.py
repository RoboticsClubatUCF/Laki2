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