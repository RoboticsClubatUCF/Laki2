def motorRPM(voltage, motorSpec, propDiameter, propPitch, speed, alpha):
    # Motor Torque is of the following form:
    #   Q = (Kt * V / R) - (Ke * Kt * omega / R)

    # Counter Torque from the propeller is derived as following:
    #   Lateral force at any cross section of the prop:
    #       Assuming a L/D of 5 at each section of the prop:
    #           Ve^2 / V_d^2 = 5
    #       F = m_dot * (omega * r) - m_dot * (V_d)
    #           m_dot = 0.5 * rho * pi * r^2 * (V_e + sin(alpha) * V_ac)
    #       
    #       F = 0.5 * rho * pi * (V_e + sin(alpha) * V_ac) * 
    #           (omega * r^3 - V_d * r^2)

    #   Small change in radius produces the following small change in force:
    #       dF = 0.5 * rho * pi * (V_e + sin(alpha) * V_ac) * 
    #           (omega * r^3 - V_d * r^2) dr
    
    #   Torque is sum of all (forces * r):
    #       Q = integral of dF from 0 to r_0
    #       *Assume propeller produces lift all the way to the center

    #   Total Torque:
    #       Q = 0.5 * rho * pi * (V_e + sin(alpha) * V_ac) * 
    #                               (1/4 * omega * r^4 - 1/3 * V_d * r^3)

    #       V_d^2 = V_e^2 / sqrt(5) 
    #       TO DO: Fix this no slip assumption when it is fixed in thrust
    #       *Assuming 20% slip:
    #          V_e = 0.8 * omega * pitch

    #       Q = 0.5 * rho * pi * (0.8 * omega * pitch + sin(alpha) * V_ac) *
    #          ((1/4 * omega * r^4) - (1/3 * 1/sqrt(5) * 0.8 * omega * pitch * r^3))

    #       Q = 0.5 * rho * pi * ((r^4 / 4) - (0.8 * pitch * r^3 / (3 * sqrt(5))) * 
    #           (0.8 * omega^2 * pitch + omega * sin(alpha) * V_ac)

    # Arbitrary parameter to store big coefficient from propeller torque eqn:
    #   beta = 0.5 * p * pi * ((r^4 / 4) - (0.8 * pitch * r^3 / (3 * sqrt(5))))
    
    #   Q = beta * (0.8 * P * omega^2 + V_ac * sin(alpha) * omega) 

    # At Steady State Q_motor = Q_prop

    # Define and calculate motor constants:
    # Motor Internal Resistance
    resistance = motorSpec[0]

    # Motor Velocity Constant
    Kv = motorSpec[1]

    # Back emf Constant
    Ke = 1 / (2 * np.pi * Kv / 60)

    # Torque Constant (assume a linear model for the moter)
    Kt = Ke

    # Define propeller constants:
    # Prop Radius in m:
    r = 0.0254 * propDiameter / 2 

    # Prop Pitch in m / (rad/s)
    pitch = propPitch * 0.0254 / (2 * np.pi / 60)

    print "Kv: ", Kv, "Ke: ", Ke, "Kt: ", Kt, "r: ", r, "pitch: ", pitch

    # Arbitrary parameter to store big coefficient from propeller torque eqn:
    beta = 0.5 * airDensity * np.pi * (
            (r**4 / 4)) #- (0.8 * pitch * r**3 / (3 * np.sqrt(5))))

    print "beta: ", beta

    # Solve for the roots of the quadratic torque equation
    quadTerm = beta * 0.8 * pitch
    linTerm  = (beta * np.sin(alpha) * speed) + (Ke * Kt / resistance)
    consTerm = Kt * voltage / resistance

    # Motor speed in rad/s
    omegas = np.roots([quadTerm, linTerm, -consTerm])

    print "omegas: ", omegas

    # If by some miracle the parabola is perfectly placed on the x-axis, that is the 
    #   motor speed
    if (len(omegas) == 1):
        omega = omegas[0]
    
    # If the solution is complex, the current draw is infinite
    elif (not np.isreal(omegas).any()):
        omega = float('inf')
    
    # If both currents are valid, the smaller one is valid
    elif (omegas[0] > 0 and omegas[1] > 0):
        omega = np.min(omegas)
        
    # If both currents are negative, something is wrong
    elif (omegas[0] < 0 and omegas[1] < 0):
        return -1
      
    # To reach this point, the roots must be real and exactly one is non-negative
    else:
        omega = max(omegas)

    # Calculate the torque required (should be same as prop eqn, but this is easier)
    Q = (Kt * voltage / resistance) - (Ke * Kt * omega / resistance)

    # Conver omega from rad/s to RPM
    omega *= 2 * np.pi

    return (omega, Q)
