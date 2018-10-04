% Calculates the total power and current consumption per motor of the given vehicle

% motorRPM and numMotors are the motor RPM and number of motors
%   respectively
% propDiameter and propPitch are the propeller diameter and pitch
%   repectively, in inches
% speed is the vehicle speed in m/s
% alpha is the angle of the vehicle in radians
% isStacked is a boolean, 0 for flat frames, 1 else
% batteryVoltage is the voltage of the battery in V

% TO DO: Add a functionality for motor resistance
% TO DO: make motor lookup return max rpm & resistance

function [power, current] = powerConsumptionCalculator(motorRPM, numMotors, propDiameter, propPitch, speed, alpha, isStacked, batteryVoltage)
    % Calculate the thrust produced by the vehicle, and the exit velocity
    %   of the air it produces
    [thrustX, thrustY, exitVelocity] = thrustCalculator(motorRPM, propDiameter, propPitch, speed, alpha, isStacked);
    totalThrustX = thrustX * numMotors;
    totalThrustY = thrustY * numMotors;
    totalThrust = sqrt(totalThrustX^2 + totalThrustY^2);
    
    % If the x thrust is negative, it is actually generating power
    % Don't use sign because if thrustX is 0, Y direction can still be using power
    if (totalThrustX < 0)
        totalThrust = totalThrust * -1;
    end
    
    % Vp, 0.5*(Ve + Vo) in the canonical case of the vehicle speed 
    %   perpendicular to the oncoming aire
    propellerVelocity = 0.5 * (exitVelocity + sin(alpha) * speed);
    
    % Power required by a propeller with no frictional or swirl effects
    propPowerOpt = totalThrust * propellerVelocity;
    
    % Assume the linear drag coefficient of the propeller to be roughly
    %   that of what is expected from a slender, traditional airfoil
    linDragCoef = 0.035;
    
    % Assume air density to be 1.225
    airDensity = 1.225;
        
    % Frictional torque produced by drag on the propeller
    counterTorque = 0.2 * pi * linDragCoef * airDensity * (propDiameter/2 * 0.0254)^5 * (motorRPM / 60 * 2 * pi);%^2;
    
    % Total losses of the propeller
    % Does not include swirl, or viscous shear
    propPowerLoss = counterTorque * (motorRPM / 60 * 2 * pi);
    
    % Total power consumed by the propeller
    propPower = propPowerOpt + propPowerLoss;
    
    % Losses due to the losses of the motor are of the following form:
    % P_in = P_out + I^2 * R_o + I_o * V
    % where I^2 * R_m are the "copper losses" due to the no load resistance
    % of the motor, controller, and battery
    % and I_o * V are the "iron losses" are due to the no load current required by the motor to produce 0 work
    
    % Assumed to be constant, but make a spreadsheet of motor Kv,
    % resistance, etc
    motorResistance = 0.172;
    
    % Assumed controller resistance
    controllerResistance = 0.0004;
    
    % Assumed battery resistance
    batteryResistance = 0.0006 / numMotors;
    
    % Total internal resistance of components from battery to motor
    totalResistance = motorResistance + controllerResistance + batteryResistance;
    
    % Assumed the required no load power of the motor is that given on the
    % data sheet, this should also be spreadsheet driven
    noLoadPower = 0.5 * 10;
        
    propPower = propPower + noLoadPower;
    
    % The RMS voltage of the controller is equal to the battery voltage, so
    % P_in = V * I
    
    current = propPower / batteryVoltage;
    
    % Add the copper losses (first degree approximation)
    power = propPower + current^2 * totalResistance;
    
    % Recalculate the music
    current = power / batteryVoltage;
    
    % The solution to the equation can have either 2 real or 2 imaginary
    % solutions. If they are imaginary, the current required for the
    % internal resistance is unstable and theoretically infinite
    
    %{
    current = roots([totalResistance, -batteryVoltage, propPower]);
    
    if ~isreal(current)
        current = inf;
    else
        % If both currents are valid, the smaller one is valid
        if (current(1) > 0 && current(2) > 0)
            current = min(current);
        else
            % If only one solution is positive, it is the valid one
            
            if (current(1) > 0 && current(2) < 0)
                current = current(1);
            
            elseif (current(1) < 0 && current(2) > 0)
                current = current(2);
            
            % If both solutions are negative there is no real solution
            else
                current = inf;
            end
        end
    end
    
    power = current * batteryVoltage;
    %}
        
    % Add a conservative estimate power of all electronics
    power = power + 50;
end