% Returns the minimum radius of turn the vehicle is capable of achieving

function [radiusOfTurn, maxMotorRPM] = radiusOfTurnCalculator(weight, numBatteryCells, numMotors, numArms, propDiameter, propPitch, speed, maxCurrent, isStacked)
    % Find the motor RPM to produce steady level flight at that speed
    steadyLevelFlightMotorRPM = motorRPMAtSteadyLevelFlight(weight, numBatteryCells, numMotors, numArms, propDiameter, propPitch, speed, maxCurrent, isStacked);

    % Find the alpha at steady level flight
    alpha = alphaCalculator(weight, numMotors, steadyLevelFlightMotorRPM, propDiameter, propPitch, speed, isStacked);    

    % Find the max RPM in order to generate the most thrust possible
    maxMotorRPM = maxMotorRPMAtSpeedCalculator(numBatteryCells, numMotors, numArms, propDiameter, propPitch, speed, alpha, maxCurrent);

    % steady level flight motor RPM will be -1 if it is not possible to
    %   achieve that speed
    if (steadyLevelFlightMotorRPM < 0)
        radiusOfTurn = Inf;
        return
    end
    
    % Find the thrust produced at this speed to sustain lift and speed
    [thrustX, thrustY] = thrustCalculator(steadyLevelFlightMotorRPM, propDiameter, propPitch, speed, alpha, isStacked);
        
    % Find the maximum thrust producible at this speed (excess thrust
    %   causes roll, not pitch, so no change in alpha
    [maxThrustX, maxThrustY] = thrustCalculator(maxMotorRPM, propDiameter, propPitch, speed, alpha, isStacked);

    % Total thrust produced to sustain list and speed
    totalThrust = norm([thrustX; thrustY]) * numMotors;

    % Maximum thrust producible at this speed
    totalMaxThrust = norm([maxThrustX; maxThrustY]) * numMotors;

    % To turn, only half the motors are at max thrust, 
    %   so only half the excess thrust is usable.
    % excess thrust = max thrust - thrust required to sustain steady level flight
    excessThrust = (totalMaxThrust - totalThrust) / 2; 
    
    % Load Factor
    n = (excessThrust + weight) / weight;
    
    % Gravity
    g = 9.81;
    
    % Radius of Turn only valid for vehicles with excess thrust
    if (n < 1)
        radiusOfTurn = Inf;
    else
        % https://goo.gl/oy8vzW
        radiusOfTurn = speed^2 / (g * sqrt(n^2 - 1));
    end
end

