function [radius, speed, motorRPM] = fastestTurnCalculator(weight, numBatteryCells, numMotors, numArms, propDiameter, propPitch, maxSpeed, maxCurrent)
    isStacked = ~(numMotors == numArms);   
    
    % Do newton's method to find speed that minimizes time
    iter = 0;
    convError = 1;
    absError = 1;
    
    % Initial guesses
    
    
    % Use Newton's Method to solve for alpha and speed
    while(absError > 1e-8 || convError > 1e-10)
        % Test runs converged within about 9 iterations. If it takes more
        %   than 1000 something is wrong
        iter = iter + 1;        
        if (iter > 1000)
            return
        end
    
        % Find the max RPM in order to generate the most thrust possible
        maxMotorRPM = maxMotorRPMAtSpeedCalculator(numBatteryCells, numMotors, numArms, propDiameter, propPitch, speed, alpha, maxCurrent);

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
        usefulTotalMaxThrust = ((totalMaxThrust - totalThrust) / 2) + totalThrust; 

        % Load Factor
        n = usefulTotalMaxThrust / totalThrust;

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

end