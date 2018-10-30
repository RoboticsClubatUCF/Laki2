function [motorRPM, current] = motorRPMAtSteadyLevelFlight(weight, numBatteryCells, numMotors, numArms, propDiameter, propPitch, targetSpeed, maxCurrent, isStacked)
    % Find the max motor RPM and max speed
    maxMotorRPM = lookupMaxMotorRPM(numBatteryCells);
    maxSpeed = speedCalculator(weight, numMotors, numArms, maxMotorRPM, propDiameter, propPitch);

    % If at max motor RPM, it cannot achieve the required speed, it is not
    %   possible to achieve that speed
    if (maxSpeed < targetSpeed)
        motorRPM = -1;
        current = -1;
        return
    end
    
    %% Bisection (because speedCalculator is numerical)
    minMotorRPM = 0;
    motorRPM = 0;
    
    convError = 1;
    absError = 1;
    iter = 0;
    
    while(convError > 1e-10 && absError > 1e-8)
        % Usually takes about 50 iterations to converge, if it is taking 1000 
        %   something is wronge
        iter = iter + 1;
        if (iter > 1e3)
            break
        end
        
        motorRPM_old = motorRPM;
        motorRPM = (maxMotorRPM + minMotorRPM)/2;
        
        convError = abs(motorRPM_old - motorRPM);
        [speed, alpha] = speedCalculator(weight, numMotors, numArms, motorRPM, propDiameter, propPitch);
        
        % If speed is less than the target speed, motor RPM is too low
        if (speed < targetSpeed)
            minMotorRPM = motorRPM;
        else
            maxMotorRPM = motorRPM;
        end
    end
   
    batteryVoltage = numBatteryCells * 3.7;
    [~, current] = powerConsumptionCalculator(motorRPM, numMotors, propDiameter, propPitch, speed, alpha, isStacked, batteryVoltage);

    % If at that speed, it required a current pull greater than allowed, it
    %   is not possible to travel at this speed
    if (current > maxCurrent)
        motorRPM = -1;
        current = -1;
        return
    end
    
end

