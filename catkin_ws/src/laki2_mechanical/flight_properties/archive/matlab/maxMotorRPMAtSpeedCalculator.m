%% Find max RPM limited by Kv and current

function [motorRPM, current] = maxMotorRPMAtSpeedCalculator(numBatteryCells, numMotors, numArms, propDiameter, propPitch, speed, alpha, maxCurrent)
% Get other Prioris based on Vehicle Configuration Information
isStacked = ~(numMotors == numArms);
batteryVoltage = numBatteryCells * 3.7;

%% Bisection method (because alpha is numerically calculated)
convErrorTol = 1e-8;
absErrorTol = 1e-6;

% Kv is an upper limit
maxMotorRPM = lookupMaxMotorRPM(numBatteryCells);
minMotorRPM = 0;
motorRPM = 0;

convError = 1;
absError = 1;
iter = 0;

while(convError > convErrorTol && absError > absErrorTol)
    iter = iter + 1;
    if (iter > 1000)
        break
    end

    motorRPM_old = motorRPM;
    motorRPM = (maxMotorRPM + minMotorRPM)/2;
    
    % Calculate current pull
    [~, current] = powerConsumptionCalculator(motorRPM, numMotors, propDiameter, propPitch, speed, alpha, isStacked, batteryVoltage);

    % Update Numerical Error Values
    convError = abs(motorRPM_old - motorRPM);
    absError = abs(current - maxCurrent);

    % If current pull > max current, RPM is too high
    if (current > maxCurrent)
        maxMotorRPM = motorRPM;
    else
        minMotorRPM = motorRPM;
    end
end

% Calculate current after iterations to make sure value returned uses the
%   motor RPM that is being returned
[~, current] = powerConsumptionCalculator(motorRPM, numMotors, propDiameter, propPitch, speed, alpha, isStacked, batteryVoltage);

end