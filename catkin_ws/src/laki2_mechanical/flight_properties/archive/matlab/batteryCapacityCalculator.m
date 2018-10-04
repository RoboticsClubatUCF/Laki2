% Calculates the Required Number of Batteries and the Time of Flight

% speeds, and currents are 2 x 1 vectors with the pre and post  speeds 
%   and currents
% canHover is a 2 x 1 boolean vector denoting if it is possible to hover
%   pre and post 
% batteryCapacity is the mAh required to complete the competition

% motorRPMPre, and motorRPMPost
% numMotors, numArms, numBatteryCells, are the number of motors, number of
%   arms, and number of battery cells respectively
% propDiameter, and propPitch are the propeller diameter and pitch
%   respectively, in inches
% distPre, and distPost are the distances needed to travel by the
%   vehicle pre and post payload , in meters
%   command window, 1 to allow, 0 to disallow

function [batteryCapacity, totalTimeOfFlight, currents, speeds, canHover] = batteryCapacityCalculator(motorRPMPre, motorRPMPost, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost)
    %% Constants:
    % isStacked is based off the given number of motors and arms
    isStacked = ~(numMotors == numArms);
    
    % Battery Voltage in V
    batteryVoltage = 3.7 * numBatteryCells;
    
    % Ridiculous number of batteries as an upper limit to the optomization
    % Set max at 100 Ah capacity
    maxBatteryCapacity = 100000;

    %% Optomize number of batteries for the given vehicle configuration
    minBatteryCapacity = 0;
    convError = 1;
    absError = 1;
    remainingEnergy = 1;
    batteryCapacity = 0;
    
    currentPre = -1;
    currentPost = -1;
    totalTimeOfFlight = -1;
    speedPre = -1;
    speedPost = -1;
    iter = 0;
    
    % Uses a bisection method instead of Newton or Gradient descent because 
    %   the speedCalculator is itself a numerical solver, and thus has no
    %   derivatives
    % This also allows for an inability to hover to restrict the bounds
    while (convError > 1e-10 && absError > 1e-8)
        % Usually takes about 50 iterations to converge, if it takes more something 
        %   is wrong
        iter = iter + 1;
        if (iter > 1e3)
            break;
        end
        
        % Calculate updated values and errors
        batteryCapacity_old = batteryCapacity;
        batteryCapacity = (minBatteryCapacity + maxBatteryCapacity)/2;
        
        convError = abs(batteryCapacity_old - batteryCapacity);
        absError = abs(remainingEnergy);
        
        % Battery Energy
        %   Assumed landing with 20% of battery energy (SF 1.25)
        %   Assumed 95% depth of discharge
        %   Assumed 90% battery efficiency (actual charge / theoretical charge)
        batteryEnergy = (1.00 - 0.20) * 0.95 * 0.9 * batteryVoltage * (batteryCapacity / 1000 * 60 * 60);

        % Weight changes as a function of the number of batteries and payload attachment
        isPayloadAttached = 1;
        weightPre = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached);

        % Find speed and angle at which the drone will fly
        [speedPre, alphaPre] = speedCalculator(weightPre, numMotors, numArms, motorRPMPre, propDiameter, propPitch);
        
        canHover = [0, 0];
        
        % speedCalculator returns -1's if it cannot hover
        % Cannot hover if too heavy, the only way this function can make
        %   reduce weight is to reduce the number of batteries 
        if (speedPre == -1)
            maxBatteryCapacity = batteryCapacity;
            canHover(1) = 0;
            continue
        else
            canHover(1) = 1;
        end
        
        % Calculate energy required to finish this leg of the competition
        [powerPre, currentPre] = powerConsumptionCalculator(motorRPMPre, numMotors, propDiameter, propPitch, speedPre, alphaPre, isStacked, batteryVoltage);
        timeOfFlightPre = distPre/speedPre;
        energyPreFlight = timeOfFlightPre * powerPre;

        % Weight changes as a function of the number of batteries and payload attachment
        isPayloadAttached = 0;
        weightPost = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached);

        % Find speed and angle drone will fly at
        [speedPost, alphaPost] = speedCalculator(weightPost, numMotors, numArms, motorRPMPost, propDiameter, propPitch);

        % speedCalculator returns -1's if it cannot hover
        % Cannot hover if too heavy, the only way this function can make
        %   reduce weight is to reduce the number of batteries 
        if (speedPost == -1)
            maxBatteryCapacity = batteryCapacity;
            canHover(2) = 0;
            continue
        else
            canHover(2) = 1;
        end
                        
        % Calculate energy required to finish the second leg of the comp
        [powerPost, currentPost] = powerConsumptionCalculator(motorRPMPost, numMotors, propDiameter, propPitch, speedPost, alphaPost, isStacked, batteryVoltage);
        timeOfFlightPost = distPost/speedPost;
        energyPostFlight = timeOfFlightPost * powerPost;

        % Find the total time of flight required to finish the competition
        %   at this speed
        totalTimeOfFlight = timeOfFlightPre + timeOfFlightPost;

        % Find the total energy used in the competition
        totalEnergy = energyPreFlight + energyPostFlight;

        % Value to minimize
        remainingEnergy = batteryEnergy - totalEnergy;
        
        % if there is energy remaining, reduce the number of batteries,
        % else increase the number of batteries
        if (remainingEnergy > 0)
            maxBatteryCapacity = batteryCapacity;
        else
            minBatteryCapacity = batteryCapacity;
        end
    end
    
    % Packages the pre and post currents and speeds nicely in a matrix
    currents = [currentPre; currentPost];
    speeds = [speedPre; speedPost];
end