% Creates a plot of power consumption vs motor RPM

% Takes weight in N
% topArea and frontArea are respective top and front lateral areas in m^2
% numMotors and motorRPM are the number of motors and RPM rpm respectively
% propDiameter and propPitch are the propeller diameter and pitch
%   respecivetly, in inces
% isStacked is a boolean, 1 if frame is a stacked configuration and 0 else
% maxSpeed and maxAlpha are user inputs that determine the upper bound of
%   the figure

function [] = plotPowerAndSpeed(weight, numArms, numMotors, propDiameter, propPitch, isStacked, numBatteryCells, batteryCapacity)
    % maxMotorRPM is a function of numBatteryCells
    switch numBatteryCells
        case 2
            maxMotorRPM = 11544;
        case 3
            maxMotorRPM = 17316;
        case 4
            maxMotorRPM = 23088;
        case 5
            maxMotorRPM = 8288;
        case 6
            maxMotorRPM = 7460;    
        otherwise
            maxMotorRPM = -1;
    end

    % Create a grid of points to plot
    motorRPM = [0:maxMotorRPM/5000:maxMotorRPM];
    
    batteryVoltage = numBatteryCells * 3.7;
    batteryEnergy = (1.00 - 0.20) * 0.95 * 0.9 * batteryVoltage * (batteryCapacity / 1000* 60 * 60);
    
    % for each point in the grid, calculate the thrust
    j = 0;
    for i = 1:length(motorRPM)
        [speed, alpha] = speedCalculator(weight, numMotors, numArms, motorRPM(i), propDiameter, propPitch);
        
        [powerCons, current] = powerConsumptionCalculator(motorRPM(i), numMotors, propDiameter, propPitch, speed, alpha, isStacked, batteryVoltage);
        
        lengthOfFlight = batteryEnergy / powerCons;
        
        range = speed * lengthOfFlight;
        
        if (speed == -1)
            continue
        end
        
        if (current <= 35)
            j = j + 1;
            motorRPMs(j) = motorRPM(i);
            power(j) = powerCons;
            speeds(j) = speed;
            ranges(j) = range;
        end
            
    end
    
    subplot(2,2,1);
    plot(motorRPMs, power);
    hold on
    xlabel('motorRPM');
    ylabel('power');
    hold off

    subplot(2,2,2);
    plot(motorRPMs, speeds);
    hold on
    xlabel('motorRPM');
    ylabel('speed');
    hold off
    
    subplot(2,2,3);
    plot(speeds, power);
    hold on
    xlabel('speed');
    ylabel('power');
    hold off
    
    subplot(2,2,4);
    plot(speeds, ranges);
    hold on
    xlabel('speed');
    ylabel('range');
    hold off
    
end