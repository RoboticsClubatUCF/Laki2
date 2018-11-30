% Creates a plot of power consumption vs motor RPM

% Takes weight in N
% topArea and frontArea are respective top and front lateral areas in m^2
% numMotors and motorRPM are the number of motors and RPM rpm respectively
% propDiameter and propPitch are the propeller diameter and pitch
%   respecivetly, in inces
% isStacked is a boolean, 1 if frame is a stacked configuration and 0 else
% maxSpeed and maxAlpha are user inputs that determine the upper bound of
%   the figure

function [] = plotExcessThrust(weight, numMotors, propDiameter, propPitch, speed, isStacked, numBatteryCells)
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
    motorRPM = [0 : maxMotorRPM/5000 : maxMotorRPM];
    %motorRPM = [18000: 10000/5000 : 18200];
    
    batteryVoltage = numBatteryCells * 3.7;

    alpha = 0.97*pi/2;
    
    % for each point in the grid, calculate the thrust
    j = 0;
    for i = 1:length(motorRPM)   
        alpha = alphaCalculator(weight, numMotors, motorRPM(i), propDiameter, propPitch, speed, isStacked);

        if (alpha == -1)
            continue;
        end
        
        [powerCons, current] = powerConsumptionCalculator(motorRPM(i), numMotors, propDiameter, propPitch, speed, alpha, isStacked, batteryVoltage);
                    
        [thrustX, thrustY, exitVelocity] = thrustCalculator(motorRPM(i), propDiameter, propPitch, speed, alpha, isStacked);
        
        
        j = j + 1;
        motorRPMs(j) = motorRPM(i);
        powers(j) = powerCons;
        alphas(j) = alpha;
        currents(j) = current;
        thrustXs(j) = thrustX;
        thrustYs(j) = thrustY;
        
    end
    
    subplot(3,2,1);
    plot(motorRPMs, thrustXs);
    hold on
    xlabel('motorRPM');
    ylabel('thrustX');
    hold off

    subplot(3,2,2);
    plot(motorRPMs, thrustYs);
    hold on
    xlabel('motorRPM');
    ylabel('thrustY');
    hold off
    
    subplot(3,2,3);
    plot(motorRPMs, alphas);
    hold on
    xlabel('motorRPM');
    ylabel('alphas');
    hold off
    
    subplot(3,2,4);
    plot(motorRPMs, currents);
    hold on
    xlabel('motorRPM');
    ylabel('currents');
    hold off
    
    subplot(3,2,5);
    plot(motorRPMs, powers);
    hold on
    xlabel('motorRPM');
    ylabel('power');
    hold off
    
end