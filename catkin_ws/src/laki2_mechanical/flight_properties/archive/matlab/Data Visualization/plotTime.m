% Creates a Figure of the absolute error in the max speed calculation

% Takes weight in N
% topArea and frontArea are respective top and front lateral areas in m^2
% numMotors and motorRPM are the number of motors and RPM rpm respectively
% propDiameter and propPitch are the propeller diameter and pitch
%   respecivetly, in inces
% isStacked is a boolean, 1 if frame is a stacked configuration and 0 else
% maxSpeed and maxAlpha are user inputs that determine the upper bound of
%   the figure

function [] = plotTime(numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPreDrop, distPostDrop)
    % Create a grid of points to plot
    switch numBatteryCells
        case 2
            maxMotorRPM = 11544.0;
        case 3
            maxMotorRPM = 17316.0;
        case 4
            maxMotorRPM = 23088.0;
        case 5
            maxMotorRPM = 8288.0;
        case 6
            maxMotorRPM = 7460.0;    
        otherwise
            maxMotorRPM = -1.0;
    end
    
    postDropMotorRPM = [9200: (maxMotorRPM - 9200)/500 :maxMotorRPM];
    preDropMotorRPM = 9000;

    %time = zeros(length(postDropMotorRPM));
    
    % for each point in the grid, calculate the thrust
    for i = 1:length(postDropMotorRPM)
        [batteryCapacity, totalTimeOfFlight, currents, speeds, canHover] = batteryCapacityCalculator(preDropMotorRPM, postDropMotorRPM(i), numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPreDrop, distPostDrop);
        time(i) = totalTimeOfFlight;
    end

    % fprintf("(weight - verticalThrust): %f\tthrustX: %f\n", netVerticalThrust, thrustX);
    % fprintf("conv: %f\tabs: %f\n", convError, absError);

    plot(postDropMotorRPM, time);
    hold on
    xlabel('postDropMotorRPM');
    ylabel('time');
    hold off
end

