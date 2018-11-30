% Creates a Figure of the absolute error in the max speed calculation

% Takes weight in N
% topArea and frontArea are respective top and front lateral areas in m^2
% numMotors and motorRPM are the number of motors and RPM rpm respectively
% propDiameter and propPitch are the propeller diameter and pitch
%   respecivetly, in inces
% isStacked is a boolean, 1 if frame is a stacked configuration and 0 else
% maxSpeed and maxAlpha are user inputs that determine the upper bound of
%   the figure

function [] = plotMaxSpeedError(weight, topArea, frontArea, numMotors, motorRPM, propDiameter, propPitch, isStacked, maxSpeed, maxAlpha)
    % Create a grid of points to plot
    speed = [0:maxSpeed/500:maxSpeed];
    alpha = [0:maxAlpha/500:maxAlpha];

    % Initialize the thrust vectors
    verticalThrust = zeros(length(speed));
    horizontalThrust = zeros(length(speed));

    % for each point in the grid, calculate the thrust
    for i = 1:length(speed)
        for j = 1:length(speed)
            [thrustX, thrustY] = thrustCalculator(motorRPM, propDiameter, propPitch, speed(i), alpha(j), isStacked);

            verticalThrust(i,j) = thrustY * numMotors;
            horizontalThrust(i,j) = thrustX * numMotors;

        end
    end
    
    % Forces in the y:
    netVerticalThrust = weight - verticalThrust;

    % Forces in the x:
    % effective lateral area
    areaEff = topArea * sin(alpha) + frontArea * cos(alpha);
    % air drag
    airDrag = 0.5 * 0.9 * 1.225 * speed.^2 .* areaEff;
    netHorizontalThrust = horizontalThrust - airDrag;

    absError  = ((netVerticalThrust).^2 + (netHorizontalThrust).^2).^0.5;
    % fprintf("(weight - verticalThrust): %f\tthrustX: %f\n", netVerticalThrust, thrustX);
    % fprintf("conv: %f\tabs: %f\n", convError, absError);

    mesh(alpha, speed, absError);
    hold on
    xlabel('alpha');
    ylabel('speed');
    zlabel('absError');
    hold off
end