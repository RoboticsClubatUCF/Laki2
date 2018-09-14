% Calculates the max speed of the vehicle in m/s

% Returns -1's is vehicle cannot sustain lift
% Returns current value for inputs that don't not converge after 1000
%   iterations

% weight is the weight of the vehicle in N 
% topArea and frontArea are the lateral areas of the vehicle from the top 
%   and front respectively, in m^2
% numMotors & motorRPM are number of motors and RPM of motors respectively
% propDiameter and propPitch are diameter and pitch of propeller 
%   respectively, in inches
% isStacked is 0 for flat designs and 1 otherwise
%   1 to allow it print, 0 otherwise

% this analysis assumes that at max speed all motors are operating at the
%   same RPM. In reality they would be different, but that analysis is
%   outside the scope of this optomizer

function [climbSpeed] = climbSpeedCalculator(weight, numMotors, numArms, motorRPM, propDiameter, propPitch)
    isStacked = ~(numMotors == numArms);

    % Check if it can sustain its weight in hover
    [~, thrustY, climbSpeed] = thrustCalculator(motorRPM, propDiameter, propPitch, 0, 0, isStacked);
    thrust = thrustY * numMotors;
    
    % if the thrust is less than the weight it cannot hover
    if (thrust < weight)
        climbSpeed = -1;
        return
    end
    
    %% Calculate the Max Vertical Speed
    % Initialize Error Values
    iter = 0;
    convError = 1;
    absError = 1;
        
    % Uses Newton's Method to Fastest Possible Climbing Speed
    while(absError > 1e-8 && convError > 1e-8)
        % Test runs converged within about 5 iterations at most. 
        %   If it takes more than 1000 something is wrong
        iter = iter + 1;        
        if (iter > 1000)
            fprintf("ahh")
            break
        end
        % thrustCalculator assumes that the vehicle only has horizontal
        % speed. To calculate vertical speed we will give the calculator a
        % vehicle rotated 90 degrees, this is equivalent to if the
        % calculator assumed we were only moving vertically
        thrust = thrustCalculator(motorRPM, propDiameter, propPitch, climbSpeed, pi/2, isStacked);
        totalThrust = thrust * numMotors;
        
        % Air drag also assumes all motion is in the x directions, so again
        %   use pi/2 so that the lateral area is the top area
        airDrag = airDragCalculator(climbSpeed, pi/2, numArms);

        % Function we are trying to minimize to 0
        F = totalThrust - airDrag - weight;
        
        % Calculate the First Derivative
        climbSpeed_fd = climbSpeed + 1e-4;
        
        % Finite difference thrust
        thrust_fd = thrustCalculator(motorRPM, propDiameter, propPitch, climbSpeed_fd, pi/2, isStacked);
        totalThrust_fd = thrust_fd * numMotors;
        
        % Finite difference drag
        airDrag_fd = airDragCalculator(climbSpeed_fd, pi/2, numArms);

        % Derivative with respect to climbing speed
        D = ((totalThrust_fd - airDrag_fd - weight) - F) / (climbSpeed_fd - climbSpeed);
        
        % Update climbSpeed & errors
        speed_old = climbSpeed;
        climbSpeed = climbSpeed - F / D;
       
        convError = abs(speed_old - climbSpeed);
        absError = abs(F);
    end
end