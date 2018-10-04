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

function [speed, alpha] = speedCalculator(weight, numMotors, numArms, motorRPM, propDiameter, propPitch)
    isStacked = ~(numMotors == numArms);

    % Check if it can sustain its weight in hover
    [~, thrustY, speed] = thrustCalculator(motorRPM, propDiameter, propPitch, 0, 0, isStacked);
    thrust = thrustY * numMotors;
    
    % if the thrust is less than the weight it cannot hover
    if (thrust < weight)
        speed = -1;
        alpha = -1 ;
        return
    end
    
    %% Calculate the Max Forward Speed with Air Resistance
    iter = 0;
    convError = 1;
    absError = 1;
    
    % Initial guesses
    alpha = 0.9*pi/2;
    % speed's initial guess is the exitVelocity found when checking if it
    %   can hover
    
    % Use Newton's Method to solve for alpha and speed
    while(absError > 1e-8 || convError > 1e-10)
        % Test runs converged within about 9 iterations. If it takes more
        %   than 1000 something is wrong
        iter = iter + 1;        
        if (iter > 1000)
            return
        end
        
        % Find thrust at this speed
        [thrustX, thrustY] = thrustCalculator(motorRPM, propDiameter, propPitch, speed, alpha, isStacked);

        % Forces in the y:
        verticalThrust = thrustY * numMotors;
        netVerticalThrust = weight - verticalThrust;
        
        % Forces in the x:
        horizontalThrust = thrustX * numMotors;
        airDrag = airDragCalculator(speed, alpha, numArms);
        
        netHorizontalThrust = horizontalThrust - airDrag;
        
        % Function Vector
        F = [netHorizontalThrust;
            netVerticalThrust];
        
        % Calculate the Jacobian
        speed_fd = speed + 1e-4;
        alpha_fd = alpha + 1e-4;
       
        % Finite Difference w/ respect to speed
        [thrustX_speed, thrustY_speed] = thrustCalculator(motorRPM, propDiameter, propPitch, speed_fd, alpha, isStacked);
        verticalThrust_speed = thrustY_speed * numMotors;
        horizontalThrust_speed = thrustX_speed * numMotors;
        
        netVerticalThrust_speed = weight - verticalThrust_speed;
        
        % alpha has not changed, so areaEff has not changed
        airDrag_speed = airDragCalculator(speed_fd, alpha, numArms);
        netHorizontalThrust_speed = horizontalThrust_speed - airDrag_speed;
        
        % Finite difference values w/ respect to speed
        fd_netVerticalThrust_speed = (netVerticalThrust - netVerticalThrust_speed)/(speed - speed_fd);
        fd_netHorizontalThrust_speed = (netHorizontalThrust - netHorizontalThrust_speed)/(speed - speed_fd); 
        
        % Finite Difference w/ respect to alpha
        [thrustX_alpha, thrustY_alpha] = thrustCalculator(motorRPM, propDiameter, propPitch, speed, alpha_fd, isStacked);
        verticalThrust_alpha = thrustY_alpha * numMotors;
        horizontalThrust_alpha = thrustX_alpha * numMotors;
        
        netVerticalThrust_alpha = weight - verticalThrust_alpha;
        
        airDrag_alpha = airDragCalculator(speed, alpha_fd, numArms);
        netHorizontalThrust_alpha = horizontalThrust_alpha - airDrag_alpha;
        
        fd_netVerticalThrust_alpha = (netVerticalThrust - netVerticalThrust_alpha)/(alpha - alpha_fd);
        fd_netHorizontalThrust_alpha = (netHorizontalThrust - netHorizontalThrust_alpha)/(alpha - alpha_fd); 
        
        J = [fd_netHorizontalThrust_speed, fd_netHorizontalThrust_alpha;
            fd_netVerticalThrust_speed, fd_netVerticalThrust_alpha];
        
        % Calculate Error
        % Equivalent to X = inv(J) * F
        %   but significantly faster and more accurate
        X = J \ F;
        
        alpha_old = alpha;
        speed_old = speed;

        % Update Speed and Alpha
        speed = speed - X(1);
        alpha = alpha - X(2);
        
        % Restrict speed to be a positive number
        speed = abs(speed);
    
        % Restrict alpha to be between 0 and pi/2
        alpha = abs(mod(alpha, pi/2));

        % Update Error Values
        convError = sqrt((alpha_old - alpha)^2 + (speed_old - speed)^2);
        absError  = sqrt((netVerticalThrust)^2 + (netHorizontalThrust)^2);
    end
end