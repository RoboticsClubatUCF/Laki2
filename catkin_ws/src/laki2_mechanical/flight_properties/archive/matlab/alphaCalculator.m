% Calculates the angle of attack of a vehicle at a given speed

% Returns -1's if vehicle cannot sustain lift
% Returns -1's if iterator does not converge

% numMotors & motorRPM are number of motors and RPM of motors respectively
% propDiameter and propPitch are diameter and pitch of propeller 
%   respectively, in inches
% speed is the vehicle speed in m/s
% isStacked is 0 for flat designs and 1 otherwise
% isFreeSpeech is a boolean that allows it to print to the command window
%   1 to allow it print, 0 otherwise

% this analysis assumes that at max speed all motors are operating at the
%   same RPM. In reality they would be different, but that analysis is
%   outside the scope of this optomizer

function [alpha] = alphaCalculator(weight, numMotors, motorRPM, propDiameter, propPitch, speed, isStacked)
    % check if it can sustain its weight in hover
    [thrustX, thrustY] = thrustCalculator(motorRPM, propDiameter, propPitch, speed, 0, isStacked);
    thrust = thrustY * numMotors;
    
    if (thrust < weight)                
        alpha = -1 ;
        return
    end
    
    %% Calculate the Max Vertical Speed Neglecting Air Resistance
    iter = 0;
    convError = 1;
    absError = 1;
    maxAlpha = pi/2;
    minAlpha = 0;
    alpha = 0;
    
    while(absError > 1e-16 && convError > 1e-16)
        % Test runs converged within ~15 iterations max. If it takes more
        % than 1000 something is wrong
        iter = iter + 1;        
        if (iter > 1000)
            alpha = -1;
            break
        end
    
        alpha_old = alpha;
        alpha = (minAlpha + maxAlpha)/2;
        
        convError = abs(alpha_old - alpha);
        
        [~, thrustY] = thrustCalculator(motorRPM, propDiameter, propPitch, speed, alpha, isStacked);
        totalThrustY = thrustY * numMotors;
        
        absError = abs(totalThrustY - weight);
                
        if (totalThrustY > weight)
            minAlpha = alpha;
        else
            maxAlpha = alpha;
        end
        
    end    
end