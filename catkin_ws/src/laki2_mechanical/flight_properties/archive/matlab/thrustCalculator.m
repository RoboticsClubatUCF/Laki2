% Returns the thrust of specified motor/prop combination in N
% Returns the angle of the thrust relative to the horizontal in radians

% NOTE: Will return a negative number if the speed is creater than the exit
% velocity of the propeller

% NOTE: Assumes vehicleSpeed is only in horizontal direction. 

% NOTE: Thrust does not necessarily act perpendicular to frame

% NOTE: Will return a value for a negative vehicleSpeed

% Takes prop_diameter and prop_pitch in inches
% Tales vehicle_speed in m/s
% propDiameter and propPitch in inches
% alpha is the pitch angle of the vehicle
% isStacked is boolean, 0 for flat frame, 1 else

function [thrustX, thrustY, exitVelocity] = thrustCalculator(motorRPM, propDiameter, propPitch, speed, alpha, isStacked)
    % Assume that two motors on the same arm each operate at n% efficiency
    stackedCoeff = 0.8;

    % exitVelocity is the speed in m/s of the air after is has gone thru 
    %   the prop
    exitVelocity = motorRPM * propPitch * 0.0254/60;
    
    if isStacked
        exitVelocity = exitVelocity * stackedCoeff;
    end
    
    % diskArea is the area of the circle created by the spinning propeller
    diskArea = pi * (0.0254 * propDiameter / 2)^2;
    
    % mass flow rate = density * area * propeller velocity
    massFlowRate = (1.225 * diskArea) * (0.5 *  (exitVelocity + sin(alpha) * speed));
    
    % To Do: Accurately predict propeller slip and do away with the
    %   correction factor
    
    % This is an empirically found value that corrects the thrust based on
    %   motor/propeller thrust data from OS Engines
    correctionFactor = 0.344804953656331 * (propDiameter/propPitch)^1.38152091029528;
    correctionFactor= 1;
    
    % Sans the correction factor, this is the analytical thrust of the prop
    % The correction factor serves to try to make the thrust estimate a
    %   little bit more accurate to empirical data  
    thrustX = massFlowRate * (exitVelocity * sin(alpha) - speed) * correctionFactor;
    thrustY = massFlowRate * exitVelocity * cos(alpha) * correctionFactor;
end