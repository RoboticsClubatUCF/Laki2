clc
clear
fprintf('>> %s\n', mfilename)

%% Gives some example inputs to functions. Created 7/3/18

%% Independent Parameters:
% This is a continuous value. We will assume that whatever value the
%    analysis finds can be created with a combination of batteries
numBatteries = 1;

% This is a discrete value 2-4
numBatteryCells = 3;

% These two have discrete values based on the available props from 
%   hobby king. 
propDiameter = 10;
propPitch = 4.5;

% This is continuous value. We will select a motor with a Kv rating that
%   can support the RPM at the given voltage
motorRPM = 9000;

% Determined by the style of frame:
numMotors = 4;
numArms = 4;

%% Constants:
% Battery capacity in mAH
batteryCapacity = 6000;

% a ridiculous number of batteries as an upper limit to the optomization
maxBatteries = 10;

% Lateral areas of drone:
topArea = 0.0323465160; % m^2
frontArea = 0.006939136; % m^2

% Distances travelled:
%   vehicle has to travel the listed distance with the payload, drop it at 
%   the target, then travel the listed distance without the payload
distanceWithPayload = 2.5627652;
distanceWOutPayload = 0.78; % + scouringSearchSpace(areaSeenByCamera);

% Max Speed Allowed By Competition
maxAllowedSpeed = 70*0.514444; % Converts 70 KIAS to m/s

%% Dependent Parameters:
% Higher end motor: https://hobbyking.com/en_us/multistar-viking-brushless-outrunner-drone-racing-motor-2208-2600kv-cw.html
if numBatteryCells == 2
    maxRPM = 11544;

elseif numBatteryCells == 3
    maxRPM = 17316;

elseif numBatteryCells == 4
    maxRPM = 23088;

else
    maxRPM = -1;
end

%% Calculations
% Have to fly with the water bottle payload
isWaterBottle = 1;
weight = weightCalculator(numMotors, numArms, numBatteries, numBatteryCells, propDiameter, isWaterBottle);

% Flat designs -> isStacked = 0, else 1
isStacked = 0;


% Plot Max Speed Error
plotMaxSpeedError(weight, topArea, frontArea, numMotors, motorRPM, propDiameter, propPitch, isStacked, 20, 1.1*pi/2);

%{
[thrustX, thrustY, exitVelocity] = thrustCalculator(motorRPM, propDiameter, propPitch, 0, 0, isStacked);
thrust = [thrustX, thrustY];
thrust = norm(thrust);

approximateMaxSpeed = exitVelocity/sqrt(1 + (weight/(thrust*numMotors))^4);
%}

% Calculates the max speeds during climb, no air drag, and with air drag
isFreeSpeech = 1;
[maxSpeed, alpha] = maxSpeedCalculator(weight, topArea, frontArea, numMotors, motorRPM, propDiameter, propPitch, isStacked, isFreeSpeech);

[thrustX, thrustY, exitVelocity] = thrustCalculator(motorRPM, propDiameter, propPitch, maxSpeed, alpha, isStacked);
thrust = [thrustX, thrustY];
thrust = norm(thrust);

% range at maxSpeed
[range, timeOfFlight] = rangeCalculator(thrust, exitVelocity, maxSpeed, alpha, numMotors, numBatteryCells, batteryCapacity, numBatteries);

% current per motor at maxSpeed
current = currentCalculator(weight, thrust, maxSpeed, propDiameter, numMotors, numBatteryCells);