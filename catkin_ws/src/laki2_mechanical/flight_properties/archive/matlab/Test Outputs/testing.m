clear
clc

numMotors = 8;
numArms = 8;
batteryCapacity = 20167.3658938358;
numBatteryCells = 4;
propDiameter = 8;
propPitch = 5;
isPayloadAttached = 1;
isStacked = 0;

motorRPM = [9200: (20200 - 9200) / 5000: 20200];

profile off
profile on
weight = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached);
%[thrustX, thrustY, exitVelocity] = thrustCalculator(motorRPM, propDiameter, propPitch, 0, 0, isStacked);
%totalThrustY = thrustY * numMotors;

for i = 1:length(motorRPM)
    [speed, alpha] = speedCalculator(weight, numMotors, numArms, motorRPM(i), propDiameter, propPitch);
    [~, ~, exitVelocity] = thrustCalculator(motorRPM(i), propDiameter, propPitch, 0, 0.9*pi/2, isStacked);    
end

%alpha  = alphaCalculator(weight, numMotors, motorRPM, propDiameter, propPitch, speed, isStacked);

profile off
profile viewer
