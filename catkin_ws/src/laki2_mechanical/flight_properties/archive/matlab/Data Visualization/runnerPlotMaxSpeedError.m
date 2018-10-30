numMotors = 8;
numArms = 8;
batteryCapacity = 20000;
numBatteryCells = 4;
propDiameter = 14;
propPitch = 4.7;
isPayloadAttached = 0;

motorRPM = 10000;
maxSpeed = 36.011;
maxAlpha = 1.25*pi/2;

weight = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached);

[topArea, frontArea] = lookupArea(numArms);

plotMaxSpeedError(weight, topArea, frontArea, numMotors, motorRPM, propDiameter, propPitch, isStacked, maxSpeed, maxAlpha)
