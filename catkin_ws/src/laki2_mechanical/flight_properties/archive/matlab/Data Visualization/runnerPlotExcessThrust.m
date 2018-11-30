numBatteryCells = 4;
numMotors = 8;
numArms = 8;
propDiameter = 14;
propPitch = 4.7;
batteryCapacity = 31598;
isPayloadAttached = 1;

speed = 36.011075;

weight = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached);

plotExcessThrust(weight, numMotors, propDiameter, propPitch, speed, isStacked, numBatteryCells);