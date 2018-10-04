numMotors = 8;
numArms = 8;
numBatteryCells = 4;
propDiameter = 26;
propPitch = 5.5;
isStacked = 0;
isPayloadAttached = 1;

distPreDrop = 4124.4 * 1.4; 
distPostDrop = 5613.2 * 1.4;

% maxMotorRPM is a function of numBatteryCells
switch numBatteryCells
    case 2
        maxMotorRPM = 11544;
    case 3
        maxMotorRPM = 17316;
    case 4
        maxMotorRPM = 23088;
    case 5
        maxMotorRPM = 8288;
    case 6
        maxMotorRPM = 7460;    
    otherwise
        maxMotorRPM = -1;
end

[batteryCapacity, totalTimeOfFlight, currents, speeds, canHover] = batteryCapacityCalculator(maxMotorRPM, maxMotorRPM, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPreDrop, distPostDrop);
batteryCapacity = 31381.865516305;


weight = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached);

plotPowerAndSpeed(weight, numArms, numMotors, propDiameter, propPitch, isStacked, numBatteryCells, batteryCapacity)