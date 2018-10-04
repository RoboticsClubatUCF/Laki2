% Calculates how far the drone can fly in meters at a given speed

% thrust is the thrust produced by one motor in N
% exitVelocity is the final velocity of the air after it has gone thru the
%   propeller, in m/s
% alpha is the angle of the vehicle relative to the horizontal in radians
% numMotors, numBatterCells, batteryCapacity, and numBatteries are the 
%   number of motors, number of battery cells, battery capacity, and number 
%   of batteries respectively

function [range, timeOfFlight] = rangeCalculator(thrust, exitVelocity, vehicleSpeed, alpha, numMotors, numBatteryCells, batteryCapacity, numBatteries)
    totalThrust = thrust * numMotors;
    
    propellerVelocity = 0.5 * (exitVelocity + sin(alpha) * vehicleSpeed);

    power = totalThrust * propellerVelocity; 
    
    % Assume 85% efficiency
    power = power/0.85;
    
    batteryVoltage = 3.7 * numBatteryCells;
    batteryEnergy = batteryVoltage * batteryCapacity / 1000 * numBatteries * 60 * 60;
    
    timeOfFlight = batteryEnergy/power;
    range = vehicleSpeed * timeOfFlight;
end