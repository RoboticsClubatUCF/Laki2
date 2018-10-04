clear
clc

motorRPM = 15000;
numMotors = 8;
numArms = 8;
batteryCapacity = 20167.3658938358;
numBatteryCells = 4;
propDiameter = 8;
propPitch = 5;
isPayloadAttached = 1;
isStacked = 0;
weight = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached);


climbSpeed = [0: 0.001: 50];

for i = 1:length(climbSpeed)
    thrust = thrustCalculator(motorRPM, propDiameter, propPitch, climbSpeed(i), pi/2, isStacked);
    totalThrust = thrust * numMotors;
    airDrag = airDragCalculator(climbSpeed(i), 0, numArms);

    F(i) = totalThrust - airDrag - weight;
    
    climbSpeed_fd = climbSpeed(i) + 1e-5;
    
    thrust_fd = thrustCalculator(motorRPM, propDiameter, propPitch, climbSpeed_fd, pi/2, isStacked);
    totalThrust_fd = thrust_fd * numMotors;
    
    airDrag_fd = airDragCalculator(climbSpeed_fd, 0, numArms);
    
    D(i) = ((totalThrust_fd - airDrag_fd - weight) - F(i)) / (climbSpeed_fd - climbSpeed(i));
    
end

plot(climbSpeed, F, climbSpeed, D);
hold on
xlabel('climbSpeed');
ylabel('F & D');
hold off