% Returns the weight of specified vehicle in N

% numMotors and numArms are the number of motors and arms respectively
% numBatteries and numBatteryCells are the number of batteries and cells in
%   those batteries respectively
% propDiameter is the diameter of each prop in inches
% isPayloadAttached is a boolean, 1 when arrying the payload, else 0

% This estimate assumes that the weight of the batteries scales linearly
%   with the number of battery cells and the number of batteries. This is
%   reasonable, and allows the analysis to test different voltages and
%   capacities

% All constants for item weights are in lb. Converted to N in last step
function [totalWeight] = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, isPayloadAttached)
    
    % Weight of 16mm carbon tube based on creo model
    armPerIn = 0.037486529988/8.66142;

    % Estimate from Creo Model (includes weight of clamps on both sides of
    %   arm
    genericArm = 0.06986696;
    armWeight = genericArm + (armPerIn * ((propDiameter/2)-0.75));
    
    % Values obtained from gram scale at robotics
    motor = 0.1000899;
    odroid = 0.132277;
    gps = 0.0396832;
    pix = 0.0749572;
    remoteReceiver = 0.0132277;
    powerDistribution = 0.0859803;
    camera = 0.0220462;
    
    % Estimated Values
    waterBottleMechanism = 0.5;
    electronicsHardware = 0.15;
    radioTransceiver = 0.0507063;
    landingLegs = 0.202825;
    oneTB_SSD_Harddrive = 0.18;
    
    % Weights of various frames
    % Does not include weight of clamps for arms, that is included in
    %   weight of arm
    switch numArms
        case 3
            centerFrame = 0.636163313; 
            
        case 4
            centerFrame = 0.472710296;

        case 6
            centerFrame = 0.710648931;
    
        case 8
            centerFrame = 0.699119006;
        
        otherwise
            centerFrame = 9999999;
    end
    
    % Center of Frame, includes all electronics
    torso = odroid + gps + pix + remoteReceiver + radioTransceiver + powerDistribution + oneTB_SSD_Harddrive + waterBottleMechanism + landingLegs + electronicsHardware + camera + centerFrame;
    
    % Payload Weight
    if isPayloadAttached
        waterBottle = 0.51;
        payload = 0.49;
    else
        waterBottle = 0;
        payload = 0;
    end
    
    % Estimate Battery Weights
    weightPerCellPermAh = 0.988/3/6000;
    batteryWeight = numBatteryCells * batteryCapacity * weightPerCellPermAh;
    
    % totalWeight in lb
    totalWeight = armWeight * numArms + motor * numMotors + torso + waterBottle + payload + batteryWeight;
    
    % Convert from lb to N
    totalWeight = 4.45 * totalWeight;
end