% Finds various properties of a drone configuration

% Clear workspace and command window
clc
clear
fprintf('>> %s\n', mfilename)

% Distances travelled:
%   Vehicle has to travel the listed distance with the payload,  it at 
%   the target, then travel the listed distance without the payload
%   These distances are in m

% Assue that when travelling, no fly zones (and possibly other sources) 
%   force the drone to fly some amount facter than straight line paths
geodesicFactor = 1.2;

% Distance required to travel waypoints then get to drop off location
distPre = 4124.4;
distPre = 3413.2;
% Post dist is an estimate of the linear distance required to scour the
%   entire search area, seeing every point in the area twice
distPost = 5613.2;
distPost = 4613.2;

% Apply the geodesic factor
distPre  = distPre  * geodesicFactor;
distPost = distPost * geodesicFactor;

% Read in the available Hobby King Propellers 
diameters = xlsread('Hobby_King_Items.xlsx', 'Prop_Data_Extracted', 'A:A');
pitches = xlsread('Hobby_King_Items.xlsx', 'Prop_Data_Extracted', 'B:B');

% Initialize motor and arm configurations
% First value of each row is num motors, second value is num arms
motorsAndArms = [4, 4; 6, 3; 6, 6; 8, 4; 8, 8; 12, 6; 16, 8];

% Put motors and armsw in their own vectors
motors = motorsAndArms(:, 1);
arms = motorsAndArms(:, 2);

% Initialize number of batteries
numBatteryCellsInit = [2:1:6];

% Max Current, limited by ESC's and motor
maxCurrent = 35;

% Max Speed, limited by competition rules
% Conversion from 70 KIAS to m/s
maxSpeed = 70 * 0.514444;

%% Find the optimal RPM for every configuration (~3676 Permutations)

% Set the file in which to write to
dateString = datestr(datetime('now'));
file = strcat(dateString, '.csv');
fileID = fopen(file,'a');

% Start the first line with titles for the data
properties = ["numBatteryCells", "numMotors", "numArms", "propDiameter", "propPitch", "weightPre", "weightPost", "batteryCapacity", "motorRPMPre", "motorRPMPost", "speeds(1)", "speeds(2)", "alphaPre", "alphaPost", "currents(1)", "currents(2)", "motorRPMExcessThrustPre", "motorRPMExcessThrustPost", "radiusOfTurnPre", "radiusOfTurnPost", "toc"];
numProperties = length(properties);

for i = 1:numProperties
    fprintf(fileID, "%s, ", properties(i));
end
fprintf(fileID, "\n");

% The number of configurations will be used to tell the user the % finished
numConfs = (6 - 2 + 1) * length(motorsAndArms) * length(diameters);
% A counter for the number of finished configurations
kount = 0;

% Reset the profiler, and start it again for user optomization
profile off;
profile on;

% Delete starting here to the block comments
numBatteryCells = 8;
numMotors = 8;
numArms = 8;
propDiameter = 18;
propPitch = 4.5;

[motorRPMPre, motorRPMPost, batteryCapacity, timeOfFlight, speeds, currents] = leastTimeOfFlightCalculator(numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost, maxSpeed, maxCurrent)

weightPre = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, 1)
weightPost = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, 0);

alphaPre  = alphaCalculator(weightPre,  numMotors, motorRPMPre, propDiameter, propPitch, speeds(1), 1)
alphaPost = alphaCalculator(weightPost, numMotors, motorRPMPost, propDiameter, propPitch, speeds(2), 0)

[thrustX, thrustY, exitVelocity] = thrustCalculator(motorRPMPre, propDiameter, propPitch, speeds(1), alphaPre, 0)
[thrustX, thrustY, exitVelocity] = thrustCalculator(motorRPMPost, propDiameter, propPitch, speeds(1), alphaPost, 0)

fprintf("done\n")



%{
% The battery can have between 2 and 6 cells (inclusive)
for i = 1:length(numBatteryCellsInit)
        numBatteryCells = numBatteryCellsInit(i);
    
    % Iterate through all the available propellers
    for j = 1:length(diameters)
        propDiameter = diameters(j);
        propPitch = pitches(j);
            
        % Write calculated properties will be stored in A inside the
        %   following loop
        % Each row will have a different frame configuration, each
        %   column is a vehicle property
        % Do this bc it:
        %   A) reduces the number of fopens without using much more memory
        %   B) allows the parallelization of vechicle calculations
        A = zeros(length(motorsAndArms), numProperties);
        
        % Output to screen for user sanity
        fprintf("\n%0.3f%% Complete \tRunning Configuration: numBatteryCells: %d, propDiameter: %0.1f, propPitch: %0.1f\n", kount/numConfs*100, numBatteryCells, propDiameter, propPitch);  
        
        % Choosen to parallelize frame configurations over other loops bc
        %   for a given number of battery cells and propeller, usually
        %   either all or none of them will fly. This means the parrallel
        %   threads will usually all take around the same amount of time
        % The propeller loop might actually be better because if one gets 
        %   stuck there are like 88 others to run
        % Num battery cells is definitely the worst to run in parallel
 
        % Iterate through all frame configurations
        parfor k = 1:length(motorsAndArms)
            numMotors = motors(k);
            numArms = arms(k);

            isStacked = ~(numMotors == numArms);

            % Skip stacked configurations until data is acquired
            if (isStacked)
                continue
            end
            
            % tic/toc gives the time required for each calculation
            tic
            [motorRPMPre, motorRPMPost, batteryCapacity, timeOfFlight, speeds, currents] = leastTimeOfFlightCalculator(numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost, maxSpeed, maxCurrent);
            
            %% Calculate weight, alpha, and radius of turn 
            %   (and values related to excess thrust)

            % Don't calculate them if this configuration is garbage
            if (batteryCapacity == -1)
                weightPre = -1;
                weightPost = -1;
                
                alphaPre = -1;
                alphaPost = -1;
                
                motorRPMExcessThrustPre = -1;
                motorRPMExcessThrustPost = -1;
                radiusOfTurnPre = -1;
                radiusOfTurnPost = -1;

            else
                weightPre = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, 1);
                weightPost = weightCalculator(numMotors, numArms, batteryCapacity, numBatteryCells, propDiameter, 0);
                                
                alphaPre  = alphaCalculator(weightPre, numMotors, motorRPMPre, propDiameter, propPitch, speeds(1), isStacked);
                alphaPost = alphaCalculator(weightPost, numMotors, motorRPMPost, propDiameter, propPitch, speeds(2), isStacked);

                [radiusOfTurnPre, motorRPMExcessThrustPre]   = radiusOfTurnCalculator(weightPre,  numBatteryCells, numMotors, numArms, propDiameter, propPitch, maxSpeed, maxCurrent, isStacked);
                [radiusOfTurnPost, motorRPMExcessThrustPost] = radiusOfTurnCalculator(weightPost, numBatteryCells, numMotors, numArms, propDiameter, propPitch, maxSpeed, maxCurrent, isStacked);
            end
            
            % Write data to a .csv file
            A(k, :) = [numBatteryCells, numMotors, numArms, propDiameter, propPitch, weightPre, weightPost, batteryCapacity, motorRPMPre, motorRPMPost, speeds(1), speeds(2), alphaPre, alphaPost, currents(1), currents(2), motorRPMExcessThrustPre, motorRPMExcessThrustPost, radiusOfTurnPre, radiusOfTurnPost, toc];
            
        end
        
        dlmwrite(file, A, '-append', 'precision','%.16f');
    
        % Increment the count of finished configurations
        kount = kount + length(motorsAndArms);  
        
    end
end

profile off
profsave(profile('info'), 'nonStacked')
profile viewer
%}

fclose(fileID);