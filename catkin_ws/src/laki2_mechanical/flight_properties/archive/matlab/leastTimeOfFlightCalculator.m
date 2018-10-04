%% Minimizes the Time of Flight of a Drone with the Given Parameters

% Goal: Given propeller specifications, number of battery cells, number of 
%       motors, number of arms, and frame configuration,
%       Find the motor RPM, alpha, number of batteries, that completes the 
%       entire competition in the least amount of time.

% Assumptions: 
%    1) The drone is in steady-level flight
%    2) The drone flies at this constant speed the entire competition
%    3) The only time parameters can chage is when the payload is ped
%       Even then, only the speed, alpha, and motor RPM can change
%    4) Drone is 85% efficient from battery to propeller
%    5) Battery can store 90% of expected energy
%    6) Battery is capable of a 95% depth of discharge
%    7) Drone should land with 20% of battery power remaining
%    8) Current is limited to 35A per motor
%    9) Drone has drag coefficient of 0.9

% Function to Manipulate:
%    1) Forces in the x sum to 0
%    2) Forces in the y sum to 0
%    3) Range able to travel = Competition Range
%    4) Minimize Time of Flight

% Additional Considerations:
%    1) Maximum number of batteries is 10 (that is already too many)
%    2) Maximum allowed airspeed is 70 KIAS (36.0111 m/s) (as per competition rules)
%    3) The upper bound of motor RPM is determined by the fastest motor on
%       hobby king at that given battery voltage
%    4) Battery weight and power estimates are based off of the 
%       Lakitu 1.0 battery

function [motorRPMPre, motorRPMPost, batteryCapacity, totalTimeOfFlight, speeds, currents] = leastTimeOfFlightCalculator(numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost, maxSpeed, maxCurrent)
    %% Constants (Hardware/Rule Limited Values)
   
    % Define numerical tolerances
    convErrorTol = 1e-10;
    absErrorTol = 1e-8;

    % Max Motor RPM limited by motor Kv
    maxMotorRPM = lookupMaxMotorRPM(numBatteryCells);
    
    %% Find Optimal RPM
    
    % If even at the max RPM of the motor, the vehicle cannot hover, there
    %   is no way to fly this vehicle
    [batteryCapacity, totalTimeOfFlight, currents, speeds, canHover] = batteryCapacityCalculator(maxMotorRPM, maxMotorRPM, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost);
    
    % If it can't hover, return all -1's
    if (canHover(1) == 0 || canHover(2) == 0)        
        motorRPMPre = -1;
        motorRPMPost = -1;
        batteryCapacity = -1;
        totalTimeOfFlight = -1;
        speeds = [-1, -1];
        currents = [-1, -1]; 
        return
    end

    % If current pull per motor is below the max allowable current, current 
    %   is not a limiting factor
    if (currents(1) < maxCurrent && currents(2) < maxCurrent)
        motorRPMPre = maxMotorRPM;
        motorRPMPost = maxMotorRPM;

    % Otherwise, calculate the max RPM, current limited
    else
        % Calculate max RPM pre and post 
        minMotorRPMPost = 0.0;
        maxMotorRPMPost = maxMotorRPM;
        motorRPMPost = minMotorRPMPost;
        postConvError = 1;
        postAbsError = 1;
        iter = 0;

        % Bisection Method (Because battery capacity is found numerically)
        % In each iteration, select a post  RPM
        %   within that iteration, optomize the pre  RPM
        % At the end of the iteration update the post motor RPM based on
        %   the current
        while(postConvError > convErrorTol && postAbsError > absErrorTol)
            % Usually takes around 30 iterations to converge, if it takes 1000, 
            %   something is wrong
            iter = iter + 1;
            if (iter > 1e3)
                break;
            end

            % Pick a post  RPM for this iteration
            old_motorRPMPost  = motorRPMPost;        
            motorRPMPost = (minMotorRPMPost + maxMotorRPMPost) / 2;

            postConvError = abs(old_motorRPMPost - motorRPMPost);
            
            % Optomize the pre  RPM
            minMotorRPMPre = 0.0;
            maxMotorRPMPre = maxMotorRPM;
            motorRPMPre = minMotorRPMPre;
            preConvError = 1;
            preAbsError = 1;
            iter1 = 0;
            
            while(preConvError > convErrorTol && preAbsError > absErrorTol) 
                % Usually takes around 30 iterations to converge, if it takes 1000, 
                %   something is wrong
                iter1 = iter1 + 1;
                if (iter1 > 1e3)
                    break;
                end

                old_motorRPMPre  = motorRPMPre;
                motorRPMPre = (minMotorRPMPre + maxMotorRPMPre) / 2;

                preConvError = abs(old_motorRPMPre - motorRPMPre);        
                
                [~, ~, currents, ~, canHover] = batteryCapacityCalculator(motorRPMPre, motorRPMPost, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost);
                
                if (canHover(1) == 0)
                    minMotorRPMPre = motorRPMPre;
                    %fprintf("canHover(1): %f\n", canHover(1), canHover(2));
                    continue
                end

                preAbsError = abs(currents(1) - maxCurrent);
                
                if (currents(1) > maxCurrent)
                    maxMotorRPMPre = motorRPMPre;
                    %fprintf("minMotorRPMPre = motorRPMPre\n");

                else
                    minMotorRPMPre = motorRPMPre;
                    %fprintf("minMotorRPMre = motorRPMPre\n");

                end
            end

            [~, ~, currents, ~, canHover] = batteryCapacityCalculator(motorRPMPre, motorRPMPost, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost);
            
            fprintf("\n\n\niter: %f, motorRPMPre: %f, motorRPMPost: %f, currents(1): %f, currents(2): %f\n", iter, motorRPMPre, motorRPMPost, currents(1), currents(2));
            
            % If it can't hover, the RPM is too low
            if (canHover(2) == 0)
                minMotorRPMPost = motorRPMPost;
                fprintf("canHover(2): %f\n", canHover(2));
                continue
            end

            % If the current is too high, the RPM is too low
            if (currents(2) > maxCurrent)
                maxMotorRPMPost = motorRPMPost;
                fprintf("maxMotorRPMPost = motorRPMPost\n");
            else
                minMotorRPMPost = motorRPMPost;
                fprintf("minMotorRPMPost = motorRPMPost\n");
            end          
    
            postAbsError = abs(currents(2) - maxCurrent);
            
        end

        [batteryCapacity, totalTimeOfFlight, currents, speeds, canHover] = batteryCapacityCalculator(motorRPMPre, motorRPMPost, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost);
        
        fprintf("\n\n\nFixed currents\n");

        
        % If at max current it can't hover, return all -1's
        if (canHover(1) == 0 || canHover(2) == 0)        
            return
        end        
    end
    
    maxRPMPreCurrent = motorRPMPre;
    maxRPMPostCurrent = motorRPMPost;
    
    %----------------------------------------------------------------------
        
    % Motor RPM is presently current limited, return if maxSpeed is not a
    %   limiting factor
    if (speeds(1) < maxSpeed && speeds(2) < maxSpeed)
        return

    % Otherwise, calculate the max RPM, speed limited
    else
        % Calculate max RPM pre and post 
        minMotorRPMPost = 0.0;
        maxMotorRPMPost = maxRPMPostCurrent;
        postConvError = 1;
        postAbsError = 1;
        iter = 0;

        % This optomizer is nearly identical to the previous in terms of
        %   optimizing post  RPM then pre  RPM
        % The only difference is that it instead optomizes for max speed
        %   instead of max current
        % Could probably combine these two somehow, but I don't know how to
        %   magically know if current or speed will be the limiting factor
        while(postConvError > convErrorTol && postAbsError > absErrorTol)
            iter = iter + 1;
            if (iter > 1e3)
                break;
            end

            % Fix a post  RPM for this iteration
            old_motorRPMPost  = motorRPMPost;        
            motorRPMPost = (minMotorRPMPost + maxMotorRPMPost) / 2;

            postConvError = abs(old_motorRPMPost - motorRPMPost);
            
            % Optomize the pre  RPM
            minMotorRPMPre = 0.0;
            maxMotorRPMPre = maxRPMPreCurrent;
            preConvError = 1;
            preAbsError = 1;
            iter1 = 0;
            
            while(preConvError > convErrorTol && preAbsError > absErrorTol)
                iter1 = iter1 + 1;
                if (iter1 > 1e3)
                    break;
                end

                old_motorRPMPre  = motorRPMPre;
                motorRPMPre = (minMotorRPMPre + maxMotorRPMPre) / 2;

                preConvError = abs(old_motorRPMPre - motorRPMPre);        

                [~, ~, currents, speeds, canHover] = batteryCapacityCalculator(motorRPMPre, motorRPMPost, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost);

                % If it can't hover, RPM is too low
                if (canHover(1) == 0)
                    minMotorRPMPre = motorRPMPre;
                    continue
                end

                % If current is too high, RPM is too high
                if (currents(1) > maxCurrent)
                    maxMotorRPMPre = motorRPMPre;
                    continue
                end
                
                % If speed is too high, RPM is too high
                if (speeds(1) > maxSpeed)
                    maxMotorRPMPre = motorRPMPre;
                else
                    minMotorRPMPre = motorRPMPre;
                end
                
                preAbsError = abs(speeds(1) - maxSpeed);

            end

            [~, ~, currents, speeds, canHover] = batteryCapacityCalculator(motorRPMPre, motorRPMPost, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost);

            % If it can't hover, RPM is too low
            if (canHover(2) == 0)
                minMotorRPMPost = motorRPMPost;
                continue
            end

            % If the current is too high, RPM is too high
            if (currents(2) > maxCurrent)
                maxMotorRPMPost = motorRPMPost;
                continue
            end
            
            % If speed is too high, RPM is too high
            if (speeds(2) > maxSpeed)
                maxMotorRPMPost = motorRPMPost;
            else
                minMotorRPMPost = motorRPMPost;
            end          

            postAbsError = abs(speeds(2) - maxSpeed);
            
        end
        
        [batteryCapacity, totalTimeOfFlight, currents, speeds, ~] = batteryCapacityCalculator(motorRPMPre, motorRPMPost, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPre, distPost);
    end  
    
end