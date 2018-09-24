
    % Calculate min RPM to hover pre Drop
    minMotorRPMPostDrop = 0;
    maxMotorRPMPostDrop = maxMotorRPM;
    motorRPMPostDrop = 0;
    postConvError = 1;
    iter = 0;

    while(postConvError > 1e-6)
        iter = iter + 1;
        if (iter > 1e3)
            break;
        end
        
        % Fix a post drop RPM for this iteration
        old_motorRPMPostDrop  = motorRPMPostDrop;        
        motorRPMPostDrop = (minMotorRPMPostDrop + maxMotorRPMPostDrop) / 2;
        
        postConvError = abs(old_motorRPMPostDrop - motorRPMPostDrop);
        
        % Optomize the pre drop RPM
        minMotorRPMPreDrop = 0;
        maxMotorRPMPreDrop = maxMotorRPM;
        motorRPMPreDrop = 0;
        preConvError = 1;
        iter1 = 0;
        while(preConvError > 1e-6) 
            iter1 = iter1 + 1;
            if (iter1 > 1e3)
                break;
            end
            
            old_motorRPMPreDrop  = motorRPMPreDrop;
            motorRPMPreDrop = (minMotorRPMPreDrop + maxMotorRPMPreDrop) / 2;
            
            preConvError = abs(old_motorRPMPreDrop - motorRPMPreDrop);        

            [numBatteries, totalTimeOfFlight, currents, speeds, canHover] = numBatteriesCalculator(motorRPMPreDrop, motorRPMPostDrop, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPreDrop, distPostDrop, isFreeSpeech);
            
            if (canHover(1) == 0)
                minMotorRPMPreDrop = motorRPMPreDrop;
            else
                maxMotorRPMPreDrop = motorRPMPreDrop;
            end
        end
        
        % Since the RPM is teetering on the edge of hovering, ensure that
        %   it hovers by giving it a tiny push
        if (canHover(1) == 0)
            motorRPMPreDrop = 1.02 * motorRPMPreDrop;
        end
        
        [numBatteries, totalTimeOfFlight, currents, speeds, canHover] = numBatteriesCalculator(motorRPMPreDrop, motorRPMPostDrop, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPreDrop, distPostDrop, isFreeSpeech);
        
        if (canHover(2) == 0)
                minMotorRPMPostDrop = motorRPMPostDrop;
        else
                maxMotorRPMPostDrop = motorRPMPostDrop;
        end            
        
    end
    
    if (canHover(2) == 0)
        motorRPMPostDrop = 1.02 * motorRPMPostDrop;
    end
    
    % Minimum RPM's required to hover
    RPMPostDropHover = motorRPMPostDrop;
    RPMPreDropHover  = motorRPMPreDrop;
    
    fprintf("RPMPreDropHover: %f, RPMPostDropHover: %f\n", RPMPreDropHover, RPMPostDropHover) 
    
    %----------------------------------------------------------------------
        
    % If the min required RPM is below the max allotted RPM, vehicle is not
    %   possible to fly
    [numBatteries, totalTimeOfFlight, currents, speeds, canHover] = numBatteriesCalculator(RPMPreDropHover, RPMPostDropHover, numMotors, numArms, propDiameter, propPitch, numBatteryCells, distPreDrop, distPostDrop, isFreeSpeech);

    
    if (currents(1)/numMotors > maxCurrent || currents(2)/numMotors > maxCurrent)
        fprintf("Current to hover is too high\n")
        motorRPMPreDrop = -1;
        motorRPMPostDrop = -1;
        numBatteries = -1;
        totalTimeOfFlight = -1;
        speeds = [-1, -1];
        currents = [-1, -1]; 
        return
    end