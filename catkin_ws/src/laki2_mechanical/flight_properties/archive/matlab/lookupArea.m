% Lookup table for lateral cross sectional area of requested drone in m^2

% Estimate values based on quick frame designs
function [topArea, frontArea] = lookupArea(numArms)
    switch numArms
        % 6 motors, 3 arms
        case 3
            topArea = 0.037739937423;
            frontArea = 0.02793626671;

        % 4 or 8 motors, 4 arms
        case 4
            topArea = 0.0323465160;
            frontArea = 0.006939136;

        % 6 or 12 motors, 6 arms
        case 6
            topArea = 0.0648640896264;
            frontArea = 0.0163591208301;

        % 8 or 16 motors, 8 arms
        case 8
            topArea = 0.0694350353232;
            frontArea = 0.0153517009094;

        % error
        otherwise
            topArea = 9999999;
            frontArea = 9999999;
    end
    
    % Estimate for lateral area of payload
    frontArea = frontArea + 0.01;
end