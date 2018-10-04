% Lookup table for motor RPM limited by Kv

function [maxMotorRPM] = lookupMaxMotorRPM(numBatteryCells)
    % Higher end motor for 2-4S:    https://hobbyking.com/en_us/multistar-viking-brushless-outrunner-drone-racing-motor-2208-2600kv-cw.html
    % 5S:                           https://hobbyking.com/en_us/4010-580kv-turnigy-multistar-22-pole-brushless-multi-rotor-motor-with-extra-long-leads.html?___store=en_us
    % 6S:                           https://hobbyking.com/en_us/turnigy-multistar-3508-640kv-14-pole-multi-rotor-outrunner-v2.html?___store=en_us
    switch numBatteryCells
        case 2
            maxMotorRPM = 11544.0;
        case 3
            maxMotorRPM = 17316.0;
        case 4
            maxMotorRPM = 23088.0;
        case 5
            maxMotorRPM = 8288.0;
        case 6
            maxMotorRPM = 3777;%7460.0;
        case 7
            maxMotorRPM = 5801.6;
        case 8
            maxMotorRPM = 6630.4;
        otherwise
            maxMotorRPM = -1.0;
    end
end

