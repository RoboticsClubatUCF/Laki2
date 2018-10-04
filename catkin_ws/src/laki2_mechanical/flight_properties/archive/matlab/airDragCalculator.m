% Returns the air drag on a vehicl of the given configuration at the given speed and angle of attack

function [airDrag] = airDragCalculator(speed, alpha, numArms)
    % Assume a drag coefficient for the vehicle
    dragCoef = 0.9;
    
    % Air density in kg/m^3
    airDensity = 1.225;

    % Populate lateral areas
    [topArea, frontArea] = lookupArea(numArms);
    
    % Calculate effective lateral area
    areaEff = topArea * sin(alpha) + frontArea * cos(alpha);
    
    % Drag Equation
    airDrag = 0.5 * dragCoef * airDensity * speed^2 * areaEff;
end

