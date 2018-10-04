clear
% Losses due to the losses of the motor are of the following form:
% P_in = P_out + I^2 * R_o + I_o * V
% where I^2 * R_m are the "copper losses" due to the no load resistance
% of the motor, controller, and battery
% and I_o * V are the "iron losses" are due to the no load current required by the motor to produce 0 work

% Assumed to be constant, but make a spreadsheet of motor Kv,
% resistance, etc
motorResistance = 0.032;

% Assumed controller resistance
controllerResistance = 0.010;

% Assumed battery resistance
batteryResistance = 0.0006 / 4;

% Total internal resistance of components from battery to motor
totalResistance = motorResistance + controllerResistance + batteryResistance;

% Assumed the required no load power of the motor is that given on the
% data sheet, this should also be spreadsheet driven
noLoadPower = 1.6 * 10;

% The RMS voltage of the controller is equal to the battery voltage, so
% P_in = V * I

propPower = linspace(0, 1500, 500);
totalResistance = linspace(0, 0.05, 500);


for i = 1:length(propPower)
    for j = 1: length(propPower)
        current = roots([totalResistance(i), -14.8, (propPower(j) + noLoadPower)]);
        y(i, j) =  min(current);
        
        if (isreal(y(i, j)) == 0)
            y(i, j) = 225;
        end
        
    end
end

mesh(totalResistance, propPower, y)
hold on
xlabel 'totalResistance'
ylabel 'propPower'
zlabel 'current'


