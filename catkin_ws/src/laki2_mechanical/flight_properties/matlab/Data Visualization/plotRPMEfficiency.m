% From the found motor data, plots the measured RPM as a % of the theoretical RPM

diameters = xlsread('motor_thrust_data.xlsx', 'Sheet1', 'A:A');
pitches = xlsread('motor_thrust_data.xlsx', 'Sheet1', 'B:B');
RPM = xlsread('motor_thrust_data.xlsx', 'Sheet1', 'C:C');
kV= xlsread('motor_thrust_data.xlsx', 'Sheet1', 'D:D');
voltage= xlsread('motor_thrust_data.xlsx', 'Sheet1', 'E:E');
RPMEfficiency= xlsread('motor_thrust_data.xlsx', 'Sheet1', 'G:G');

scatter3(diameters, pitches, RPMEfficiency);
hold on
xlabel('diameters');
ylabel('pitches');
zlabel('RPMEfficiency');
hold off