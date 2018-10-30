function [] = powerTester()
    % Create a grid of points to plot
    speed = [0 : 11.4/500 : 11.4];
    alpha = [0 : pi/2/500 : pi/2];

    % Initialize the thrust vectors
    currents = zeros(length(speed), length(alpha));

    % for each point in the grid, calculate the thrust
    for i = 1:length(speed)
        for j = 1:length(alpha)

            [thrustX, thrustY, exitVelocity] = thrustCalculator(6000, 10, 4.5, speed(i), alpha(j), 0);

            if (exitVelocity < speed(i))
                fprintf("speed: %f, exitVelocity: %f, thrustX: %f, thrustY, %f\n", speed(i), exitVelocity, thrustX, thrustY);
            end
                
            [power, current] = powerConsumptionCalculator(6000, 4, 10, 4.5, speed(i), alpha(j), 0, 11.1);
            
            currents(i,j) = current;

        end
    end

    mesh(alpha, speed, currents);
    hold on
    xlabel('alpha');
    ylabel('speed');
    zlabel('currents');
    hold off
end

