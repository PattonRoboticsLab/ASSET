% Create the figure
figure;
xlabel('Angle (Degrees)');
ylabel('Torque');
hold on;

% Define the desired torques
desired_angles = 0:1:180;
desired_torques = 5 * sind(desired_angles);
%desired_torques(desired_angles >= asind(5)) = 5; % leveling off at 20

% Plot the desired torque curve in red
plot(desired_angles, desired_torques, 'r');

% Initialize a random value for forearm_width between 1 and 5
best_forearm_width = 1 + 5.123 * rand();

% Optimization Loop (100 tries)
for iter = 1:100

    % Use fminsearch to find the optimal forearm_width
    [best_forearm_width, ~] = fminsearch(@objective_function, best_forearm_width);
    
    % Run the simulation with the best parameters
    [~, optimal_torques] = run_simulation(best_forearm_width);
    
    % Plot the optimal torques in gray
    plot(desired_angles, optimal_torques, 'Color', [0.5, 0.5, 0.5]);
    
    % Debugging
    fprintf('Iteration: %d, Initial forearm width: %f, Best forearm width: %f\n', iter, best_forearm_width, best_forearm_width);

    % Pause to allow inspection of the new curve
    pause(1);  % 1-second pause, adjust as needed

end

% Turn off the legend
legend('off');

% End of the plot
hold off;
