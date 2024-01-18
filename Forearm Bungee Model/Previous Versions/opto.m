% Create the figure
figure;
xlabel('Angle (Degrees)');
ylabel('Torque');
hold on;

% Define the desired torques
desired_angles = 0:1:180;
desired_torques = 5 * sind(desired_angles);

% Plot the desired torque curve in red
plot(desired_angles, desired_torques, 'r');

% Cost function to be minimized
function cost = objective_function(params)
    % Extract parameters
    forearm_length = params(1);
    initial_forearm_angle = params(2);
    
    % Run the simulation with the given parameters
    [~, simulation_torques] = runExoNETSimulation(forearm_length, initial_forearm_angle);
    
    % Calculate the cost as the negative of the sum of the squares of the 
    % differences between simulation torques and desired torques 
    % (since we want to maximize the torque)
    cost = -sum((simulation_torques - desired_torques).^2);
end

% Initial guesses for the parameters
initial_params = [12, 90];  % [forearm_length, initial_forearm_angle]

% Optimization using Simulated Annealing
options = optimoptions('simulannealbnd','PlotFcns',...
          {@saplotbestx,@saplotbestf,@saplotx,@saplotf});
optimal_params = simulannealbnd(@objective_function, initial_params, [10, 0], [14, 180], options);

% Run the simulation with the optimal parameters to get the optimal torques
[~, optimal_torques] = runExoNETSimulation(optimal_params(1), optimal_params(2));

% Plot the optimal torques in black
plot(desired_angles, optimal_torques, 'k', 'LineWidth', 2);

% Display the optimal parameters
disp(['Optimal Forearm Length: ', num2str(optimal_params(1))]);
disp(['Optimal Initial Forearm Angle: ', num2str(optimal_params(2))]);

% End of the plot
hold off;
