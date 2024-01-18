function obj_val = objective_function(forearm_width)
    [~, torques] = run_simulation(forearm_width);

    % Desired torques
    angle_range = 0:1:180;
    desired_torques = 5 * sind(angle_range);
    %desired_torques(desired_torques >= asind(5)) = 5; % leveling off at 20
    
    % Compute the error
    torque_error = sum((torques - desired_torques).^2);

    % Objective value
    obj_val = torque_error;
end
