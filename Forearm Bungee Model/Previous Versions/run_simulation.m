function [angles, torques] = run_simulation(forearm_width)
    % Initialize parameters
    num_frames = 181;
    wrist_width = 2.5; % replace with actual value
    forearm_length = 12; % replace with actual value
    wrist_theta = deg2rad(180);
    forearm_theta = 0; % replace with actual value
    dt = 0.1; % replace with actual value
    Kp = 1; % replace with actual value
    Ki = 0; % replace with actual value
    Kd = .1; % replace with actual value
    target_angle = deg2rad(0); % replace with actual value
    prev_error = 0; % replace with actual value
    error_sum = 0; % replace with actual value
    omega = 0.1; % replace with actual value
    alpha = 0; % replace with actual value
    I = 5; % replace with actual value
    damping_coeff = 3; % replace with actual value
    k = 57.98; % Spring constant, replace with actual value
    L0 = 0.5; % Spring resting length, replace with actual value

    angles = zeros(1, num_frames);
    torques = zeros(1, num_frames);
    
    for frame = 1:num_frames
        phi = pi/2;

        % Cartesian coordinates for wrist and forearm rods
        x_wrist = wrist_width * sin(phi) * cos(wrist_theta);
        y_wrist = wrist_width * sin(phi) * sin(wrist_theta);
        z_wrist = 0;
        
        x_forearm = forearm_width * sin(phi) * cos(forearm_theta);
        y_forearm = forearm_width * sin(phi) * sin(forearm_theta);
        z_forearm = forearm_length;
        
        % Calculations
        r_wrist = [x_wrist, y_wrist, z_wrist];
        r_forearm = [x_forearm, y_forearm, z_forearm];
        bungee = r_forearm - r_wrist;
        
        bungee_mag = norm(bungee);
        bungee_norm = bungee ./ bungee_mag;
        F_bungee = k * (bungee_mag - L0);
        F = F_bungee .* bungee_norm;

        tau = cross(r_forearm, F);
        tau = tau(3);

        error = target_angle - wrist_theta;
        error_sum = error_sum + error;
        error_diff = error - prev_error;
        tau_control = Kp * error + Ki * error_sum + Kd * error_diff;
        alpha = (tau + tau_control - damping_coeff * omega) / I;
        
        omega = omega + alpha * dt;
        wrist_theta = wrist_theta + omega * dt;
        prev_error = error;

        torques(frame) = tau;
        angles(frame) = rad2deg(wrist_theta);
    end
end
