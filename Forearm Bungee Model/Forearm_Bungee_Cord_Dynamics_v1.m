%% ExoNET Program to Compare Forearm Angle vs. Output Torque 

close all; 
clear all;
clc;

%% Initialize & Establish Input Variables

% Rod lengths
forearm_length = 12;  % length of the forearm : inches
wrist_width = 2.5;  % width of the wrist : inches
forearm_width = 4; % width of the forearm : inches

% Convert Lengths
u = symunit; % import units collection
forearm_length = double(separateUnits(unitConvert(forearm_length*u.in,u.m)));  % length of the forearm : m
wrist_width = double(separateUnits(unitConvert(wrist_width*u.in,u.m)))/2;  % width of the wrist rod : m
forearm_width = double(separateUnits(unitConvert(forearm_width*u.in,u.m)))/2; % width of the forearm rod : m

% Bungee cord properties
L0 = 0.5;  % unstretched length
k = 100;   % stiffness

% Initialize angular velocity and angular acceleration
omega = 0.1;  % Non-zero initial angular velocity in radians/s to kick-start the system
alpha = 0;  % Angular acceleration in radians/s^2
I = 5;  % Moment of inertia of the wrist rod (reduced for testing)
dt = 0.1;  % Time step, same as your pause_time
damping_coeff = 3; % Damping coefficient

% Initialize angles
wrist_theta = deg2rad(180);  % Initialize at 180 degrees
forearm_theta = 0;

% Initialize torque and angle arrays
num_frames = 90;  % total number of frames in the animation
torques = zeros(1, num_frames);
angles = zeros(1, num_frames);

% Initialize PID controller parameters
Kp = 1;
Ki = 0;
Kd = 0.1;
error_sum = 0;
prev_error = 0;

% Target angle (in radians)
target_angle = deg2rad(0);  % 0 degrees

%% Plot Results

% Initialize the plot
figure;
grid on;
axis equal;
xlim([-forearm_width, forearm_width]);
ylim([-forearm_width, forearm_width]);
zlim([0, forearm_length]);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Rods with Bungee Cord in 3D');
hold on;

% Plot the vertical rod
plot3([0, 0], [0, 0], [0, forearm_length], 'k', 'LineWidth', 2);
view(3);

% Animation loop
for frame = 1:num_frames
    
    phi = pi/2;  % Polar angle set to 90 degrees to keep the rod horizontal
    
    % Convert spherical coordinates to Cartesian for wrist rod
    x_wrist = wrist_width * sin(phi) * cos(wrist_theta);
    y_wrist = wrist_width * sin(phi) * sin(wrist_theta);
    z_wrist = 0;  % This will be zero due to phi = pi/2
    
    % Convert spherical coordinates to Cartesian for forearm rod
    x_forearm = forearm_width * sin(phi) * cos(forearm_theta);
    y_forearm = forearm_width * sin(phi) * sin(forearm_theta);
    z_forearm = forearm_length;
    
    % Segment Calculations
    r_wrist = [x_wrist, y_wrist, z_wrist]; % wrist position vector
    r_forearm = [x_forearm, y_forearm, z_forearm]; % forearm position vector
    bungee = r_forearm - r_wrist; % spring position matrix

    % Force Calculations
    bungee_mag = norm(bungee); % magnitude of bungee position
    bungee_norm = bungee ./ bungee_mag; % normalized spring position values - unit vector
    F_bungee = k*(bungee_mag-L0); % spring force calculation
    F = F_bungee .* bungee_norm; % force components
    
    % Calculate torque vector
    tau = cross(r_forearm, F);  % Torque cross-product calculation;
    tau = tau(3);  % Taking magnitude of the torque
    
    % Debugging
    fprintf('Frame: %d, Torque: %f, Omega: %f, Alpha: %f\n', frame, tau, omega, alpha);
    fprintf('r_wrist: [%f, %f, %f], r_forearm: [%f, %f, %f]\n', r_wrist(1), r_wrist(2), r_wrist(3), r_forearm(1), r_forearm(2), r_forearm(3));
    
    % PID Controller
    error = target_angle - wrist_theta;
    error_sum = error_sum + error;
    error_diff = error - prev_error;
    tau_control = Kp * error + Ki * error_sum + Kd * error_diff;
    
    % Update angular acceleration based on torque and damping
    alpha = (tau + tau_control - damping_coeff * omega) / I;
    
    % Update angular velocity and wrist angle
    omega = omega + alpha * dt;
    wrist_theta = wrist_theta + omega * dt;
     
    % Store current error for next iteration
    prev_error = error;
    
    % Store torque and angle for plotting
    torques(frame) = tau;
    angles(frame) = rad2deg(wrist_theta);
    
    % Plot the horizontal rods in their new positions
    wrist = plot3([0, x_wrist], [0, y_wrist], [0, z_wrist], 'b', 'LineWidth', 2);
    forearm = plot3([0, x_forearm], [0, y_forearm], [forearm_length, z_forearm], 'g', 'LineWidth', 2);
    
    % Plot the bungee cord
    bungee_cord = plot3([x_wrist, x_forearm], [y_wrist, y_forearm], [z_wrist, z_forearm], 'r', 'LineWidth', 2);
    
    drawnow;
    pause(dt);  % Small delay for visualization
    
    % Clear the previous frame's horizontal rods and bungee cord for the next frame
    if frame < num_frames
        delete(wrist);
        delete(forearm);
        delete(bungee_cord);
    end
end

hold off;

% Plot torque vs angle
figure(2);
plot(angles, torques);
xlabel('Wrist Rotation Angle (degrees)');
ylabel('Resulting Torque (Nm)');
title('Wrist Rotation Angle vs Resulting Torque');
grid on;

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
saveas(figure(2), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Torque_v2.png')); % save to computer
