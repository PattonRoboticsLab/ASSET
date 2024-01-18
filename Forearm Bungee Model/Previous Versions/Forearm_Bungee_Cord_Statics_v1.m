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
deltaL = 0.3;  % initial stretch

% Define the rotation parameters
rotation_angle = -180; % span of rotation for the system
num_frames = 90;  % total number of frames in the animation
pause_time = 0.1;  % pause time in seconds between frames
wrist_angle_step = rotation_angle / num_frames; % speed of wrist rotation
forearm_angle_step = 0; % final_rotation_angle / num_frames;  % speed of forearm rotation

% Initialize angles
wrist_theta = deg2rad(180);  % Initialize at 180 degrees;
forearm_theta = 0;

% Initialize torque and angle arrays
torques = zeros(1, num_frames);
angles = zeros(1, num_frames);

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
    
    % Update angles based on different speeds
    wrist_theta = wrist_theta + deg2rad(wrist_angle_step);
    forearm_theta = forearm_theta + deg2rad(forearm_angle_step);
    
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
    tau = cross(r_forearm, F); % torque cross-product calculation;
    tau = tau(3); % taking magnitude of the torque
    
    % Store torque and angle for plotting
    torques(frame) = tau;
    angles(frame) = rad2deg(wrist_theta);
    
    % Plot the horizontal rods in their new positions
    wrist = plot3([0, x_wrist], [0, y_wrist], [0, z_wrist], 'b', 'LineWidth', 2);
    forearm = plot3([0, x_forearm], [0, y_forearm], [forearm_length, z_forearm], 'g', 'LineWidth', 2);
    
    % Plot the bungee cord
    bungee_cord = plot3([x_wrist, x_forearm], [y_wrist, y_forearm], [z_wrist, z_forearm], 'r', 'LineWidth', 2);
    
    drawnow;
    pause(pause_time);  % Small delay for visualization
    
    % Clear the previous frame's horizontal rods and bungee cord for the next frame
    if frame < num_frames
        delete(wrist);
        delete(forearm);
        delete(bungee_cord);
    end
end

hold off;

% Plot torque vs angle
figure;
plot(angles, torques);
xlabel('Wrist Rotation Angle (degrees)');
ylabel('Resulting Torque (Nm)');
title('Wrist Rotation Angle vs Resulting Torque');
grid on;
