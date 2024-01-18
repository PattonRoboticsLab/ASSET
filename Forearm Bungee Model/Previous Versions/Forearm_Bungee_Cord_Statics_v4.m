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
rotation_angle = 180; % span of rotation for the system
num_frames = 90;  % total number of frames in the animation
pause_time = 0.1;  % pause time in seconds between frames
wrist_angle_step = rotation_angle / num_frames; % speed of wrist rotation
forearm_angle_step = 0; % final_rotation_angle / num_frames;  % speed of forearm rotation

% Initialize angles
wrist_theta = deg2rad(0);  % initialize at 180 degrees;
forearm_theta = 0;

% Initialize torque and angle arrays
torques = zeros(1, num_frames);
angles = zeros(1, num_frames);

% Helix Parameters
num_turns = 1;  % number of turns in the helix
pitch = forearm_length / num_turns;  % pitch of the helix


%% Plot Results

% Initialize the plot
forearm_bungee_wrap = figure; % assign figure to variable for saving purposes
forearm_bungee_wrap_sim = zeros(length(2*num_frames)); % create empty vector to write frames of simulation
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

% Create cylinder with different radii
z_cylinder = linspace(0, forearm_length, 50); % 50 points along z-axis
theta_cylinder = linspace(0, 2*pi, 50); % 50 points around the cylinder
[Z, theta] = meshgrid(z_cylinder, theta_cylinder);
R = (forearm_width - wrist_width)/forearm_length .* Z + wrist_width;
X = R .* cos(theta);
Y = R .* sin(theta);

% Plot the vertical rod
plot3([0, 0], [0, 0], [0, forearm_length], 'k', 'LineWidth', 2);
view(3);

% Animation loop
for i = 1:2*num_frames
    
    % Determine the direction of rotation
    if i <= num_frames
        wrist_theta = wrist_theta + deg2rad(wrist_angle_step);  % update wrist angle in radians
    else
        wrist_theta = wrist_theta - deg2rad(wrist_angle_step);  % reverse wrist angle in radians
    end

    % Update angles based on different speeds
    forearm_theta = forearm_theta + deg2rad(forearm_angle_step);
    phi = pi/2;  % polar angle set to 90 degrees to keep the rod horizontal
    
    % Convert spherical coordinates to Cartesian for wrist rod
    x_wrist = wrist_width * sin(phi) * cos(wrist_theta);
    y_wrist = wrist_width * sin(phi) * sin(wrist_theta);
    z_wrist = 0;  % this will be zero due to phi = pi/2
    
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
    torques(i) = tau;
    angles(i) = rad2deg(wrist_theta);

    % Rotate the cylinder
    theta_rotation = wrist_theta;
    X_rot = X .* cos(theta_rotation) - Y .* sin(theta_rotation);
    Y_rot = X .* sin(theta_rotation) + Y .* cos(theta_rotation);

    % Calculate the helix based on the current rotation angle
    t_helix_current = linspace(0, 2 * pi * (wrist_theta / (2 * pi)), 1000);  % current parameter for helix based on wrist_theta
    r_helix = (forearm_width - wrist_width) / forearm_length * (pitch * t_helix_current / (2 * pi)) + wrist_width;  % radius of helix
    x_helix = r_helix .* cos(t_helix_current);
    y_helix = r_helix .* sin(t_helix_current);
    z_helix = pitch * t_helix_current / (2 * pi);
    
    % Rotate the helix for visualization
    x_helix_rot = x_helix * cos(wrist_theta) - y_helix * sin(wrist_theta);
    y_helix_rot = x_helix * sin(wrist_theta) + y_helix * cos(wrist_theta);
    
    % Plot the current helix (bungee cord)
    helix = plot3(x_helix_rot, y_helix_rot, z_helix, 'm', 'LineWidth', 2);  % using magenta for helix
    
    % Plot the rotated cylinder (frustum)
    h_cylinder = surf(X_rot, Y_rot, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    colormap('jet');  
    set(h_cylinder, 'CData', theta);
    
    % Plot the horizontal rods in their new positions
    wrist = plot3([0, x_wrist], [0, y_wrist], [0, z_wrist], 'b', 'LineWidth', 2);
    forearm = plot3([0, x_forearm], [0, y_forearm], [forearm_length, z_forearm], 'g', 'LineWidth', 2);
    
    % Plot the bungee cord
    bungee_cord = plot3([x_wrist, x_forearm], [y_wrist, y_forearm], [z_wrist, z_forearm], 'r', 'LineWidth', 2);
    
    drawnow;
    pause(pause_time);  % small delay for visualization
    
    % Get frames of figure to save animation file
    forearm_bungee_wrap_sim(i) = getframe(forearm_bungee_wrap);
    
    % Clear the previous frame's horizontal rods and bungee cord for the next frame
    if i < 2*num_frames
        delete(wrist);
        delete(forearm);
        delete(bungee_cord);
        delete(h_cylinder);
        delete(helix);
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
