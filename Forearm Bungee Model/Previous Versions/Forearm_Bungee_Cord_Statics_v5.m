%% ExoNET Program to Compare Forearm Angle vs. Output Torque 

close all;  % Close all existing figures
clear all;  % Clear all variables
clc;       % Clear the command window

%% Initialize & Establish Input Variables

% Rod lengths and properties
forearm_length = 12;  % Length of the forearm rod in inches
wrist_width = 2.5;    % Width of the wrist rod in inches
forearm_width = 4;    % Width of the forearm rod in inches

% Convert Lengths to Meters
u = symunit;  % Create symbolic unit variable
forearm_length = double(separateUnits(unitConvert(forearm_length*u.in,u.m)));  % Convert forearm length to meters
wrist_width = double(separateUnits(unitConvert(wrist_width*u.in,u.m)))/2;  % Convert wrist width to meters and halve it
forearm_width = double(separateUnits(unitConvert(forearm_width*u.in,u.m)))/2;  % Convert forearm width to meters and halve it

% Bungee cord properties
L0 = 0.5;  % Unstretched length of the bungee cord in meters
k = 100;   % Stiffness constant of the bungee cord
deltaL = 0.3;  % Initial stretch of the bungee cord in meters

% Define the rotation parameters
rotation_angle = -180;  % Total rotation angle in degrees
num_frames = 90;  % Number of animation frames
pause_time = 0.1;  % Pause time between frames in seconds
wrist_angle_step = rotation_angle / num_frames;  % Angle step for wrist rotation

% Initialize angles
wrist_theta = deg2rad(180);  % Initial wrist angle in radians
forearm_theta = 0;  % Initial forearm angle in radians

% Initialize arrays to store torques and angles
torques = zeros(1, num_frames);  % Initialize array for storing torques
angles = zeros(1, num_frames);   % Initialize array for storing angles

% Initialize helix parameters
f_min = 0.5;  % Minimum frequency for at least 0.5 wraps of helix
f_max = 1;    % Maximum frequency to ensure less than 1 wrap of helix
num_helix_points = 100;  % Number of points for the helix
t_helix = linspace(0, 1, num_helix_points);  % Parameter for helix positions
x_helix = zeros(1, num_helix_points);  % Initialize x-coordinates of helix
y_helix = zeros(1, num_helix_points);  % Initialize y-coordinates of helix
z_helix = zeros(1, num_helix_points);  % Initialize z-coordinates of helix
initial_pitch = forearm_length;  % Initial pitch of the helix

%% Plot Results

% Initialize 3D plot
figure;  % Create new figure
grid on;  % Turn on grid
axis equal;  % Make axes equal
xlim([-forearm_width, forearm_width]);  % Set x-axis limits
ylim([-forearm_width, forearm_width]);  % Set y-axis limits
zlim([0, forearm_length]);  % Set z-axis limits
xlabel('X');  % Label x-axis
ylabel('Y');  % Label y-axis
zlabel('Z');  % Label z-axis
title('Rods with Bungee Cord in 3D');  % Add title to plot
hold on;  % Hold the plot to allow further plotting

% Initialize cylinder parameters
z_cylinder = linspace(0, forearm_length, 50);  % Z-axis positions for cylinder
theta_cylinder = linspace(0, 2*pi, 50);  % Angles for cylinder
[Z, theta] = meshgrid(z_cylinder, theta_cylinder);  % Create meshgrid for cylinder
R = (forearm_width - wrist_width)/forearm_length .* Z + wrist_width;  % Calculate cylinder radius
X = R .* cos(theta);  % Calculate x-coordinates of cylinder
Y = R .* sin(theta);  % Calculate y-coordinates of cylinder

% Plot central rod
plot3([0, 0], [0, 0], [0, forearm_length], 'k', 'LineWidth', 2);  % Plot the central rod
view(3);  % Set the 3D view

% Animation loop
for frame = 1:num_frames  % Iterate through each frame
    % Update angles and positions
    wrist_theta = wrist_theta + deg2rad(wrist_angle_step);  % Update wrist angle in radians
    phi = pi/2;  % Set phi angle (angle from z-axis)
    
    % Calculate wrist rod position
    x_wrist = wrist_width * sin(phi) * cos(wrist_theta);
    y_wrist = wrist_width * sin(phi) * sin(wrist_theta);
    z_wrist = 0;
    
    % Calculate forearm rod position
    x_forearm = forearm_width * sin(phi) * cos(forearm_theta);
    y_forearm = forearm_width * sin(phi) * sin(forearm_theta);
    z_forearm = forearm_length;
    
    % Calculate distance between wrist and forearm rods
    distance = norm([x_forearm - x_wrist, y_forearm - y_wrist, z_forearm - z_wrist]);
    
    % Calculate dynamic helix frequency based on distance
    f_dynamic = max(f_min, min(f_max, 1 / distance));
    
    % Update helix parameters
    for i = 1:num_helix_points  % Iterate through each helix point
        t = t_helix(i);  % Get parameter for current helix point
        R = (forearm_width - wrist_width) * t + wrist_width;  % Calculate radius for helix point
        x_helix(i) = R * cos(2 * pi * f_dynamic * t);  % Calculate x-coordinate for helix point
        y_helix(i) = R * sin(2 * pi * f_dynamic * t);  % Calculate y-coordinate for helix point
        z_helix(i) = initial_pitch * t;  % Calculate z-coordinate for helix point
    end
    
    % Rotate the helix according to the wrist rotation
    x_helix_rot = x_helix * cos(wrist_theta) - y_helix * sin(wrist_theta);  % Rotate x-coordinates of helix
    y_helix_rot = x_helix * sin(wrist_theta) + y_helix * cos(wrist_theta);  % Rotate y-coordinates of helix
    
    % Ensure bungee cord attaches to wrist and forearm rods
    x_helix_rot(1) = x_wrist;  % Set first x-coordinate to wrist position
    y_helix_rot(1) = y_wrist;  % Set first y-coordinate to wrist position
    z_helix(1) = z_wrist;      % Set first z-coordinate to wrist position
    x_helix_rot(end) = x_forearm;  % Set last x-coordinate to forearm position
    y_helix_rot(end) = y_forearm;  % Set last y-coordinate to forearm position
    z_helix(end) = z_forearm;      % Set last z-coordinate to forearm position
    
    % Calculate forces and torques
    F_direction = [x_forearm - x_wrist, y_forearm - y_wrist, z_forearm - z_wrist] / distance;  % Calculate force direction
    F_bungee = -k * (distance - L0);  % Calculate bungee force
    F = F_bungee * F_direction;  % Calculate total force
    r_forearm = [x_forearm, y_forearm, z_forearm];  % Position vector of forearm rod
    tau = cross(r_forearm, F);  % Calculate torque using cross product
    torques(frame) = norm(tau);  % Store the magnitude of torque for this frame

    % Plotting and animation
    h_cylinder = surf(X, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');  % Plot transparent cylinder
    colormap('jet');  % Set colormap
    
    wrist = plot3([0, x_wrist], [0, y_wrist], [0, z_wrist], 'b', 'LineWidth', 2);  % Plot wrist rod
    forearm = plot3([0, x_forearm], [0, y_forearm], [forearm_length, z_forearm], 'g', 'LineWidth', 2);  % Plot forearm rod
    
    helix_plot = plot3(x_helix_rot, y_helix_rot, z_helix, 'r', 'LineWidth', 2);  % Plot helix (bungee cord)
    
    drawnow;  % Update plot immediately
    pause(pause_time);  % Pause for a short time
    
    if frame < num_frames  % If not the last frame, delete previous plots
        delete(wrist);
        delete(forearm);
        delete(helix_plot);
        delete(h_cylinder);
    end
end

% Plot torque vs angle graph
figure;  % Create new figure
plot(angles, torques);  % Plot torque vs angle
xlabel('Wrist Rotation Angle (degrees)');  % Label x-axis
ylabel('Resulting Torque (Nm)');  % Label y-axis
title('Wrist Rotation Angle vs Resulting Torque');  % Add title
grid on;  % Turn on grid
