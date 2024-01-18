close all;
clear all;
clc;
 
%% Forearm Rotation

% Define cylinder parameters
elbow_radius = 2; % in cm
wrist_radius = 1.5; % in cm
forearm_length = 30; % in cm

% Define cylinder
N = 50; % number of points
z = linspace(0, forearm_length, N); % define z points along the length of the cylinder
theta = linspace(0, 2*pi, N); % define theta points around the circumference of the cylinder
[Z, Theta] = meshgrid(z, theta); % create a grid of (z, theta) points

% Cylinder equation in polar coordinates, radius changes linearly from elbow to wrist
R = elbow_radius + (wrist_radius - elbow_radius) * Z / forearm_length;
[X, Y] = pol2cart(Theta, R); % convert polar to cartesian coordinates

% Define ODE
rotation_speed = pi / 10; % radians per second, change as needed
rotation_ode = @(t, y) rotation_speed; % constant rotation speed

% Solve ODE
tspan = [0, 10]; % time span in seconds, change as needed
y0 = 0; % initial angle is 0
[t, y] = ode45(rotation_ode, tspan, y0);

% Define figure and initial plot
fig = figure;
h = surf(NaN(size(X)), NaN(size(Y)), NaN(size(Z))); % initial plot is empty
axis equal
title('Rotated forearm')

% For each angle, rotate forearm and update plot
for i = 1:length(t)
    % Define rotation
    rotation_angle = y(i);
    rotation_matrix = [cos(rotation_angle), -sin(rotation_angle); sin(rotation_angle), cos(rotation_angle)];

    % Rotate forearm
    rotated_XY = rotation_matrix * [X(:)'; Y(:)']; % apply rotation
    rotated_X = reshape(rotated_XY(1, :), size(X)); % reshape X
    rotated_Y = reshape(rotated_XY(2, :), size(Y)); % reshape Y

    % Update plot data
    set(h, 'XData', rotated_X, 'YData', rotated_Y, 'ZData', Z);
    drawnow % force figure to update immediately

    % Optionally, pause for a bit to slow down the animation
    pause(0.1)
end

%% Bungee Wrapping

% Define cylinder and bungee cord parameters
elbow_radius = 2; % in cm
wrist_radius = 1.5; % in cm
forearm_length = 30; % in cm
resting_length = 10; % in cm
stiffness = 1; % in N/cm, change as needed
damping = 0.1; % damping coefficient, change as needed
radius = 1; % in cm, change as needed

% Set the total bungee length as an input parameter
bungee_length = 20; % in cm, change as needed

% Define initial conditions
initial_extension = bungee_length - resting_length; % calculate the initial extension based on the total bungee length
initial_velocity = 0; % in cm/s

% Define ODEs
bungee_ode = @(t, y) [y(2); -stiffness * (y(1) - resting_length) - damping * y(2)];
rotation_ode = @(t, y) -5 * pi / 180; % 5 degrees per second

% Solve ODEs
tspan = [0, 0.5]; % time span in seconds, change as needed
[t, y] = ode45(bungee_ode, tspan, [initial_extension; initial_velocity]); % solve bungee cord ODE
[t_rot, y_rot] = ode45(rotation_ode, tspan, 0); % initial rotation angle is 0

% Define figure and initial plot
fig = figure;
set(gcf, 'Position', [100, 100, 800, 600]); % position the figure and set its size
h = plot3(NaN, NaN, NaN, 'LineWidth', 2); % initial plot is empty
axis equal
xlim([-bungee_length/2, bungee_length/2]); % set x-axis limits
ylim([-bungee_length/2, bungee_length/2]); % set y-axis limits
zlim([0, bungee_length/2]); % set z-axis limits
xlabel('X (cm)')
ylabel('Y (cm)')
zlabel('Z (cm)')
title('Bungee cord extension over time')

% Reverse time and corresponding y values
t = flip(t);
y = flip(y, 1);
y_rot = flip(y_rot);

% For each time point, update plot
for i = 1:length(t)
    % Calculate rotation
    rotation_angle = y_rot(i);
    rotation_matrix = [cos(rotation_angle), -sin(rotation_angle); sin(rotation_angle), cos(rotation_angle)];

    % Calculate helix coordinates in rotating reference frame
    n_loops = max(1, 1 + t(i) / tspan(2)); % adjust number of unwrapped turns based on current time, but never go below 1 loops
    theta = linspace(0, 3/2*pi*n_loops, 100); % start from 0 and increase to maximum theta
    radius_current = radius + y(i, 1) / (2 * (resting_length + initial_extension)); % adjust radius based on current extension
    X_rot = radius_current * cos(theta);
    Y_rot = radius_current * sin(theta);
    Z_rot = y(i, 1) * theta / (2*pi*n_loops); % adjust length of the helix

    % Adjust Z_rot to keep bottom end fixed
    Z_rot = Z_rot + (resting_length - y(i, 1));

    % Rotate helix coordinates to original reference frame
    rotated_XY = rotation_matrix * [X_rot; Y_rot]; % apply rotation
    X = reshape(rotated_XY(1, :), size(X_rot)); % reshape X
    Y = reshape(rotated_XY(2, :), size(Y_rot)); % reshape Y
    Z = Z_rot; % Z coordinates are not affected by the rotation

    % Update plot data
    set(h, 'XData', X, 'YData', Y, 'ZData', Z);
    drawnow % force figure to update immediately

    % Optionally, pause for a bit to slow down the animation
    pause(0.1)
end
