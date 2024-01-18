close all;
clear all;
clc;

%% Initialize Input Variables

% Parameters
forearm_length = 0.55;         % length of the forearm : meters
length_wrap = 0.5;            % length of the helical wrapping section : meters
radius_forearm = 0.05;        % radius of the forearm : meters
num_elements_wrap = 50;       % number of elastic elements in the helical wrapping section
pitch = 1;                    % number of rotations around the forearm for helical wrapping
tension = 100;                % bungee cord tension parameter
k = 50;                      % spring constant : N/m

%% Joint Positions & Cords

% Define the position of the wrist (origin point for the helix)
x_wrist = 0;                  % x-coordinate of the wrist
y_wrist = 0;                  % y-coordinate of the wrist
z_wrist = 0;                  % z-coordinate of the wrist

%% Torque Calculations

% Calculate the torque at each element along the forearm
theta = 2 * pi * pitch * (0:num_elements_wrap-1) / num_elements_wrap;
torque_profile = tension * ones(1, num_elements_wrap);

% Calculate the rest length for each individual segment
rest_lengths = length_wrap * ones(1, num_elements_wrap);

for i = 1:num_elements_wrap
    distance = sqrt((radius_forearm - length_wrap * sin(theta(i)))^2 + (length_wrap * cos(theta(i)))^2);
    torque_profile(i) = k * (distance - rest_lengths(i));
end


%% Plotting Functions

% Plot the forearm, wrist, and bungee cord positions
figure(1);
subplot(1,2,1);

% Plot the bungee cord
theta_cord = linspace(0, 2 * pi, num_elements_wrap);
x_cord = x_wrist + radius_forearm * cos(theta_cord);
y_cord = y_wrist + radius_forearm * sin(theta_cord);
z_cord = linspace(0, length_wrap, num_elements_wrap);
plot3(x_cord, y_cord, z_cord, 'k-', 'LineWidth', 2); % Bungee Cord
hold on;
plot3(x_cord, y_cord, z_cord, 'k.', 'MarkerSize', 15); % Bungee Cord

% Plot the physical forearm as a cylinder
z_forearm = linspace(0, forearm_length, 100);  % Z-coordinates for the cylinder
theta_forearm = linspace(0, 2 * pi, 100);  % Angles for the cylinder
[theta_forearm_grid, z_forearm_grid] = meshgrid(theta_forearm, z_forearm);
x_forearm = radius_forearm * cos(theta_forearm_grid);
y_forearm = radius_forearm * sin(theta_forearm_grid);
surf(x_forearm, y_forearm, z_forearm_grid, 'FaceColor', [0.7,0.7,0.7], 'EdgeColor', 'none');  % Plot the cylinder
alpha(0.5);  % Set transparency for better visibility

plot3(x_wrist, y_wrist, z_wrist, 'ro'); % Wrist
plot3([x_wrist, x_wrist], [y_wrist, y_wrist], [z_wrist, z_wrist + forearm_length], 'k-'); % Forearm
plot3(x_wrist, y_wrist, forearm_length, 'ro'); % Wrist
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Bungee Cord and Forearm Positions');
legend('Bungee Cord', 'Forearm', 'Wrist');
hold off;

% Plot the bungee torque profile
subplot(1,2,2);
plot(1:num_elements_wrap, torque_profile, 'b.-');
xlabel('Element');
ylabel('Torque (Nm)');
title('Bungee Torque Profile');

% Adjust the subplot spacing
subplot(1,2,1);
pbaspect([1 1 1]);
subplot(1,2,2);
pbaspect([1 1 1]);

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
saveas(figure(1), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v3.png')); % save to computer
