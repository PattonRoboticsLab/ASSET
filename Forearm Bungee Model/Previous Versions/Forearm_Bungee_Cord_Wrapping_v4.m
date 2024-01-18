close all;
clear all;
clc;

%% Initialize Input Variables

% Parameters
forearm_length = 0.55;         % length of the forearm : meters
length_wrap = forearm_length;            % length of the helical wrapping section : meters
radius_forearm_wrist = 0.04;   % radius of the forearm at the wrist : meters
radius_forearm_forearm = 0.05; % radius of the forearm at the forearm : meters
num_elements_wrap = 25;       % number of elastic elements in the helical wrapping section
pitch = 1;                    % number of rotations around the forearm for helical wrapping
tension = 100;                % bungee cord tension parameter
k = 50;                       % spring constant : N/m

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
rest_lengths = length_wrap / num_elements_wrap * ones(1, num_elements_wrap);

% Calculate the torque contribution of each spring segment
segment_torques = zeros(1, num_elements_wrap);

for i = 1:num_elements_wrap
    distance = sqrt((radius_forearm_wrist + (radius_forearm_forearm - radius_forearm_wrist) * z_wrist/forearm_length - length_wrap * sin(theta(i)))^2 + (length_wrap * cos(theta(i)))^2);
    segment_torques(i) = k * (distance - rest_lengths(i));
end

% Calculate the overall bungee torque as the summation of segment torques
overall_torque = sum(segment_torques);

%% Plotting Functions

% Plot the forearm, wrist, and bungee cord positions
figure(1);
subplot(1,2,1);

% Plot the bungee cord segments as springs
z_cord = linspace(0, length_wrap, num_elements_wrap);
colors = {'r', 'b'};  % Colors for alternating segments

for i = 1:num_elements_wrap-1
    x_cord_start = x_wrist + (radius_forearm_wrist + (radius_forearm_forearm - radius_forearm_wrist) * z_cord(i)/forearm_length) * cos(theta(i));
    y_cord_start = y_wrist + (radius_forearm_wrist + (radius_forearm_forearm - radius_forearm_wrist) * z_cord(i)/forearm_length) * sin(theta(i));
    x_cord_end = x_wrist + (radius_forearm_wrist + (radius_forearm_forearm - radius_forearm_wrist) * z_cord(i+1)/forearm_length) * cos(theta(i+1));
    y_cord_end = y_wrist + (radius_forearm_wrist + (radius_forearm_forearm - radius_forearm_wrist) * z_cord(i+1)/forearm_length) * sin(theta(i+1));
    z_cord_start = z_cord(i);
    z_cord_end = z_cord(i+1);
    
    x_segment = [x_cord_start, x_cord_end];
    y_segment = [y_cord_start, y_cord_end];
    z_segment = [z_cord_start, z_cord_end];
    
    % Alternate the colors of the bungee cord segments
    color_index = mod(i, 2) + 1;
    color = colors{color_index};
    
    plot3(x_segment, y_segment, z_segment, 'Color', color, 'LineWidth', 2); % Bungee Cord Segment
    hold on;
    plot3(x_cord_start, y_cord_start, z_cord_start, '.', 'Color', color, 'MarkerSize', 15); % Bungee Cord Segment
end

% Plot the physical forearm as a cylinder
z_forearm = linspace(0, forearm_length, 100);  % Z-coordinates for the cylinder
theta_forearm = linspace(0, 2 * pi, 100);  % Angles for the cylinder

% Calculate the varying radius along the forearm
radius_forearm = radius_forearm_wrist + (radius_forearm_forearm - radius_forearm_wrist) * z_forearm/forearm_length;

for i = 1:numel(z_forearm)
    x_forearm = radius_forearm(i) * cos(theta_forearm);
    y_forearm = radius_forearm(i) * sin(theta_forearm);
    z_forearm_current = z_forearm(i) * ones(size(theta_forearm));
    
    % Plot each segment of the forearm cylinder
    if i > 1
        surf([x_forearm_prev; x_forearm], [y_forearm_prev; y_forearm], [z_forearm_prev; z_forearm_current], 'FaceColor', [0.7,0.7,0.7], 'EdgeColor', 'none');
        alpha(0.5);  % Set transparency for better visibility
    end
    
    x_forearm_prev = x_forearm;
    y_forearm_prev = y_forearm;
    z_forearm_prev = z_forearm_current;
end

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
plot(1:num_elements_wrap, segment_torques, 'b.-');
xlabel('Element');
ylabel('Segment Torque (Nm)');
title('Bungee Segment Torque Profile');

% Display overall bungee torque
disp(['Overall Bungee Torque: ', num2str(overall_torque), ' Nm']);

% Adjust the subplot spacing
subplot(1,2,1);
pbaspect([1 1 1]);
subplot(1,2,2);
pbaspect([1 1 1]);

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
saveas(figure(1), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v4.png')); % save to computer
