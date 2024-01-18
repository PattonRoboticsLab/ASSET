close all;
clear all;
clc;

%% Initialize Input Variables

% Parameters
forearm_length = 0.55;         % length of the forearm : meters
length_wrap = forearm_length;  % length of the helical wrapping section : meters
radius_wrist = 0.04;           % radius of the forearm at the wrist : meters
radius_forearm = 0.05;         % radius of the forearm at the forearm : meters
num_elements_wrap = 25;        % number of elastic elements in the helical wrapping section
pitch = 1;                     % number of rotations around the forearm for helical wrapping
tension = 100;                 % bungee cord tension parameter
k = 50;                        % spring constant : N/m

%% Joint Positions & Cords

% Define the position of the wrist (origin point for the helix)
x_wrist = 0;                   % x-coordinate of the wrist
y_wrist = 0;                   % y-coordinate of the wrist
z_wrist = 0;                   % z-coordinate of the wrist

% Specify the starting coordinate of the helical wrapping
x_start = 0.04;                % x-coordinate of the starting point
y_start = 0;                   % y-coordinate of the starting point
z_start = 0;                   % z-coordinate of the starting point

%% Torque Calculations

% Calculate the starting angle based on the specified coordinate
start_theta = atan2(y_start - y_wrist, x_start - x_wrist);

% Calculate the theta values for the helical wrapping configuration
theta = start_theta + 2 * pi * pitch * (0:num_elements_wrap-1) / num_elements_wrap;

% Calculate the torque at each element along the forearm for the helical wrapping configuration
torque_profile_wrap = tension * ones(1, num_elements_wrap);

% Calculate the rest length for each individual segment for the helical wrapping configuration
rest_lengths_wrap = length_wrap / num_elements_wrap * ones(1, num_elements_wrap);

% Calculate the torque contribution of each spring segment for the helical wrapping configuration
segment_torques_wrap = zeros(1, num_elements_wrap);

for i = 1:num_elements_wrap
    distance = sqrt((radius_wrist + (radius_forearm - radius_wrist) * z_wrist/forearm_length - length_wrap * sin(theta(i)))^2 + (length_wrap * cos(theta(i)))^2);
    segment_torques_wrap(i) = k * (distance - rest_lengths_wrap(i));
end

% Calculate the overall bungee torque as the summation of segment torques for the helical wrapping configuration
overall_torque_wrap = sum(segment_torques_wrap);

% Calculate the energy for the helical wrapping configuration
energy_wrap = 0.5 * sum(segment_torques_wrap.^2) / k;

% Calculate the torque for the single bungee configuration
torque_single_bungee = tension * forearm_length;

% Calculate the offset of the bungee cord from the centerline of the forearm
offset_single_bungee = (radius_forearm - radius_wrist) * z_wrist / forearm_length;

% Calculate the torque contribution of the single bungee configuration
segment_torque_single = k * (offset_single_bungee - length_wrap);

% Calculate the overall bungee torque for the single bungee configuration
overall_torque_single = segment_torque_single;

% Calculate the energy for the single bungee configuration
energy_single_bungee = 0.5 * segment_torque_single^2 / k;

%% Figure 1 Plotting Functions

% Plot the forearm, wrist, and bungee cord positions
figure(1);
subplot(1,2,1);

% Plot the bungee cord segments for the helical wrapping configuration as springs
z_cord_wrap = linspace(0, length_wrap, num_elements_wrap);
colors_wrap = {'r', 'b'};  % Colors for alternating segments

for i = 1:num_elements_wrap-1
    x_cord_start = x_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_cord_wrap(i)/forearm_length) * cos(theta(i));
    y_cord_start = y_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_cord_wrap(i)/forearm_length) * sin(theta(i));
    x_cord_end = x_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_cord_wrap(i+1)/forearm_length) * cos(theta(i+1));
    y_cord_end = y_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_cord_wrap(i+1)/forearm_length) * sin(theta(i+1));
    z_cord_start = z_cord_wrap(i);
    z_cord_end = z_cord_wrap(i+1);
    
    x_segment = [x_cord_start, x_cord_end];
    y_segment = [y_cord_start, y_cord_end];
    z_segment = [z_cord_start, z_cord_end];
    
    % Alternate the colors of the bungee cord segments
    color_index = mod(i, 2) + 1;
    color_wrap = colors_wrap{color_index};
    
    plot3(x_segment, y_segment, z_segment, 'Color', color_wrap, 'LineWidth', 2); % Bungee Cord Segment (Helical Wrapping)
    hold on;
    plot3(x_cord_start, y_cord_start, z_cord_start, '.', 'Color', color_wrap, 'MarkerSize', 15); % Bungee Cord Segment (Helical Wrapping)
end

% Plot the bungee cord for the single bungee configuration (straight line)
x_cord_single = [x_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_wrist/forearm_length) * cos(start_theta), ...
                x_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_wrist/forearm_length) * cos(start_theta + 2*pi*pitch)];
y_cord_single = [y_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_wrist/forearm_length) * sin(start_theta), ...
                y_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_wrist/forearm_length) * sin(start_theta + 2*pi*pitch)];
z_cord_single = [z_wrist, z_wrist + length_wrap];

plot3(x_cord_single, y_cord_single, z_cord_single, 'g-', 'LineWidth', 2); % Bungee Cord (Single)

% Plot the physical forearm as a cylinder
z_forearm = linspace(0, forearm_length, 100);  % Z-coordinates for the cylinder
theta_forearm = linspace(0, 2 * pi, 100);  % Angles for the cylinder

% Calculate the varying radius along the forearm
radius_forearm = radius_wrist + (radius_forearm - radius_wrist) * z_forearm/forearm_length;

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
title('Bungee Cord Configurations on Forearm');
legend('Bungee Cord (Helical Wrapping)', 'Bungee Cord (Single)', 'Forearm', 'Wrist');
hold off;

% Plot the energy states
subplot(1,2,2);
energy_states = [energy_wrap, energy_single_bungee];
bar(1:numel(energy_states), energy_states);
xticklabels({'Helical Wrapping', 'Single Bungee'});
ylabel('Energy (J)');
title('Energy States');

% Display the overall bungee torque and energy states
disp(['Overall Bungee Torque (Helical Wrapping): ', num2str(overall_torque_wrap), ' Nm']);
disp(['Energy (Helical Wrapping): ', num2str(energy_wrap), ' J']);
disp(['Torque (Single Bungee): ', num2str(overall_torque_single), ' Nm']);
disp(['Energy (Single Bungee): ', num2str(energy_single_bungee), ' J']);

% Adjust the subplot spacing
subplot(1,2,1);
pbaspect([1 1 1]);
subplot(1,2,2);
pbaspect([1 1 1]);

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
saveas(figure(1), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v5_Energy.png')); % save to computer

 
% %% Figure 2 Plotting Functions
% 
% % Plot the forearm, wrist, and bungee cord positions
% figure(2);
% subplot(1,2,1);
% 
% % Plot the bungee cord segments for the helical wrapping configuration as springs
% z_cord_wrap = linspace(0, length_wrap, num_elements_wrap);
% colors_wrap = {'r', 'b'};  % Colors for alternating segments
% 
% for i = 1:num_elements_wrap-1
%     x_cord_start = x_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_cord_wrap(i)/forearm_length) * cos(theta(i));
%     y_cord_start = y_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_cord_wrap(i)/forearm_length) * sin(theta(i));
%     x_cord_end = x_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_cord_wrap(i+1)/forearm_length) * cos(theta(i+1));
%     y_cord_end = y_wrist + (radius_wrist + (radius_forearm - radius_wrist) * z_cord_wrap(i+1)/forearm_length) * sin(theta(i+1));
%     z_cord_start = z_cord_wrap(i);
%     z_cord_end = z_cord_wrap(i+1);
%     
%     x_segment = [x_cord_start, x_cord_end];
%     y_segment = [y_cord_start, y_cord_end];
%     z_segment = [z_cord_start, z_cord_end];
%     
%     % Alternate the colors of the bungee cord segments
%     color_index = mod(i, 2) + 1;
%     color_wrap = colors_wrap{color_index};
%     
%     plot3(x_segment, y_segment, z_segment, 'Color', color_wrap, 'LineWidth', 2); % Bungee Cord Segment (Helical Wrapping)
%     hold on;
%     plot3(x_cord_start, y_cord_start, z_cord_start, '.', 'Color', color_wrap, 'MarkerSize', 15); % Bungee Cord Segment (Helical Wrapping)
% end
% 
% % Plot the physical forearm as a cylinder
% z_forearm = linspace(0, forearm_length, 100);  % Z-coordinates for the cylinder
% theta_forearm = linspace(0, 2 * pi, 100);  % Angles for the cylinder
% 
% % Calculate the varying radius along the forearm
% radius_forearm = radius_wrist + (radius_forearm - radius_wrist) * z_forearm/forearm_length;
% 
% for i = 1:numel(z_forearm)
%     x_forearm = radius_forearm(i) * cos(theta_forearm);
%     y_forearm = radius_forearm(i) * sin(theta_forearm);
%     z_forearm_current = z_forearm(i) * ones(size(theta_forearm));
%     
%     % Plot each segment of the forearm cylinder
%     if i > 1
%         surf([x_forearm_prev; x_forearm], [y_forearm_prev; y_forearm], [z_forearm_prev; z_forearm_current], 'FaceColor', [0.7,0.7,0.7], 'EdgeColor', 'none');
%         alpha(0.5);  % Set transparency for better visibility
%     end
%     
%     x_forearm_prev = x_forearm;
%     y_forearm_prev = y_forearm;
%     z_forearm_prev = z_forearm_current;
% end
% 
% plot3(x_wrist, y_wrist, z_wrist, 'ro'); % Wrist
% plot3([x_wrist, x_wrist], [y_wrist, y_wrist], [z_wrist, z_wrist + forearm_length], 'k-'); % Forearm
% plot3(x_wrist, y_wrist, forearm_length, 'ro'); % Wrist
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Bungee Cord on Forearm');
% legend('Bungee Cord (Helical Wrapping)', 'Forearm', 'Wrist');
% hold off;
% 
% % Plot the bungee torque profile
% subplot(1,2,2);
% plot(1:num_elements_wrap, segment_torques_wrap, 'b.-');
% xlabel('Element');
% ylabel('Segment Torque (Nm)');
% title('Bungee Segment Torque Profile');
% 
% % Display overall bungee torque
% disp(['Overall Bungee Torque: ', num2str(overall_torque_wrap), ' Nm']);
% 
% % Adjust the subplot spacing
% subplot(1,2,1);
% pbaspect([1 1 1]);
% subplot(1,2,2);
% pbaspect([1 1 1]);
% 
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% saveas(figure(2), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v5_Torque.png')); % save to computer
