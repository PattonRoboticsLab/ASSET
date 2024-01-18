close all;
clear all;
clc;

%% Initialize Input Variables

% Parameters
length_wrap = 0.3;            % Length of the helical wrapping section (in meters)
length_straight = 0.2;        % Length of the straight section (in meters)
radius_forearm = 0.05;        % Radius of the forearm (in meters)
num_elements_wrap = 100;      % Number of elastic elements in the helical wrapping section
num_elements_straight = 50;   % Number of elastic elements in the straight section
pitch = 1;                    % Number of rotations around the forearm for helical wrapping
tension = 100;                % Bungee cord tension parameter


%% Joint Positions & Cords

% Define the position of the wrist
x_wrist = 0;                  % x-coordinate of the wrist
y_wrist = 0;                  % y-coordinate of the wrist
z_wrist = 0;                  % z-coordinate of the wrist

% Calculate the length of each elastic element
element_length_wrap = length_wrap / num_elements_wrap;
element_length_straight = length_straight / num_elements_straight;

% Initialize arrays to store the position and torque of each element along the forearm
x_cord = zeros(1, num_elements_wrap + num_elements_straight);
y_cord = zeros(1, num_elements_wrap + num_elements_straight);
z_cord = zeros(1, num_elements_wrap + num_elements_straight);
torque_profile = zeros(1, num_elements_wrap + num_elements_straight);

% Calculate the helical wrapping of the bungee cord
theta = 2 * pi * pitch * (0:num_elements_wrap-1) / num_elements_wrap;
x_cord(1:num_elements_wrap) = x_wrist + radius_forearm * cos(theta);
y_cord(1:num_elements_wrap) = y_wrist + radius_forearm * sin(theta);
z_cord(1:num_elements_wrap) = linspace(0, length_wrap, num_elements_wrap);

% Calculate the position of the first point on the straight section
x_cord(num_elements_wrap+1) = x_wrist + radius_forearm * cos(theta(end));
y_cord(num_elements_wrap+1) = y_wrist + radius_forearm * sin(theta(end));
z_cord(num_elements_wrap+1) = length_wrap;

% Calculate the tangent vector of the straight section
tangent_vector_straight = [0, 0, element_length_straight];

% Calculate the positions of the remaining points on the straight section
for i = num_elements_wrap+2:num_elements_wrap+num_elements_straight
    x_cord(i) = x_cord(i-1) + tangent_vector_straight(1);
    y_cord(i) = y_cord(i-1) + tangent_vector_straight(2);
    z_cord(i) = z_cord(i-1) + tangent_vector_straight(3);
end


%% Torque Calculations

% Calculate the torque at each element along the forearm (including the straight section)
torque_profile = tension * ones(1, num_elements_wrap + num_elements_straight);
for i = 1:num_elements_wrap
    distance = sqrt((radius_forearm - z_cord(i) * sin(theta(i)))^2 + (z_cord(i) * cos(theta(i)))^2);
    torque_profile(i) = tension * distance;
end
for i = num_elements_wrap+1:num_elements_wrap+num_elements_straight
    distance = sqrt((radius_forearm - length_wrap)^2 + (z_cord(i) - length_wrap)^2);
    torque_profile(i) = tension * distance;
end

% Modify the torque profile at the transition point to ensure continuity
torque_profile(num_elements_wrap) = torque_profile(num_elements_wrap + 1);


%% Plotting Functions

% Plot the forearm, wrist, and bungee cord positions
figure(1);
subplot(1,2,1);
plot3([x_wrist, x_wrist], [y_wrist, y_wrist], [z_wrist, z_wrist + length_wrap + length_straight], 'k-'); % Forearm
hold on;
plot3(x_wrist, y_wrist, z_wrist, 'ro'); % Wrist
plot3(x_cord, y_cord, z_cord, 'b.-'); % Bungee Cord
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Bungee Cord Positions');
legend('Forearm', 'Wrist', 'Bungee Cord', 'Location', 'north');
hold off;

% Plot the bungee torque profile
subplot(1,2,2);
plot(1:(num_elements_wrap + num_elements_straight), torque_profile, 'b.-');
xlabel('Element');
ylabel('Torque (Nm)');
title('Bungee Torque Profile');

% Adjust the subplot spacing
subplot(1,2,1);
pbaspect([1 1 1]);
subplot(1,2,2);
pbaspect([1 1 1]);

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
saveas(figure(1),fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v1.png')); % save to computer

