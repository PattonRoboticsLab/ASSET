%% ExoNET Program to Model a Bungee Cord Attached Between Wrist and Forearm for Supination

close all;
clear all;
clc;


%% Initialize Input Variables

% Constants
k = 100; % spring constant of the bungee cord (N/m)
L0 = 0.1; % resting length of the bungee cord (m)
leverArm = 0.08; % distance from the elbow joint to the axis of rotation (m)

% Define the attachment points around the elbow
numPoints = 25; % number of attachment points
wrist_radius = 0.075; % radius of the attachment points (m)
forearm_radius = 0.1; % radius of the forearm attachment points (m)
theta = linspace(0, 360, numPoints+1); % angular positions
wrist_band_location = 0; % set wrist band location along length of forearm
forearm_band_location = 0.2; % set forearm band location along length of forearm

% Initialize torque field
torqueField = zeros(numPoints, numPoints);

% Initialize arrays to store attachment point coordinates
forearm_band_points = zeros(numPoints, 3);
wrist_band_points = zeros(numPoints, 3);


%% Identify Bungee Attachment Point Positions & Resulting Torques

% Loop through attachment points
for i = 1:numPoints
    for j = 1:numPoints
        % Check if attachment points form a radial combination
        if abs(theta(i) - theta(j)) <= 90 || abs(theta(i) - theta(j)) >= 270
            % Calculate position of attachment points
            x_forearm_band = wrist_band_location;
            y_forearm_band = forearm_radius * sind(theta(i));
            z_forearm_band = forearm_radius * cosd(theta(i));
            x_wrist_band = forearm_band_location;
            y_wrist_band = wrist_radius * sind(theta(j));
            z_wrist_band = wrist_radius * cosd(theta(j));

            % Store attachment point coordinates
            forearm_band_points(i, :) = [x_forearm_band, y_forearm_band, z_forearm_band];
            wrist_band_points(j, :) = [x_wrist_band, y_wrist_band, z_wrist_band];

            % Calculate distance between attachment points
            L = sqrt((x_wrist_band - x_forearm_band)^2 + (y_wrist_band - y_forearm_band)^2 + (z_wrist_band - z_forearm_band)^2);

            % Calculate elongation of the bungee cord
            elongation = L - L0;

            % Calculate the force generated by the bungee cord
            force = k * elongation;

            % Calculate the torque generated by the bungee cord
            torque = force * leverArm;

            % Store torque value in the torque field
            torqueField(i, j) = torque;
        end
    end
end


%% Plots

% Plot the torque field
figure(1);
surf(torqueField);
xlabel('Wrist Band Attachment Points');
ylabel('Forearm Band Attachment Points');
zlabel('Torque (N.m)');
title('Torque Field Generated by Bungee Cord');
saveas(figure(1),fullfile([pwd '\MATLAB Figures'],'Torque_Field.png')); % save to computer

% Find the maximum torque and its corresponding attachment points
[maxTorque, idx] = max(torqueField(:));
[max_i, max_j] = ind2sub(size(torqueField), idx);
maxWristAttachment = theta(max_j);
maxElbowAttachment = theta(max_i);

% Display the maximum torque and its corresponding attachment points
fprintf('Maximum Torque: %.2f N.m\n', maxTorque);
fprintf('Wrist Band Attachment Angle: %.2f radians\n', maxWristAttachment);
fprintf('Forearm Band Attachment Angle: %.2f radians\n', maxElbowAttachment);

% Plot the attachment points and bungee cords in 3D
figure(2);
hold on;
scatter3(forearm_band_points(:, 1), forearm_band_points(:, 2), forearm_band_points(:, 3), 'ro', 'filled');
scatter3(wrist_band_points(:, 1), wrist_band_points(:, 2), wrist_band_points(:, 3), 'bo', 'filled');
for i = 1:numPoints
    for j = 1:numPoints
        % Check if attachment points form a radial combination
        if abs(theta(i) - theta(j)) <= 90 || abs(theta(i) - theta(j)) >= 270
            x_line = [forearm_band_points(i, 1), wrist_band_points(j, 1)];
            y_line = [forearm_band_points(i, 2), wrist_band_points(j, 2)];
            z_line = [forearm_band_points(i, 3), wrist_band_points(j, 3)];
            line(x_line, y_line, z_line, 'Color', 'k', 'LineWidth', 1);
        end
    end
end

hold off;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Attachment Points and Bungee Cords in 3D');
grid on;
axis equal;
view(3); % default view for 3d plots can also use this "view(-37.5,30);"
saveas(figure(2),fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Visualization.png')); % save to computer





