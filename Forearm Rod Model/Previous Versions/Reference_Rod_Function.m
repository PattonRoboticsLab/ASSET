clear all;
close all; 
clc;

% Define the points and rod length
point1 = [1, 4, 5];
point2 = [0, 0, 0];
rod_length = 10; % Replace this with the actual length of your rod

% Calculate the distance between the two points
d = norm(point2 - point1);

% Check if the rod can be bent to fit between the two points
if rod_length < d
    error('The rod is too short to fit between the points.');
end

% Initialize the height of the peak deformation (sagitta)
h = (rod_length^2 - d^2) / (2 * rod_length);  % Initial guess

% The vector from point1 to point2
vec = point2 - point1;

% Find a vector that is perpendicular to vec and has a positive z component
perp_vec = cross(cross(vec, [0 0 1]), vec);
perp_vec = perp_vec / norm(perp_vec);  % Normalize the perpendicular vector

% Ensure the deformation is upwards by checking the z-component
if perp_vec(3) < 0
    perp_vec = -perp_vec;
end

% Define tolerance and initialize error
tolerance = 1e-5;
error = inf;

% Iterative approach to adjust 'h'
while error > tolerance
    % Create the parameter t for interpolation
    t = linspace(0, 1, 100);

    % Initialize the arc points
    arc_points = zeros(3, length(t));

    % Calculate the arc points
    for i = 1:length(t)
        % Interpolate linearly between the two points
        arc_points(:, i) = point1' * (1 - t(i)) + point2' * t(i);
        
        % Add the deformation in the direction perpendicular to the straight line
        arc_points(:, i) = arc_points(:, i) + (4 * h * (t(i) - t(i)^2)) * perp_vec';
    end

    % Initialize the length of the deformed rod
    deformed_rod_length = 0;

    % Calculate the length of the deformed rod
    for i = 2:length(t)
        % Calculate the distance between consecutive points
        segment_length = norm(arc_points(:, i) - arc_points(:, i - 1));
        % Add the segment length to the total length
        deformed_rod_length = deformed_rod_length + segment_length;
    end
    
    % Update the error
    error = abs(rod_length - deformed_rod_length);
    
    % Adjust 'h' based on the error
    if deformed_rod_length > rod_length
        h = h * (1 - error / deformed_rod_length);
    else
        h = h * (1 + error / rod_length);
    end
end

% Calculate the midpoint of the line connecting the two points
midpoint = (point1 + point2) / 2;

% Plot the deformed rod
plot3(arc_points(1, :), arc_points(2, :), arc_points(3, :), 'b-', 'LineWidth', 2);
hold on;

% Plot the points (endpoints)
plot3(point1(1), point1(2), point1(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(point2(1), point2(2), point2(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Plot the straight line between the two points
plot3([point1(1), point2(1)], [point1(2), point2(2)], [point1(3), point2(3)], 'g--', 'LineWidth', 2);

% Plot the midpoint
plot3(midpoint(1), midpoint(2), midpoint(3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% Plot the deformed midpoint
deformed_midpoint = midpoint + h * perp_vec;
plot3(deformed_midpoint(1), deformed_midpoint(2), deformed_midpoint(3), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm');

% Plot the perpendicular line
plot3([midpoint(1), deformed_midpoint(1)], [midpoint(2), deformed_midpoint(2)], [midpoint(3), deformed_midpoint(3)], 'm--', 'LineWidth', 2);

hold off;

% Set the plot properties
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Deformed Rod Between Two Points');
axis equal;
view(3); % Adjust the view to 3D

% Display the length of the deformed rod
fprintf('Length of the deformed rod: %f\n', deformed_rod_length);


