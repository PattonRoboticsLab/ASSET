%% ExoNET Program to Compare Forearm Angle vs. Output Torque 

close all; % close all existing figures
clear all; % clear all variables
clc; % clear the command window
warning('off', 'MATLAB:audiovideo:VideoWriter:mp4FramePadded'); % turn off warning


%% Initialize & Establish Input Variables

global S P plt

% Rod lengths and properties
P.L_forearm = 12; % measured forearm length : in
P.L_upper_arm = 14; % measured upper_arm length : in
P.L_rod = 15; % measured length of the actuator rod : in

P.r_wrist = 3; % measured wrist width : in
P.r_forearm = 5; % measured forearm width : in
P.r_upper_arm = 5; % measured upper arm width : in
P.r_wrist_rod = P.r_wrist; % bungee wrist attachment moment arm : in
P.r_forearm_bungee = P.r_forearm; % bungee forearm attachment moment arm : in
P.r_upper_arm_rod = P.r_upper_arm; % rod attachment 

% Bungee cord properties
P.L_bungee_rest = 6; % measured unstretched length of the bungee cord : in
P.k_bungee = 100; % bungee cord stiffness constant

% Actuator Rod Properties
P.E = 5.5e6; % Young's modulus in Pa (for fiberglass)
P.I = 1e-6; % Area moment of inertia in m^4 (I = (1/4)*pi*r^4)) REPLACE WITH CALC

% Convert Lengths to Meters
u = symunit; % create symbolic unit variable
P.L_forearm = double(separateUnits(unitConvert(P.L_forearm*u.in,u.m))); % convert to meters
P.L_bungee_rest = double(separateUnits(unitConvert(P.L_bungee_rest*u.in,u.m))); % convert to meters and halve it
P.L_upper_arm = double(separateUnits(unitConvert(P.L_upper_arm*u.in,u.m))); % convert to meters
P.L_rod = double(separateUnits(unitConvert(P.L_rod*u.in,u.m))); % convert to meters

P.r_wrist = double(separateUnits(unitConvert(P.r_wrist*u.in,u.m)))/2; % convert to meters and halve it
P.r_forearm = double(separateUnits(unitConvert(P.r_forearm*u.in,u.m)))/2; % convert to meters and halve it
P.r_upper_arm = double(separateUnits(unitConvert(P.r_upper_arm*u.in,u.m)))/2; % convert to meters and halve it
P.r_upper_arm_rod = double(separateUnits(unitConvert(P.r_upper_arm_rod*u.in,u.m)))/2; % convert to meters and halve it
P.r_wrist_rod = double(separateUnits(unitConvert(P.r_wrist_rod*u.in,u.m)))/2; % convert to meters and halve it
P.r_forearm_bungee = double(separateUnits(unitConvert(P.r_forearm_bungee*u.in,u.m)))/2; % convert to meters and halve it

% Define the rotation parameters
affected_arm = 'L'; % options = R (right arm) or L (left arm) ----RIGHT ARM DOES NOT WORK RIGHT NOW----
rotation_pattern = 'Supination';  % options = 'Supination', 'Pronation', 'Supination-Pronation', 'Pronation-Supination'
rotationPattern = affected_arm + "-" + rotation_pattern; % concatenate options together
P.rotation_angle = 180; % total angle of forearm rotation : degrees
P.num_data_points = P.rotation_angle/2; % number of data points in matrices
P.phi_twist_step = P.rotation_angle/P.num_data_points; % angle step for twist
pause_time = 0.01; % pause time between frames in seconds

switch rotationPattern
    case 'L-Supination'
        P.total_frames = P.num_data_points;
        initial_wrist_angle = 540; % initial wrist angle
        initial_forearm_angle = 90; % initial forearm angle
        initial_elbow_angle = 90; % initial elbow angle
        P.caseValue = 'L-Supination';

    case 'L-Pronation-Supination'
        P.total_frames = 2*P.num_data_points; 
        initial_wrist_angle = 360; % initial wrist angle
        initial_forearm_angle = 90; % initial forearm angle
        initial_elbow_angle = 90; % initial elbow angle
        P.caseValue = 'L-Pronation-Supination';

end

% Initialize angles
P.theta_wrist = deg2rad(initial_wrist_angle); % initial wrist angle in radians
P.theta_forearm = deg2rad(initial_forearm_angle); % initial forearm angle in radians
P.theta_elbow = deg2rad(initial_elbow_angle); % elbow angle in radians
P.beta = deg2rad(90); % set angle from z-axis

% Initialize helix parameters
P.num_helix_points = 100; % number of points for the helix
S.t_helix = linspace(0, 1, P.num_helix_points); % parameter for helix positions
S.x_helix = zeros(1, P.num_helix_points); % initialize x-coordinates of helix
S.y_helix = zeros(1, P.num_helix_points); % initialize y-coordinates of helix
S.z_helix = zeros(1, P.num_helix_points); % initialize z-coordinates of helix
P.pitch = P.L_forearm; % pitch of the helix

%% Setup the Plots

setupPlots; % set up the plots for animation
% figure(1); subplot(1,2,1); % set figure for stl plotting
% plotSTLforearm; % plot stl of forearm

%% Animation Loop

L_rod_deformed = zeros(P.num_data_points,1); % create an empty matrix to store helix lengths
F_direction = zeros(P.num_data_points,3); % create an empty matrix to store direction of force
F_origin = zeros(P.num_data_points,3); % create an empty matrix to store wrist bungee attachment position
F_magnitude = zeros(P.num_data_points,1); % create an empty matrix to store magnitude of force
Tau = zeros(P.num_data_points,1); % create an empty matrix to store torque values
Tau_vec = zeros(P.num_data_points,3); % create an empty matrix to store torque vectors
angles = zeros(P.num_data_points,1); % create an empty matrix to store angles
r_torque = zeros(P.num_data_points,3); % create an empty matrix to store torque radius
F_rod = zeros(P.num_data_points,3); % create an empty matrix to store bungee force
forearm_bungee_wrap_sim(P.num_data_points) = struct('cdata',[],'colormap',[]); % pre-allocate the struct array

figure(1); % call figure 1 for plotting

for i = 1:P.total_frames % iterate through each frame

    % Identify system rotation pattern and determine direction of rotation
    switch rotationPattern
        case 'L-Supination'
            if i == 1
                P.phi_twist = P.theta_wrist - deg2rad(P.phi_twist_step);
            else
                P.phi_twist = P.phi_twist - deg2rad(P.phi_twist_step);
            end

        case 'L-Pronation-Supination'
            if i == 1
                P.phi_twist = P.theta_wrist + deg2rad(P.phi_twist_step);
            elseif (i > 1) && (i <= P.num_data_points)
                P.phi_twist = P.phi_twist + deg2rad(P.phi_twist_step);
            else
                P.phi_twist = P.phi_twist - deg2rad(P.phi_twist_step);
            end
    end
    
    % Calculate rod configuration
    [L_rod_mag, F_direction_vec] = deformed_rod(P.r_wrist_rod, P.L_forearm, P.L_rod, P.phi_twist); % run helix configuration function
    L_rod_deformed(i) = L_rod_mag; % append length of current helix iteration to helix length matrix
    F_direction(i,:) = F_direction_vec; % append direction of current force iteration to direction matrix
    F_origin(i,:) = [S.x_wrist_rod, S.y_wrist_rod, S.z_wrist_rod]; % append force origin position to origin matrix 
      
    % Calculate F_magnitude
    F_magnitude(i,:) = rod_force(P.L_rod, L_rod_deformed(i), S.dist_coordinates); % calculate magnitude of spring force

    % Calculate Torque
    Tau(i) = rod_torque(F_magnitude(i,:), F_direction(i,:), F_origin(i,:), P.L_forearm); % calculate torque on forearm by bungee
    Tau_vec(i,:) = S.torque;
    r_torque(i,:) = S.r_torque;
    F_rod(i,:) = S.F_rod;
    
    % Plot Forearm Animation.................................................

    exonetAnimation;
    angles(i) = abs(rad2deg(P.phi_twist) - initial_wrist_angle);

    % Draw torque plot alongside forearm rotation..........................

    if i == 1
        subplot(1,2,2); % torque plot
        torquePlot = plot(angles(i), Tau(i), 'k', 'LineWidth', 4 ); 
        hold on; % keep the plot for further updates
        
        % Create plot handles for the individual torque components
        xTorquePlot = plot(angles(i), Tau_vec(i,1), 'LineWidth', 2);
        yTorquePlot = plot(angles(i), Tau_vec(i,2), 'LineWidth', 2);
        zTorquePlot = plot(angles(i), Tau_vec(i,3), 'LineWidth', 2);

        lgd = legend('Overall Torque', 'X-Torque', 'Y-Torque', 'Z-Torque', 'Desired Torque');
        lgd.FontSize = 14;

    else
        % Update the XData and YData properties of the plot using the handle
        torquePlot.XData = angles(1:i); 
        torquePlot.YData = Tau(1:i); 
        
        % Also update the individual torque component plots
        xTorquePlot.XData = angles(1:i);
        xTorquePlot.YData = Tau_vec(1:i,1);
        
        yTorquePlot.XData = angles(1:i);
        yTorquePlot.YData = Tau_vec(1:i,2);
        
        zTorquePlot.XData = angles(1:i);
        zTorquePlot.YData = Tau_vec(1:i,3);

%         desiredTorquePlot.XData = desired_angles(i);
%         desiredTorquePlot.YData = S.desired_torques(i);

    end

    % Finish Plotting......................................................
    
    drawnow; % update plot immediately
    
    % Insert pause time between datapoints and rotation transition
    switch rotationPattern 
        case {'L-Supination', 'R_Supination'}
            if i == P.total_frames % if changing between pronation and supination
                pause(15*pause_time); % pause for a long time
            else
                pause(pause_time); % pause for a short time
            end
    
        case {'L-Pronation-Supination', 'R-Pronation-Supination'}
            if i == P.total_frames/2 % if changing between pronation and supination
                pause(15*pause_time); % pause for a long time
            else
                pause(pause_time); % pause for a short time
            end
    end
    
    forearm_bungee_wrap_sim(i) = getframe(gcf); % get frames of figure to save animation file

    if i < P.total_frames  % if not the last frame, delete previous plots
        fields = fieldnames(plt);
        for a = 1:length(fields)
            delete(plt.(fields{a}));
        end
    end

end


%% Save Animation

% Write animation to MP4 file
videofile = fullfile([pwd '\MATLAB Figures'], 'ExoNET_Rod_v1_Movie.mp4'); % file save path
forearm_bungee_wrap_movie = VideoWriter(videofile,'MPEG-4'); % create an .mp4 file
forearm_bungee_wrap_movie.FrameRate = 20;
open(forearm_bungee_wrap_movie);
writeVideo(forearm_bungee_wrap_movie, forearm_bungee_wrap_sim);
close(forearm_bungee_wrap_movie);

saveas(figure(1), fullfile([pwd '\MATLAB Figures'],'ExoNET_Rod_v1_Torque.png')); % save to computer


%% Plot Figure 2

figure(2);

% Torque vs. angle
subplot(3,1,1); 
plot(angles, Tau,'k', 'LineWidth', 4);

% Moment Arm vs. Angle - You need to calculate and store the radius values in a variable named 'radius'
subplot(3,1,2); 
plot(angles, vecnorm(r_torque'), 'k', 'LineWidth', 4); 
hold on; % keep the plot for further updates
plot(angles, r_torque(:,1),'LineWidth', 2 ); 
plot(angles, r_torque(:,2),'LineWidth', 2 ); 
plot(angles, r_torque(:,3),'LineWidth', 2 ); 
grid on;
lgd1 = legend('Magnitude', 'X-Component', 'Y-Component', 'Z-Component', 'FontSize', 12);
lgd1.FontSize = 14;
hold off;

% Force Direction Components & Force Magnitude vs. Angle
subplot(3,1,3); 
plot(angles, F_magnitude, 'k', 'LineWidth', 4); 
hold on;
plot(angles, F_rod(:,1), 'LineWidth', 2);
plot(angles, F_rod(:,2), 'LineWidth', 2);
plot(angles, F_rod(:,3), 'LineWidth', 2);
grid on;
lgd2 = legend('Magnitude', 'X-Component', 'Y-Component', 'Z-Component', 'FontSize', 12);
lgd2.FontSize = 14;
hold off;

saveas(figure(2), fullfile([pwd '\MATLAB Figures'],'ExoNET_Rod_v1_Supplemental.png')); % save to computer


%% Functions

function setupPlots

% SETUP PLOTS TO BE USED IN ANALYSIS.

%% Initialize Global

global S P

%% Setup Figure 1 Plot

% Initialize 3D plot
figure(1);
subplot(1,2,1); % forearm rotation subplot
grid on; % turn on grid
axis equal; % make axes equal
xlim([-P.L_upper_arm, 3*P.r_forearm_bungee]); % set x-axis limits
ylim([-3*P.r_forearm_bungee, 3*P.r_forearm_bungee]); % set y-axis limits
zlim([0, 2*P.L_forearm]); % set z-axis limits
xlabel('X', 'FontSize', 14); % label x-axis
ylabel('Y', 'FontSize', 14); % label y-axis
zlabel('Z', 'FontSize', 14); % label z-axis
title('Forearm with ExoNET in 3D', 'FontSize', 16); % add title to plot
view(3); % set the 3D view        
ax = gca; ax.FontSize = 14; % set axis font size
hold on;

% Initialize forearm parameters
z_frustrum = linspace(0, P.L_forearm, 50); % z-axis positions for frustrum
theta_frustrum = linspace(0, 2*pi, 50); % angles for frustrum
[S.Z, S.theta] = meshgrid(z_frustrum, theta_frustrum); % create meshgrid for frustrum
R = -(P.r_forearm - P.r_wrist)/P.L_forearm .* S.Z + P.r_forearm; % calculate frustrum radius
S.X = R .* cos(S.theta); % calculate x-coordinates of frustrum
S.Y = R .* sin(S.theta); % calculate y-coordinates of frustrum

% Plot forearm centerline
forearm_centerline = plot3([0, 0], [0, 0], [0, P.L_forearm], 'k', 'LineWidth', 2); % plot forearm centerline

% Calculate the endpoint of the upper arm based on given parameters
S.x_upper_arm = P.L_upper_arm * cos(P.theta_elbow + P.beta); % shoulder x-coordinate
S.y_upper_arm = 0; % shoulder y-coordinate
S.z_upper_arm = P.L_upper_arm * sin(P.theta_elbow + P.beta); % shoulder z-coordinate

% Plot the line representing the upper arm's centerline
upper_arm_centerline = plot3([S.x_upper_arm, 0], [S.y_upper_arm, 0], [S.z_upper_arm, 0], 'r', 'LineWidth', 2);

% Initialize parameters for constructing the cylinder
theta_cylinder = linspace(0, 2*pi, 50); % angles for the circular cross-section
x_cylinder = linspace(0, P.L_upper_arm, 50); % points along the height of the cylinder
[Theta_cylinder, X_cylinder] = meshgrid(theta_cylinder, x_cylinder); % create a grid for the cylinder surface
X_cylinder = -X_cylinder; % cylinder surface x-coordinates
Y_cylinder = P.r_upper_arm * cos(Theta_cylinder); % cylinder surface y-coordinates
Z_cylinder = P.r_upper_arm * sin(Theta_cylinder); % cylinder surface z-coordinates

% Plot the upper arm cylinder
cylinder_surface = surf(X_cylinder, Y_cylinder, Z_cylinder, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
set(cylinder_surface, 'CData', Theta_cylinder); % set color based on angle around the cylinder
colormap('jet'); % use the jet colormap for coloring the cylinder

% Initialize Torque Plot
subplot(1,2,2); % <-- Added subplot for torque plot
grid on; % turn on grid
xlim([0, P.rotation_angle]); % set x-axis limits
% ylim([-1, 2.5]); % set y-axis limits
xlabel('Wrist Rotation Angle (degrees)', 'FontSize', 14); 
ylabel('Resulting Torque (Nm)', 'FontSize', 14); 
title('Wrist Rotation Angle vs Resulting Torque', 'FontSize', 16); 
ax = gca; ax.FontSize = 14; % set axis font size
grid on; hold on;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);


%% Setup Figure 2 Plot

% Create a new figure
figure(2);

% Torque vs. angle
subplot(3,1,1); 
xlabel('Angle (degrees)', 'FontSize', 14);
ylabel('Torque (Nm)', 'FontSize', 14);
title('Torque vs. Angle', 'FontSize', 16);
grid on; hold on;
ax = gca; ax.FontSize = 12;  % set axis font size

% Moment Arm vs. Angle 
subplot(3,1,2); 
xlabel('Angle (degrees)', 'FontSize', 14);
ylabel('Moment Arm (m)', 'FontSize', 14);
title('Moment Arm vs. Angle', 'FontSize', 16);
hold on;
ax = gca; ax.FontSize = 12;  % set axis font size

% Force Direction Components & Force Magnitude vs. Angle
subplot(3,1,3); 
xlabel('Angle (degrees)', 'FontSize', 14);
ylabel('Force (N)', 'FontSize', 14);
title('Force Components and Magnitude vs. Angle', 'FontSize', 16);
grid on; hold on;
ax = gca; ax.FontSize = 12;  % set axis font size

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);

end

function [L_rod_deformed, F_direction] = deformed_rod(r_wrist_rod, L_forearm, L_rod, phi_twist)

% ACTUATOR ROD CONFIGURATION. 
% r_wrist_rod : radius of user's wrist;
% L_forearm : length of forearm
% L_rod : initial undeformed length of the rod
% phi_twist : twist angle of the wrist
% L_rod_deformed : length of the rod under deformation (tells us how much deformation in the rod during rotation)
% F_direction : direction of the rod force at the wrist

%% Initialize Global

global S P

%% Identify Current Position of Wrist

% Calculate wrist bungee attachment position
S.x_wrist_rod = r_wrist_rod * sin(P.beta) * cos(phi_twist);
S.y_wrist_rod = r_wrist_rod * sin(P.beta) * sin(phi_twist);
S.z_wrist_rod = L_forearm;

%% Calculate Upper Arm Moment Arm

% Create Moment Arm on Upper Arm
S.x_moment_arm = S.x_upper_arm / 2;
S.y_moment_arm = S.y_upper_arm / 2;
S.z_moment_arm = S.z_upper_arm / 2;

% Calculate the end point of the moment arm
S.x_moment_arm_end = S.x_moment_arm;
S.y_moment_arm_end = S.y_moment_arm;
S.z_moment_arm_end = S.z_moment_arm + P.r_upper_arm_rod;
    

%% Calculate Undeformed Rod Endpoints

S.rod_start = [S.x_moment_arm_end, S.y_moment_arm_end, S.z_moment_arm_end]; % upper arm rod position
S.rod_end = [S.x_wrist_rod, S.y_wrist_rod, S.z_wrist_rod]; % wrist rod position
    

%% Calculate Deformed Rod Configuration

% Number of points used to create the rod
num_points = 100;

% Calculate the midpoint of the line connecting the two points
S.midpoint = (S.rod_start + S.rod_end) / 2;

% Calculate the distance between S.rod_start and S.rod_end
S.dist_coordinates = norm(S.rod_end - S.rod_start);

% Initialize the deflection of the rod
h = (L_rod^2 - S.dist_coordinates^2) / (2 * L_rod); % Initial guess for deflection of the rod

% Calculate the vector from start to end and its perpendicular
vec = S.rod_end - S.rod_start;
perp_vec = cross(cross(vec, [0 0 1]), vec);
perp_vec = perp_vec / norm(perp_vec); % Normalize the perpendicular vector

% Ensure the deformation is upwards by checking the z-component
if perp_vec(3) < 0
    perp_vec = -perp_vec;
end

% Define tolerance for the iterative process
tolerance = 1e-5;
error = inf;

% Iterative approach to adjust 'h'
while error > tolerance
    % Initialize the arc points
    S.arc_points = zeros(3, num_points);

    % Create the parameter t for interpolation
    t = linspace(0, 1, num_points);

    % Calculate the arc points
    for j = 1:num_points
        % Interpolate linearly between the two points
        S.arc_points(:, j) = S.rod_start' * (1 - t(j)) + S.rod_end' * t(j);

        % Add the deformation in the direction perpendicular to the straight line
        S.arc_points(:, j) = S.arc_points(:, j) + (4 * h * (t(j) - t(j)^2)) * perp_vec';
    end

    % Initialize the length of the deformed rod
    L_rod_deformed = 0;

    % Calculate the length of the deformed rod
    for k = 2:num_points
        % Calculate the distance between consecutive points
        segment_length = norm(S.arc_points(:, k) - S.arc_points(:, k - 1));
        % Add the segment length to the total length
        L_rod_deformed = L_rod_deformed + segment_length;
    end

    % Update the error
    error = abs(L_rod - L_rod_deformed);

    % Adjust 'h' based on the error
    if L_rod_deformed > L_rod
        h = h * (1 - error / L_rod_deformed);
    else
        h = h * (1 + error / L_rod);
    end
end

S.deformed_midpoint = S.midpoint + h * perp_vec; % calculate deformed midpoint

    
%% Check Arc Length Compared to Rod Original Length
% % Ensure arc length is equal to the original length of the rod

if abs(L_rod_deformed - L_rod) > tolerance
    disp('The length of the deformed rod is not constant.');
    disp(['Deformed rod length: ', num2str(L_rod_deformed)]);
    disp(['Original rod length: ', num2str(L_rod)]);
end


%% Calculate Direction of Force at the Last Node 
% % Force will act along the tangent to the arc at the wrist, away from the center of curvature

dx_force = S.arc_points(1, end) - S.arc_points(1, end - 1); % x-distance b/w last two nodes of rod
dy_force = S.arc_points(2, end) - S.arc_points(2, end - 1); % y-distance b/w last two nodes of rod
dz_force = S.arc_points(3, end) - S.arc_points(3, end - 1); % z-distance b/w last two nodes of rod

% Calculate magnitude of force at wrist
tangent_vec = [dx_force, dy_force, dz_force]; % tangent vector of the force at the wrist
magnitude = norm(tangent_vec); % calculate magnitude of vector
F_direction = tangent_vec / magnitude; % normalize to get unit vector of force


%% Ensure the direction is away from the center of curvature (the rod is trying to straighten out)

if dot(tangent_vec, vec) > 0
    F_direction = -F_direction;
end

disp(['Direction of the tensile force (unit vector): [', num2str(F_direction(1)), ', ', ...
      num2str(F_direction(2)), ', ', num2str(F_direction(3)), ']']);

end

function F_magnitude = rod_force(L_rod, L_rod_deformed, dist_coordinates)

% BUNGEE FORCE CALCULATION. 
% L_rod : undeformed length of the rod prior to deformation;
% L_rod_deformed : length of the rod under deformation (tells us how much deformation in the rod during rotation)
% dist_coordinates : distance between the mounting points on the upper arm and wrist
% F_magnitude : magnitude of the bungee cord force

%% Initialize Global

global P

%% Calculate Spring Constant & Identify Rod Delta Length

k = (3 * P.E * P.I) / L_rod^3; % calculate spring constant for the rod
delta_L = L_rod_deformed - dist_coordinates; % calculate the deflection at the wrist

%% Caclulate Rod Spring Force

F_rod = k * delta_L; % calculate the restorative force acting at the wrist due to rod deformation
F_magnitude = norm(F_rod); % calculate the bungee force magnitude


%% Output the calculated force and direction

disp(['Tensile restorative force acting on the wrist due to rod deformation: ', num2str(F_magnitude), ' N']);


end

function Tau = rod_torque(F_magnitude, F_direction, F_origin, L_forearm)

% BUNGEE TORQUE CALCULATION. 
% F_magnitude : magnitude of the rod force
% F_direction : direction of the rod force at the wrist
% Tau : torque induced by the bungee cord when rotating the forearm

%% Initialize Global

global S

%% Identify Force and Radius Matrices of Bungee Cord

S.F_rod = F_magnitude * F_direction; % calculate force vector

S.rotation_origin = [0, 0, L_forearm]; % calculate origin of rotation 
S.r_torque = F_origin - S.rotation_origin; % calculate distance from origin of forearm bungee attachment to point of force application

%% Calculate Resulting Torque

S.torque = cross(S.r_torque, S.F_rod); % calculate torque using cross product
Tau = norm(S.torque);

end

function exonetAnimation

% SETUP ROD PLOTS TO BE USED IN ANALYSIS.

%% Initialize Global

global S P plt

%% Plot First Figure

subplot(1,2,1); % call first figure for plotting helix animation

% Rotate frustrum
X_rot = S.X .* cos(P.phi_twist) - S.Y .* sin(P.phi_twist);
Y_rot = S.X .* sin(P.phi_twist) + S.Y .* cos(P.phi_twist);

% Plot rotated frustrum
plt.h_frustrum = surf(X_rot, Y_rot, S.Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % plot transparent frustrum
colormap('jet'); % set colormap
set(plt.h_frustrum, 'CData', S.theta);
plt.wrist_bungee = plot3([0, S.x_wrist_rod], [0, S.y_wrist_rod], [P.L_forearm, S.z_wrist_rod], 'b', 'LineWidth', 2); % plot wrist bungee attachment


%% Plot Rod and Components

% Plot upper arm moment arm 
plt.upper_arm_moment_arm = plot3([S.x_moment_arm, S.x_moment_arm_end], [S.y_moment_arm, S.y_moment_arm_end], [S.z_moment_arm, S.z_moment_arm_end], 'b', 'LineWidth', 2);

% Plot deformed rod and midpoint
plt.deformed_rod = plot3(S.arc_points(1, :), S.arc_points(2, :), S.arc_points(3, :), 'r-', 'LineWidth', 2);
plt.deformed_midpoint = plot3(S.deformed_midpoint(1), S.deformed_midpoint(2), S.deformed_midpoint(3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');

% Plot initial, midpoint & final points of the rod
plt.rod_start_coordinate = plot3(S.rod_start(1), S.rod_start(2), S.rod_start(3), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
plt.rod_end_coordinate = plot3(S.rod_end(1), S.rod_end(2), S.rod_end(3), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');

% Plot the straight line between the two points
plt.norm_rod = plot3([S.rod_start(1), S.rod_end(1)], [S.rod_start(2), S.rod_end(2)], [S.rod_start(3), S.rod_end(3)], 'g--', 'LineWidth', 2);
plt.midpoint = plot3(S.midpoint(1), S.midpoint(2), S.midpoint(3), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');

% Plot the perpendicular line
plt.midpoint_line = plot3([S.midpoint(1), S.deformed_midpoint(1)], [S.midpoint(2), S.deformed_midpoint(2)], [S.midpoint(3), S.deformed_midpoint(3)], 'g--', 'LineWidth', 2);

end

