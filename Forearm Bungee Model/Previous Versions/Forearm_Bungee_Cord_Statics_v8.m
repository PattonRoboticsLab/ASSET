%% ExoNET Program to Compare Forearm Angle vs. Output Torque 

close all; % close all existing figures
clear all; % clear all variables
clc; % clear the command window
warning('off', 'MATLAB:audiovideo:VideoWriter:mp4FramePadded'); % turn off warning


%% Initialize & Establish Input Variables

global S

% Rod lengths and properties
S.L_forearm = 12; % length of the forearm rod in inches
S.r_wrist = 3; % width of the wrist rod in inches
S.r_forearm = 5; % width of the forearm rod in inches

% Bungee cord properties
S.L_rest = 6; % unstretched length of the bungee cord in inches
S.k_bungee = 100; % stiffness constant of the bungee cord

% Convert Lengths to Meters
u = symunit; % create symbolic unit variable
S.L_forearm = double(separateUnits(unitConvert(S.L_forearm*u.in,u.m))); % convert forearm length to meters
S.L_rest = double(separateUnits(unitConvert(S.L_rest*u.in,u.m))); % convert bungee resting length to meters and halve it
S.r_wrist = double(separateUnits(unitConvert(S.r_wrist*u.in,u.m)))/2; % convert wrist width to meters and halve it
S.r_forearm = double(separateUnits(unitConvert(S.r_forearm*u.in,u.m)))/2; % convert forearm width to meters and halve it

% Define the rotation parameters
affected_arm = 'L'; % options = R (right arm) or L (left arm) ----RIGHT ARM DOES NOT WORK RIGHT NOW----
rotation_pattern = 'Supination';  % options = 'Supination', 'Pronation', 'Supination-Pronation', 'Pronation-Supination'
rotationPattern = affected_arm + "-" + rotation_pattern; % concatenate options together
S.rotation_angle = 180; % total angle of forearm rotation : degrees
S.num_data_points = S.rotation_angle/2; % number of data points in matrices
S.phi_twist_step = S.rotation_angle/S.num_data_points; % angle step for twist
pause_time = 0.01; % pause time between frames in seconds

switch rotationPattern
    case 'L-Supination'
        S.total_frames = S.num_data_points;
        initial_wrist_angle = 540; % initial wrist angle
        initial_forearm_angle = 90;   % initial forearm angle
        S.caseValue = 'L-Supination';

    case 'L-Pronation-Supination'
        S.total_frames = 2*S.num_data_points; 
        initial_wrist_angle = 360; % initial wrist angle
        initial_forearm_angle = 90;   % initial forearm angle
        S.caseValue = 'L-Pronation-Supination';

end

% Initialize angles
S.theta_wrist = deg2rad(initial_wrist_angle); % initial wrist angle in radians
S.theta_forearm = deg2rad(initial_forearm_angle); % initial forearm angle in radians
S.beta = pi/2; % set angle from z-axis

% Initialize helix parameters
S.num_helix_points = 100; % number of points for the helix
S.t_helix = linspace(0, 1, S.num_helix_points); % parameter for helix positions
S.x_helix = zeros(1, S.num_helix_points); % initialize x-coordinates of helix
S.y_helix = zeros(1, S.num_helix_points); % initialize y-coordinates of helix
S.z_helix = zeros(1, S.num_helix_points); % initialize z-coordinates of helix
S.pitch = S.L_forearm; % pitch of the helix


%% Optimization Algorithm

% Define the desired torques
desired_angles = 0:S.phi_twist_step:S.rotation_angle-1;
S.desired_torques = 1 * sind(desired_angles);

% Initial guesses for the parameters
initial_params = S.theta_forearm;  % initial forearm angle

% Bounds for the parameters
lb = 0;   % lower bounds
ub = 180; % upper bounds

% Optimization using fmincon
optimal_forearm_angle = fmincon(@objective_function, initial_params, [], [], [], [], lb, ub);
disp(['Optimal Forearm Angle: ', num2str(optimal_forearm_angle)]);
S.theta_forearm = deg2rad(optimal_forearm_angle);

%% Setup the Plots

setupPlots; % set up the plots for animation

%% Animation Loop

L_helix = zeros(S.num_data_points,1); % create an empty matrix to store helix lengths
F_direction = zeros(S.num_data_points,3); % create an empty matrix to store direction of force
F_origin = zeros(S.num_data_points,3); % create an empty matrix to store wrist rod position
F_magnitude = zeros(S.num_data_points,1); % create an empty matrix to store magnitude of force
Tau = zeros(S.num_data_points,1); % create an empty matrix to store torque values
Tau_vec = zeros(S.num_data_points,3); % create an empty matrix to store torque vectors
angles = zeros(S.num_data_points,1); % create an empty matrix to store angles
r_torque = zeros(S.num_data_points,3); % create an empty matrix to store torque radius
F_bungee = zeros(S.num_data_points,3); % create an empty matrix to store bungee force
forearm_bungee_wrap_sim(S.num_data_points) = struct('cdata',[],'colormap',[]); % pre-allocate the struct array

figure(1); % call figure 1 for plotting

for i = 1:S.total_frames % iterate through each frame

    % Identify system rotation pattern and determine direction of rotation
    switch rotationPattern
        case 'L-Supination'
            if i == 1
                phi_twist = S.theta_wrist - deg2rad(S.phi_twist_step);
            else
                phi_twist = phi_twist - deg2rad(S.phi_twist_step);
            end

        case 'L-Pronation-Supination'
            if i == 1
                phi_twist = S.theta_wrist + deg2rad(S.phi_twist_step);
            elseif (i > 1) && (i <= S.num_data_points)
                phi_twist = phi_twist + deg2rad(S.phi_twist_step);
            else
                phi_twist = phi_twist - deg2rad(S.phi_twist_step);
            end
    end
    
    % Calculate helix configuration
    [L_helix_mag, F_direction_vec] = helix(S.r_wrist, S.r_forearm, S.L_forearm, phi_twist, S.theta_forearm); % run helix configuration function
    L_helix(i) = L_helix_mag; % append length of current helix iteration to helix length matrix
    F_direction(i,:) = F_direction_vec; % append direction of current force iteration to direction matrix
    F_origin(i,:) = [S.x_wrist, S.y_wrist, S.z_wrist]; % append force origin position to origin matrix 
    
    % Calculate F_magnitude
    F_magnitude(i,:) = spring_force(S.k_bungee, S.L_rest, L_helix(i)); % calculate magnitude of spring force

    % Calculate Torque
    Tau(i) = exonet_torque(F_magnitude(i,:), F_direction(i,:), F_origin(i,:), S.L_forearm); % calculate torque on forearm by bungee
    Tau_vec(i,:) = S.torque;
    r_torque(i,:) = S.r_torque;
    F_bungee(i,:) = S.F_bungee;

    %% Plot Helix Animation

    subplot(1,2,1); % call first figure for plotting helix animation

    % Rotate the frustrum
    X_rot = S.X .* cos(phi_twist) - S.Y .* sin(phi_twist);
    Y_rot = S.X .* sin(phi_twist) + S.Y .* cos(phi_twist);
    angles(i) = abs(rad2deg(phi_twist) - initial_wrist_angle);
    
    % Plot frustrum
    h_frustrum = surf(X_rot, Y_rot, S.Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % plot transparent frustrum
    colormap('jet'); % set colormap
    set(h_frustrum, 'CData', S.theta);

    wrist = plot3([0, S.x_wrist], [0, S.y_wrist], [S.L_forearm, S.z_wrist], 'b', 'LineWidth', 2); % plot wrist rod
    forearm = plot3([0, S.x_forearm], [0, S.y_forearm], [0, S.z_forearm], 'g', 'LineWidth', 2); % plot forearm rod
    helix_plot = plot3(S.x_helix, S.y_helix, S.z_helix, 'r', 'LineWidth', 2); % plot helix (bungee cord)
    
    
    %% Draw torque plot alongside forearm rotation

    if i == 1
        subplot(1,2,2); % torque plot
        torquePlot = plot(angles(i), Tau(i), 'k', 'LineWidth', 4 ); 
        hold on; % keep the plot for further updates
        
        % Create plot handles for the individual torque components
        xTorquePlot = plot(angles(i), Tau_vec(i,1), 'LineWidth', 2);
        yTorquePlot = plot(angles(i), Tau_vec(i,2), 'LineWidth', 2);
        zTorquePlot = plot(angles(i), Tau_vec(i,3), 'LineWidth', 2);
        desiredTorquePlot = plot(desired_angles,S.desired_torques, 'LineWidth', 2);

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


%% Finish Plotting
    drawnow; % update plot immediately
    
    % Insert pause time between datapoints and rotation transition
    switch rotationPattern 
        case {'L-Supination', 'R_Supination'}
            if i == S.total_frames % if changing between pronation and supination
                pause(15*pause_time); % pause for a long time
            else
                pause(pause_time); % pause for a short time
            end
    
        case {'L-Pronation-Supination', 'R-Pronation-Supination'}
            if i == S.total_frames/2 % if changing between pronation and supination
                pause(15*pause_time); % pause for a long time
            else
                pause(pause_time); % pause for a short time
            end
    end
    
    forearm_bungee_wrap_sim(i) = getframe(gcf); % get frames of figure to save animation file

    if i < S.total_frames  % if not the last frame, delete previous plots
        delete(wrist);
        delete(forearm);
        delete(helix_plot);
        delete(h_frustrum);
    end

end


%% Save Animation

% Write animation to MP4 file
videofile = fullfile([pwd '\MATLAB Figures'], 'ExoNET_Bungee_Wrapping_v8_Movie.mp4'); % file save path
forearm_bungee_wrap_movie = VideoWriter(videofile,'MPEG-4'); % create an .mp4 file
forearm_bungee_wrap_movie.FrameRate = 20;
open(forearm_bungee_wrap_movie);
writeVideo(forearm_bungee_wrap_movie, forearm_bungee_wrap_sim);
close(forearm_bungee_wrap_movie);

saveas(figure(1), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v8_Torque.png')); % save to computer


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
plot(angles, F_bungee(:,1), 'LineWidth', 2);
plot(angles, F_bungee(:,2), 'LineWidth', 2);
plot(angles, F_bungee(:,3), 'LineWidth', 2);
grid on;
lgd2 = legend('Magnitude', 'X-Component', 'Y-Component', 'Z-Component', 'FontSize', 12);
lgd2.FontSize = 14;
hold off;

saveas(figure(2), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v8_Supplemental.png')); % save to computer


%% Functions

function cost = objective_function(params)

% COST FUNCTION CALCULATION.
% params : parameters to be optimized : initial_forearm_angle

%% Initialize Global Variables
global S

%% Extract parameters
initial_forearm_angle = params;

%% Run the simulation with the given parameters

S.tau = zeros(1,S.num_data_points); % create an empty matrix to store torque values

for i = 1:S.total_frames % iterate through each frame

    % Identify system rotation pattern and determine direction of rotation
    if strcmp(S.caseValue, 'L-Supination')
            if i == 1
                phi_twist = S.theta_wrist - deg2rad(S.phi_twist_step);
            else
                phi_twist = phi_twist - deg2rad(S.phi_twist_step);
            end

    elseif strcmp(S.caseValue, 'L-Pronation-Supination')
            if i == 1
                phi_twist = S.theta_wrist + deg2rad(S.phi_twist_step);
            elseif (i > 1) && (i <= S.num_data_points)
                phi_twist = phi_twist + deg2rad(S.phi_twist_step);
            else
                phi_twist = phi_twist - deg2rad(S.phi_twist_step);
            end
    end
    
    % Calculate helix configuration
    [L_helix_mag, F_direction_vec] = helix(S.r_wrist, S.r_forearm, S.L_forearm, phi_twist, initial_forearm_angle); % run helix configuration function
    F_origin = [S.x_wrist, S.y_wrist, S.z_wrist]; % append force origin position to origin matrix 
    
    % Calculate F_magnitude
    F_magnitude = spring_force(S.k_bungee, S.L_rest, L_helix_mag); % calculate magnitude of spring force

    % Calculate Torque
    S.tau(i) = exonet_torque(F_magnitude, F_direction_vec, F_origin, S.L_forearm); % calculate torque on forearm by bungee

end

% Calculate the cost as the negative of the sum of the squares of the differences between simulation torques and desired torques 
cost = -sum((S.tau - S.desired_torques).^2);

end

function setupPlots

% SETUP PLOTS TO BE USED IN ANALYSIS.

%% Initialize Global

global S

%% Setup Figure 1 Plot

% Initialize 3D plot
figure(1);
subplot(1,2,1); % forearm rotation subplot
grid on; % turn on grid
axis equal; % make axes equal
xlim([-S.r_forearm, S.r_forearm]); % set x-axis limits
ylim([-S.r_forearm, S.r_forearm]); % set y-axis limits
zlim([0, S.L_forearm]); % set z-axis limits
xlabel('X', 'FontSize', 14); % label x-axis
ylabel('Y', 'FontSize', 14); % label y-axis
zlabel('Z', 'FontSize', 14); % label z-axis
title('Forearm with ExoNET in 3D', 'FontSize', 16); % add title to plot
view(3); % set the 3D view        
ax = gca; ax.FontSize = 14; % set axis font size
hold on;

% Initialize frustrum parameters
z_frustrum = linspace(0, S.L_forearm, 50); % z-axis positions for frustrum
theta_frustrum = linspace(0, 2*pi, 50); % angles for frustrum
[S.Z, S.theta] = meshgrid(z_frustrum, theta_frustrum); % create meshgrid for frustrum
R = -(S.r_forearm - S.r_wrist)/S.L_forearm .* S.Z + S.r_forearm; % calculate frustrum radius
S.X = R .* cos(S.theta); % calculate x-coordinates of frustrum
S.Y = R .* sin(S.theta); % calculate y-coordinates of frustrum

% Plot central rod
plot3([0, 0], [0, 0], [0, S.L_forearm], 'k', 'LineWidth', 2); % plot the central rod

% Initialize Torque Plot
subplot(1,2,2); % <-- Added subplot for torque plot
grid on; % turn on grid
xlim([0, S.rotation_angle]); % set x-axis limits
ylim([-1, 2.5]); % set y-axis limits
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

function [L_helix, F_direction] = helix(r_wrist, r_forearm, L_forearm, phi_twist, initial_theta_forearm)

% BUNGEE HELIX CONFIGURATION. 
% r_wrist : radius of user's wrist (ExoNET wrist band location);
% r_forearm : radius of user's forearm just below elbow (ExoNET forearm band location);
% L_forearm : length of forearm (distance between wrist and forearm bands)
% phi_twist : twist angle of the bungee cord (dynamic frequency of bungee)
% L_helix : length of the helix in single iteration (tells us how much stretch in the bungee during rotation
% F_direction : direction of the bungee force at the wrist

%% Initialize Global

global S

%% Identify Current Positions of Wrist and Forearm

% Calculate wrist rod position
S.x_wrist = r_wrist * sin(S.beta) * cos(phi_twist);
S.y_wrist = r_wrist * sin(S.beta) * sin(phi_twist);
S.z_wrist = L_forearm;

% Calculate forearm rod position
S.x_forearm = r_forearm * sin(S.beta) * cos(initial_theta_forearm);
S.y_forearm = r_forearm * sin(S.beta) * sin(initial_theta_forearm);
S.z_forearm = 0;
    
%% Calculate Helix Configuration

% Initialize the length of the helix to 0
L_helix = 0; % total length of helix for single iteration

% Update helix parameters
for j = 1:S.num_helix_points % iterate through each helix point
    t = S.t_helix(j); % get parameter for current helix point
    R = (r_wrist - r_forearm) * t + r_forearm; % calculate radius for helix point

    if j == 1
        S.x_helix(j) = S.x_forearm; % set first x-coordinate to forearm position
        S.y_helix(j) = S.y_forearm; % set first y-coordinate to forearm position
        S.z_helix(j) = S.z_forearm; % set first z-coordinate to forearm position

    elseif j == length(S.num_helix_points)
        S.x_helix(j) = S.x_wrist; % set last x-coordinate to wrist position
        S.y_helix(j) = S.y_wrist; % set last y-coordinate to wrist position
        S.z_helix(j) = S.z_wrist; % set last z-coordinate to wrist position

    else
        S.x_helix(j) = R * cos((phi_twist - initial_theta_forearm) * t + initial_theta_forearm); % calculate x-coordinate for helix point
        S.y_helix(j) = R * sin((phi_twist - initial_theta_forearm) * t + initial_theta_forearm); % calculate y-coordinate for helix 
        S.z_helix(j) = S.pitch * t; % calculate z-coordinate for helix point
        
    end
    
    % If it's not the first point, calculate the distance to the previous point
    if j > 1
        dx_helix = S.x_helix(j) - S.x_helix(j - 1); % distance between x-coordinates
        dy_helix = S.y_helix(j) - S.y_helix(j - 1); % distance between y-coordinates
        dz_helix = S.z_helix(j) - S.z_helix(j - 1); % distance between z-coordinates
        distance = norm([dx_helix, dy_helix, dz_helix]); % distance between two points on helix
        L_helix = L_helix + distance;  % add the distance to the total length
    end
    
end

% Calculate direction of force at the last node 
% Calculation uses difference b/w last two points to approximate tangent vector
dx_force = S.x_helix(end) - S.x_helix(end - 1); % x-distance b/w last two nodes of helix
dy_force = S.y_helix(end) - S.y_helix(end - 1); % y-distance b/w last two nodes of helix
dz_force = S.z_helix(end) - S.z_helix(end - 1); % z-distance b/w last two nodes of helix

magnitude = norm([dx_force, dy_force, dz_force]); % calculate magnitude of vector
F_direction = [dx_force, dy_force, dz_force] / magnitude; % normalize to get unit vector of force

end

function F_magnitude = spring_force(k_bungee, L_rest, L_helix)

% BUNGEE FORCE CALCULATION. 
% L_rest : rest length of the bungee cord helix prior to mounting on forearm;
% k_bungee : material properties of the bungee cord;
% L_helix : length of the helix in single iteration (tells us how much stretch in the bungee during rotation
% F_magnitude : magnitude of the bungee cord force

%% Identify Bungee Cord Delta Length

deltaL = L_helix - L_rest; % calculate the difference between helical length and resting length

%% Calculate Bungee Spring Force

F_spring = -k_bungee * deltaL; % calculate bungee force
F_magnitude = norm(F_spring); % calculate the bungee force magnitude

end

function Tau = exonet_torque(F_magnitude, F_direction, F_origin, L_forearm)


% BUNGEE TORQUE CALCULATION. 
% r2 : moment arm from the wrist attachment point to the origin of bungee
% F_magnitude : magnitude of the bungee cord force
% F_direction : direction of the bungee force at the wrist
% Tau : torque induced by the bungee cord when rotating the forearm

%% Initialize Global

global S

%% Identify Force and Radius Matrices of Bungee Cord

S.F_bungee = F_magnitude * F_direction; % calculate force vector

S.rotation_origin = [0, 0, L_forearm]; % calculate origin of rotation 
S.r_torque = F_origin - S.rotation_origin; % calculate distance from origin of forearm rod to point of force application

%% Calculate Resulting Torque

S.torque = cross(S.r_torque, S.F_bungee); % calculate torque using cross product
Tau = norm(S.torque);

end
