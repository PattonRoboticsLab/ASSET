%% ExoNET Program to Compare Forearm Angle vs. Output Torque 

close all; % close all existing figures
clear all; % clear all variables
clc; % clear the command window
warning('off', 'MATLAB:audiovideo:VideoWriter:mp4FramePadded'); % turn off warning


%% Initialize & Establish Input Variables

global S P plt

affected_arm = 'L'; % options = R (right arm) or L (left arm) ----RIGHT ARM DOES NOT WORK RIGHT NOW----
rotation_pattern = 'Supination';  % options = 'Supination', 'Pronation', 'Supination-Pronation', 'Pronation-Supination'
rotationPattern = affected_arm + "-" + rotation_pattern; % concatenate options together

P.rotation_angle = 180; % total angle of forearm rotation : degrees
P.num_data_points = P.rotation_angle/2; % number of data points in matrices

switch rotationPattern
    case 'L-Supination'
        P.total_frames = P.num_data_points;
        P.init_wrist_angle = 540; % initial wrist angle
        P.init_forearm_angle = 90; % initial forearm angle
        P.init_elbow_angle = 90; % initial elbow angle
        P.caseValue = 'L-Supination';
        parameters;

    case 'L-Pronation-Supination'
        P.total_frames = 2*P.num_data_points; 
        P.init_wrist_angle = 360; % initial wrist angle
        P.init_forearm_angle = 90; % initial forearm angle
        P.init_elbow_angle = 90; % initial elbow angle
        P.caseValue = 'L-Pronation-Supination';
        parameters;

end


%% Optimization Algorithm

% Define the desired torques
desired_angles = 0:P.phi_twist_step:P.rotation_angle-1;
S.desired_torques = 1 * sind(desired_angles);

% Initial guesses for the parameters
init_params = [P.L_bungee_rest, P.theta_forearm];  % [initial forearm angle, resting bungee length]

% Bounds for the parameters
L_rest_lb = 4; % lower bound of bungee resting length
L_rest_ub = 10; % upper bounds of bungee resting length
forearm_theta_lb = 180; % lower bound of forearm theta
forearm_theta_ub = 270; % upper bounds of forearm theta

% Convert input parameters
u = symunit; % create symbolic unit variable
L_rest_lb = double(separateUnits(unitConvert(L_rest_lb*u.in,u.m))); % convert to meters
L_rest_ub = double(separateUnits(unitConvert(L_rest_ub*u.in,u.m))); % convert to meters
forearm_theta_lb = deg2rad(forearm_theta_lb); % convert to radians
forearm_theta_ub = deg2rad(forearm_theta_ub); % convert to radians

% Loop fmincon optimization
bestCost=1e5; % best cost allowed by optimization
nTries = 10; % number of times running the optimization

fprintf('\n\n Begin optimizations:  ');
options = optimoptions('fmincon','Display','iter-detailed');
for TRY=1:nTries
    fprintf('Opt#%d of %d..',TRY,nTries);
    [optimal_params, cost, exitflag, output] = fmincon(@objective_function, init_params, [], [], [], [], [L_rest_lb, forearm_theta_lb], [L_rest_ub, forearm_theta_ub], [], options);
    if cost < bestCost % if lower cost
        fprintf(' cost =%g, ',cost);
        bestCost = cost; 
        init_params = optimal_params; % update initial parameters to new optimal parameters
    else
        fprintf(' (not an improvement) \n ');
    end
end

disp(['Optimal Bungee Length: ', num2str(double(separateUnits(unitConvert(optimal_params(1)*u.m,u.in))))]);
P.L_bungee_rest = optimal_params(1);

disp(['Optimal Forearm Angle: ', num2str(rad2deg(optimal_params(2)))]);
P.theta_forearm = optimal_params(2);


%% Setup the Plots

setupPlots; % set up the plots for animation
figure(1); subplot(1,2,1); % set figure for stl plotting
plotSTLforearm; % plot stl of forearm


%% Animation Loop

L_helix = zeros(P.num_data_points,1); % create an empty matrix to store helix lengths
F_direction = zeros(P.num_data_points,3); % create an empty matrix to store direction of force
F_origin = zeros(P.num_data_points,3); % create an empty matrix to store wrist bungee attachment position
F_magnitude = zeros(P.num_data_points,1); % create an empty matrix to store magnitude of force
Tau = zeros(P.num_data_points,1); % create an empty matrix to store torque values
Tau_vec = zeros(P.num_data_points,3); % create an empty matrix to store torque vectors
angles = zeros(P.num_data_points,1); % create an empty matrix to store angles
r_torque = zeros(P.num_data_points,3); % create an empty matrix to store torque radius
F_bungee = zeros(P.num_data_points,3); % create an empty matrix to store bungee force
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
    
    % Calculate helix configuration
    [L_helix_mag, F_direction_vec] = bungee_helix(P.r_wrist_bungee, P.r_forearm_bungee, P.L_forearm, P.phi_twist, P.theta_forearm); % run helix configuration function
    L_helix(i) = L_helix_mag; % append length of current helix iteration to helix length matrix
    F_direction(i,:) = F_direction_vec; % append direction of current force iteration to direction matrix
    F_origin(i,:) = [S.x_wrist_bungee, S.y_wrist_bungee, S.z_wrist_bungee]; % append force origin position to origin matrix 
    
    % Calculate F_magnitude
    F_magnitude(i,:) = bungee_force(P.k_bungee, P.L_bungee_rest, L_helix(i)); % calculate magnitude of spring force

    % Calculate Torque
    Tau(i) = bungee_torque(F_magnitude(i,:), F_direction(i,:), F_origin(i,:), P.L_forearm); % calculate torque on forearm by bungee
    Tau_vec(i,:) = S.torque;
    r_torque(i,:) = S.r_torque;
    F_bungee(i,:) = S.F_bungee;
   
    % Plot Forearm Animation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    exonetAnimation;
    angles(i) = abs(rad2deg(P.phi_twist) - P.init_wrist_angle);

    % Draw torque plot alongside forearm rotation %%%%%%%%%%%%%%%%%%%%%%%%%

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

        % desiredTorquePlot.XData = desired_angles(i);
        % desiredTorquePlot.YData = S.desired_torques(i);

    end

    % Finish Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    drawnow; % update plot immediately
    
    % Insert pause time between datapoints and rotation transition
    switch rotationPattern 
        case {'L-Supination', 'R_Supination'}
            if i == P.total_frames % if changing between pronation and supination
                pause(15*P.pause_time); % pause for a long time
            else
                pause(P.pause_time); % pause for a short time
            end
    
        case {'L-Pronation-Supination', 'R-Pronation-Supination'}
            if i == P.total_frames/2 % if changing between pronation and supination
                pause(15*P.pause_time); % pause for a long time
            else
                pause(P.pause_time); % pause for a short time
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
videofile = fullfile([pwd '\MATLAB Figures'], 'ExoNET_Bungee_Wrapping_v9_Movie.mp4'); % file save path
forearm_bungee_wrap_movie = VideoWriter(videofile,'MPEG-4'); % create an .mp4 file
forearm_bungee_wrap_movie.FrameRate = 20;
open(forearm_bungee_wrap_movie);
writeVideo(forearm_bungee_wrap_movie, forearm_bungee_wrap_sim);
close(forearm_bungee_wrap_movie);

saveas(figure(1), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v9_Torque.png')); % save to computer


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

saveas(figure(2), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v9_Supplemental.png')); % save to computer


%% Functions

function parameters

% SETUP PARAMETERS TO BE USED IN ANALYSIS.

%% Initialize Global

global S P

%% Initialize Parameters

% Rod lengths and properties
P.L_forearm = 12; % measured forearm length : in
P.r_wrist = 3; % measured wrist width : in
P.r_forearm = 5; % measured forearm width : in
P.r_wrist_bungee = P.r_wrist; % bungee wrist attachment moment arm : in
P.r_forearm_bungee = P.r_forearm; % bungee forearm attachment moment arm : in

% Bungee cord properties
P.L_bungee_rest = 6; % measured unstretched length of the bungee cord : in
P.k_bungee = 100; % bungee cord stiffness constant

% Convert Lengths to Meters
u = symunit; % create symbolic unit variable
P.L_forearm = double(separateUnits(unitConvert(P.L_forearm*u.in,u.m))); % convert to meters
P.L_bungee_rest = double(separateUnits(unitConvert(P.L_bungee_rest*u.in,u.m))); % convert to meters and halve it

P.r_wrist = double(separateUnits(unitConvert(P.r_wrist*u.in,u.m)))/2; % convert to meters and halve it
P.r_forearm = double(separateUnits(unitConvert(P.r_forearm*u.in,u.m)))/2; % convert to meters and halve it
P.r_wrist_bungee = double(separateUnits(unitConvert(P.r_wrist_bungee*u.in,u.m)))/2; % convert to meters and halve it
P.r_forearm_bungee = double(separateUnits(unitConvert(P.r_forearm_bungee*u.in,u.m)))/2; % convert to meters and halve it

% Define the rotation parameters
P.phi_twist_step = P.rotation_angle/P.num_data_points; % angle step for twist
P.pause_time = 0.01; % pause time between frames in seconds

% Initialize angles
P.theta_wrist = deg2rad(P.init_wrist_angle); % initial wrist angle in radians
P.theta_forearm = deg2rad(P.init_forearm_angle); % initial forearm angle in radians
P.theta_elbow = deg2rad(P.init_elbow_angle); % elbow angle in radians
P.beta = deg2rad(90); % set angle from z-axis

% Initialize helix parameters
P.num_helix_points = 100; % number of points for the helix
S.t_helix = linspace(0, 1, P.num_helix_points); % parameter for helix positions
S.x_helix = zeros(1, P.num_helix_points); % initialize x-coordinates of helix
S.y_helix = zeros(1, P.num_helix_points); % initialize y-coordinates of helix
S.z_helix = zeros(1, P.num_helix_points); % initialize z-coordinates of helix
P.pitch = P.L_forearm; % pitch of the helix

end

function cost = objective_function(params)

% COST FUNCTION CALCULATION.
% params : parameters to be optimized : initial_forearm_angle

%% Initialize Global Variables
global S P

%% Extract parameters

init_L_bungee = params(1);
init_forearm_angle = params(2);

%% Run the simulation with the given parameters

S.tau = zeros(1,P.num_data_points); % create an empty matrix to store torque values

for i = 1:P.total_frames % iterate through each frame

    % Identify system rotation pattern and determine direction of rotation
    if strcmp(P.caseValue, 'L-Supination')
            if i == 1
                P.phi_twist = P.theta_wrist - deg2rad(P.phi_twist_step);
            else
                P.phi_twist = P.phi_twist - deg2rad(P.phi_twist_step);
            end

    elseif strcmp(P.caseValue, 'L-Pronation-Supination')
            if i == 1
                P.phi_twist = P.theta_wrist + deg2rad(P.phi_twist_step);
            elseif (i > 1) && (i <= P.num_data_points)
                P.phi_twist = P.phi_twist + deg2rad(P.phi_twist_step);
            else
                P.phi_twist = P.phi_twist - deg2rad(P.phi_twist_step);
            end
    end
    
    % Calculate helix configuration
    [L_helix_mag, F_direction_vec] = bungee_helix(P.r_wrist_bungee, P.r_forearm_bungee, P.L_forearm, P.phi_twist, init_forearm_angle); % run helix configuration function
    F_origin = [S.x_wrist_bungee, S.y_wrist_bungee, S.z_wrist_bungee]; % append force origin position to origin matrix 
    
    % Calculate F_magnitude
    F_magnitude = bungee_force(P.k_bungee, init_L_bungee, L_helix_mag); % calculate magnitude of spring force

    % Calculate Torque
    S.tau(i) = bungee_torque(F_magnitude, F_direction_vec, F_origin, P.L_forearm); % calculate torque on forearm by bungee

end

% Calculate the cost as the negative of the sum of the squares of the differences between simulation torques and desired torques 
cost = -sum((S.tau - S.desired_torques).^2);

end

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
xlim([-3*P.r_forearm_bungee, 3*P.r_forearm_bungee]); % set x-axis limits
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

function plotSTLforearm

% SETUP AND PLOT STL OF FOREARM.

%% Initialize Global

global S P

%% Import STL & Extract Faces & Vertices

[F,V] = stlread('CAD Files/L_Forearm_Compressed.STL'); % read the stl file
S.F = F;

%% Apply Rotation Matrix to Correctly Orient Z-Axis

R = [cos(P.beta), 0, -sin(P.beta); 0, -1, 0; -sin(P.beta), 0, cos(P.beta)]; % rotate STL to align z-axis with frustrum
V = (R * V')';  % apply the rotation

%% Translate to Correctly Align Origins

z_translation = -min(V(:, 3)); % identify min value of forearm stl
V = V + [0, 0, z_translation]; % apply transformation stl vertices

%% Identify STL Forearm Length

S.z_lower_bound = min(V(:,3)); % identify point of forearm nearest to elbow
S.z_upper_bound = 0.525*max(V(:,3)); % identify point of forearm nearest to wrist
current_height = S.z_upper_bound-S.z_lower_bound;  % calculate distance of stl forearm

%% Scale STL Based on STL Forearm Length & Desired

scaling_factor = P.L_forearm / current_height; % calculate scaling factor based on stl forearm length
V = V * scaling_factor; % apply the scaling to stl vertices

%% Identify STL Z-Axis Centerline & Translate to Align with Frustrum Centerline

x_center = mean(V(:,1)); % calculate x-center of forearm
y_center = mean(V(:,2)); % calculate y-center of forearm
x_translation = 0 - x_center + 0.005; % x-translate forearm to fit in frustrum
y_translation = 0 - y_center; % y-translate forearm to fit in frustrum
S.V = V + [x_translation, y_translation, 0]; % apply transformations to stl vertices

%% Recalculate STL Centerline

S.x_center = mean(S.V(:,1)) - 0.005; % recalculate x-center of forearm stl vertices
S.y_center = mean(S.V(:,2)); % recalculate y-center of forearm stl vertices

%% Setup Plotting Environment

camlight('headlight'); % Add camera light
material('dull'); % Tone down the specular highlighting
% axis('image'); % fix the axis 

%% Optional Plotting

% patch('Vertices', S.V, 'Faces', S.F, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none'); % Plot the 3D model
% line([S.x_center, S.x_center], [S.y_center, S.y_center], [0,10], 'Color', 'r', 'LineWidth', 2); % plot line indicating centerline


%% Align STL

V_centered = S.V - [S.x_center, S.y_center, 0]; % center the object along X and Y axes
init_rotation_matrix = [cos(P.theta_wrist), -sin(P.theta_wrist), 0; ...
                           sin(P.theta_wrist), cos(P.theta_wrist), 0; ...
                           0, 0, 1]; % create a 2D rotation matrix in the XY plane
V_centered = (init_rotation_matrix * V_centered')'; % apply the rotation to the centered vertices
S.V_centered = V_centered + [S.x_center, S.y_center, 0]; % reposition the rotated object to its original location


end

function [L_helix, F_direction] = bungee_helix(r_wrist_bungee, r_forearm_bungee, L_forearm, phi_twist, init_theta_forearm)

% BUNGEE HELIX CONFIGURATION. 
% r_wrist_bungee : radius of user's wrist (ExoNET wrist band location);
% r_forearm_bungee : radius of user's forearm just below elbow (ExoNET forearm band location);
% L_forearm : length of forearm (distance between wrist and forearm bands)
% phi_twist : twist angle of the bungee cord (dynamic frequency of bungee)
% L_helix : length of the helix in single iteration (tells us how much stretch in the bungee during rotation
% F_direction : direction of the bungee force at the wrist

%% Initialize Global

global S P

%% Identify Current Positions of Wrist and Forearm

% Calculate wrist bungee attachment position
S.x_wrist_bungee = r_wrist_bungee * sin(P.beta) * cos(phi_twist);
S.y_wrist_bungee = r_wrist_bungee * sin(P.beta) * sin(phi_twist);
S.z_wrist_bungee = L_forearm;

% Calculate forearm bungee attachment position
S.x_forearm_bungee = r_forearm_bungee * sin(P.beta) * cos(init_theta_forearm);
S.y_forearm_bungee = r_forearm_bungee * sin(P.beta) * sin(init_theta_forearm);
S.z_forearm_bungee = 0;
    
%% Calculate Helix Configuration

% Initialize the length of the helix to 0
L_helix = 0; % total length of helix for single iteration

% Update helix parameters
for j = 1:P.num_helix_points % iterate through each helix point
    t = S.t_helix(j); % get parameter for current helix point
    R = (r_wrist_bungee - r_forearm_bungee) * t + r_forearm_bungee; % calculate radius for helix point

    if j == 1
        S.x_helix(j) = S.x_forearm_bungee; % set first x-coordinate to forearm position
        S.y_helix(j) = S.y_forearm_bungee; % set first y-coordinate to forearm position
        S.z_helix(j) = S.z_forearm_bungee; % set first z-coordinate to forearm position

    elseif j == length(P.num_helix_points)
        S.x_helix(j) = S.x_wrist_bungee; % set last x-coordinate to wrist position
        S.y_helix(j) = S.y_wrist_bungee; % set last y-coordinate to wrist position
        S.z_helix(j) = S.z_wrist_bungee; % set last z-coordinate to wrist position

    else
        S.x_helix(j) = R * cos((phi_twist - init_theta_forearm) * t + init_theta_forearm); % calculate x-coordinate for helix point
        S.y_helix(j) = R * sin((phi_twist - init_theta_forearm) * t + init_theta_forearm); % calculate y-coordinate for helix 
        S.z_helix(j) = P.pitch * t; % calculate z-coordinate for helix point
        
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

function F_magnitude = bungee_force(k_bungee, L_bungee_rest, L_helix)

% BUNGEE FORCE CALCULATION. 
% L_bungee_rest : rest length of the bungee cord helix prior to mounting on forearm;
% k_bungee : material properties of the bungee cord;
% L_helix : length of the helix in single iteration (tells us how much stretch in the bungee during rotation
% F_magnitude : magnitude of the bungee cord force

%% Identify Bungee Cord Delta Length

deltaL = L_helix - L_bungee_rest; % calculate the difference between helical length and resting length

%% Calculate Bungee Spring Force

F_spring = -k_bungee * deltaL; % calculate bungee force
F_magnitude = norm(F_spring); % calculate the bungee force magnitude

end

function Tau = bungee_torque(F_magnitude, F_direction, F_origin, L_forearm)


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
S.r_torque = F_origin - S.rotation_origin; % calculate distance from origin of forearm bungee attachment to point of force application

%% Calculate Resulting Torque

S.torque = cross(S.r_torque, S.F_bungee); % calculate torque using cross product
Tau = norm(S.torque);

end

function exonetAnimation

% PLOT EXONET ANIMATION FOR ANALYSIS.

%% Initialize Global

global S P plt

%% Plot First Figure

subplot(1,2,1); % call first figure for plotting helix animation

% Plot forearm centerline
plt.forearm_centerline = plot3([0, 0], [0, 0], [0, P.L_forearm], 'k', 'LineWidth', 2); % plot forearm centerline

% Rotate frustrum
X_rot = S.X .* cos(P.phi_twist) - S.Y .* sin(P.phi_twist);
Y_rot = S.X .* sin(P.phi_twist) + S.Y .* cos(P.phi_twist);

% Plot rotated frustrum
plt.frustrum = surf(X_rot, Y_rot, S.Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % plot transparent frustrum
colormap('jet'); % set colormap
set(plt.frustrum, 'CData', S.theta);
plt.wrist_bungee = plot3([0, S.x_wrist_bungee], [0, S.y_wrist_bungee], [P.L_forearm, S.z_wrist_bungee], 'b', 'LineWidth', 2); % plot wrist bungee attachment
plt.forearm_bungee = plot3([0, S.x_forearm_bungee], [0, S.y_forearm_bungee], [0, S.z_forearm_bungee], 'g', 'LineWidth', 2); % plot forearm bungee attachment
plt.helix = plot3(S.x_helix, S.y_helix, S.z_helix, 'r', 'LineWidth', 2); % plot helix (bungee cord)    

% Rotate the STL forearm
Rz = [cos(P.phi_twist), -sin(P.phi_twist), 0; sin(P.phi_twist), cos(P.phi_twist), 0; 0, 0, 1];
V_rotated = (Rz * S.V_centered')';
plt.forearm_stl = patch('Vertices', V_rotated, 'Faces', S.F, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none','FaceAlpha', 0.2);
% forearm_pointcloud = scatter3(V_rotated(:,1), V_rotated(:,2), V_rotated(:,3), 10, 'k', 'filled', 'MarkerFaceAlpha', 0.015, 'MarkerEdgeAlpha', 0.015);


end

