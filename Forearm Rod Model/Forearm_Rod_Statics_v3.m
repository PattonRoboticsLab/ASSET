%% ExoNET Program to Compare Forearm Angle vs. Output Torque 

close all; % close all existing figures
clear all; % clear all variables
clc; % clear the command window
warning('off', 'MATLAB:audiovideo:VideoWriter:mp4FramePadded'); % turn off warning


%% Initialize & Establish Input Variables

global P S plt

affected_arm = 'L'; % options = R (right arm) or L (left arm) ----RIGHT ARM DOES NOT WORK RIGHT NOW----
rotation_pattern = 'Supination';  % options = 'Supination', 'Pronation', 'Supination-Pronation', 'Pronation-Supination'
rotationPattern = affected_arm + "-" + rotation_pattern; % concatenate options together

P.rotation_angle = 180; % total angle of forearm rotation : degrees
P.num_data_points = P.rotation_angle/2; % number of data points in matrices

switch rotationPattern
    case 'L-Supination'
        P.total_frames = P.num_data_points;
        P.init_wrist_rod_angle = 270; % initial wrist rod angle
        P.init_elbow_angle = 80; % initial elbow angle
        P.caseValue = 'L-Supination';
        parameters;

    case 'L-Pronation-Supination'
        P.total_frames = 2*P.num_data_points; 
        P.init_wrist_rod_angle = 360; % initial wrist rod angle
        P.init_elbow_angle = 90; % initial elbow angle
        P.caseValue = 'L-Pronation-Supination';
        parameters;
        
end

%% Optimization Algorithm

% Define the desired torques
desired_angles = 0:P.phi_twist_step:P.rotation_angle-1;
S.desired_torques = 0.01 * (desired_angles).^2; % sinusoidal top doesnt make sense here, should try linear, half trapezoid and sinusoidal crossing over 0 at 90 deg

% Initial guesses for the parameters
init_params = [P.phi_moment_arm, P.moment_arm_location, P.L_rod];  % [upper arm moment arm angle, attachment point on upper arm length, rod length]

% Bounds for the parameters
phi_moment_arm_lb = -90; % lower bound of moment arm angle
phi_moment_arm_ub = 90; % upper bound of moment arm angle
moment_arm_location_lb = 0.1; % lower bound of moment arm location on upper arm
moment_arm_location_ub = 0.9; % upper bound of moment arm location on upper arm
L_rod_lb = 8; % lower bound of rod length 
L_rod_ub = 15; % upper bound of rod length

% Convert input parameters
u = symunit; % create symbolic unit variable
phi_moment_arm_lb = deg2rad(phi_moment_arm_lb); % convert to radians
phi_moment_arm_ub = deg2rad(phi_moment_arm_ub); % convert to radians
L_rod_lb = double(separateUnits(unitConvert(L_rod_lb*u.in,u.m))); % convert to meters
L_rod_ub = double(separateUnits(unitConvert(L_rod_ub*u.in,u.m))); % convert to meters

% Loop fmincon optimization
bestCost = 1e5; % best cost allowed by optimization
nTries = 10; % number of times running the optimization

fprintf('\n\n Begin optimizations:  ');
% options = optimoptions('fmincon','Display','iter-detailed');
options = optimset('Display', 'iter');

for TRY =1:nTries
    fprintf('Opt#%d of %d..',TRY,nTries);

    % [optimal_params, cost, exitflag, output] = fmincon(@objective_function, init_params, [], [], [], [], [phi_moment_arm_lb, moment_arm_location_lb, L_rod_lb], [phi_moment_arm_ub, moment_arm_location_ub, L_rod_ub], [], options);
    [optimal_params, cost, exitflag, output] = fminsearch(@objective_function, init_params, options);

    % init_params = random(combination of parameter values) % this will help prevent getting stuck in local minimum
    % random(iqr(bounds)) % this will help lessen reliance on regularization

    if cost < bestCost % if lower cost
        fprintf(' cost =%g, ',cost);
        bestCost = cost; 
        init_params = optimal_params; % update initial parameters to new optimal parameters
    else
        fprintf(' (not an improvement) \n ');
    end
end

disp(['Optimal Moment Arm Angle: ', num2str(rad2deg(optimal_params(1)))]);
P.phi_moment_arm = optimal_params(1);

disp(['Optimal Moment Arm Location: ', num2str(optimal_params(2))]);
P.moment_arm_location = optimal_params(2);

disp(['Optimal Rod Length: ', num2str(double(separateUnits(unitConvert(optimal_params(3)*u.m,u.in))))]);
P.L_rod = optimal_params(3);


%% Setup the Plots

setupPlots; % set up the plots for animation
figure(1); subplot(1,2,1); % set figure for stl plotting
plotSTLforearm; % plot stl of forearm


%% Animation Loop

L_rod_deformed = zeros(P.num_data_points,1); % create an empty matrix to store helix lengths
F_direction = zeros(P.num_data_points,3); % create an empty matrix to store direction of force
F_origin = zeros(P.num_data_points,3); % create an empty matrix to store wrist rod attachment position
F_magnitude = zeros(P.num_data_points,1); % create an empty matrix to store magnitude of force
Tau = zeros(P.num_data_points,1); % create an empty matrix to store torque values
Tau_vec = zeros(P.num_data_points,3); % create an empty matrix to store torque vectors
angles = zeros(P.num_data_points,1); % create an empty matrix to store angles
r_torque = zeros(P.num_data_points,3); % create an empty matrix to store torque radius
F_rod = zeros(P.num_data_points,3); % create an empty matrix to store rod force
forearm_rod_sim(P.num_data_points) = struct('cdata',[],'colormap',[]); % pre-allocate the struct array

figure(1); % call figure 1 for plotting

for i = 1:P.total_frames % iterate through each frame

    % Identify system rotation pattern and determine direction of rotation
    switch rotationPattern
        case 'L-Supination'
            if i == 1
                P.phi_twist = P.theta_wrist_rod - deg2rad(P.phi_twist_step);
            else
                P.phi_twist = P.phi_twist - deg2rad(P.phi_twist_step);
            end

        case 'L-Pronation-Supination'
            if i == 1
                P.phi_twist = P.theta_wrist_rod + deg2rad(P.phi_twist_step);
            elseif (i > 1) && (i <= P.num_data_points)
                P.phi_twist = P.phi_twist + deg2rad(P.phi_twist_step);
            else
                P.phi_twist = P.phi_twist - deg2rad(P.phi_twist_step);
            end
    end
    
    % Calculate rod configuration
    [L_rod_mag, F_direction_vec] = deformed_rod(P.r_wrist_rod, P.L_forearm, P.L_rod, P.phi_twist, P.phi_moment_arm, P.moment_arm_location, P.theta_elbow); % run helix configuration function
    L_rod_deformed(i) = L_rod_mag; % append length of current helix iteration to helix length matrix
    F_direction(i,:) = F_direction_vec; % append direction of current force iteration to direction matrix
    F_origin(i,:) = [S.x_wrist_rod, S.y_wrist_rod, S.z_wrist_rod]; % append force origin position to origin matrix 
      
    % Calculate F_magnitude
    F_magnitude(i,:) = rod_force(P.L_rod, L_rod_deformed(i), S.dist_coordinates); % calculate magnitude of spring force

    % Calculate Torque
    Tau(i) = rod_torque(F_magnitude(i,:), F_direction(i,:), F_origin(i,:), P.L_forearm); % calculate torque on forearm by rod
    Tau_vec(i,:) = S.torque;
    r_torque(i,:) = S.r_torque;
    F_rod(i,:) = S.F_rod;
    
    % Plot Forearm Animation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    exonetAnimation;
    angles(i) = abs(rad2deg(P.phi_twist) - P.init_wrist_rod_angle);
    
    % Plot the arrow for the direction of the force %%%%%%%%%%%%%%%%%%%%%%%
   
    arrowLength = 0.15; % Change this to your desired length
    plt.forceArrow = quiver3(F_origin(i,1), F_origin(i,2), F_origin(i,3), ...
                             F_direction(i,1)*arrowLength, ...
                             F_direction(i,2)*arrowLength, ...
                             F_direction(i,3)*arrowLength, ...
                             'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % plot force vector as a unit vector arrow
   
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

        desiredTorquePlot.XData = desired_angles(i);
        desiredTorquePlot.YData = S.desired_torques(i);

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
    
    forearm_rod_sim(i) = getframe(gcf); % get frames of figure to save animation file

    if i < P.total_frames  % if not the last frame, delete previous plots
        fields = fieldnames(plt);
        for a = 1:length(fields)
            delete(plt.(fields{a}));
        end
    end

end


%% Save Animation

% Write animation to MP4 file
videofile = fullfile([pwd '\MATLAB Figures'], 'ExoNET_Rod_v3_Movie.mp4'); % file save path
forearm_rod_movie = VideoWriter(videofile,'MPEG-4'); % create an .mp4 file
forearm_rod_movie.FrameRate = 20;
open(forearm_rod_movie);
writeVideo(forearm_rod_movie, forearm_rod_sim);
close(forearm_rod_movie);

saveas(figure(1), fullfile([pwd '\MATLAB Figures'],'ExoNET_Rod_v3_Torque.png')); % save to computer


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

saveas(figure(2), fullfile([pwd '\MATLAB Figures'],'ExoNET_Rod_v3_Supplemental.png')); % save to computer


%% Functions

function parameters

% SETUP PARAMETERS TO BE USED IN ANALYSIS.

%% Initialize Global

global S P

%% Initialize Parameters

% Rod lengths and properties
P.L_forearm = 12; % measured forearm length : in
P.L_up_arm = 14; % measured upper arm length : in
P.L_rod = 12; % measured length of the actuator rod : in

P.r_wrist = 3; % measured wrist width : in
P.r_forearm = 5; % measured forearm width : in
P.r_up_arm = 5; % measured upper arm width : in
P.r_wrist_rod = P.r_wrist; % rod wrist attachment moment arm : in
P.r_up_arm_rod = P.r_up_arm; % rod attachment 

% Actuator Rod Properties
P.E = 5.5e6; % Young's modulus : psi (for fiberglass)
P.E = P.E * 6894.76; % Young's modulus : Pascals (for fiberglass)
P.I = 1e-6; % area moment of inertia in m^4 (I = (1/4)*pi*r^4)) REPLACE WITH CALC
P.moment_arm_location = 0.5; % location along the upper arm from origin of rod attachment
P.phi_moment_arm = 0; % angle of rotation around upper arm of moment arm

% Convert Lengths to Meters
u = symunit; % create symbolic unit variable
P.L_forearm = double(separateUnits(unitConvert(P.L_forearm*u.in,u.m))); % convert to meters
P.L_up_arm = double(separateUnits(unitConvert(P.L_up_arm*u.in,u.m))); % convert to meters
P.L_rod = double(separateUnits(unitConvert(P.L_rod*u.in,u.m))); % convert to meters

P.r_wrist = double(separateUnits(unitConvert(P.r_wrist*u.in,u.m)))/2; % convert to meters and halve it
P.r_forearm = double(separateUnits(unitConvert(P.r_forearm*u.in,u.m)))/2; % convert to meters and halve it
P.r_up_arm = double(separateUnits(unitConvert(P.r_up_arm*u.in,u.m)))/2; % convert to meters and halve it
P.r_up_arm_rod = double(separateUnits(unitConvert(P.r_up_arm_rod*u.in,u.m)))/2; % convert to meters and halve it
P.r_wrist_rod = double(separateUnits(unitConvert(P.r_wrist_rod*u.in,u.m)))/2; % convert to meters and halve it

% Define the rotation parameters
P.phi_twist_step = P.rotation_angle/P.num_data_points; % angle step for twist
P.pause_time = 0.01; % pause time between frames in seconds

% Initialize angles
P.theta_wrist_rod = deg2rad(P.init_wrist_rod_angle); % initial wrist rod angle in radians
P.theta_elbow = deg2rad(P.init_elbow_angle); % elbow angle in radians
P.beta = deg2rad(90); % set angle from z-axis
P.phi_moment_arm = deg2rad(P.phi_moment_arm); % convert to radians

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

init_phi_moment_arm = params(1);
init_moment_arm_location = params(2);
init_L_rod = params(3);


%% Run the simulation with the given parameters

S.tau = zeros(1,P.num_data_points); % create an empty matrix to store torque values

for i = 1:P.total_frames % iterate through each frame

    % Identify system rotation pattern and determine direction of rotation
    if strcmp(P.caseValue, 'L-Supination')
            if i == 1
                P.phi_twist = P.theta_wrist_rod - deg2rad(P.phi_twist_step);
            else
                P.phi_twist = P.phi_twist - deg2rad(P.phi_twist_step);
            end

    elseif strcmp(P.caseValue, 'L-Pronation-Supination')
            if i == 1
                P.phi_twist = P.theta_wrist_rod + deg2rad(P.phi_twist_step);
            elseif (i > 1) && (i <= P.num_data_points)
                P.phi_twist = P.phi_twist + deg2rad(P.phi_twist_step);
            else
                P.phi_twist = P.phi_twist - deg2rad(P.phi_twist_step);
            end
    end
    
    % Calculate rod configuration
    [L_rod_mag, F_direction_vec] = deformed_rod(P.r_wrist_rod, P.L_forearm, init_L_rod, P.phi_twist, init_phi_moment_arm, init_moment_arm_location, P.theta_elbow); % run helix configuration function
    F_origin = [S.x_wrist_rod, S.y_wrist_rod, S.z_wrist_rod]; % append force origin position to origin matrix 
      
    % Calculate F_magnitude
    F_magnitude = rod_force(init_L_rod, L_rod_mag, S.dist_coordinates); % calculate magnitude of spring force

    % Calculate Torque
    S.tau(i) = rod_torque(F_magnitude, F_direction_vec, F_origin, P.L_forearm); % calculate torque on forearm by rod
    
end

% Calculate the cost as the negative of the sum of the squares of the differences between simulation torques and desired torques 
cost = sum((S.tau - S.desired_torques).^2);

% cost = -sum((S.tau - S.desired_torques).^2); % use this if we are doing a maximization problem instead of a minimization problem
% can try mean or median instead of sum (use median over mean because cannot have prior assumptions over distribution)
% at the end to punish system if using fminserch: cost = cost + 100*param^2 (L2 norm = regularization)

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
xlim([-P.L_up_arm, 3*P.r_forearm]); % set x-axis limits
ylim([-3*P.r_forearm, 3*P.r_forearm]); % set y-axis limits
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

R = [cos(P.beta), 0, sin(P.beta); 0, 1, 0; -sin(P.beta), 0, cos(P.beta)]; % rotate STL to align z-axis with frustrum
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
init_rotation_matrix = [cos(P.theta_wrist_rod), -sin(P.theta_wrist_rod), 0; ...
                           sin(P.theta_wrist_rod), cos(P.theta_wrist_rod), 0; ...
                           0, 0, 1]; % create a 2D rotation matrix in the XY plane
V_centered = (init_rotation_matrix * V_centered')'; % apply the rotation to the centered vertices
S.V_centered = V_centered + [S.x_center, S.y_center, 0]; % reposition the rotated object to its original location


end

function [L_rod_deformed, F_direction] = deformed_rod(r_wrist_rod, L_forearm, L_rod, phi_twist, phi_moment_arm, moment_arm_location, theta_elbow)

% ACTUATOR ROD CONFIGURATION. 
% r_wrist_rod : radius of user's wrist;
% L_forearm : length of forearm
% L_rod : initial undeformed length of the rod
% phi_twist : twist angle of the wrist
% phi_moment_arm : 
% theta_elbow : 
% moment_arm_location : 
% L_rod_deformed : length of the rod under deformation (tells us how much deformation in the rod during rotation)
% F_direction : direction of the rod force at the wrist

%% Initialize Global

global S P

%% Identify Current Position of Wrist

% Calculate wrist rod attachment position
S.x_wrist_rod = r_wrist_rod * sin(P.beta) * cos(phi_twist);
S.y_wrist_rod = r_wrist_rod * sin(P.beta) * sin(phi_twist);
S.z_wrist_rod = L_forearm;

% Calculate the endpoint of the upper arm based on given parameters
S.x_shoulder = P.L_up_arm * cos(theta_elbow + P.beta); % shoulder x-coordinate
S.y_shoulder = 0; % shoulder y-coordinate
S.z_shoulder = P.L_up_arm * sin(theta_elbow + P.beta); % shoulder z-coordinate


%% Calculate Upper Arm Momement Arm Configuration

% Idenitfy upper arm centerline direction and perpendicular vector
shoulder_vec = [S.x_shoulder, S.y_shoulder, S.z_shoulder]; % centerline direction vector of the upper arm
S.up_arm_centerline_dir = shoulder_vec / norm(shoulder_vec); % normalize centerline direction vector
up_arm_perp_vec = cross(S.up_arm_centerline_dir, [0, -1, 0]);

% Create moment arm on upper arm
S.x_moment_arm = moment_arm_location * S.x_shoulder;
S.y_moment_arm = moment_arm_location * S.y_shoulder;
S.z_moment_arm = moment_arm_location * S.z_shoulder;

% Identify rotation axis for upper arm moment arm
rotAxis = S.up_arm_centerline_dir; % rotation axis
theta = phi_moment_arm;  % rotation angle

% Apply Rodrigues' Rotation Formula
K = [0 -rotAxis(3) rotAxis(2); rotAxis(3) 0 -rotAxis(1); -rotAxis(2) rotAxis(1) 0];
R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K ^ 2);

% Rotate the perpendicular vector around the rotation axis
up_arm_perp_vec_rotated = R * up_arm_perp_vec';

% Calculate the end point of the moment arm
S.x_moment_arm_end = S.x_moment_arm + P.r_up_arm_rod * up_arm_perp_vec_rotated(1);
S.y_moment_arm_end = S.y_moment_arm + P.r_up_arm_rod * up_arm_perp_vec_rotated(2);
S.z_moment_arm_end = S.z_moment_arm + P.r_up_arm_rod * up_arm_perp_vec_rotated(3);


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

% if abs(L_rod_deformed - L_rod) > tolerance
%     disp('The length of the deformed rod is not constant.');
%     disp(['Deformed rod length: ', num2str(L_rod_deformed)]);
%     disp(['Original rod length: ', num2str(L_rod)]);
% end


%% Calculate Direction of Force at the Last Node 
% % Force will act along the tangent to the arc at the wrist, away from the center of curvature

dx_force = S.arc_points(1, end) - S.arc_points(1, end - 1); % x-distance b/w last two nodes of rod
dy_force = S.arc_points(2, end) - S.arc_points(2, end - 1); % y-distance b/w last two nodes of rod
dz_force = S.arc_points(3, end) - S.arc_points(3, end - 1); % z-distance b/w last two nodes of rod

% Calculate magnitude of force at wrist
S.tangent_vec = [dx_force, dy_force, dz_force]; % tangent vector of the force at the wrist
magnitude = norm(S.tangent_vec); % calculate magnitude of vector
F_direction = S.tangent_vec / magnitude; % normalize to get unit vector of force


%% Ensure the direction is away from the center of curvature (the rod is trying to straighten out)

%%% THIS MAY NOT BE CORRECT HERE
% if dot(S.tangent_vec, vec) > 0
%     F_direction = -F_direction;
% end

% disp(['Direction of the tensile force (unit vector): [', num2str(F_direction(1)), ', ', ...
%       num2str(F_direction(2)), ', ', num2str(F_direction(3)), ']']);

end

function F_magnitude = rod_force(L_rod, L_rod_deformed, dist_coordinates)

% ROD FORCE CALCULATION. 
% L_rod : undeformed length of the rod prior to deformation;
% L_rod_deformed : length of the rod under deformation (tells us how much deformation in the rod during rotation)
% dist_coordinates : distance between the mounting points on the upper arm and wrist
% F_magnitude : magnitude of the rod force

%% Initialize Global

global P

%% Calculate Spring Constant & Identify Rod Delta Length

k = (3 * P.E * P.I) / L_rod^3; % calculate spring constant for the rod
delta_L = L_rod_deformed - dist_coordinates; % calculate the deflection at the wrist

%% Caclulate Rod Spring Force

F_rod = k * delta_L; % calculate the restorative force acting at the wrist due to rod deformation
F_magnitude = norm(F_rod); % calculate the rod force magnitude

%% Output the calculated force and direction

% disp(['Tensile restorative force acting on the wrist due to rod deformation: ', num2str(F_magnitude), ' N']);


end

function Tau = rod_torque(F_magnitude, F_direction, F_origin, L_forearm)

% ROD TORQUE CALCULATION. 
% F_magnitude : magnitude of the rod force
% F_direction : direction of the rod force at the wrist
% Tau : torque induced by the rod when rotating the forearm

%% Initialize Global

global S

%% Identify Force and Radius Matrices of Bungee Cord

S.F_rod = F_magnitude * F_direction; % calculate force vector

S.rotation_origin = [0, 0, L_forearm]; % calculate origin of rotation 
S.r_torque = F_origin - S.rotation_origin; % calculate distance from origin of rod attachment to point of force application

%% Calculate Resulting Torque

S.torque = cross(S.r_torque, S.F_rod); % calculate torque using cross product
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
plt.wrist_rod = plot3([0, S.x_wrist_rod], [0, S.y_wrist_rod], [P.L_forearm, S.z_wrist_rod], 'b', 'LineWidth', 2); % plot wrist rod attachment

% Rotate the STL forearm
Rz = [cos(P.phi_twist), -sin(P.phi_twist), 0; sin(P.phi_twist), cos(P.phi_twist), 0; 0, 0, 1];
V_rotated = (Rz * S.V_centered')';
plt.forearm_stl = patch('Vertices', V_rotated, 'Faces', S.F, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none','FaceAlpha', 0.2);
% plt.forearm_pointcloud = scatter3(V_rotated(:,1), V_rotated(:,2), V_rotated(:,3), 10, 'k', 'filled', 'MarkerFaceAlpha', 0.015, 'MarkerEdgeAlpha', 0.015);


%% Plot Upper Arm Cylinder

% Plot upper arm centerline
plt.up_arm_centerline = plot3([S.x_shoulder, 0], [S.y_shoulder, 0], [S.z_shoulder, 0], 'r', 'LineWidth', 2);

% Create the cylinder mesh 
[x_cylinder, y_cylinder, z_cylinder] = cylinder(P.r_up_arm, 50); % create a cylinder with radius
z_cylinder = z_cylinder * P.L_up_arm; % scale the Z-coordinates to match the upper arm length

% Compute the rotation axis and angle
rotAxis = cross([0 0 1], S.up_arm_centerline_dir); % axis perp to both matrices
rotAngle = acos(dot([0 0 1], S.up_arm_centerline_dir)); % rotation needed to align direction vec with Z-axis

% Ensure the rotation axis is not a zero vector
if norm(rotAxis) ~= 0
    rotAxis = rotAxis / norm(rotAxis);  % normalize the rotation axis

    % Create the rotation matrix using the Rodrigues' rotation formula
    K = [0 -rotAxis(3) rotAxis(2); rotAxis(3) 0 -rotAxis(1); -rotAxis(2) rotAxis(1) 0];
    R = eye(3) + sin(rotAngle) * K + (1 - cos(rotAngle)) * (K ^ 2);

    % Apply the rotation to each point in the cylinder mesh
    for i = 1:size(x_cylinder, 2)
        for j = 1:2
            point = [x_cylinder(j, i); y_cylinder(j, i); z_cylinder(j, i)];
            rotated_point = R * point;
            x_cylinder(j, i) = rotated_point(1);
            y_cylinder(j, i) = rotated_point(2);
            z_cylinder(j, i) = rotated_point(3);
        end
    end
end

cylinder_surface = surf(x_cylinder, y_cylinder, z_cylinder, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
cylinder_colors = linspace(0, 2*pi, 50); % angles for the circular cross-section
set(cylinder_surface, 'CData', cylinder_colors); % set color based on angle around the cylinder
colormap('jet'); % use the jet colormap for coloring the cylinder


%% Plot Rod and Components

% Plot upper arm moment arm 
plt.up_arm_moment_arm = plot3([S.x_moment_arm, S.x_moment_arm_end], [S.y_moment_arm, S.y_moment_arm_end], [S.z_moment_arm, S.z_moment_arm_end], 'b', 'LineWidth', 2);

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

