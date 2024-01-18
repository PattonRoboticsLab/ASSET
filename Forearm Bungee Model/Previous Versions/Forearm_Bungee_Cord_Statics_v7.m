%% ExoNET Program to Compare Forearm Angle vs. Output Torque 

close all; % close all existing figures
clear all; % clear all variables
clc; % clear the command window
warning('off', 'MATLAB:audiovideo:VideoWriter:mp4FramePadded'); % turn off warning

%% Initialize & Establish Input Variables

global S

% Rod lengths and properties
L_forearm = 12; % length of the forearm rod in inches
r_wrist = 2.5; % width of the wrist rod in inches
r_forearm = 4; % width of the forearm rod in inches

% Bungee cord properties
L_rest = 10; % unstretched length of the bungee cord in inches
k_bungee = 100; % stiffness constant of the bungee cord

% Convert Lengths to Meters
u = symunit; % create symbolic unit variable
L_forearm = double(separateUnits(unitConvert(L_forearm*u.in,u.m))); % convert forearm length to meters
L_rest = double(separateUnits(unitConvert(L_rest*u.in,u.m))); % convert bungee resting length to meters and halve it
r_wrist = double(separateUnits(unitConvert(r_wrist*u.in,u.m)))/2; % convert wrist width to meters and halve it
r_forearm = double(separateUnits(unitConvert(r_forearm*u.in,u.m)))/2; % convert forearm width to meters and halve it

% Define the rotation parameters
rotation_angle = 180; % total rotation angle in degrees
S.num_frames = 90; % number of angles total rotation is segmented
pause_time = 0.01; % pause time between frames in seconds
phi_twist_step = 2; % angle step for twist

% Initial angles in degrees
initial_wrist_angle = 360; % initial wrist angle
initial_forearm_angle = 0;   % initial forearm angle

% Initialize angles
theta_wrist = deg2rad(initial_wrist_angle); % initial wrist angle in radians
S.theta_forearm = deg2rad(initial_forearm_angle); % initial forearm angle in radians
S.beta = pi/2; % set angle from z-axis

% Initialize helix parameters
S.num_helix_points = 100; % number of points for the helix
S.t_helix = linspace(0, 1, S.num_helix_points); % parameter for helix positions
S.x_helix = zeros(1, S.num_helix_points); % initialize x-coordinates of helix
S.y_helix = zeros(1, S.num_helix_points); % initialize y-coordinates of helix
S.z_helix = zeros(1, S.num_helix_points); % initialize z-coordinates of helix
S.pitch = L_forearm; % pitch of the helix

%% Plot Results

% Initialize 3D plot
figure;
grid on; % turn on grid
axis equal; % make axes equal
xlim([-r_forearm, r_forearm]); % set x-axis limits
ylim([-r_forearm, r_forearm]); % set y-axis limits
zlim([0, L_forearm]); % set z-axis limits
xlabel('X'); % label x-axis
ylabel('Y'); % label y-axis
zlabel('Z'); % label z-axis
title('Rods with Bungee Cord in 3D'); % add title to plot
view(3); % set the 3D view
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
hold on; % hold the plot to allow further plotting

% Initialize frustrum parameters
z_frustrum = linspace(0, L_forearm, 50); % z-axis positions for frustrum
theta_frustrum = linspace(0, 2*pi, 50); % angles for frustrum
[Z, theta] = meshgrid(z_frustrum, theta_frustrum); % create meshgrid for frustrum
R = -(r_forearm - r_wrist)/L_forearm .* Z + r_forearm; % calculate frustrum radius
X = R .* cos(theta); % calculate x-coordinates of frustrum
Y = R .* sin(theta); % calculate y-coordinates of frustrum

% Plot central rod
plot3([0, 0], [0, 0], [0, L_forearm], 'k', 'LineWidth', 2); % plot the central rod

%% Animation Loop

L_helix = zeros(S.num_frames,1); % create an empty matrix to store helix lengths
F_direction = zeros(S.num_frames,3); % create an empty matrix to store direction of force
F_origin = zeros(S.num_frames,3); % create an empty matrix to store wrist rod position
forearm_bungee_wrap_sim(S.num_frames) = struct('cdata',[],'colormap',[]); % pre-allocate the struct array

for i = 1:S.num_frames % iterate through each frame
    
    % Determine the direction of rotation
    if i == 1
        phi_twist = theta_wrist + deg2rad(phi_twist_step); % update wrist angle in radians
        
    elseif (i > 1) && (i <= S.num_frames)
        phi_twist = phi_twist + deg2rad(phi_twist_step); % update wrist angle in radians
    
    else
        phi_twist = phi_twist - deg2rad(phi_twist_step); % reverse wrist angle in radians
        
    end
    
    % Create the helix configuration
    [L_helix_single, F_direction_single] = helix(r_wrist, r_forearm, L_forearm, phi_twist); % run helix configuration function
    L_helix(i) = L_helix_single; % append length of current helix iteration to helix length matrix
    F_direction(i,:) = [F_direction_single(1),F_direction_single(2),F_direction_single(3)]; % append direction of current force iteration to direction matrix
    F_origin(i,:) = [S.x_wrist, S.y_wrist, S.z_wrist]; % append force origin position to origin matrix 

    % Rotate the frustrum
    X_rot = X .* cos(phi_twist) - Y .* sin(phi_twist);
    Y_rot = X .* sin(phi_twist) + Y .* cos(phi_twist);
    angles(i) = rad2deg(phi_twist);
    
    % Plotting and animation
    h_frustrum = surf(X_rot, Y_rot, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % plot transparent frustrum
    colormap('jet'); % set colormap
    set(h_frustrum, 'CData', theta);
    
    % Initialize an empty array to store handles for the vertical lines
    line_handles = [];
    
%     % Draw black vertical lines on the frustrum
%     num_vertical_lines = 25; % number of vertical lines
%     theta_lines = linspace(0, 2*pi, num_vertical_lines); % angles for vertical lines
%     for t_line = theta_lines
%         R_line = (r_forearm - r_wrist)/L_forearm .* z_frustrum + r_wrist; % calculate radius
%         X_line = R_line * cos(t_line); % x-coordinates for line
%         Y_line = R_line * sin(t_line); % y-coordinates for line
%         % Apply rotation to the line coordinates
%         X_line_rot = X_line * cos(phi_twist) - Y_line * sin(phi_twist);
%         Y_line_rot = X_line * sin(phi_twist) + Y_line * cos(phi_twist);
%         lh = plot3(X_line_rot, Y_line_rot, z_frustrum, 'Color', [0.7 0.7 0.7]); % plot vertical line and store handle
%         line_handles = [line_handles, lh]; % append handle to array
%     end
    
    wrist = plot3([0, S.x_wrist], [0, S.y_wrist], [L_forearm, S.z_wrist], 'b', 'LineWidth', 2); % plot wrist rod
    forearm = plot3([0, S.x_forearm], [0, S.y_forearm], [0, S.z_forearm], 'g', 'LineWidth', 2); % plot forearm rod
    
    helix_plot = plot3(S.x_helix, S.y_helix, S.z_helix, 'r', 'LineWidth', 2); % plot helix (bungee cord)
    
    drawnow; % update plot immediately
    
    if i == S.num_frames % if changing between pronation and supination
        pause(15*pause_time); % pause for a long time
    else
        pause(pause_time); % pause for a short time
    end
    
    forearm_bungee_wrap_sim(i) = getframe(gcf); % get frames of figure to save animation file

    if i < S.num_frames  % if not the last frame, delete previous plots
        delete(wrist);
        delete(forearm);
        delete(helix_plot);
        delete(h_frustrum);
        delete(line_handles);
    end
end

F_magnitude = spring_force(k_bungee, L_rest, L_helix); % calculate magnitude of spring force
Tau = exonet_torque(F_magnitude, F_direction, F_origin, L_forearm); % calculate torque on forearm by bungee


%% Final Plots

% Write animation to MP4 file
videofile = fullfile([pwd '\MATLAB Figures'], 'ExoNET_Bungee_Wrapping_v7_Movie.mp4'); % file save path
forearm_bungee_wrap_movie = VideoWriter(videofile,'MPEG-4'); % create an .mp4 file
forearm_bungee_wrap_movie.FrameRate = 20;
open(forearm_bungee_wrap_movie);
writeVideo(forearm_bungee_wrap_movie, forearm_bungee_wrap_sim);
close(forearm_bungee_wrap_movie);

% Plot torque vs angle graph
figure; % create new figure
plot(angles, Tau); % plot torque vs angle
xlabel('Wrist Rotation Angle (degrees)'); % label x-axis
ylabel('Resulting Torque (Nm)'); % label y-axis
title('Wrist Rotation Angle vs Resulting Torque'); % add title
grid on; % turn on grid

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
saveas(figure(2), fullfile([pwd '\MATLAB Figures'],'ExoNET_Bungee_Wrapping_v7_Torque.png')); % save to computer


%% Functions 

function [L_helix, F_direction] = helix(r_wrist, r_forearm, L_forearm, phi_twist)

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
S.x_forearm = r_forearm * sin(S.beta) * cos(S.theta_forearm);
S.y_forearm = r_forearm * sin(S.beta) * sin(S.theta_forearm);
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
        S.x_helix(j) = R * cos((phi_twist - S.theta_forearm) * t + S.theta_forearm); % calculate x-coordinate for helix point
        S.y_helix(j) = R * sin((phi_twist - S.theta_forearm) * t + S.theta_forearm); % calculate y-coordinate for helix 
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

L_rest = L_rest * ones(length(L_helix),1); % convert bungee cord resting length to matrix
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

rotation_origin = [0, 0, L_forearm] .* ones(S.num_frames,3); % calculate origin of rotation 
S.r_torque = F_origin - rotation_origin; % calculate distance from origin of forearm rod to point of force application
    
%% Calculate Resulting Torque

torque = cross(S.r_torque, S.F_bungee); % calculate torque using cross product

Tau = zeros(length(torque),1);
for i = 1:length(torque)
    Tau(i) = norm(torque(i,:));
end

end

