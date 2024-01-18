function tau = single_joint(L1,r,theta,phi1,k,x0)

% COMPUTE TORQUE OF SINGLE JOINT EXONET. 
% L1: segment length : m
% r : initial element attachment length from ground : m
% theta : angle between both grounding points : degrees
% phi1 : segment angle range : degrees
% k : spring constant
% x0 : resting length of elastic element : m

global P

%% Segment Calculations
P.r = [r*cosd(theta), r*sind(theta), 0]; % r position
P.L = [L1*cosd(phi1), L1*sind(phi1), 0]; % l position
P.x = P.L-P.r; % spring position matrix

%% Force Calculations
P.x_mag = norm(P.x); % magnitude of spring position
P.x_norm = P.x./P.x_mag; % normalized spring position values - unit vector
P.T_spring = -k*(P.x_mag-x0); % spring force calculation
P.T = P.T_spring .* P.x_norm; % force components

%% Torque Calculation
P.tau = cross(P.L,P.T); % torque cross-product calculation
tau = P.tau(3); % taking magnitude of the torque

end
