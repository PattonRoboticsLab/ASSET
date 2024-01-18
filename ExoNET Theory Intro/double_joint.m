function [tau1,tau2] = double_joint(L1,L2,r,theta,phi1,phi2,k,x0)

% COMPUTE TORQUE OF DOUBLE JOINT EXONET. 
% L1 : segment length 1 : m
% L2 : segment length 2 : m
% r : initial element attachment length from ground : m
% theta : angle between both grounding points : degrees
% phi1 : segment 1 angle range : degrees
% phi2 : segment 2 angle range : degrees
% k : spring constant
% x0 : resting length of elastic element : m

global P

%% Segment Calculations
P.r = [r*cosd(theta), r*sind(theta), 0]; % r position
P.L1 = [L1*cosd(phi1), L1*sind(phi1), 0]; % segment 1 position
P.L2 = [L2*cosd(phi2), L2*sind(phi2), 0]; % segment 2 position
P.L = P.L1 + P.L2; % length of attachment point from origin
P.x = P.L-P.r; % spring position matrix

%% Force Calculations
P.x_mag = norm(P.x); % magnitude of spring position
P.x_norm = P.x./P.x_mag; % normalized spring position values - unit vector
P.T_spring = -k*(P.x_mag-x0); % spring force calculation
P.T = P.T_spring .* P.x_norm; % force components

%% Torque Calculation
P.tau1 = cross(P.L,P.T); % joint 1 torque cross-product calculation
P.tau2 = cross(P.L2,P.T); % joint 2 torque cross-product calculation
tau1 = P.tau1(3); % joint 1 torque
tau2 = P.tau2(3); % joint 2 torque

end
