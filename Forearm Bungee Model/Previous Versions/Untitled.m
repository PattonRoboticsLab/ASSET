clear all;
clc;

% function visualize_cylinder_rotation_3D_with_helical_bungee()
    % Parameters
    R = 1; % Radius of the cylinder
    H = 2; % Height of the cylinder
    L = 2; % Length of the rod

    % Create a figure and set up 3D view
    figure('Name', '3D Visualization of Cylinder, Rod, and Helical Bungee', 'NumberTitle', 'off');
    axis equal;
    xlim([-R-L, R+L]);
    ylim([-R-L, R+L]);
    zlim([-H, H]);
    hold on;
    view(3); % 3D view

    % Draw cylinder
    [X, Y, Z] = cylinder(R);
    Z = Z * H - H/2; % Center the cylinder at z = 0
    surf(X, Y, Z, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

    % Initialize plot handles for rod and bungee
    hRod = plot3([0, 0], [0, 0], [H/2, H/2], '-r', 'LineWidth', 2);
    hBungee = plot3([0, 0], [0, 0], [-H/2, -H/2], '-g', 'LineWidth', 2);

    title('3D Visualization of Cylinder with Attached Rod and Helical Bungee Cord');

    % Rotate the rod and bungee cord to visualize
    for theta = 0:0.01:2*pi
        % Update rod position
        rod_x = [0, L*cos(theta)];
        rod_y = [0, L*sin(theta)];
        set(hRod, 'XData', rod_x, 'YData', rod_y);

        % Calculate bungee cord helical path
        [bungee_x, bungee_y, bungee_z] = compute_helical_bungee_path(theta, R, H, L);
        set(hBungee, 'XData', bungee_x, 'YData', bungee_y, 'ZData', bungee_z);

        drawnow;
        pause(0.01);
    end

    hold off;
% end

function [x, y, z] = compute_helical_bungee_path(theta, R, H, L)
    % Define helix parameters
    numTurns = theta / (2*pi); % Number of turns based on theta
    helixHeight = numTurns * H; % Total height of the helix
    
    % Generate helical path for the bungee
    t = linspace(0, theta, 100);
    x_helix = R * cos(t);
    y_helix = R * sin(t);
    z_helix = linspace(-H/2, -H/2 + helixHeight, 100);
    
    % Straight segment from end of helix to rod's tip
    x_end = L * cos(theta);
    y_end = L * sin(theta);
    z_end = z_helix(end);
    
    % Combine the helical and straight segments
    x = [x_helix, x_end];
    y = [y_helix, y_end];
    z = [z_helix, z_end];
end
