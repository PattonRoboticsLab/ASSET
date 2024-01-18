%% ExoNET Program to Model a Bungee Cord Attached Between Wrist and Forearm for Supination

close all;
clear all;
clc;


%% Initialize Input Variables  

% Bungee cord parameters
momentOfInertia = 0.1; % moment of inertia of the forearm (in kg*m^2)
springConstant = 5; % torsional spring constant (in N*m/rad)
dampingCoefficient = 0.2; % damping coefficient (in N*m*s/rad)
initialAngle = 0; % initial angle of the forearm (in rad)

% Simulation parameters
timeStep = 0.01; % time step for simulation (in s)
totalTime = 5; % total simulation time (in s)

% Angle range
minAngle = 0; % Minimum angle (rad)
maxAngle = pi/2; % Maximum angle (rad)

% Initialize variables
time = 0:timeStep:totalTime;
numSteps = numel(time);
angle = linspace(minAngle, maxAngle, numSteps);
angularVelocity = zeros(1, numSteps);
angularAcceleration = zeros(1, numSteps);
restoringTorque = zeros(1, numSteps);

% Set initial conditions
angularVelocity(1) = 0; % Starting with zero angular velocity
angularAcceleration(1) = 0; % Starting with zero angular acceleration


%% Supination Motion of the Forearm

for i = 2:numSteps
    
    % Calculate the torque
    torqueSpring = springConstant * angle(i-1);
    torqueDamping = dampingCoefficient * angularVelocity(i-1);
    restoringTorque(i) = -torqueSpring - torqueDamping;
    
    % Calculate angular acceleration using Newton's second law for rotation
    angularAcceleration(i) = restoringTorque(i) / momentOfInertia;
    
    % Update angular velocity and angle using Euler's method
    angularVelocity(i) = angularVelocity(i-1) + angularAcceleration(i) * timeStep;
    
end

%% Plotting

% Angle vs. time
subplot(2, 2, 1);
plot(time, angle);
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Angle vs. Time');
grid on;

% Angular acceleration vs. time
subplot(2, 2, 2);
plot(time, angularAcceleration);
xlabel('Time (s)');
ylabel('Angular Acceleration (rad/s^2)');
title('Angular Acceleration vs. Time');
grid on;

% Angular velocity vs. time
subplot(2, 2, 3);
plot(time, angularVelocity);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity vs. Time');
grid on;

% Restoring torque vs. angle
subplot(2, 2, 4);
plot(angle, restoringTorque);
xlabel('Angle (rad)');
ylabel('Restoring Torque (N*m)');
title('Restoring Torque vs. Angle');
grid on;
