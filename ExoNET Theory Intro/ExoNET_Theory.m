%% ExoNET Program to Compare Element Attachment Angle vs. Output Torque 

close all;
clear all;
clc;

%% Initialize Input Variables (pre-defined values) 
% % use either this section or next section but not both together

global P
L1 = 4; % segment 1 length : m
L2 = 4; % segment 2 attachment point length : m
r = 2; % initial element attachment length from ground : m
theta = 60; % ground angle : degrees
phi1 = 0:10:360; % segment 1 angle range : degrees
phi2 = 0:10:360; % segment 2 angle range : degrees
k = 1; % spring constant
x0 = 0; % resting length of elastic element : m 

%% Single Joint Torque Caclulations

TAU = ones(1,length(phi1));
for i = 1:length(TAU)
    TAU(i) = single_joint(L1,r,theta,phi1(i),k,x0);    
end

%% Single Joint Plotting 

figure(1); % create figure number
plot(phi1,TAU,'k','LineWidth', 3, 'DisplayName', 'Link 1');
title('Single Joint ExoNET Torque Profile','FontSize',12); % figure title
axis auto
xlabel('Segment Angle (degrees)'); 
ylabel('Output Torque (Nm)');
saveas(figure(1),fullfile([pwd '\MATLAB Figures'],'Single_Joint_ExoNET.png')); % save to computer

%% Double Joint Torque Calculations

phi = [phi1;phi2];
TAU = ones(2,length(phi));
for i = 1:length(TAU)
    [tau1,tau2] = double_joint(L1,L2,r,theta,phi1(i),phi2(i),k,x0);
    TAU(1,i) = tau1;
    TAU(2,i) = tau2;
end

%% Double Joint Plotting

figure(2); % create figure number
plot(phi1,TAU(1,:),'k','LineWidth', 3, 'DisplayName', 'Joint 1');
hold on 
plot(phi2,TAU(2,:),'r','LineWidth', 3, 'DisplayName', 'Joint 2');
hold off
title(['Double Joint ExoNET Torque Profile'],'FontSize',12); % figure title
axis auto
legend
xlabel('Segment Angle (degrees)'); 
ylabel('Output Torque (Nm)');
saveas(figure(2),fullfile([pwd '\MATLAB Figures'],'Double_Joint_ExoNET.png')); % save to computer
