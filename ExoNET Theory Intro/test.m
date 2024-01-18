clc
clear all
close all

% spring
l0=0;
k=1;

% segment1

theta1=60;
r1=2;
R1=[r1*cosd(theta1) r1*sind(theta1) 0];%r1_pos(1)+

deg1=0:10:360;%0:5:90;
phi1=deg1;%-90+deg1;
l1=4;

% computing torques
TAU=ones(length(phi1));
for i=1:length(phi1)
    L=[l1*cosd(phi1(i)), l1*sind(phi1(i)) ,0];%l_pos(2)+
    T=R1-L;
    T_norm = norm(T);
    F_vers = T./T_norm;
    F_mod=k*(T_norm-l0);
    F=F_mod*F_vers;
    torque=cross(L,F);
    TAU(i) = torque(3);
end

figure(1); % create figure number
plot(phi1,TAU,'k','LineWidth', 3, 'DisplayName', 'Link 1');
title(['Single Joint ExoNET Torque Profile'],'FontSize',12); % figure title
axis auto
xlabel('Segment Angle (degrees)'); 
ylabel('Output Torque (Nm)');