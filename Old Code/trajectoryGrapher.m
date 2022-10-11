%%
% Run paramaeters_init before this. This is only for creating the trajectory grapher

close all; clc
% out = sim('trajectorySimulator.slx',t_simulation); % most basic way to simulate with command script.

%% Calculations to get the Roll/pitch values vs time
X = q(1,:);
Y = q(2,:);
Z = q(3,:);

n = size(q,2);
phi_des = [];
for i = 1:n
    i;
    phi_des(i) = 1 / g * ( X(i) * sin(yaw_des.signals.values(i)) - Y(i)*cos(yaw_des.signals.values(i)));
    theta_des(i) = 1 / g *( X(i) * cos(yaw_des.signals.values(i)) + Y(i)*sin(yaw_des.signals.values(i)));
end
%% XYZ plot
figure(1)
sgtitle('$\zeta_{des}$', 'Interpreter','latex');

s1 = plot3(q(1,:), q(2,:), q(3,:));
grid on;

%% Angles plot
figure(2)
sgtitle('Yaw vs Time')
a2 = subplot(3,1,1);
a2 = plot(yaw_des.time, yaw_des.signals.values);

b2 = subplot(3,1,2);
b2 = plot(yaw_des.time, phi_des);

c2 = subplot(3,1,3);
c2 = plot(yaw_des.time, theta_des);
