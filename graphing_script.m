%% Graphing function
close all; clc
out = sim('flightSimulator',t_simulation) % most basic way to simulate with command script.
XYZ = out.XYZ.Data;
attitude_actual = out.gamma.Data;


%% zeta plot
fig1 = figure(1);
sgtitle('$\zeta$ and $\zeta_{des}$ vs Time', 'Interpreter','latex');
s1 = subplot(3,1,1);
d1 = plot(t,zeta_des.signals.values(:,1), 'DisplayName', 'X des');
hold on
a1 = plot(out.tout, XYZ(:,1), 'DisplayName', 'X');
title("X");
legend();

s2 = subplot(3,1,2);
d2 = plot(t,zeta_des.signals.values(:,2), 'DisplayName', 'Y des');
hold on
a2 = plot(out.tout, XYZ(:,2), 'DisplayName','Y');
legend();
title("Y");
s3 = subplot(3,1,3);
d3 = plot(t,zeta_des.signals.values(:,3), 'DisplayName', 'Z des');
hold on
a3 = plot(out.tout, XYZ(:,3),'DisplayName', 'Z');
legend();
title("Z");
%% Attitude Plot

fig2 = figure(2);
sgtitle('Attitude and $Attitude_{des}$ vs Time', 'Interpreter', 'latex');
s4 = subplot(3,1,1);
% d4 = plot(t,attitude_des.signals.values(:,1), 'DisplayName', 'X des');
hold on;
a4 = plot(out.tout, attitude_actual(:,1), 'DisplayName', 'X');
title("Roll (phi)");
legend();

s5 = subplot(3,1,2);
% d5 = plot(t,attitude_des.signals.values(:,2), 'DisplayName', 'Y des');
hold on
a5 = plot(out.tout, attitude_actual(:,2), 'DisplayName','Y');
legend();
title("Pitch")
s6 = subplot(3,1,3);
% d6 = plot(t,attitude_des.signals.values(:,3), 'DisplayName', 'Z des');
hold on
a6 = plot(out.tout, attitude_actual(:,3),'DisplayName', 'Z');
legend();
title("Yaw");