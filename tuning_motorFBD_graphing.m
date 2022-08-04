%% Graphing function
close all;
out = sim('tuning_motorFBD',t_simulation); % most basic way to simulate with command script.
%%

XYZ = out.XYZ.Data;
attitude_actual = out.attitude.Data;
% ThrustTau_des = out.ThrustTau_des.Data;
ThrustTau = out.ThrustTau.Data;
acceleration = out.acceleration.Data;
%% zeta plot
fig1 = figure(1);
fig1.Position = [50 50 1200 900];
sgtitle('$\zeta$ and $\zeta_{des}$ vs Time', 'Interpreter','latex');
s1 = subplot(3,1,1);
% d1 = plot(t,zeta_des.signals.values(:,1), 'DisplayName', 'X des');
hold on
a1 = plot(out.tout, XYZ(:,1), 'DisplayName', 'X');
title("X");
legend();

s2 = subplot(3,1,2);
% d2 = plot(t,zeta_des.signals.values(:,2), 'DisplayName', 'Y des');
hold on
a2 = plot(out.tout, XYZ(:,2), 'DisplayName','Y');
legend();
title("Y");
s3 = subplot(3,1,3);
% d3 = plot(t,zeta_des.signals.values(:,3), 'DisplayName', 'Z des');
hold on
a3 = plot(out.tout, XYZ(:,3),'DisplayName', 'Z');
legend();
title("Z");



%% Attitude Plot

fig2 = figure(2);
fig2.Position = [1250 50 1200 900];
sgtitle('Attitude and $Attitude_{des}$ vs Time', 'Interpreter', 'latex');
s4 = subplot(3,1,1);
% d4 = plot(t,attitude_des.signals.values(:,1), 'DisplayName', 'X des');
hold on;
a4 = plot(out.tout, attitude_actual(:,1), 'DisplayName', 'Phi');
title("Roll (phi)");
legend();

s5 = subplot(3,1,2);
% d5 = plot(t,attitude_des.signals.values(:,2), 'DisplayName', 'Y des');
hold on
a5 = plot(out.tout, attitude_actual(:,2), 'DisplayName','Theta');
legend();
title("Pitch")
s6 = subplot(3,1,3);
% d6 = plot(t,attitude_des.signals.values(:,3), 'DisplayName', 'Z des');
hold on
a6 = plot(out.tout, attitude_actual(:,3),'DisplayName', 'Yaw');
legend();
title("Yaw");


%% Thrust and Tau Plot
fig3 = figure(3);
fig3.Position = [50 250 1200 900];

sgtitle('Thrust/$\tau$ and Thrust/$\tau_{des}$ vs Time', 'Interpreter', 'latex');
s7 = subplot(4,1,1);
a7 = plot(out.tout, ThrustTau(:,1), 'DisplayName', 'Thrust');
% hold on 
% d5 = plot(out.tout, ThrustTau_des(:,1), 'DisplayName', 'ThrustDes');

title("Thrust");
% legend();

s8 = subplot(4,1,2);
a8 = plot(out.tout, ThrustTau(:,2), 'DisplayName', 'actual');
% hold on
% d6 = plot(out.tout, ThrustTau_des(:,2), 'DisplayName', 'TauX');
title("\tau_x");
% legend();

s9 = subplot(4,1,3);
 
a9 = plot(out.tout, ThrustTau(:,3), 'DisplayName', 'actual');
% hold on
% d7 = plot(out.tout, ThrustTau_des(:,3), 'DisplayName', 'TauY');
title("\tau_y");
% legend();

s10 = subplot(4,1,4);


a10 = plot(out.tout, ThrustTau(:,4), 'DisplayName', 'actual');
% hold on 
% d8 = plot(out.tout, ThrustTau_des(:,4), 'DisplayName', 'TauZ');
title("\tau_z");
% legend();

%% Acceleration Plot
fig4 = figure(4);
fig4.Position = [1250 250 1200 900];
sgtitle('$\ddot{\zeta}$ vs Time', 'Interpreter', 'latex');
s11 = subplot(3,1,1);
a11 = plot(out.tout, acceleration(:,1), 'DisplayName', '$\ddot{\zeta}_x$');
title('X Acceleration');

s12 = subplot(3,1,2);
a12 = plot(out.tout, acceleration(:,2), 'DisplayName', '$\ddot{\zeta}_y$');
title("Y Acceleration")
s13 = subplot(3,1,3);
a13 = plot(out.tout, acceleration(:,3), 'DisplayName', '$\ddot{\zeta}a_z$');
title("Z Acceleration")

