%% Graphing function
close all;
out = sim('flightSimulator2.slx',t_simulation); % most basic way to simulate with command script.
XYZ = out.XYZ.Data;
attitude_actual = out.attitude.Data;
attitude_des = out.attitude_des.Data;
omega_actual = out.omegas.Data;
acceleration_actual = out.acceleration.Data;
ThrustTau = out.ThrustTau.Data;
% u1 = out.u1.Data;
u2 = out.u2.Data; 
e_R = out.error_R.Data;
% velocity_actual = out.velocities_Data;


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
% 
fig2 = figure(2);
sgtitle('Attitude and $Attitude_{des}$ vs Time', 'Interpreter', 'latex');
s4 = subplot(3,1,1);
d4 = plot(out.tout,attitude_des(:,1), 'DisplayName', 'phi des');
hold on;
a4 = plot(out.tout, attitude_actual(:,1), 'DisplayName', 'X');
title("\phi (Roll)");
legend();

s5 = subplot(3,1,2);
d5 = plot(out.tout,attitude_des(:,2), 'DisplayName', 'Y des');
hold on
a5 = plot(out.tout, attitude_actual(:,2), 'DisplayName','Y');
legend();
title("\theta (Pitch)")
s6 = subplot(3,1,3);
d6 = plot(out.tout,attitude_des(:,3), 'DisplayName', 'Z des');
hold on
a6 = plot(out.tout, attitude_actual(:,3),'DisplayName', 'Z');
legend();
title("\psi (Yaw)");

%% Thrust and Tau Plot
fig3 = figure(3);
% fig3.Position = [50 250 1200 900];

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
% fig4 = figure(4);
% % fig4.Position = [1250 250 1200 900];
% sgtitle('$\ddot{\zeta}$ vs Time', 'Interpreter', 'latex');
% s11 = subplot(3,1,1);
% a11 = plot(out.tout, acceleration_actual(:,1), 'DisplayName', '$\ddot{\zeta}_x$');
% title('X Acceleration');
% 
% s12 = subplot(3,1,2);
% a12 = plot(out.tout, acceleration_actual(:,2), 'DisplayName', '$\ddot{\zeta}_y$');
% title("Y Acceleration")
% s13 = subplot(3,1,3);
% a13 = plot(out.tout, acceleration_actual(:,3), 'DisplayName', '$\ddot{\zeta}a_z$');
% title("Z Acceleration")

%% u2 plot
fig5 = figure(5);
sgtitle('$u_2$ vs Time', 'Interpreter', 'latex');
s14 = subplot(3,1,1);
a14 = plot(out.tout, u2(:,1));
title('\tau_x^{des}');
s15 = subplot(3,1,2);
a15 = plot(out.tout, u2(:,2));
title('\tau_y^{des}');
s16 = subplot(3,1,3);
a16 = plot(out.tout, u2(:,3));
title('\tau_z^{des}');

%% error plot
fig6 = figure(6);
sgtitle('$e_R$ vs Time', 'Interpreter', 'latex');
s17 = subplot(3,1,1);
a17 = plot(out.tout, e_R(:,1));
title('e_{\phi}');
s18 = subplot(3,1,2);
a18 = plot(out.tout, e_R(:,2));
title('e_{\theta}');
s19 = subplot(3,1,3);
a19 = plot(out.tout, e_R(:,3));
title('e_{\psi}');
%% u1 plot
% fig7 = figure(7);
% s20 = plot(out.tout, u1(:,1));
% title('$u_1$     vs Time', 'Interpreter', 'latex')