%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load constants
g = 9.81; %m/s^2


%% Describing rotation matrix calculations w/example

%% Attitude Controller Block (Working)
Kp_attitude = 0.5;
Kd_attitude = 0.01;

t = [0:.01:10.0]'; % Do not delete, these are so simulink thinks each value has a time associated with it
n = length(t);

attitude.signals.values = [0.1.*ones(n,1), 0.*ones(n,1), 0.*ones(n,1)];
attitude.time = t;

attitude_des.signals.values = [0.*ones(n,1), 0.*ones(n,1), 0.*ones(n,1)];
attitude_des.time = t; 


omega.signals.values = [0.*ones(n,1), 0.*ones(n,1), 0.*ones(n,1)];
omega.time = t;
% u2_test = (R_des - R)*Kp_attitude - (omega*Kd_attitude)




%% Motor Controller Block
% Test properties of quadcopter paper (see resource literature)
% Taken from Geometric tracking of a Quadcopter paper
J = diag([0.082, 0.0845, 0.1377]);
m = 4.34;
d = 0.315;
c_taoF = 8.004*10^-4;
cT = c_taoF; % Thrust coefficient (Couples motor speed (omega^2) to thrust), determined from static thrust tests
cQ = cT*10^-3; % Reaction torque (due to rotor drag acting on airframe omega^2 to moment), also determined from static thrust tests
x_0 = [0, 0, 0];
v_0 = [0, 0, 0];
R_0 = eye(3);
omega_0 = [0, 0, 0];
k_x = 16*m;
k_v = 5.6*m;

k_motors = 1;
Gamma = ...
    [cT, cT, cT, cT;
     0, d*cT, 0 -d*cT;
     -d*cT, 0 d*cT, 0;
     -cQ, cQ, -cQ, cQ;];
GammaInv = inv(Gamma); % 

u1.signals.values = [5*m*g*ones(n,1)];
u1.time = t;
%% Free Body Simulator

fake_motors.signals.values = [m*g*1.001.*ones(n,1), 0.*t, 0.*t, 0.*t];
fake_motors.time = t;
XYZ_initial_condition = [0, 0, 0];

% phi = x axis
% psi = z axis
% theta = y axis
phi = 0; theta = pi/4; psi = 0;
gamma = [phi, theta, psi]; % [rad]
R_BmapToA = eul2rotm(gamma, "ZYX"); % ZYX angle rotation order, gamma HAS to be in radians
% v_I = R_BmapToA*[0, 0, 1]';



% % [R_BmapA, R_AmapB]= fcn(gamma)
% % NOTE: Gamma is a vector of roll, pitch, yaw angles in RADIANS
% % gamma = [phi, theta, psi] [rad]
% R_BmapA = eul2rotm(gamma, "ZYX"); % ZYX angle rotation order, gamma HAS to be in radians
% R_AmapB = inv(R_BmapA);


%% Trajectory Planning

wpts = [0, 1;
        0, 0;
        1, 1;];
tpts = [0, 10];
[q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, t);

eta_des.signals.values = q';
eta_des.time = t;
yaw_des.signals.values = 0.*ones(n,1);
yaw_des.time = t;

eta_fake.signals.values = q'.*0;
eta_fake.time = t;

% figure(1)
% plot3(q(1,:), q(2,:), q(3,:))
Kp_z_eta = 0.5;
Kd_z_eta = 0;  

Kp_eta = 0.5;
Kd_eta = 0;




fprintf('Finished loading program parameters!\n')