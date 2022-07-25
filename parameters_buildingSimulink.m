%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load constants
g = 9.81; %m/s^2


%% Attitude Controller Block (Working)
Kp_attitude = 0.5;
Kd_attitude = 0.01;

t = [0:.01:10.0]'; % Do not delete, these are so simulink thinks each value has a time associated with it
n = length(t);

R.signals.values = [ones(n,1), ones(n,1), ones(n,1)];
R.time = t;

R_des.signals.values = [3.*ones(n,1), 5.*ones(n,1), 10.*ones(n,1)];
R_des.time = t;


omega.signals.values = [.1.*ones(n,1), .3.*ones(n,1), 1.*ones(n,1)];
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

u1.signals.values = [m*g*ones(n,1)];
u1.time = t;
%%
fprintf('Finished loading program parameters!\n')