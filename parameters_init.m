%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load global constants
g = 9.81; %m/s^2
m = 4.34;
d = 0.315;
c_taoF = 8.004*10^-4;
L = d/2;
t = [0:.01:10.0]'; % Do not delete, these are so simulink thinks each value has a time associated with it
n = length(t);
cT = c_taoF; % Thrust coefficient (Couples motor speed (omega^2) to thrust), determined from static thrust tests
cQ = cT*10^-3; % Reaction torque (due to rotor drag acting on airframe omega^2 to moment), also determined from static thrust tests
J = diag([0.082, 0.0845, 0.1377]); %[kgm^2] Inertia matrix about {B} frame (body frame)
J_inv = inv(J);

Gamma = ...
    [cT, cT, cT, cT;
     0, d*cT, 0 -d*cT;
     -d*cT, 0 d*cT, 0;
     -cQ, cQ, -cQ, cQ;];
GammaInv = inv(Gamma);
%% Free Body Simulator
syms x
z_piecewise = piecewise(x < 5, m*g*1.1, x>= 5, m*g*.9);
zVals = double(subs(z_piecewise, x, t));
fake_motors.signals.values = [zVals, 0.*t, 0.*t, 0.*t];
fake_motors.time = t;
XYZ_initial_condition = [0, 0, 0];

%% Gains for attitude and position stuff 
Gain.attitude.Kp = [3000, 3000, 3000]; % Coming directly from project_report
Gain.attitude.Kd = [300, 300, 300];
Gain.position.Kp = [5, 5, 20];
Gain.position.Kd = [5, 5, 10];
Gain.motors.Kp = 1;



%%

fprintf('Finished loading program parameters!\n')

tau = [5, 5, 5]';
omega = [3, 3, 3]';
omega_dot = J_inv*tau - J_inv*(cross(omega,J*omega));