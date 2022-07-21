%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load constants
g = 9.81; %m/s^2


%% Attitude Controller Confirm Working
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

fprintf('Finished loading program parameters!\n')