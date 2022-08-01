%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load global constants
g = 9.81; %m/s^2
m = 0.030; %[kg]
d = 0.092; %[m]
L = d/2;
t = [0:.01:10.0]'; % Do not delete, these are so simulink thinks each value has a time associated with it
n = length(t);
J = diag([1.43e-5, 1.43e-5, 2.89e-5]); % Inertia matrix about {B} frame (body frame)
%% Free Body Simulator

fake_motors.signals.values = [m*g*1.001.*ones(n,1), 0.*t, 0.*t, 0.*t];
fake_motors.time = t;
XYZ_initial_condition = [0, 0, 0];

%% Gains for attitude and position stuff 
Gain.attitude.Kp = [3000, 3000, 3000]; % Coming directly from project_report
Gain.attitude.Kd = [300, 300, 300];
Gain.position.Kp = [5, 5, 20];
Gain.position.Kd = [5, 5, 10];

%%

fprintf('Finished loading program parameters!\n')

K = [-5, 5, 0]

u2 = J*(Gain.attitude.Kp.*K)'