%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load global constants
g = 9.81; %m/s^2
m = 0.9928; %[kg]
d = 0.092; %[m]
L = d/2;
t = [0:.01:10.0]'; % Do not delete, these are so simulink thinks each value has a time associated with it
n = length(t);
J = diag([0.00963, 0.00963, .019]); %[kgm^2] Inertia matrix about {B} frame (body frame)
thrustCoeff = 3.599e-5; %[kgm] thrust coefficient (thrust / motor rotation speed^2)
dragTorqueCoeff = 2.081e-6; %[kgm^2/s^2] drag torque coefficient
d = .1; %[kg/s] Air resistance

%% Free Body Simulator

fake_motors.signals.values = [m*g*1.001.*ones(n,1), 0.*t, 0.*t, 0.*t];
fake_motors.time = t;
XYZ_initial_condition = [0, 0, 0];


Gamma = ...
    [thrustCoeff, thrustCoeff, thrustCoeff, thrustCoeff;
     0, d*thrustCoeff, 0 -d*thrustCoeff;
     -d*thrustCoeff, 0 d*thrustCoeff, 0;
     -cQ, cQ, -cQ, cQ;];
%% Gains for attitude and position stuff 
Gain.attitude.Kp = [3000, 3000, 3000]; % Coming directly from project_report
Gain.attitude.Kd = [300, 300, 300];
Gain.position.Kp = [5, 5, 20];
Gain.position.Kd = [5, 5, 10];

%%

fprintf('Finished loading program parameters!\n')

% K = [-5, 5, 0]

% u2 = J*(Gain.attitude.Kp.*K)'