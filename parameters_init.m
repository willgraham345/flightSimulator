%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load global constants
g = 9.81; %m/s^2

%% Attitude Controller 
Kp_attitude = 0.5; % Based on PID Control theory
Kd_attitude = 0.01;

fprintf('Finished loading program parameters!\n')