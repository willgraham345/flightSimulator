%% Playing around with trajectory planning
close all; clear; clc; format compact;

%% Make rot matrix
phi = 0
theta = 0
psi = deg2rad(90)
R = eul2rotm([phi, theta, psi], "ZYX")
R_inv = inv(R)


vX_b = 0,; vY_b = 0; vZ_b = 1;

v_B = [vX_b, vY_b, vZ_b]';


v_A = R_inv*v_B