%% Playing around with trajectory planning
close all; clear; clc; format compact;

%% Make rot matrix
% eul = [phi, theta, psi]
% stuff = eul2quat(eul, "ZYX")

%%
% R = eul2rotm([psi, theta, psi], 'ZYX');
R = angle2dcm(0, 0, pi/2, "XYZ")
R_inv = inv(R)


vX_b = 0;
vY_b = 0; 
vZ_b = 1;

v_B = [vX_b, vY_b, vZ_b]'


v_A = R_inv*v_B