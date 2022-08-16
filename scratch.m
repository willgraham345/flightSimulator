%% Playing around with trajectory planning
close all; clear; clc; format compact;
m = 1;
c = 0.2;
A = [0, 1; 0,-c/m]; B = [0, 1/m]';


Q = diag([1, 1]);
R = [0.01];

C = [1, .1]; D = [0];

[K, S, e] = lqr(A, B, Q, R);
x0 = [pi; -2];