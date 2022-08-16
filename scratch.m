%% Playing around with trajectory planning
close all; clear; clc; format compact;
A = [0, 1; 0,-1/5]; B = [0, 1]';
Q = [1, 0; 0, 1]; R = [0.01];


[K, S, e] = lqr(A, B, Q, R)