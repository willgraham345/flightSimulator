% A = cat(2, zeros(3,3), ones(3))
% A = cat(1, A, zeros(3,6))
% 
% 
% % B = cat(1, zeros(3), diag([6.993e4, 6.993e4, 3.46e4]))
% % B = cat(2, zeros(6,3), B)
% B = [0, 0, 0, 6.993e4, 6.993e4, 3.46e4]'
% 
% C = cat(2, diag([1, 1, 1]), zeros(3))
% D = zeros(3,6)';
% 
% M = ctrb(A, B);
% rank(M)
% obsv(A, C)
% % N = obsv(A, B);
% rank(N)
% % sys = ss(A, B, C, D)
% [T, D] = eig(A, diag(B));

%%
% clear
% clc;
% close all;
% format compact;
% g = 9.81;
% m = 0.003;
% J = diag([0.005, 0.005, 0.001]);
% J = cat(2, [0 0 0]', J);
% A1 = cat(2,zeros(3,3), eye(3), zeros(3,6));
% A2 = cat(2,zeros(3,6), [0 -g 0; g 0 0; 0 0 0], zeros(3,3));
% A3 = cat(2, zeros(3,9), eye(3));
% A4 = zeros(3,12);
% A = cat(1, A1, A2, A3, A4);
% B = cat(1, zeros(4,4), [1/m 0 0 0], zeros(4,4), J);
% C1 = cat(2, eye(3), zeros(3,9));
% C2 = cat(2, zeros(3,6), eye(3), zeros(3));
% C = cat(1, C1, C2);
% D = zeros(6,4);
% sys = ss(A, B, C, D);
% [T, D_eig] = eig(A);
% state_penalties = ones([1,12]);
% input_penalties = [1, 1, 1, 1];
% 
% Q = diag([state_penalties]);
% R =  diag([1, 1, 1, 1]);
% M = ctrb(A, B);
% N = obsv(A, C);
% rank(M)
% rank(N)
% N = zeros(12, 4)
% 
% X = [Q N; N' R]
% [Abar,Bbar,Cbar,T,k] = ctrbf(sys.A,sys.B,sys.C)
% % A = [0, 1, 0; 0 0 1; -04. -4.2 -2.1]
% % B = [0 0 1]'
% % 
% % C = ones(3);
% % D = [0, 0, 0]'
% % sys = ss(A, B, C, D);
% % 
% % Q = eye(3);
% % R = 0.1;

% X = [Q ]
% [K,S,CLP] = lqr(SYS,Q,R, N)


%%
clear; clc; format compact
Ap  = [0 1 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0;];
Bp = [0 0 0;
    1 0 0;
    0 0 0
    0 1 0;
    0 0 0;
    0 0 1;];
Qp = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;];
Rp = diag([1, 1, 1]);
[Kp Sp CLPp] = lqr(Ap, Bp, Qp, Rp);


Aa  = [0 1 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0;];
Ba = [0 0 0 0;
    1 0 -1 0 
    0 0 0 0;
    0 1 0 -1;
    0 0 0 0;
    .5 -.5 .5 -.5];

Qa = diag([12, 1, 12, 1, 10, 0.5]);
Ra = diag([.1 .1 .1 .1]);

[Ka,S_a,CLP_a] = lqr(Aa, Ba, Qa, Ra);

t_sampling = 1 / 250;
g = -9.81;
m = 0.5;
b = 1;

