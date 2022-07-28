%% Playing around with trajectory planning
close all; clear; clc; format compact;

%% Cubic Polynomial Trajectories
% Defines path between two points w/cubic polynomial

A = @(t0, tf) ...
    [1, t0, t0^2, t0^3, t0^4, t0^5;
    0, 1, 2*t0, 3*t0^2, 4*t0^4, 5*t0^5;
    0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
    1, tf, tf^2, tf^3, tf^4, tf^5;
    0, 1, 2*tf, 3*tf^2, 4*tf^4, 5*tf^5;
    0, 0, 2, 6*tf, 12*tf^2, 20*tf^3;]

b = A(0, 6)

vec = [3, 1, 0, 20, 2, 0]'
aVec = inv(b)*vec




t = [0:.1:6];
qVals = polyval(flip(aVec),t);


%% Matlab version
wpts = [3, 20];
tpts = [0, 6];
tvec = [0:.1:6];
[q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, tvec);

figure(1)
plot(tvec, q)
figure(2)
plot(tvec, qd)
figure(3)
plot(tvec, qdd)
figure(7)
plot(t, qVals)




