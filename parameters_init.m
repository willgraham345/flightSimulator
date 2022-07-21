%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load constants
g = 9.81; %m/s^2

%% Load data for quadthruster (DO NOT USE ON SIMULATION UNTIL ITS RUNNING)

% % *Inertia Tensor Matrix for Ionocraft*
g = 9.8;        % [m/s^2], acceleration of gravity
m = 60e-6;      % [kg], body mass  Changed from 10e-6 9/5/2017    5-e-6 is with IMU + flexboard
lx = 1e-2;      % [m], x distance to body center of mass
ly = 1e-2;      % [m], y distance to body center of mass
lz = 20e-6;     % [m], z distance to body center of mass
rho = 1.01;     % [kg/m^3] 

% %% Quadcopter Test Inputs (These are used to test simulink setup)
% % *Test properties for quadcopter*
% % Taken from Geometric tracking of a Quadcopter paper
% J = diag([0.082, 0.0845, 0.1377]);
% m = 4.34;
% d = 0.315;
% c_taoF = 8.004*10^-4;
% cT = c_taoF; % Thrust coefficient (Couples motor speed (omega^2) to thrust), determined from static thrust tests
% cQ = cT*10^-3; % Reaction torque (due to rotor drag acting on airframe omega^2 to moment), also determined from static thrust tests
% x_0 = [0, 0, 0];
% v_0 = [0, 0, 0];
% R_0 = eye(3);
% omega_0 = [0, 0, 0];
% k_x = 16*m;
% k_v = 5.6*m;
% % *Positive definite gain matricies for Rotation and Angular Velocity Error
% % Gains*
% k_R = 8.81;
% k_omega = 2.54;
% k_motor = 1.0;
% Vff = .1;
% motorDistances = d*ones([1,4]);
% motorRotationDirection = [-1, 1, -1, 1]; % Direction +1 = CW, -1 = CCW
% motorLocationAngles = deg2rad([0, 90, 180, 270]); %explanation below
% motorForceVector = [0 0 0 0]';
% motorOmegasSquared = [0 0 0 0]';
% forceVector = [];
% % motor 1 on x axis, 2 on y axis, 3 on negative x, axis, 4 on negative y
% % axis (assumed to be a 90 degree angle, converted to radians for testing)
% 
% % Hard codes desired states to try
% state_des = [0; 0; 0.1 ; 0; .00; .00; 0; 0; 0; 0; 0; 0]; %x_eq
% % NOTE: The system will not converge with a nonzero roll or pitch desired
% % state. Have not tested angular or linear velocities. For now, desired
% % state sould be a vector of the form [X,Y,Z,YAW] with 0's appended as need
% 
% t = [0:.01:10];
% trajectory_XYZ.signals.values = timeseries([0*t;0*t; 1*log(1+t);]', t); % this will slowly rise to 1m
% trajectory_XYZ.time = t;
% trajectory_yaw.time = t;
% trajectory_yaw.signals.values = timeseries([0*t]', t);
% % *Quadcopter gamma matrix*
% % Used when going from motor speeds to forces
Gamma = ...
    [cT, cT, cT, cT;
     0, d*cT, 0 -d*cT;
     -d*cT, 0 d*cT, 0;
     -cQ, cQ, -cQ, cQ;];
% 
% GammaInv = inv(Gamma);
% 
% Kp_eta = 0;
% Kd_eta = 0;
% 
% 
% % input desired trajectories
% t = [1:.01:10];
% x_d = [.4*t, 0.4*sin(pi*t), 0.6*cos(pi*t)];
% b_1d = [cos(pi*t), sin(pi*t), 0];

%% Attitude Controller Variables 
% *(adjusted for Numerical example from Geometric Trakcing Control of a
% Quadcopter)*

% *Rotation Matrix from {A} to {B} (global to body)*
% theta = 10.0;

psi = 20.0;
phi = 70.0;
del_theta = 0;
del_phi = 0;

C_psi = cos(psi);
S_psi = sin(psi);
C_phi = cos(phi);
S_phi = sin(phi);
C_theta = cos(theta);
S_theta = sin(theta);

% Linearized dynamics about nominal hover position 
% Assumes roll and pitch are close to zero, and angular velocities are negligible
R_AtoB = ...
    [cos(psi)   -sin(psi) del_theta*cos(psi)+del_phi*sin(psi);
     sin(psi),   cos(psi) del_theta*sin(psi)-del_phi*cos(psi);
     -del_theta -del_phi 1;];

R_BtoA = R_AtoB';

angularVelocity = ...
    [0, 0, 0]';







%% Creation of waypoints
%{
waypoint creation help -
https://www.mathworks.com/help/fusion/ref/waypointtrajectory-system-object.html
%}



fprintf('Finished loading program parameters!\n')