%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load constants, determine simulation time, simulation dynamics
% Determine time of simulation here
t_simulation = .5;
model = 'crazyflie'; % crazyflie
trajectoryType = 'hover'; % hover or linearX (future work should include linearY and box and circle)
formatSpec = "Simulation time of: %2.2f seconds \n";


%% Simulation Constants
fprintf(formatSpec, t_simulation)
g = 9.81;
t = [0:.001:t_simulation]'; % Do not delete, these are so simulink thinks each value has a time associated with it
n = length(t);


fprintf("Current dynamics of: %s\n", model)

% Crazyflie dymamics
Crazyflie.m = 0.0030; %[kg]
Crazyflie.J = diag([1.43e-5, 1.43e-5, 2.89e-5]); % %[kgm^2] Inertia matrix about {B} frame (body frame)
Crazyflie.d = 0.046; %[m] distance from center of mass to rotor
Crazyflie.k_M = 1.5e-9; %[Nm/rpm^2]
Crazyflie.k_f = 6.11e-8;     %[N/rpm^2]
Crazyflie.k_motor = 20; %[1/second]
Crazyflie.RotMatrix = "ZXY"; %Order of rotations
Crazyflie.drag = 0.1; %[kg/s]


%% Determine which model to load
switch model
    case 'crazyflie'
    m = Crazyflie.m;
    J = Crazyflie.J;
    d = Crazyflie.d;
    J_inv = inv(J);
    cT = Crazyflie.k_f; % Thrust coefficient (Couples motor speed (omega^2) to thrust), determined from static thrust tests
    cQ = Crazyflie.k_M; % Moment coefficient [look @ Crazyflie for units

end

Gamma = [cT, cT, cT, cT;
    0, d*cT, 0, -d*cT;
     -d*cT, 0 d*cT, 0;
     -cQ, cQ, -cQ, cQ;];
GammaInv = inv(Gamma);


%% Attitude stuff
theta = 0;
phi = 0;
psi = 0;

R = angle2dcm(phi, theta, psi, "XYZ");

phi_des = 0.6;
theta_des= .60;

psi_des = 0.7;
R_des = angle2dcm(phi_des, theta_des, psi_des, "XYZ");

% e_r = .5.*((R_des')*R - R'*R_des)



%% State Space Matrix Definitions



% A_attitude = cat(2, zeros(6,3), cat(1, eye(3), zeros(3,3)));
% B_attitude = cat(1, zeros(3,4), Gamma(2:4,:))
% C_attitude = cat(2, eye(3), zeros(3,3));
% D_attitude = zeros(3,4);
% x0_attitude = zeros(6,1);
% 
% 
% A_plant = cat(2, zeros(12,3), cat(1, eye(3), zeros(9,3)), zeros(12,3), cat(1, zeros(6,3), eye(3), zeros(3,3)) );
% A_plant(4,8) = -g; A_plant(5,7) = g
% J_forB = cat(2, zeros(3,1), J)
% B_plant = cat(1, zeros(4,4), [1/m, 0, 0, 0], zeros(4,4), J_forB)
% C_plant = cat(2, cat(1, eye(3), zeros(3,3)), zeros(6,3), cat(1,zeros(3,3), eye(3)), zeros(6,3))
% D_plant = zeros(6,4)
% x0 = [0, 0, 0, .1, 0, 0];
% 
% A = cat(2, zeros(6,3), cat(1,eye(3), zeros(3,3)))
% B = cat(1, zeros(3,4), Gamma(2:end,:))
% C = cat(2, eye(3), zeros(3,3))
% D = zeros(3,4)
% 
% M = ctrb(A_attitude, B_attitude); rank(M)
% N = obsv(A_attitude, C_attitude); rank(N)
% 
% Q = diag([10, 10, 10, .001, 0.001, 0.001])
% R = diag([.1, .1, .1, .1])
% 
% [K, S, e] = lqr(A_attitude, B_attitude, Q, R)
% 
% 
% 
% sys = ss(A_attitude - B_attitude*K, B_attitude, C_attitude, D_attitude)
% step(sys);
% 
% 
% 
% 
% u_matrix = [1, 1, 1.1, 1];


%% Gains section
Gain.attitude.Kp = [300, 300, 3000]; %[3000, 3000, 3000]; % Coming directly from project_report
Gain.attitude.Kd = [300, 300, 300];%[0.013, 0.013, 0.03]; %[300, 300, 300];
Gain.attitude.Kd = [0, 0, 0];
Gain.attitude.LQR = [1, 1, 1];
Gain.position.Kp = [5, 5, 20];
Gain.position.Kd = [5, 5, 10];
Gain.motors.Kp = 1/cT;

Gain.saturation.attitude.Thrust = [1,-1];
Gain.saturation.attitude.tauX = [1,-1];
Gain.saturation.attitude.tauY = [1,-1];
Gain.saturation.attitude.tauZ = [1,-1];
%% Drag Section
dx = .1; % val taken from Usman Muhammad model
dy = .1; % val taken from Usman Muhammad model
dragConstants = [dx, dy, 0]; % Random values, just threw these in
projectionMatrix = [1, 0, 0; 0, 1, 0];
% 
% 

%% Trajectory Control Section
fprintf("Trajectory is: %s\n", trajectoryType)
switch trajectoryType
    case 'hover'
        wpts = [0, 0; 0, 0; 0, 0;];
        tpts = [0; 10];
        [q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, t);
        zeta_des.signals.values = q';
        zeta_des.time = t;
        
        % still needs yaw control inputs...
        yaw_des.signals.values = zeros(n,1);
        yaw_des.time = t;
    case 'linearX'
        wpts = [0, 1; 0, 0; 0, 0;];
        tpts = [0; 5];
        [q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, t);
        zeta_des.signals.values = q';
        zeta_des.time = t;
        
        % still needs yaw control inputs...
        yaw_des.signals.values = zeros(n,1);
        yaw_des.time = t;
    case 'diagonalXY'
        wpts = [0, 1; 0, 1; 0, 0;];
        tpts = [0; 5];
        [q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, t);
        zeta_des.signals.values = q';
        zeta_des.time = t;
        
        % still needs yaw control inputs...
        yaw_des.signals.values = zeros(n,1);
        yaw_des.time = t;

end


%%
fprintf('Finished loading program parameters!\n')