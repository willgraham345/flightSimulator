%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load constants, determine simulation time, simulation dynamics
% Determine time of simulation here
t_simulation = 1;
formatSpec = "Simulation time of: %2.2f seconds \n";
trajectoryType = 'hover'; % hover or linearX (future work should include linearY and box and circle)
fprintf(formatSpec, t_simulation)
g = 0; %% CHANGE THIS
t = linspace(0,t_simulation, 1000)'; % Do not delete, these are so simulink thinks each value has a time associated with it
n = length(t);


model = 'crazyflie'
fprintf("Current dynamics of: %s\n", model)


% Crazyflie dymamics
Crazyflie.m = 0.0030; %[kg]
Crazyflie.J = diag([1.43e-5, 1.43e-5, 2.89e-5]); % %[kgm^2] Inertia matrix about {B} frame (body frame)
Crazyflie.d = 0.046; %[m] distance from center of mass to rotor
Crazyflie.k_M = 1.5e-9; %[Nm/rpm^2]
Crazyflie.k_f = 6.11e-8; %[N/rpm^2]
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

Gamma = ...
    [cT, cT, cT, cT;
     0, d*cT, 0 -d*cT;
     -d*cT, 0 d*cT, 0;
     -cQ, cQ, -cQ, cQ;];
GammaInv = inv(Gamma);
%% Gains section
Gain.attitude.Kp = [.06, .06, .01]; %[3000, 3000, 3000]; % Coming directly from project_report
Gain.attitude.Kd = [.013, .013, .03];%[0.013, 0.013, 0.03]; %[300, 300, 300];
Gain.position.Kp = [5, 5, 20];
Gain.position.Kd = [5, 5, 10];
Gain.motors.Kp = 1/cT;

%% Drag Section
dx = .1; % val taken from Usman Muhammad model
dy = .1; % val taken from Usman Muhammad model
dragConstants = [dx, dy, 0]; % Random values, just threw these in
projectionMatrix = [1, 0, 0; 0, 1, 0]; % 
% flapMatrix = [A1c, -A1s, 0; A1s, A1c, 0; 0, 0, 0];





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