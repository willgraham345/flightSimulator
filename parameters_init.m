%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load constants, determine simulation time, simulation dynamics
% Determine time of simulation here
t_simulation = 3;
formatSpec = "Simulation time of: %2.2f seconds \n";
fprintf(formatSpec, t_simulation)
g = 9.81;
t = [0:.01:t_simulation]'; % Do not delete, these are so simulink thinks each value has a time associated with it
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
Gain.attitude.Kp = [3000, 3000, 3000]; % Coming directly from project_report
Gain.attitude.Kd = [300, 300, 300];
Gain.position.Kp = [5, 5, 20];
Gain.position.Kd = [5, 5, 10];
Gain.motors.Kp = 1/cT;

%% Free Body Simulator Motor stuff (probably can delete later)
syms x
z_piecewise = piecewise(x < 5, m*g*1.1, x>= 5, m*g*.9);
zVals = double(subs(z_piecewise, x, t));
fake_motors.signals.values = [zVals, 0.*t, 0.*t, 0.*t];
fake_motors.time = t;
XYZ_initial_condition = [0, 0, 0];



%% Trajectory Control Section

wpts = [0, 0; 0, 0; 0, 0;];
tpts = [0; 10];
[q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, t);
zeta_des.signals.values = q';
zeta_des.time = t;
% still needs yaw control inputs...
yaw_des.signals.values = zeros(n,1);
yaw_des.time = t;


%%

fprintf('Finished loading program parameters!\n')