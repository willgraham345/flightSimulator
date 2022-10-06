%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load yconstants, determine simulation time, simulation dynamics
% Determine time of simulation here
t_simulation = 10;
formatSpec = "Simulation time of: %2.2f seconds \n";
model = 'crazyflie';
trajectoryType = 'linearX'; % hover or linearX (future work should include linearY and box and circle)
fprintf(formatSpec, t_simulation)
g = 9.81;
t = linspace(0,t_simulation, 100)'; % Do not delete, these are so simulink thinks each value has a time associated with it
n = length(t);


fprintf("Current model dynamics: %s\n", model)


% Crazyflie dymamics
Crazyflie.m = 0.0030; %[kg]
Crazyflie.J = diag([1.43e-5, 1.43e-5, 2.89e-5]); % %[kgm^2] Inertia matrix about {B} frame (body frame)
Crazyflie.d = 0.046; %[m] distance from center of mass to rotor
Crazyflie.k_M = 1.5e-9; %[Nm/rpm^2]
Crazyflie.k_f = 6.11e-8;     %[N/rpm^2]
Crazyflie.k_motor = 20; %[1/second]
Crazyflie.RotMatrix = "ZYX"; %Order of rotations
Crazyflie.drag = [0.1, 0.1, .5]; %[kg/s]
Crazyflie.X0 = [0, 0, 0];
Crazyflie.v0 = [0, 0, 0];
Crazyflie.RPY_0 = [0, 0, 0];
Crazyflie.pqr_0 = [0, 0, 0];


%% Determine which model to load
switch model
    case 'crazyflie'
    m = Crazyflie.m;
    J = Crazyflie.J;
    d = Crazyflie.d;
    J_inv = inv(J);
    cT = Crazyflie.k_f; % Thrust coefficient (Couples motor speed (omega^2) to thrust), determined from static thrust tests
    cQ = Crazyflie.k_M; % Moment coefficient [look @ Crazyflie for units
    Gamma = [cT, cT, cT, cT;
    0, d*cT, 0, -d*cT;
     -d*cT, 0 d*cT, 0;
     -cQ, cQ, -cQ, cQ;];
    GammaInv = inv(Gamma);
    dragConstants = [1, 1, 3]; %[N/ms] Assuming these here (dx, dy, dz)
    case 'example'
        J  = diag([0.082, 0.0845, 0.1377]);
        m = 4.34;
        d = 3.15;
        ct_f = 8.004e-4;
        kx = 16*m;
        kv = 5.6*m;
        kR = 8.81;
        komega = 2.54;

end



%% Angle Trajectories
tspan = [0:.1:t_simulation]';
angleTrajs.time = tspan; fn = @(t) pi/2*cos(0.1*t); fn2 = @(t)0*t;
angleTrajs.signals.values = [fn2(tspan),  fn2(tspan),  fn2(tspan)];


fprintf('Finished loading program parameters!\n')

%% running simulation and taking out params
fprintf('Running Simulation\n')
out = sim("flightSimulator.slx", t_simulation);
Xe = out.Xe;
RPY = out.RPY;
e_RPY = out.e_RPY;
u1_u2 = out.u1_u2;
% quatDirections = out.quatDirections;
fprintf('Simulation Finished :) \n')