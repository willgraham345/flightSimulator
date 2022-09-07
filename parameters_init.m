%% 
close all; clear all; clc; format compact;

fprintf('Started loading program parameters...\n')
%% Load constants, determine simulation time, simulation dynamics
% Determine time of simulation here
t_simulation = 5;
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
Crazyflie.RotMatrix = "ZXY"; %Order of rotations
Crazyflie.drag = [0.1, 0.1, .5]; %[kg/s]

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

end



%% State Space Definitions


%% Gains section
Gain.attitude.Kp = 1.*ones(1,3); %[3000, 3000, 3000]; % Coming directly from project_report
Gain.attitude.Kd = 300.*ones(1,3);%[0.013, 0.013, 0.03]; %[300, 300, 300];
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





%% Trajectory Control Section
fprintf("Trajectory is: %s\n", trajectoryType)
switch trajectoryType
    case 'hover'
        wpts = [0, 0; 0, 0; 0, 0;];
        tpts = [0, 10; 0, 10; 0, 10];
        [q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, t);
        zeta_des.signals.values = q';
        zeta_des.time = t;
        
        % still needs yaw control inputs...
        yaw_des.signals.values = zeros(n,1);
        yaw_des.time = t;
    case 'linearX'
        wpts = [0, 0, 1; 0, 0, 0; 0, 1, 1;];
        tpts = [0, 3, 5];
        [q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, t);
        traj.signals.values = q;
        traj.time = cat(1, linspace(0, tpts(end), size(qd,2)), linspace(0, tpts(end), size(qd,2)), linspace(0, tpts(end), size(qd,2)));
%         traj_ddot.signals.values = qdd;
%         traj_ddot.time = linspace(0, tpts(end), size(qd,2));
%         zeta_des.signals.values = q';
%         zeta_des.time = t;
        
        % still needs yaw control inputs...
        yaw_des.signals.values = zeros(n,1);
        yaw_des.time = t;
    case 'diagonalXY'
        wpts = [0, 1; 0, 1; 0, 0;];
        tpts = [0, 5];
        [q,qd,qdd,pp] = quinticpolytraj(wpts,tpts, t);
        
        zeta_des.signals.values = q';
        zeta_des.time = t;
        
        % still needs yaw control inputs...
        yaw_des.signals.values = zeros(n,1);
        yaw_des.time = t;

end

%% RPY from calculations stuff
X_des.signals.values = q(1,:)'; X_des.time = t;
X_ddot_des.signals.values = qdd(1,:)'; X_ddot_des.time = t;
Y_des.signals.values = q(2,:)'; Y_des.time = t;
Y_ddot_des.signals.values = qdd(2,:)'; Y_ddot_des.time = t;

Z_des.signals.values = q(3,:)'; Z_des.time = t;
Z_ddot_des.signals.values = qdd(1,:)'; Z_ddot_des.time = t;
n = size(q,2);
for i = 1:n
    phi(i) = 1 / g * ( X_ddot_des.signals.values(i) * sin(yaw_des.signals.values(i)) - Y_ddot_des.signals.values(i)*cos(yaw_des.signals.values(i)));
    theta(i) = 1 / g *( X_ddot_des.signals.values(i) * cos(yaw_des.signals.values(i)) + Y_ddot_des.signals.values(i)*sin(yaw_des.signals.values(i)));
end
phi_des.signals.values = phi'; phi_des.time = t;
theta_des.signals.values = theta'; theta_des.time = t;
% attitude_des = [phi_des.signals.values, theta_des.signals.values, yaw_des.signals.values];

%%

fprintf('Finished loading program parameters!\n')