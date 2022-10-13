%% 
clear;
clc;
close all;
format compact;

fprintf('Started loading program parameters...\n')
%% Simulation Parameters
t_simulation = 3;
model = 'crazyflie'; % crazyflie or quadthruster or example
trajectoryType = 'linearX'; % hover or linearX (future work should include linearY and box and circle)
outputDataAnalysis = 1;

% Simulation characteristics
seed = randi(1000, [1,5]);
Moment_noise = 1e-8;%5e-8;
Force_noise = 1e-8;%6e-8;



% constants across simulations
g = 9.81;
t = linspace(0,t_simulation, 100)'; % Generate for simulink trajectories
t_sample = t(2) - t(1);
n = length(t);

% output simulation stuff
formatSpec = "Simulation time of: %2.2f seconds \n";
fprintf(formatSpec, t_simulation)
fprintf("Current model dynamics: %s\n", model)

%% Determine which model to load
switch model
    case 'crazyflie'
        m = 0.0030; %[kg]
        J = diag([1.43e-5, 1.43e-5, 2.89e-5]); % %[kgm^2] Inertia matrix about {B} frame (body frame)
        d = 0.046; %[m] distance from center of mass to rotor
        J_inv = inv(J);
%         k_M = 1.5e-9; %[Nm/rpm^2]
%         k_f = 6.11e-8;     %[N/rpm^2]
%         k_motor = 20; %[1/second]
%         drag = [0.1, 0.1, .5]; %[kg/s]
%         inertial2body_rotationOrder = 'ZYX';


%         cT = k_f; % Thrust coefficient (Couples motor speed (omega^2) to thrust), determined from static thrust tests
%         cQ = k_M; % Moment coefficient [look @ Crazyflie for units
        X0 = [0, 0, 0];
        v0 = [0, 0, 0];
        RPY_0 = [0, 0, 0];
        pqr_0 = [0, 0, 0];

%         Gamma = [cT, cT, cT, cT;
%             0, d*cT, 0, -d*cT;
%             -d*cT, 0 d*cT, 0;
%             -cQ, cQ, -cQ, cQ;];
%         GammaInv = inv(Gamma);
%         dragConstants = [1, 1, 3]; %[N/ms] Assuming these here (dx, dy, dz)
    case 'quadthruster'
        m = 0.05 * 1e-3; % [kg] (1 g = 1e-3kg)
        J = diag([2.38, 2.58, 4.75]) * 1e-9; %[kg*m^2] (1 g*mm^2 = 1e-9 kg*m^2)
        J_inv = inv(J);
        d = 9.86 * 1e-3; %m (mm to meter conversion)
        X0 = [0, 0, 0];
        v0 = [0, 0, 0];
        RPY_0 = [0, 0, 0];
        pqr_0 = [0, 0, 0];

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

fprintf('Finished loading program parameters!\n')

%% running simulation and taking out params
% fprintf('Running Simulation\n')
% out = sim("flightSimulator.slx", t_simulation);
% Xe = out.Xe;
% RPY = out.RPY;
% e_RPY = out.e_RPY;
% u1_u2 = out.u1_u2;
% omega_b = out.omega_b
% % quatDirections = out.quatDirections;
% fprintf('Simulation Finished :) \n')


%%
% if (outputDataAnalysis == 1)
%     fprintf('Starting Data Analysis \n')
%     dimensionName = ["X (mm)", "Y (mm)", "Z (mm)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"]';
%     mean = [Xe.mean'* 1e3; RPY.mean'* (180/pi)];
%     std = [Xe.std'* 1e3; RPY.std'* (180/pi)];
%     max = [Xe.max'* 1e3; RPY.max'* (180/pi)];
% 
% 
% 
%     T = table(field, mean,std, max);
% 
% 
% 
% end

[out, mean, std, max] = runSim(t_simulation)
% noiseParams = [1e-8, 1e-8;
%     1e-10, 1e-10];
% 
% for i = [1:size(noiseParams(:,1))]
% 
% end

%%

function [out, mean, std, max] = runSim(t_simulation, noiseVec)
fprintf('Starting function')
out = sim("flightSimulator.slx", t_simulation);
Xe = out.Xe;
RPY = out.RPY;
e_RPY = out.e_RPY;
u1_u2 = out.u1_u2;
omega_b = out.omega_b
fprintf('Starting Data Analysis \n')
dimensionName = ["X (mm)", "Y (mm)", "Z (mm)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)", ...
    "Roll Vel (deg/s)", "Pitch Vel (deg/s)", "Yaw Vel(deg/s)"]';
mean = [Xe.mean'* 1e3; RPY.mean'* (180/pi); omega_b.mean'*(180/pi)];
std = [Xe.std'* 1e3; RPY.std'* (180/pi); omega_b.std'*(180/pi)];
max = [Xe.max'* 1e3; RPY.max'* (180/pi); omega_b.max'*(180/pi)];
end
