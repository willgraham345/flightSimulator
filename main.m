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
numSims = 100
momentNoiseRange = logspace(-12, -3, 100);
forceNoiseRange = zeros([1,n]);

noise = [momentNoiseRange', forceNoiseRange'];
numSims = size(noise,1)
Moment_noise = 1e-8;%5e-8;
Force_noise = 1e-8;%6e-8;
meanOut = [];
stdOut = [];
maxOut = [];
for i = [1:numSims]
    fprintf("simulation: ")
    i
%     fprintf('\n')
    Moment_noise = noise(i,1);
    Force_noise = noise(i,2);
    [out, mean_vals, std_vals, max_vals] = runSim(t_simulation);
    meanOut = [meanOut; mean_vals];
    stdOut = [stdOut; std_vals];
    maxOut = [maxOut; max_vals];
end


statsOut = [meanOut, stdOut, maxOut];
%%
XYZ_means = [statsOut(1:9:end, 1), statsOut(2:9:end, 1), statsOut(3:9:end, 1)];
XYZ_std = [statsOut(1:9:end, 2), statsOut(2:9:end, 2), statsOut(3:9:end, 2)];
XYZ_max = [statsOut(1:9:end, 3), statsOut(2:9:end, 3), statsOut(3:9:end, 3)];
RPY_means = [statsOut(4:9:end, 1), statsOut(5:9:end, 1), statsOut(6:9:end, 1)];
RPY_std = [statsOut(4:9:end, 2), statsOut(5:9:end, 2), statsOut(6:9:end, 2)];
RPY_max = [statsOut(4:9:end, 3), statsOut(5:9:end, 3), statsOut(6:9:end, 3)];
RPYrate_means = [statsOut(7:9:end, 1), statsOut(8:9:end, 1), statsOut(9:9:end, 1)];
RPYrate_std = [statsOut(7:9:end, 2), statsOut(8:9:end, 2), statsOut(9:9:end, 2)];
RPYrate_max = [statsOut(7:9:end, 3), statsOut(8:9:end, 3), statsOut(9:9:end, 3)];


%%
f1 = figure(1)
sgtitle('XYZ Noise and Stability')
subplot(3,1,1)
title("Mean Displacement")
semilogx(momentNoiseRange, XYZ_max)
legend('X Max', 'Y Max', 'Z Max');
xlabel("Noise [N*m]")
ylabel("Max Value [mm]")
ylim([-100, 100])

subplot(3,1,2)
title('Max Displacement')
semilogx(momentNoiseRange, XYZ_means)
legend('X Mean', 'Y Mean', 'Z Mean');
xlabel("Noise [N*m]")
ylabel("Max Value [mm]")
subplot(3,1,3)
semilogx(momentNoiseRange,XYZ_std)
legend('X STD', 'Y STD', 'Z STD');
xlabel("Noise [N*m]")
ylabel("Max Value [mm]")

f2 = figure(2)
sgtitle('RPY Noise and Stability')
subplot(3,1,1)
title("Mean Displacement")
semilogx(momentNoiseRange, RPY_max)
legend('Roll Max', 'Pitch Max', 'Yaw Max');
xlabel("Noise [N*m]")
ylabel("Max Value [deg]")
ylim([-30, 30]);

subplot(3,1,2)
title('Max Displacement')
semilogx(momentNoiseRange, RPY_means)
legend('Roll Mean', 'Pitch Mean', 'Yaw Mean');
xlabel("Noise [N*m]")
ylabel("Max Value [deg]")
ylim([-30, 30]);
subplot(3,1,3)
semilogx(momentNoiseRange,RPY_std)
legend('Roll STD', 'Pitch STD', 'Yaw STD');
xlabel("Noise [N*m]")
ylabel("Max Value [deg]")
ylim([-30, 30]);

f3 = figure(3)
sgtitle('RPY Rate Noise and Stability')
subplot(3,1,1)
title("Mean Displacement")
semilogx(momentNoiseRange, RPYrate_max)
legend('Roll Rate Max', 'Pitch Rate Max', 'Yaw Rate Max');
xlabel("Noise [N*m]")
ylabel("Max Value [deg/s]")
ylim([-30, 30]);

subplot(3,1,2)
title('Max Displacement')
semilogx(momentNoiseRange, RPYrate_means)
legend('Roll Rate Mean', 'Pitch Rate Mean', 'Yaw Rate Mean');
xlabel("Noise [N*m]")
ylabel("Max Value [deg/s]")
ylim([-30, 30]);
subplot(3,1,3)
semilogx(momentNoiseRange,RPYrate_std)
legend('Roll Rate STD', 'Pitch Rate STD', 'Yaw Rate STD');
xlabel("Noise")
ylabel("Max Value [deg/s]")
ylim([-30, 30]);

%%
% [X, Y, Z, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate] = plotResults(statsOut, noise);

%%

function [out, mean_vals, std_vals, max_vals] = runSim(t_simulation, noiseVec)
% fprintf('Starting function')
out = sim("flightSimulator.slx", t_simulation);
Xe = out.Xe;
RPY = out.RPY;
e_RPY = out.e_RPY;
u1_u2 = out.u1_u2;
omega_b = out.omega_b;
% fprintf('Starting Data Analysis \n')
dimensionName = ["X (mm)", "Y (mm)", "Z (mm)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)", ...
    "Roll Vel (deg/s)", "Pitch Vel (deg/s)", "Yaw Vel(deg/s)"]';
mean_vals = [Xe.mean'* 1e3; RPY.mean'* (180/pi); omega_b.mean'*(180/pi)];
std_vals = [Xe.std'* 1e3; RPY.std'* (180/pi); omega_b.std'*(180/pi)];
max_vals = [Xe.max'* 1e3; RPY.max'* (180/pi); omega_b.max'*(180/pi)];
end


function [X, Y, Z, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]= plotResults(statsOut, noise)





f1 = figure(1)
sgtitle("Max Noise vs Deflection")
a1 = subplot(1,3,1)
    loglog(noise(1), X(:,1))
    hold on
    loglog(noise(1), Y(:,1))
    loglog(noise(1), Z(:, 1))
%     hold off
    legend('X Max', 'Y Max', 'Z Max');
    xlabel("Noise")
    ylabel("Max Value [mm]")
    xlim([min(noise(:,1)), max(noise(:,1))])
    ylim([min(Z(:,1)), max(Z(:,1))])

a2 = subplot(1,3,2)
    plot(noise(1), roll(:,1))
    plot(noise(1), pitch(:,1))
    plot(noise(1), yaw(:,1))
    legend("Roll Max", "Pitch Max", "Yaw Max")
    xlabel("Noise")
    ylabel("Max Value [deg]")
    xlim([min(noise(:,1)), max(noise(:,1))])
    ylim([min(Z(:,1)), max(Z(:,1))])


a3 = subplot(1,3,3)
    plot(noise(1), roll_rate(:,1))
    hold on
    plot(noise(1), pitch_rate(:,1))
    plot(noise(1), yaw_rate(:,1))
    legend("Roll_Rate Max", "Pitch_Rate Max", "Yaw_Rate Max")
    xlabel("Noise")
    ylabel("Max Value [deg/s]")
   xlim([min(noise(:,1)), max(noise(:,1))])
    ylim([min(Z(:,1)), max(Z(:,1))])
    % figure()
end
