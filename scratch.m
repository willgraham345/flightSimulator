% %% Playing around with trajectory planning
% load parking-garage-posegraph.mat pg
% disp(pg);
% title('Original Pose Graph')
% show(pg,'IDs','off');
% view(-30,45)
% 
% 
% %%
% updatedPG = optimizePoseGraph(pg);
% figure
% title('Updated Pose Graph')
% show(updatedPG,'IDs','off');
% view(-30,45)

%%
% clear; clc; close all; format compact;
% ld = load('rpy_9axis.mat')
% accel = ld.sensorData.Acceleration;
% gyro = ld.sensorData.AngularVelocity;    
% Fs  = ld.Fs;
% decim = 2;
% fuse = imufilter('SampleRate',Fs,'DecimationFactor',decim);
% pose = fuse(accel,gyro);
% 
% tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
% op = orientationPlotter(tp,'DisplayName','Fused Data',...
%     'LocalAxesLength',2);
% 
% 
% for i=1:numel(pose)
%     plotOrientation(op, pose(i))
%     drawnow
% end
% 

%% Quaternion garbage
clear; close all; format compact; clc;
A = rotz(45);

eul = [0 pi/2 0;]
q = [1 0 1 0];
r = [1 1 1];
n = quatrotate(q, r);

eul = [0 pi/2 0];
qZYX = eul2quat(eul)
qZYX_1 = quatconj(qZYX)

n1 = qZYX*r


%% 
clear; close all; format compact; clc;
% A = zeros(4,4)
n = 1000;
acc = zeros(n, 3);
tspan = linspace(0, 5, n);
xAngVel = @(t) sin(3.*t) + 4.*t.^2;
yAngVel = @(t) -5.*t
zAngVel = @(t) -3.*t.^3 - 13.*t.^2 + 13;
angVel.signals.values = [xAngVel(tspan); yAngVel(tspan); zAngVel(tspan)]';
angVel.time = tspan;
psi0 = 0; theta0 = 0; phi0 = 0;
q0 = angle2quat(psi0,theta0, phi0)
%% figures
razyflie.m = 0.0030; %[kg]
Crazyflie.J = diag([1.43e-5, 1.43e-5, 2.89e-5]); % %[kgm^2] Inertia matrix about {B} frame (body frame)
Crazyflie.d = 0.046; %[m] distance from center of mass to rotor
Crazyflie.k_M = 1.5e-9; %[Nm/rpm^2]
Crazyflie.k_f = 6.11e-8;     %[N/rpm^2]
Crazyflie.k_motor = 20; %[1/second]
Crazyflie.RotMatrix = "ZYX"; %Order of rotations
Crazyflie.drag = [0.1, 0.1, .5]; %[kg/s]

X = zeros(12,1); U = ones(4,1);
% cf_firstOrderStateMdl(X, U, J_matrix, Gamma)
