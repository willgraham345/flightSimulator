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