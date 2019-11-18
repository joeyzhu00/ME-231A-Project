%% Script to Simulate Autonomous Vehicle on Highway With a Bicycle Model
clc,clear,close all

%% Vehicle/Path/Obstacle Parameters
%%%%%%%%%%%%%%%%
% Vehicle Path %
%%%%%%%%%%%%%%%%
% coordinates to make a straight path in xy-coordinates
vehiclePath(1,:) = 0:0.1:500; % [m]
vehiclePath(2,:) = zeros(length(vehiclePath(1,:)), 1); % [m]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacle Bounds/Centroid %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinates of the obstacle centroid (x,y)
obstacleCentroid = [50; 0]; % [m]
% obstacle vertex boundaries (assuming everything is rectangular)
% [xMin, xMax, yMin, yMax]
obstacleBounds = [-1, 1, -1, 1]; % [m]

%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dimensions %
%%%%%%%%%%%%%%%%%%%%%%
% vehicle wheelbase
VehicleParams.lf = 4.65/2; % [m]
VehicleParams.lr = 4.65/2; % [m]
% vehicle track width
VehicleParams.trackWidth = 1.78; % [m]

%% MPC Parameters
% sampling time
sampleTime = 0.05; % [sec]
% MPC Horizon
M = 200;
% CFTOC Horizon
N = 10;
% initial conditions
% [x-pos; y-pos; speed; vehicle heading]
z0 = [0; 0; 30; 0]; % [m; m; m/s; rad]

% do the MPC
[feas, zOpt, uOpt] = mpc_kinematic_bike(M, N, z0, vehiclePath, sampleTime, VehicleParams);

% figure()
% plot(vehiclePath(:,1), vehiclePath(:,2));
% grid on
% xlabel('X-Position [m]');
% ylabel('Y-Position [m]');





