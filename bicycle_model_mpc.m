%% Script to Simulate Autonomous Vehicle on Highway With a Bicycle Model
clc,clear,close all

%% Vehicle/Path/Obstacle Parameters
%%%%%%%%%%%%%%%%
% Vehicle Path %
%%%%%%%%%%%%%%%%
% coordinates to make a straight path in xy-coordinates
vehiclePath(1,:) = 0:0.1:500; % [m]
vehiclePath(2,:) = zeros(length(vehiclePath(1,:)), 1); % [m]
% vehiclePath(2,:) = sin(0.1*vehiclePath(1,:)); % [m]

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
sampleTime = 0.1; % [sec]
% MPC Horizon
M = 100;
% CFTOC Horizon
N = 10;
% initial conditions
% [x-pos; y-pos; speed; vehicle heading]
z0 = [0; 0; 50; 0]; % [m; m; m/s; rad]

% stop condition
stopCondition = 10; % [m]
% do the MPC
[feas, zOpt, uOpt] = mpc_kinematic_bike(M, N, z0, vehiclePath, sampleTime, VehicleParams, stopCondition);

%% Do some plotting
time = 0:sampleTime:sampleTime*length(zOpt)-sampleTime;
figure(1)
subplot(3,1,1);
plot(vehiclePath(1,:), vehiclePath(2,:));
hold on
plot(zOpt(1,:), zOpt(2,:), 'o');
hold off
grid on
xlabel('X-Position [m]');
ylabel('Y-Position [m]');
legend('Vehicle Path', 'Vehicle State');
title('Vehicle Path vs Vehicle State');

subplot(3,1,2);
plot(time, zOpt(3,:));
grid on
xlabel('Time [sec]');
ylabel('Speed [m/s]');
title('Vehicle Speed');

subplot(3,1,3);
plot(time, zOpt(4,:));
grid on
xlabel('Time [sec]');
ylabel('Heading Angle [rad]');
title('Vehicle Heading Angle');

inputTime = time(1:end-1);
figure(2)
subplot(2,1,1);
plot(inputTime, uOpt(1,:));
grid on
xlabel('Time [sec]');
ylabel('Longitudinal Accel [m/s^2]');
title('Vehicle Longitudinal Acceleration');

subplot(2,1,2);
plot(inputTime, uOpt(2,:));
grid on
xlabel('Time [sec]');
ylabel('Steering Angle [rad]');
title('Steering Input');






