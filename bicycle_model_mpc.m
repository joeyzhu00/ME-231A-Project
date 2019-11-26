%% Script to Simulate Autonomous Vehicle on Highway With a Bicycle Model
clc,clear,close all

%% Vehicle/Path/Obstacle Parameters
%%%%%%%%%%%%%%%%
% Vehicle Path %
%%%%%%%%%%%%%%%%
% coordinates to make a straight path in xy-coordinates, for a straight
% line path, set all the yCoordinates to 0
xCoordinates = 0:50:500;
yCoordinates = zeros(1, length(xCoordinates));
% yCoordinates = [0, 10, 10, 15, 20, 30, 30, 15, 10, -5, -10];

% generate linearly interpolated x-coordinates
xInterpCoordinates = linspace(xCoordinates(1), xCoordinates(end), 500);
% generate a spline of the y-coordinates
yInterpCoordinates = spline(xCoordinates, yCoordinates, xInterpCoordinates);

vehiclePath(1,:) = xInterpCoordinates;
vehiclePath(2,:) = yInterpCoordinates;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacle Bounds/Centroid %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinates of the obstacle centroid (x,y)
ObstacleParams(1).centroids = [110; 0]; % [m]
% obstacle vertex boundaries (assuming everything is rectangular)
% [xMin, xMax, yMin, yMax]
ObstacleParams(1).bounds = [-1, 1, -0.5, 0.5]; % [m]

ObstacleParams(2).centroids = [110; 1.5]; % [m]
ObstacleParams(2).bounds = [-1, 1, -0.5, 0.5]; % [m]

ObstacleParams(3).centroids = [120; -1.5]; % [m]
ObstacleParams(3).bounds = [-1, 1, -1, 1]; % [m]

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
M = 50;
% CFTOC Horizon
N = 5;
% initial conditions
% [x-pos; y-pos; speed; vehicle heading]
z0 = [0; 0; 30; 0]; % [m; m; m/s; rad]

% stop condition
stopCondition = 10; % [m]

tic
% do the MPC
[feas, zOpt, uOpt, JOpt] = mpc_kinematic_bike(M, N, z0, vehiclePath, sampleTime, VehicleParams, stopCondition, ObstacleParams);
toc
%% Do some plotting
time = 0:sampleTime:sampleTime*length(zOpt)-sampleTime;
figure(1)
subplot(3,1,1);
plot(vehiclePath(1,:), vehiclePath(2,:));
hold on
plot(zOpt(1,:), zOpt(2,:), '.', 'MarkerSize', 8);
hold on
plotLegend = {'Vehicle Path', 'Vehicle State'};

% plot the obstacles
for k = 1:length(ObstacleParams)
    rectangle('Position', [ObstacleParams(k).centroids(1)+ObstacleParams(k).bounds(1), ObstacleParams(k).centroids(2)+ObstacleParams(k).bounds(3), ...
                          -ObstacleParams(k).bounds(1)+ObstacleParams(k).bounds(2), -ObstacleParams(k).bounds(3)+ObstacleParams(k).bounds(4)]);
    hold on
    plotLegend{k+2} = sprintf('Obstacle #%d', k);
    plot(ObstacleParams(k).centroids(1), ObstacleParams(k).centroids(2), 'o');
    hold on
end
hold off
grid on
xlabel('X-Position [m]');
ylabel('Y-Position [m]');
legend(plotLegend);
title('Vehicle Path vs Vehicle State');

% plot the inputs and the cost
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
subplot(3,1,1);
plot(inputTime, uOpt(1,:));
grid on
xlabel('Time [sec]');
ylabel('Longitudinal Accel [m/s^2]');
title('Vehicle Longitudinal Acceleration');

subplot(3,1,2);
plot(inputTime, uOpt(2,:));
grid on
xlabel('Time [sec]');
ylabel('Steering Angle [rad]');
title('Steering Input');

subplot(3,1,3)
plot(inputTime, JOpt);
grid on
xlabel('Time [sec]');
ylabel('Cost');
title('Cost Over Time');







