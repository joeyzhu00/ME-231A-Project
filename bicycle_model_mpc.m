%% Script to Simulate Autonomous Vehicle on Highway With a Bicycle Model
clc,clear,close all

%% Vehicle/Path/Obstacle Parameters
%%%%%%%%%%%%%%%%
% Vehicle Path %
%%%%%%%%%%%%%%%%
% coordinates to make a straight path in xy-coordinates, for a straight
% line path, set all the yCoordinates to 0
xCoordinates = 0:50:200;
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
ObstacleParams(1).centroids = [35; 0.5]; % [m]
% obstacle vertex boundaries (assuming everything is rectangular)
% [xMin, xMax, yMin, yMax]
ObstacleParams(1).bounds = [-1.5, 1.5, -0.5, 0.5]; % [m]

ObstacleParams(2).centroids = [65; -1]; % [m]
ObstacleParams(2).bounds = [-1.5, 1.5, -0.5, 0.5]; % [m]

% ObstacleParams(3).centroids = [120; -1.5]; % [m]
% ObstacleParams(3).bounds = [-1, 1, -1, 1]; % [m]

% % coordinates of the obstacle centroid (x,y)
% ObstacleParams(1).centroids = [110; 0]; % [m]
% % obstacle vertex boundaries (assuming everything is rectangular)
% % [xMin, xMax, yMin, yMax]
% ObstacleParams(1).bounds = [-1, 1, -0.5, 0.5]; % [m]
% 
% ObstacleParams(2).centroids = [110; 1.5]; % [m]
% ObstacleParams(2).bounds = [-1, 1, -0.5, 0.5]; % [m]
% 
% ObstacleParams(3).centroids = [120; -1.5]; % [m]
% ObstacleParams(3).bounds = [-1, 1, -1, 1]; % [m]

%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dimensions %
%%%%%%%%%%%%%%%%%%%%%%
% vehicle wheelbase
VehicleParams.lf=2.4739;
VehicleParams.lfl = 2.4739; % [m]
VehicleParams.lfr = 2.4739; % [m]
VehicleParams.bf = 2.4739;

VehicleParams.lr = 3.7761; % [m]
VehicleParams.lrl = 3.7761; % [m]
VehicleParams.lrr = 3.7761; % [m]
VehicleParams.br = 3.7761;
% vehicle track width
VehicleParams.trackWidth = 1.245; % [m]
% vehicle cornering coefficients
VehicleParams.Cfl = 151550; % [N/rad]
VehicleParams.Cfr = 151550; % [N/rad]

VehicleParams.Crl = 52020; % [N/rad]
VehicleParams.Crr = 52020; % [N/rad]

% yaw inertia
VehicleParams.Izz = 93343; % [kg*m^2]
% vehicle mass
VehicleParams.mass = 17130; % [kg]

%% MPC Parameters
% sampling time
sampleTime = 0.1; % [sec]
% MPC Horizon
M = 30;
% CFTOC Horizon
N = 5;
% initial conditions
% [x-pos; y-pos; speed; vehicle heading]
z0 = [0; 0; 30; 0]; % [m; m; m/s; rad]

% stop condition
stopCondition = 10; % [m]

tic
% do the MPC
% [feas, zOpt, uOpt, JOpt, pursuitPoints] = mpc_two_level(M, N, z0, vehiclePath, sampleTime, VehicleParams, stopCondition, ObstacleParams);
[feas, zOpt, uOpt, JOpt] = mpc_kinematic_bike(M, N, z0, vehiclePath, sampleTime, VehicleParams, stopCondition, ObstacleParams);
toc
%% Do some plotting
close all
% % High level open loop predictions
% figure("Name", "High Level Open Loop Predictions")
% for i = 1:M
%     plot(pursuitPoints(1,:,i), pursuitPoints(2,:,i))
%     hold on
% end
% for k = 1:length(ObstacleParams)
%     rectangle('Position', [ObstacleParams(k).centroids(1)+ObstacleParams(k).bounds(1), ObstacleParams(k).centroids(2)+ObstacleParams(k).bounds(3), ...
%                           -ObstacleParams(k).bounds(1)+ObstacleParams(k).bounds(2), -ObstacleParams(k).bounds(3)+ObstacleParams(k).bounds(4)]);
%     plot(ObstacleParams(k).centroids(1), ObstacleParams(k).centroids(2), 'o');
% end
% xlabel('X-Position [m]');
% ylabel('Y-Position [m]');

% Path, Speed, and heading angle
time = 0:sampleTime:sampleTime*length(zOpt)-sampleTime;
figure("Name", "Vehicle Path, Speed, and Heading Angle")
subplot(3,1,1);
plot(vehiclePath(1,:), vehiclePath(2,:));
hold on
plot(zOpt(1,:), zOpt(2,:), '.', 'MarkerSize', 8);
plotLegend = {'Vehicle Path', 'Vehicle State'};

% plot the obstacles
for k = 1:length(ObstacleParams)
    rectangle('Position', [ObstacleParams(k).centroids(1)+ObstacleParams(k).bounds(1), ObstacleParams(k).centroids(2)+ObstacleParams(k).bounds(3), ...
                          -ObstacleParams(k).bounds(1)+ObstacleParams(k).bounds(2), -ObstacleParams(k).bounds(3)+ObstacleParams(k).bounds(4)]);
    plotLegend{k+2} = sprintf('Obstacle #%d', k);
    plot(ObstacleParams(k).centroids(1), ObstacleParams(k).centroids(2), 'o');
end

%Rotate about z axis
direction = [0 0 1];
for k = 1:M+1
    v = [zOpt(1,k)-VehicleParams.lrl, zOpt(2,k)-VehicleParams.trackWidth/2;...
        zOpt(1,k)-VehicleParams.lrl, zOpt(2,k)+VehicleParams.trackWidth/2;...
         zOpt(1,k)+VehicleParams.lfl, zOpt(2,k)+VehicleParams.trackWidth/2; ...
        zOpt(1,k)+VehicleParams.lfl, zOpt(2,k)-VehicleParams.trackWidth/2];
    veh = patch('Vertices',v, 'Faces',[1 2 3 4], 'EdgeColor', 'black', 'FaceColor', 'none');
    rotate(veh, direction, zOpt(4,k)*180/pi, [zOpt(1,k), zOpt(2,k),0]);
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
figure("Name", "Vehicle Acceleration, Steering, and Cost")
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







