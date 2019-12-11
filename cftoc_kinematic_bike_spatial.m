function [feas, zOpt, uOpt, JOpt] = cftoc_kinematic_bike_spatial(N, z0, sampleTime, VehicleParams, IneqConstraints, ObstacleParams, rhoS, avoidTune, trackTune)

% %%% DELETE THESE LATER. FOR TESTING ONLY %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% N = 10;
% z0 = [0; 0; 30; 0]; 
% sampleTime = 0.1; % [sec]
% VehicleParams.lf = 4.65/2; % [m]
% VehicleParams.lr = 4.65/2; % [m]
% VehicleParams.trackWidth = 1.78; % [m]
% rhoS = 10000; % 10km radius of curvature path. Approximate a straight road
% 
% 
% %Constraints 
% %upper state constraints
% IneqConstraints.zMax = [10000; 5; z0(3)+1; 45*pi/180];
% % upper state constraints
% IneqConstraints.zMin = [0; -5; z0(3)-1; -45*pi/180];
% % lower input constraints
% IneqConstraints.uMin = [-0.5; -30*pi/180];
% % upper input constraints
% IneqConstraints.uMax = [0.5; 30*pi/180];
% % limit on difference b/t current and previous steering input
% IneqConstraints.betaRange = 0.05; % [rad]
% % limit on longitudinal acceleration
% IneqConstraints.longAccelRange = 0.06; % [m/s^2]
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Obstacle Bounds/Centroid %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
% % coordinates of the obstacle centroid (x,y)
% ObstacleParams(1).centroids = [20; .5]; % [m]
% % obstacle vertex boundaries (assuming everything is rectangular)
% %xMin, xMax, yMin, yMax]
% ObstacleParams(1).bounds = [-1.5, 1.5, -0.5, 0.5]; % [m]
% % ObstacleParams(2).centroids = [40; .5]; % [m]
% % ObstacleParams(2).bounds = [-1, 1, -1, 1]; % [m]
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{ 
% Function to solve the constrained finite time optimal control problem
% with a kinematic bicycle model to track a global path. Using spatial
% state instead of time-states. 
% 
% ***NOTE***: Straight Line Only. To make it work for any path, will need
% more math so the s_coordinate is a vector that is the path
%
% INPUT:
%       N - double
%          CFTOC horizon
%       
%       z0 - double (4x1)
%          Initial Conditions: [x-pos; y-pos; speed; vehicle heading]
%                              [m; m; m/s; rad]
%       sampleTime - double
%          Sampling Time [sec]
%
%       VehicleParams - struct 
%          Contains: lf - distance from CM to front wheel [m]
%                    lr - distance from CM to rear wheel [m]
%                    trackWidth - Axle width [m]
%
%       IneqConstraints - struct
%          Contains: zMin - double (4x1) 
%                    zMax - double (4x1)
%                    uMin - double (2x1)
%                    uMax - double (2x1)
%                    betaRange - double
%                    longAccelRange - double
%
%       ObstacleParams - struct
%           Contains: centroids - position of centroids in global frame
%                     bounds - boundaries of the obstacle surrounding the
%                              centroid in the local obstacle frame
%
% OUTPUTS:
%      feas - bool
%          CFTOC feasibility
%
%      zOpt - double (4,N+1)
%          State trajectory
%
%      uOpt - double (2,N)
%          Input trajectory
%      
%      JOpt - double (1,N)
%          Cost
%}

% Convert time-dependent state to spatial states


sampleDist = sampleTime*z0(3); % initialize the be the same at k=0

ePsi0 = z0(4); %epsi0 = heading- psiRoad, psiRoad=0 at initial
ey0 = z0(2); % eY0 = VehGlobalY-RoadGlobalY. Setting the road at the x-axis to Global Frame, if not, need to subtract.
vx0 = z0(3); % speed in body-frame
zSpatial0 = [ePsi0; ey0; vx0];

% number of inputs [Veh heading in road-spatial-frame; 
%                   Veh y position in road-spatial-fram]
nz = length(zSpatial0);

% Number of inputs [longitudinal accel; steering angle]. 
nu = 2;

z = sdpvar(nz, N+1);
u = sdpvar(nu, N);

% initial condition constraint
constraints = z(:,1) == zSpatial0;
% initialize the cost
inputCostTune = 1;
% tune this to add padding around obstacles, higher = MORE padding
% too small will make MPC not feasible! 

%Terminal cost
cost = trackTune*z(2,N+1)^2 + inputCostTune*u(2,N)^2;
%Terminal Constraints
constraints = [constraints,...
    -pi/6<=z(1,N+1)<=pi/6]; % soft constraints on final heading

for k=1:N;
% System Dynamics Constraints
constraints = [constraints,...
    z(1,k+1) == z(1,k)+sampleDist*(((rhoS-z(2,k))*sin(u(2,k)))/(rhoS*z(3,k)*cos(z(1,k)))-1/rhoS),...
    z(2,k+1) == z(2,k)+sampleDist*(sin(z(1,k))*(rhoS-z(2,k)))/(rhoS*cos(z(1,k))),...
    z(3,k+1) == z(3,k)+0*sampleDist*u(1,k)]; % NO BRAKING FOR NOW
% State Constraints
constraints = [constraints,...
    IneqConstraints.zMin(4)<=z(1,k+1)<=IneqConstraints.zMax(4),... %Heading
    IneqConstraints.zMin(2)<=z(2,k+1)<=IneqConstraints.zMax(2),... %Deviation from lane
    IneqConstraints.zMin(3)<=z(3,k+1)<=IneqConstraints.zMax(3)]; %Speed
% Input Constraints
constraints = [constraints, IneqConstraints.uMin(1)<=u(1,k)<=IneqConstraints.uMax(1),...
    IneqConstraints.uMin(2)<=u(2,k)<=IneqConstraints.uMax(2)];

% Cost
cost = cost + trackTune*z(2,k)^2 + inputCostTune*u(2,k)^2;
end

% Avoidance heuristic and calculations. 
% Structure with information about direction to dodge and
% obstacles s-bounds, any-bounds from given initial condition z0
avoid = avoidHeuristic(ObstacleParams, z0, sampleDist);



for j=1:size(ObstacleParams,2); %loop through all obstacles 
    for k=avoid.s(j,1):avoid.s(j,2) %loop through state s of each obstacle
        if and(avoid.mode(j,1)==1,k<=N+1)
            constraints = [constraints, z(2,k)>=(avoid.y(j,2)+VehicleParams.trackWidth/2)];
            cost = cost - avoidTune*(z(2,k)-(avoid.y(j,2)+VehicleParams.trackWidth/2))^2;
        elseif and(avoid.mode(j,1)==2,k<=N+1)
            constraints = [constraints, z(2,k)<=(avoid.y(j,1)-VehicleParams.trackWidth/2)];
            cost = cost - avoidTune*(z(2,k)-(avoid.y(j,2)-VehicleParams.trackWidth/2))^2;
        end
    end
end

options = sdpsettings('verbose', 0, 'solver', 'ipopt');
diagnostics = optimize(constraints, cost, options);


interpTime=zeros(1,N+1);
if (diagnostics.problem == 0)
    feas = true;
    zOptSpatial = double(z);
    uOptSpatial = double(u);
    JOpt = double(cost);
    for k=1:N;
        interpTime(1,k+1)=interpTime(1,k)+sampleDist*(rhoS-zOptSpatial(2,k))/(rhoS*zOptSpatial(3,k)*cos(zOptSpatial(1,k)));
    end
else
    diagnostics.problem;
    feas = false;
    zOpt = [];
    JOpt = [];
    uOpt = [];
    return
end

% Convert Spatial States to Time-Dependent State
zOpt(1,:) = z0(1)+interp1(interpTime,linspace(0,N*sampleDist,N+1),linspace(0,N*sampleTime,N+1)); %x global
zOpt(2,:) = interp1(interpTime,zOptSpatial(2,:),linspace(0,N*sampleTime,N+1)); %y global
zOpt(3,:) = interp1(interpTime,zOptSpatial(3,:),linspace(0,N*sampleTime,N+1)); %speed 
zOpt(4,:) = interp1(interpTime,zOptSpatial(1,:),linspace(0,N*sampleTime,N+1)); %heading

uOpt(1,:) = interp1(interpTime(1:end-1),uOptSpatial(1,:),linspace(0,N*sampleTime,N)); %Steering Angle 
uOpt(2,:) = interp1(interpTime(1:end-1),uOptSpatial(2,:),linspace(0,N*sampleTime,N)); %Speed Change


%returns interpolated values of a 1-D function at specific query points using linear interpolation. Vector x contains the sample points, and v contains the corresponding values, v(x). Vector xq contains the coordinates of the query points.

% figure('name', 'position')
% % plot the obstacles
% for k = 1:length(ObstacleParams)
%     rectangle('Position', [ObstacleParams(k).centroids(1)+ObstacleParams(k).bounds(1), ObstacleParams(k).centroids(2)+ObstacleParams(k).bounds(3), ...
%                           -ObstacleParams(k).bounds(1)+ObstacleParams(k).bounds(2), -ObstacleParams(k).bounds(3)+ObstacleParams(k).bounds(4)]);
%     hold on
%     plotLegend{k+2} = sprintf('Obstacle #%d', k);
%     plot(ObstacleParams(k).centroids(1), ObstacleParams(k).centroids(2), 'o');
%     hold on
% end
% plot(sampleDist*(0:N), zOptSpatial(2,:))
% hold on
% plot(zOpt(1,:), zOpt(2,:),'o')
% hold off

% figure('name', 'heading')
% subplot(2,1,1)
% plot(sampleDist*(0:N), zOptSpatial(1,:))
% legend('heading vs spatial position')
% subplot(2,1,2)
% plot(linspace(0,N*sampleTime,N+1), zOpt(4,:))
% legend('heading vs time')







