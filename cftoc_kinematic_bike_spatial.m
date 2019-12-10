%function [feas, zOpt, uOpt, JOpt] = cftoc_kinematic_bike_spatial(N, z0, sampleTime, VehicleParams, IneqConstraints, ObstacleParams, rhoS)

%%% DELETE THESE LATER. FOR TESTING ONLY %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 15;

% [x-pos; y-pos; speed; vehicle heading]
z0 = [0; 0; 30; 0]; 
% sampleTime = 0.1; % [sec]
sampleDist = 3; %3 meter because 30m/s * 0.1sec

% vehicle wheelbase
VehicleParams.lf = 4.65/2; % [m]
VehicleParams.lr = 4.65/2; % [m]

% vehicle track width
VehicleParams.trackWidth = 1.78; % [m]

%Path
xCoordinates = 0:50:500;
yCoordinates = zeros(1, length(xCoordinates));
% yCoordinates = [0, 10, 10, 15, 20, 30, 30, 15, 10, -5, -10];

% generate linearly interpolated x-coordinates
xInterpCoordinates = linspace(xCoordinates(1), xCoordinates(end), 500);
% generate a spline of the y-coordinates
yInterpCoordinates = spline(xCoordinates, yCoordinates, xInterpCoordinates);

vehiclePath(1,:) = xInterpCoordinates;
vehiclePath(2,:) = yInterpCoordinates;

rhoS = 100; % 10km radius of curvature path. Approximate a straight road

% Constraints 
% upper state constraints
IneqConstraints.zMax = [vehiclePath(1,end); 3; 40; 2*pi];
% lower input constraints
IneqConstraints.uMin = [-0.5; -30*pi/180];
% upper input constraints
IneqConstraints.uMax = [0.5; 30*pi/180];
% limit on difference b/t current and previous steering input
IneqConstraints.betaRange = 0.05; % [rad]
% limit on longitudinal acceleration
IneqConstraints.longAccelRange = 0.06; % [m/s^2]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacle Bounds/Centroid %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinates of the obstacle centroid (x,y)
ObstacleParams(1).centroids = [20; 0]; % [m]
% obstacle vertex boundaries (assuming everything is rectangular)
% [xMin, xMax, yMin, yMax]
ObstacleParams(1).bounds = [-1.5, 1.5, -0.5, 0.5]; % [m]

ObstacleParams(2).centroids = [35; 0]; % [m]
% obstacle vertex boundaries (assuming everything is rectangular)
% [xMin, xMax, yMin, yMax]
ObstacleParams(2).bounds = [-1.5, 1.5, -0.5, 0.5]; % [m]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
trackCostTune = 1;
inputCostTune = 1;
cost = 0;

for k=1:N-1;
% State Constraints
constraints = [constraints,...
    z(1,k+1) == z(1,k)+sampleDist*(((rhoS-z(2,k))*sin(u(2,k)))/(rhoS*z(3,k)*cos(z(1,k)))-1/rhoS),...
    z(2,k+1) == z(2,k)+sampleDist*(sin(z(1,k))*(rhoS-z(2,k)))/(rhoS*cos(z(1,k))),...
    z(3,k+1) == z(3,k)+sampleDist*u(1,k)];
% Input Constraints
constraints = [constraints, IneqConstraints.uMin(1)<=u(1,k)<=IneqConstraints.uMax(1),...
    IneqConstraints.uMin(2)<=u(2,k)<=IneqConstraints.uMax(2)];
% Cost

% FIX THIS LATER
cost = cost + trackCostTune*z(2,k)^2 + inputCostTune*u(2,k)^2;
end


%Terminal Constraints
constraints = [constraints,...
    -pi/6<=z(1,N+1)<=pi/6, % soft constraints on final heading?
    -1<=z(2,N+1)<=1] % soft constraints on final position

% Obstacle Contraints
% Loop through each obstacles
% Estimate Obs bound in s-space (round down for safety)
% avoid.s = [s_min of obstacle, s_max of obstacle]. #rows=#obstacles
% avoid.y = [y_min of obstacle, y_max of obstacle]. #rows=#obstacles

for j=1:size(ObstacleParams,2)
avoid.s(j,:) = [floor((ObstacleParams(j).centroids(1)+ObstacleParams(j).bounds(1))/sampleDist),...
    ceil((ObstacleParams(j).centroids(1)+ObstacleParams(j).bounds(2))/sampleDist)]; 
avoid.y(j,:) = [ObstacleParams(j).centroids(2)+ObstacleParams(j).bounds(3), ...
    ObstacleParams(j).centroids(2)+ObstacleParams(j).bounds(4)];
end

%for j=size(avoid.s,1)
% This is to dodge left. Need to do HEURISTIC before this for left vs right

for j=1:size(ObstacleParams,2); %loop through all obstacles 
    for k=avoid.s(j,1):avoid.s(j,2) %loop through state s of each obstacle
        constraints = [constraints, z(2,k)>=avoid.y(j,2)];
    end
end

%end

% % only apply cost for the last 4 points
% for i = N:-1:N-3
%     cost = cost + norm(z(:,i)-pursuitPoint(:,i))^2;
% end
% 
% % loop through the horizon
% for i = 1:N
%     if ~minDistanceFlag
%         for j = 1:size(minObstaclePoint,1)
%              % min x position
%             objXMin = ObstacleParams(j).bounds(1) + ObstacleParams(j).centroids(1,1);
%             % max x position
%             objXMax = ObstacleParams(j).bounds(2) + ObstacleParams(j).centroids(1,1);
%             % min y position
%             objYCenter = ObstacleParams(j).centroids(2,1);
%             % max y position
%             objYOffset = ObstacleParams(j).bounds(3); 
%             %cost = cost + 10*z(3,i)/(((-1)*(minObstaclePoint(j,2)-z(2,i))*sin(z(4,i)) + (minObstaclePoint(j,1)-z(1,i))*cos(z(4,i))) + 0.00001);
%             constraints = [constraints,
%                 implies(objXMin<=z(1,i)<=objXMax, abs(z(2,i)-objYCenter)>=objYOffset)];
%            
%         end
%     end
%     constraints = [constraints, ...
%                    IneqConstraints.zMin <= z(:,i) <= IneqConstraints.zMax, ...         % state constraints
%                    IneqConstraints.uMin <= u(:,i) <= IneqConstraints.uMax, ...         % input constraints
%                    z(:,i+1) == bike_model(z(:,i), u(:,i), sampleTime, VehicleParams)]; % state dynamics
%     if i <= N-1
%         % input constraints up to N-1
%         constraints = [constraints, ...
%                        -IneqConstraints.longAccelRange <= u(1,i+1) - u(1,i) <= IneqConstraints.longAccelRange, ...
%                        -IneqConstraints.betaRange <= u(2,i+1) - u(2,i) <= IneqConstraints.betaRange];            
%     end
% end
%

options = sdpsettings('verbose', 1, 'solver', 'ipopt');
diagnostics = optimize(constraints, cost, options);



if (diagnostics.problem == 0)
    feas = true;
    zOpt = double(z);
    uOpt = double(u);
    JOpt = double(cost);
else
    diagnostics.problem;
    feas = false;
    zOpt = [];
    JOpt = [];
    uOpt = [];
    return
end


% plot the obstacles
for k = 1:length(ObstacleParams)
    rectangle('Position', [ObstacleParams(k).centroids(1)+ObstacleParams(k).bounds(1), ObstacleParams(k).centroids(2)+ObstacleParams(k).bounds(3), ...
                          -ObstacleParams(k).bounds(1)+ObstacleParams(k).bounds(2), -ObstacleParams(k).bounds(3)+ObstacleParams(k).bounds(4)]);
    hold on
    plotLegend{k+2} = sprintf('Obstacle #%d', k);
    plot(ObstacleParams(k).centroids(1), ObstacleParams(k).centroids(2), 'o');
    hold on
end
plot(sampleDist*(1:N+1), zOpt(2,:))

%end




