function [feas, zOpt, uOpt, JOpt] = cftoc_kinematic_bike_spatial(N, z0, sampleTime, VehicleParams, IneqConstraints, ObstacleParams, rhoS, avoidTune, trackTune)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to solve the constrained finite time optimal control problem
% with a kinematic bicycle model to track straight line path using spatial
% state instead of time-states. 
% 
% ***NOTE***: Straight Line Only. To make it work for any path, will need
% more math for using curvilinear coordinate system
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
%       rhoS - double
%           Radius of curviture of the road. Keep this number HIGH (ex:
%           10000) so this will work. Maybe change in future to make this
%           workable on a constant curve of radius rhoS. Can also make rhoS
%           change at each function call if radius of curvature is
%           changing.
%       avoidTune - double
%           Tuning knob to determine how closely the vehicle dodge the
%           obstacle. Used in customer function when the vehicle is passing
%           obstacles.
%       trackTune - double
%           Tuning knon to adjust how aggressively the vehicle try to track
%           the centerline. Used at each step and at terminal step cost. 
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Convert time-dependent state to spatial states
sampleDist = sampleTime*z0(3); % initialize the be the same at k=0

ePsi0 = z0(4); %epsi0 = heading- psiRoad, psiRoad=0 at initial
ey0 = z0(2); % eY0 = VehGlobalY-RoadGlobalY. Setting the road at the x-axis to Global Frame, if not, need to subtract.
vx0 = z0(3); % speed in body-frame. meter per meter. 
zSpatial0 = [ePsi0; ey0; vx0];

% number of states [Veh heading in road-spatial-frame; 
%                   Veh y position in road-spatial-frame;
%                   Veh speed. meter per meter]
nz = length(zSpatial0);

% Number of inputs [longitudinal accel; steering angle]. 
nu = 2;

z = sdpvar(nz, N+1);
u = sdpvar(nu, N);

% initial condition constraint
constraints = z(:,1) == zSpatial0;
% initialize the cost

% Consider tuning this to smooth or inputs, or just use u(k+1)-u(k)
% constraints instead.
inputCostTune = 1;

% Terminal cost
% Currently only penalizing y-position tracking.
cost = trackTune*z(2,N+1)^2; 

%Terminal Constraints
constraints = [constraints,...
    -pi/6<=z(1,N+1)<=pi/6]; % soft constraints on final heading

for k=1:N
    % System Dynamics Constraints
    constraints = [constraints,...
        z(1,k+1) == z(1,k)+sampleDist*(((rhoS-z(2,k))*sin(u(2,k)))/(rhoS*z(3,k)*cos(z(1,k)))-1/rhoS),...
        z(2,k+1) == z(2,k)+sampleDist*(sin(z(1,k))*(rhoS-z(2,k)))/(rhoS*cos(z(1,k))),...
        z(3,k+1) == z(3,k)+0*sampleDist*u(1,k)]; % NO BRAKING FOR NOW. Remove zero to add braking/accel
    % State Constraints
    constraints = [constraints,...
        IneqConstraints.zMin(4)<=z(1,k+1)<=IneqConstraints.zMax(4),... %Heading
        IneqConstraints.zMin(2)<=z(2,k+1)<=IneqConstraints.zMax(2),... %Deviation from lane
        IneqConstraints.zMin(3)<=z(3,k+1)<=IneqConstraints.zMax(3)]; %Speed
    % Input Constraints
    constraints = [constraints, IneqConstraints.uMin(1)<=u(1,k)<=IneqConstraints.uMax(1),...
        IneqConstraints.uMin(2)<=u(2,k)<=IneqConstraints.uMax(2)];
    % Cost
    % Currently only penalizing y-position tracking and steering input.
    cost = cost + trackTune*z(2,k)^2 + inputCostTune*u(2,k)^2;

end

% Avoidance heuristic and calculations. 
% Structure with information about direction to dodge and
% obstacles s-bounds, any-bounds from given initial condition z0
avoid = avoidHeuristic(ObstacleParams, z0, sampleDist);

% State Constraints to avoid obstacle.
% Obstacle x coordinates have been converted to spatial s
% Obstacle y coordinate have been converted to ey-distance from road
% centerline 
for j=1:size(ObstacleParams,2) %loop through all obstacles 
    for k=avoid.s(j,1):avoid.s(j,2) %loop through space s of each obstacle
        if and(avoid.mode(j,1)==1,k<=N+1) %avoid left if obstacle is right of Veh heading
            % vehicle y position greater than obstacle bound y max position
            constraints = [constraints, z(2,k)>=(avoid.y(j,2)+VehicleParams.trackWidth/2)];
            cost = cost - avoidTune*(z(2,k)-(avoid.y(j,2)+VehicleParams.trackWidth/2))^2;
        elseif and(avoid.mode(j,1)==2,k<=N+1) %avoid right if obstacle is left of Veh heading
            % vehicle y position less than obstacle bound y min position
            constraints = [constraints, z(2,k)<=(avoid.y(j,1)-VehicleParams.trackWidth/2)];
            cost = cost - avoidTune*(z(2,k)-(avoid.y(j,2)-VehicleParams.trackWidth/2))^2;
        end
    end
end

options = sdpsettings('verbose', 0, 'solver', 'ipopt');
diagnostics = optimize(constraints, cost, options);

% Convert spatial s to time t. See reference literature for more discussion
interpTime=zeros(1,N+1);
if (diagnostics.problem == 0)
    feas = true;
    zOptSpatial = double(z);
    uOptSpatial = double(u);
    JOpt = double(cost);
    for k=1:N
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

% Convert Spatial States to Time-Dependent State using linear interpolation
zOpt(1,:) = z0(1)+interp1(interpTime,linspace(0,N*sampleDist,N+1),linspace(0,N*sampleTime,N+1)); %x global
zOpt(2,:) = interp1(interpTime,zOptSpatial(2,:),linspace(0,N*sampleTime,N+1)); %y global
zOpt(3,:) = interp1(interpTime,zOptSpatial(3,:),linspace(0,N*sampleTime,N+1)); %speed 
zOpt(4,:) = interp1(interpTime,zOptSpatial(1,:),linspace(0,N*sampleTime,N+1)); %heading
% Convert Spatial input to Time-Dependent input
uOpt(1,:) = interp1(interpTime(1:end-1),uOptSpatial(1,:),linspace(0,N*sampleTime,N)); %Steering Angle 
uOpt(2,:) = interp1(interpTime(1:end-1),uOptSpatial(2,:),linspace(0,N*sampleTime,N)); %Speed Change

end