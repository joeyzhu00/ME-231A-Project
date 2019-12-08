function [feas, zOpt, uOpt, JOpt] = cftoc_kinematic_bike(N, z0, sampleTime, VehicleParams, IneqConstraints, pursuitPoint, minDistance, minObstaclePoint, ObstacleParams)
% Function to solve the constrained finite time optimal control problem
% with a kinematic bicycle model to track a global path.
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
%       pursuitPoint - double (2x1)
%           XY-points to pursue: [x-pos; y-pos]
%                                [m; m]
%
%       minDistance - double (Px1)
%           Distance from the front of the vehicle to the object bounds in
%           the body frame. P denotes the number of obstacles that are
%           present. 
%
%       minObstaclePoint - double(Px2)
%           Location of each obstacle point that is closest to the vehicle
%           in the global frame
%               [x-pos, y-pos]
%               [m, m]
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

% number of states
nz = length(z0);
% number of inputs [longitudinal accel; steering angle]
nu = 2;

z = sdpvar(nz, N+1);
u = sdpvar(nu, N);

% initial condition constraint
constraints = z(:,1) == z0;

% initialize the cost
cost = 0;

minDistanceFlag = 1;
if (all(minDistance) >= 0) & (all(minDistance) <= 1000)
    minDistanceFlag = 0;
end
% only apply cost for the last 4 points
for i = N:-1:N-3
    cost = cost + norm(z(:,i)-pursuitPoint(:,i))^2;
end
% loop through the horizon
for i = 1:N
    if ~minDistanceFlag
        for j = 1:size(minObstaclePoint,1)
             % min x position
            objXMin = ObstacleParams(j).bounds(1) + ObstacleParams(j).centroids(1,1);
            % max x position
            objXMax = ObstacleParams(j).bounds(2) + ObstacleParams(j).centroids(1,1);
            % min y position
            objYCenter = ObstacleParams(j).centroids(2,1);
            % max y position
            objYOffset = ObstacleParams(j).bounds(3); 
            %cost = cost + 10*z(3,i)/(((-1)*(minObstaclePoint(j,2)-z(2,i))*sin(z(4,i)) + (minObstaclePoint(j,1)-z(1,i))*cos(z(4,i))) + 0.00001);
            constraints = [constraints,
                implies(objXMin<=z(1,i)<=objXMax, abs(z(2,i)-objYCenter)>=objYOffset)];
           
        end
    end
    constraints = [constraints, ...
                   IneqConstraints.zMin <= z(:,i) <= IneqConstraints.zMax, ...         % state constraints
                   IneqConstraints.uMin <= u(:,i) <= IneqConstraints.uMax, ...         % input constraints
                   z(:,i+1) == bike_model(z(:,i), u(:,i), sampleTime, VehicleParams)]; % state dynamics
    if i <= N-1
        % input constraints up to N-1
        constraints = [constraints, ...
                       -IneqConstraints.longAccelRange <= u(1,i+1) - u(1,i) <= IneqConstraints.longAccelRange, ...
                       -IneqConstraints.betaRange <= u(2,i+1) - u(2,i) <= IneqConstraints.betaRange];            
    end
end

options = sdpsettings('verbose', 0, 'debug', 1, 'solver', 'ipopt');
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


end




