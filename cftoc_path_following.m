function [feas, zOpt, uOpt, JOpt] = cftoc_path_following(N, z0, sampleTime, VehicleParams, IneqConstraints, pursuitPoint)
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
%       pursuitPoint - double (4x1)
%           XY-points to pursue: [x-pos; y-pos; speed; vehicle heading]
%                                [m; m; m/s; rad]
%
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
CH = 2;


z = sdpvar(nz, N+1);
u = sdpvar(nu, N);
v = sdpvar(nz, 1);


% initial condition constraint
constraints = z(:,1) == z0;

% initialize the cost
cost = 0;

% only apply cost for the last 4 points
for i = N:-1:N-3
    cost = cost + norm(z(:,i)-pursuitPoint(:,i))^2;
end
for i = 1:CH
    if i == 1
        cost = cost + norm(u(:,i))^2;
    else
        cost = cost + norm(u(:,i) - u(:,i-1))^2;
    end
end

% add soft constraint
cost = cost + 0.5 * norm(v,1)^2 + norm(v,1);

% loop through the horizon
for i = 1:N
    constraints = [constraints, ...
                   IneqConstraints.zMin - v <= z(:,i) <= IneqConstraints.zMax + v, ...         % state constraints
                   IneqConstraints.uMin <= u(:,i) <= IneqConstraints.uMax, ...         % input constraints
                   z(:,i+1) == bike_model(z(:,i), u(:,i), sampleTime)]; % state dynamics
    if i <= CH
        % input constraints up to N-1
        constraints = [constraints, ...
                       -IneqConstraints.longAccelRange <= u(1,i+1) - u(1,i) <= IneqConstraints.longAccelRange, ...
                       -IneqConstraints.betaRange <= u(2,i+1) - u(2,i) <= IneqConstraints.betaRange];            
    end
    if i > CH
        constraints = [constraints, u(:,i-1) == u(:,i)];
    end
end

% soft constraint
constraints = [constraints, v >= 0];

options = sdpsettings('verbose', 0, 'solver', 'ipopt');
diagnostics = optimize(constraints, cost, options);

if (diagnostics.problem == 0)
    feas = true;
    zOpt = double(z);
    uOpt = double(u);
    JOpt = double(cost);
else
    feas = false;
    zOpt = [];
    JOpt = [];
    uOpt = [];
    return
end


end




