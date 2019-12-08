function [feas, zOpt, uOpt, JOpt] = mpc_kinematic_bike(M, N, z0, vehiclePath, sampleTime, VehicleParams, stopCondition, ObstacleParams)
% Function to facilitate MPC for the kinematic bicycle to track a global
% path. The lower state constraints are currently static to force the
% vehicle to keep moving along the path. An obstacle avoidance cost is
% incorporated from Professor Borrelli's "Predictive Control of Autonomous
% Ground Vehicles With Obstacle Avoidance On Slippery Roads". 
% 
% INPUTS: 
%       M - double
%          MPC simulation horizon
%     
%       N - double
%          CFTOC horizon
%       
%       z0 - double (4x1)
%          Initial Conditions: [x-pos; y-pos; speed; vehicle heading]
%                              [m; m; m/s; rad]
%
%       vehiclePath - double (2xS)
%          Globally planned vehicle path: [x-pos; y-pos]
%                                         [m; m]
%
%       VehicleParams - struct 
%          Contains: lf - distance from CM to front wheel [m]
%                    lr - distance from CM to rear wheel [m]
%                    trackWidth - Axle width [m]
%
%       stopCondition - double
%           Threshold to stop the MPC when the XY-positions are within the
%           stopCondition 
%
%       ObstacleParams - struct
%           Contains: centroids - position of centroids in global frame
%                     bounds - boundaries of the obstacle surrounding the
%                              centroid in the local obstacle frame
%
% OUTPUTS:
%      feas - bool (1,M)
%          MPC feasibility
%
%      zOpt - double (4,M+1)
%          Closed-loop state trajectory
%
%      uOpt - double (2,M)
%          Closed-loop inputs
%
%      JOpt - double (1,M)
%          Cost from each iteration
%
%
% NOTE: Currently only allows the vehicle to track a global path,
% vehiclePath

% number of states
nz = length(z0);
% number of inputs [longitudinal accel; steering angle]
nu = 2;

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

zOpt = zeros(nz, M+1);
uOpt = zeros(nu, M);
JOpt = zeros(1,M);
feas = false([1, M]);

% initial conditions
zOpt(:,1) = z0;
uOpt(:,1) = [0;0];
for i = 1:M
    fprintf('Working on MPC Iteration #%d \n', i);
    % calculate the minimum distance from the vehicle to the object(s) in
    % the body frame
    [minDistance, minObstaclePoint] = min_distance_calc(zOpt(:,i), uOpt, VehicleParams, N, sampleTime, ObstacleParams);
    % display(minObstaclePoint);
    
    % find the point to pursue
    % pursuitPoint = find_pursuit_point(zOpt(:,i), uOpt, VehicleParams, vehiclePath, N, sampleTime);
    for j = 1:N
        pursuitPoint(:,j) = [zOpt(1,i)+(j-1)*sampleTime*30; 0; 30; zOpt(4,i)];
    end
    % display(pursuitPoint);
    % lower state constraint is dynamic
    IneqConstraints.zMin = [zOpt(1,i); -3; 0; -2*pi];
    % solve the cftoc problem for a kinematic bicycle model and run in "open-loop"
    [feas(i), z, u, cost] = cftoc_kinematic_bike(N, zOpt(:,i), sampleTime, VehicleParams, IneqConstraints, pursuitPoint, minDistance, minObstaclePoint, ObstacleParams);
    
    if ~feas(i)
        disp('Infeasible region reached!');
        return
    end
    
    % closed loop predictions
    zOpt(:,i+1) = z(:,2);
    uOpt(:,i) = [u(1,1); u(2,1)];
    JOpt(:,i) = cost;
    
    % exit the for loop if the vehicle positions are within the stopCondition threshold
    if (abs(zOpt(1,i+1)-vehiclePath(1,end)) <= stopCondition) && (abs(zOpt(2,i+1)-vehiclePath(2,end)) <= stopCondition)
        disp('Reached the end of the path');
        break;
    end
end

end