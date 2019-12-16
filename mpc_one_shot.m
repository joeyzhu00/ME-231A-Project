function [feas, zOpt, uOpt, JOpt, pursuitPoints] = mpc_one_shot(M, N, z0, vehiclePath, sampleTime, VehicleParams, stopCondition, ObstacleParams)
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
%                    Cf - Front tire cornering coefficient [N/rad]
%                    Cr - Rear tire cornering coefficient [N/rad]
%                    Izz - Yaw Inertia [kg*m^2]
%                    mass - Vehicle Mass [kg]
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
IneqConstraints.zMax = [vehiclePath(1,end); 6; 40; 2*pi];
% lower input constraints
IneqConstraints.uMin = [-0.5; -30*pi/180];
% upper input constraints
IneqConstraints.uMax = [0.5; 30*pi/180];
% limit on difference b/t current and previous steering input
IneqConstraints.betaRange = 0.05; % [rad]
% limit on longitudinal acceleration
IneqConstraints.longAccelRange = 0.06; % [m/s^2]

% tune this to add padding around obstacles, higher = MORE padding
% too small will make MPC not feasible easier! We can't prove where this
% algorithm is persistently feasible :(. It is not in certain conditions. 
% THERE IS POSSIBILITY TO HEAD STRAIGHT FOR OBSTACLE IF N IS NOT LONG
% ENOUGH TO "SEE" IT IN THE CFTOC. DISCUSS THIS IN PAPER WRITE UP!
avoidTune = 1;

% Tune how closely the vehicle track the path. Higher is aggressive
% tracking
trackTune = 0.2;


%Path - ALMOST STRAIGHT LINE 
rhoS = 10000; % 10km radius of curvature path. Approximate a straight road

zOpt = zeros(nz, M+1);
uOpt = zeros(nu, M);
JOpt = zeros(1,M);
feas = false([1, M]);
planFeas = false([1, M]);
pursuitPoints = zeros(nz, N+1, M);

% initial conditions
zOpt(:,1) = z0;
uOpt(:,1) = [0;0];

figure()

for i = 1:M
    fprintf('Working on MPC Iteration #%d \n', i);
    % find the point to pursue
%     pursuitPoint = find_pursuit_point(zOpt(:,i), uOpt, VehicleParams, vehiclePath, N, sampleTime);
    IneqConstraints.zMin = [zOpt(1,i); -6; 0; -2*pi];
    
    % calculate the minimum distance from the vehicle to the object(s) in
    % the body frame
    [minDistance, minObstaclePoint] = min_distance_calc(zOpt(:,i), [], VehicleParams, [], [], ObstacleParams);

    % find the point to pursue
    pursuitPoint = find_pursuit_point(zOpt(:,i), uOpt, VehicleParams, vehiclePath, N, sampleTime);
    % do the high-level MPC to figure out the path
%     [planFeas(i), zSpatial, ~, ~] = cftoc_kinematic_bike_spatial(N, zOpt(:,i), sampleTime, VehicleParams, IneqConstraints, ObstacleParams, rhoS, avoidTune, trackTune);

%     if ~planFeas(i)
%         disp('High Level infeasible region reached!');
%         return;
%     end
%     pursuitPoints(:,:,i) = zSpatial;
    
    % lower state constraint is dynamic
    [feas(i), z, u, cost] = cftoc_path_following(N, z0, sampleTime, VehicleParams, IneqConstraints, pursuitPoint, minDistance, minObstaclePoint);
    
    if ~feas(i)
        disp('Infeasible region reached!');
        return
    end    
    % closed loop predictions
    uOpt(:,i) = [u(1,1); u(2,1)];
    JOpt(:,i) = cost;
%     zOpt(:,i+1) = z(:,2);
    % update state dynamics with the dynamic bicycle model
    zOpt(:,i+1) = dynamic_vehicle_model(zOpt(:,i), uOpt(:,i), sampleTime, VehicleParams);
    
    % exit the for loop if the vehicle positions are within the stopCondition threshold
    if (abs(zOpt(1,i+1)-vehiclePath(1,end)) <= stopCondition) && (abs(zOpt(2,i+1)-vehiclePath(2,end)) <= stopCondition)
        disp('Reached the end of the path');
        break;
    end
end

end