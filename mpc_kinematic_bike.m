%{
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
%}
clear all

% THIS ONLY WORK FOR STRAIGHT LINE PATH!!! %
% Need to decide workflow from here. Pass the z(:,N+1) at each step OR
% wait until MPC complete and pass zOpt(:,M+1)?

N=5;
M=30;
z0 = [0; 0; 30; 0]; 
sampleTime = 0.1; % [sec]
% vehicle wheelbase
VehicleParams.lf = 4.65/2; % [m]
VehicleParams.lr = 4.65/2; % [m]
% vehicle track width
VehicleParams.trackWidth = 1.78; % [m]
%Path - ALMOST STRAIGHT LINE 
rhoS = 10000; % 10km radius of curvature path. Approximate a straight road

ObstacleParams(1).centroids = [30; 0.5]; % [m]
% obstacle vertex boundaries (assuming everything is rectangular)
% [xMin, xMax, yMin, yMax]
ObstacleParams(1).bounds = [-2.5, 2.5, -0.5, 0.5]; % [m]

ObstacleParams(2).centroids = [55; 0];
ObstacleParams(2).bounds = [-1.5, 1.5, -0.5, 0.5]; % [m]
% INTERESTINGLY this fail at [60,0], car try to return to center too fast
% and becomes infeasible (heading straight at 2nd obstacle because
% N-horizon is too short and don't see it until too late!

% number of states
nz = length(z0);
% number of inputs [longitudinal accel; steering angle]
nu = 2;

IneqConstraints.zMax = [10000; 5; z0(3)+1; 60*pi/180];
% upper state constraints
IneqConstraints.zMin = [0; -5; z0(3)-1; -60*pi/180];
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

% tune this to add padding around obstacles, higher = MORE padding
% too small will make MPC not feasible easier! We can't prove where this
% algorithm is persistently feasible :(. It is not in certain conditions. 
% THERE IS POSSIBILITY TO HEAD STRAIGHT FOR OBSTACLE IF N IS NOT LONG
% ENOUGH TO "SEE" IT IN THE CFTOC. DISCUSS THIS IN PAPER WRITE UP!
avoidTune = 1;

% Tune how closely the vehicle track the path. Higher is aggressive
% tracking
trackTune = avoidTune/5;

figure('name', 'position')
for i = 1:M
    fprintf('Working on MPC Iteration #%d \n', i);
    tic
    % lower state constraint is dynamic, 
    % (not needed) IneqConstraints.zMin = [zOpt(1,i); -3; 0; -2*pi];
    % solve the cftoc problem for a kinematic bicycle model and run in "open-loop"
    [feas(i), z, u, cost] = cftoc_kinematic_bike_spatial(N, zOpt(:,i), sampleTime, VehicleParams, IneqConstraints, ObstacleParams, rhoS, avoidTune, trackTune);
    toc
    
    if ~feas(i)
        disp('Infeasible region reached!');
        return
    end
    
    %plot open loops
    plot(z(1,:),z(2,:))
    hold on
    % closed loop predictions
    zOpt(:,i+1) = z(:,2);
    uOpt(:,i) = [u(1,1); u(2,1)];
    JOpt(:,i) = cost;
%    pause()
    % exit the for loop if the vehicle positions are within the stopCondition threshold
%     if (abs(zOpt(1,i+1)-vehiclePath(1,end)) <= stopCondition) && (abs(zOpt(2,i+1)-vehiclePath(2,end)) <= stopCondition)
%         disp('Reached the end of the path');
%         break;
%     end

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

%plot closed loop
plot(zOpt(1,:),zOpt(2,:),'o')

zOpt
uOpt
