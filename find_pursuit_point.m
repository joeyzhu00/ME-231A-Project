function [pursuitPoint] = find_pursuit_point(zOpt, uOpt, VehicleParams, vehiclePath, elapsedTime)
% Function to find the specific point(s) that the controller should pursue.
% Currently assumes "constant acceleration" and some kinematics to find the
% point(s) to pursue. Calculates the norm from the current point versus the
% point chosen from kinematics and figures out the point along vehiclePath
% to follow.
%   NOTE: If we choose to follow more than one point (we probably will)
%   then we can calculate multiple kinematicPoints and wrap the path index
%   calculation in a for loop.
%
% INPUTS: 
%       zOpt - double (4x1)
%           State from controller: [x-pos; y-pos; speed; vehicle heading]
%                                  [m; m; m/s; rad]
%
%       uOpt - double (2x1)
%           Input from controller: [longitudinal accel; steering angle
%                                  [m/s^2; rad]
%
%       VehicleParams - struct 
%          Contains: lf - distance from CM to front wheel [m]
%                    lr - distance from CM to rear wheel [m]
%                    trackWidth - Axle width [m]
%
%       vehiclePath - double (2xS)
%          Globally planned vehicle path: [x-pos; y-pos]
%                                         [m; m]
%
%       elapsedTime - double
%          Time elapsed in open-loop
%
% OUTPUT:
%      pursuitPoint - double (2,1)
%          XY-points for the controller to pursue: [x-pos; y-pos]
%                                                  [m; m]

% velocities
xVelocity = zOpt(3) * cos(uOpt(2)+zOpt(4));
yVelocity = zOpt(3) * sin(uOpt(2)+zOpt(4));

% accelerations
xAccel = uOpt(1) * cos(uOpt(2)+zOpt(4)); 
yAccel = uOpt(1) * sin(uOpt(2)+zOpt(4));

% calculate the point with kinematics that the vehicle will end up in given the elapsed time, 
% assuming constant acceleration (this can be improved and is open to discussion)
% x pursuit point
kinematicsPoint(1) = zOpt(1) + xVelocity*(elapsedTime) + xAccel*(elapsedTime^2);
% y pursuit point
kinematicsPoint(2) = zOpt(2) + yVelocity*(elapsedTime) + yAccel*(elapsedTime^2);

% logic to find the closest point along path to this given point
% loop through each point in the vehicle path
pathNorm = zeros(length(vehiclePath));
for k = 1:length(vehiclePath)
    pathNorm(k) = sqrt((vehiclePath(1,k)-kinematicsPoint(1))^2 + (vehiclePath(2,k)-kinematicsPoint(2))^2);
end
% find the index of the minimum norm 
[~, pathIndex] = min(pathNorm);
pursuitPoint = vehiclePath(:,pathIndex(1));

end