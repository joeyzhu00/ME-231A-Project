function [pursuitPoint] = find_pursuit_point(zOpt, uOpt, VehicleParams, vehiclePath, N, sampleTime)
% Function to find the specific point(s) that the controller should pursue.
% Currently assumes "constant acceleration" and some kinematics to find the
% point(s) to pursue. Calculates the norm from the current point versus the
% point chosen from kinematics and figures out the point along vehiclePath
% to follow. Calculates a cubic spline from the XY waypoints and then uses
% that information to calculate the desired vehicle heading. Desired
% vehicle speed is current static. 
%
% INPUTS: 
%       zOpt - double (4x1)
%          State from controller: [x-pos; y-pos; speed; vehicle heading]
%                                  [m; m; m/s; rad]
%
%       uOpt - double (2x1)
%          Input from controller: [longitudinal accel; steering angle
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
%       N - double
%          CFTOC horizon
%
%       sampleTime - double
%          Sampling Time [sec]
%
% OUTPUT:
%      pursuitPoint - double (4,N)
%          XY-points for the controller to pursue: [x-pos; y-pos; velocity; heading]
%                                                  [m; m; m/s; rad]

% velocities
xVelocity = zOpt(3) * cos(uOpt(2)+zOpt(4));
yVelocity = zOpt(3) * sin(uOpt(2)+zOpt(4));

% accelerations
xAccel = uOpt(1) * cos(uOpt(2)+zOpt(4)); 
yAccel = uOpt(1) * sin(uOpt(2)+zOpt(4));

kinematicsPoint = zeros(2,N);
unprocessedPoints = zeros(2,N);
pursuitPoint = zeros(4,N);
for i = 1:N
    % elapsed time until end of horizon
    elapsedTime = i*sampleTime;

    % calculate the point with kinematics that the vehicle will end up in given the elapsed time, 
    % assuming constant acceleration (this can be improved and is open to discussion)
    % x pursuit point
    kinematicsPoint(i,1) = zOpt(1) + xVelocity*(elapsedTime) + xAccel*(elapsedTime^2);
    % y pursuit point
    kinematicsPoint(i,2) = zOpt(2) + yVelocity*(elapsedTime) + yAccel*(elapsedTime^2);
    
    % logic to find the closest point along path to this given point
    % loop through each point in the vehicle path
    pathNorm = zeros(length(vehiclePath));
    for k = 1:length(vehiclePath)
        pathNorm(k) = sqrt((vehiclePath(1,k)-kinematicsPoint(i,1))^2 + (vehiclePath(2,k)-kinematicsPoint(i,2))^2);
    end
    % find the index of the minimum norm 
    [~, pathIndex] = min(pathNorm);
    unprocessedPoints(:,i) = [vehiclePath(1,pathIndex(1)); vehiclePath(2,pathIndex(1))];
end
% calculate cubic polynomial coefficients for points
cubicPolyCoeff = polyfit(unprocessedPoints(1,:), unprocessedPoints(2,:), 3);

% calculate the heading desired from the vehicle path given the
% unprocessedPoints
headingDesired = zeros(1,N);
for i = 1:N
    headingDesired(1,i) = atan(3*cubicPolyCoeff(1)*unprocessedPoints(1,i)^2 + 2*cubicPolyCoeff(2)*unprocessedPoints(1,i) + cubicPolyCoeff(3)*unprocessedPoints(1,i));
end

pursuitPoint(1,:) = unprocessedPoints(1,:);
pursuitPoint(2,:) = unprocessedPoints(2,:);
% velocity target will be static for now
pursuitPoint(3,:) = 30;
pursuitPoint(4,:) = headingDesired;

end