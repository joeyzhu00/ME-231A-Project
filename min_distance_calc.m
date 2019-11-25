function [minDistance, minObstaclePoint] = min_distance_calc(zOpt, uOpt, VehicleParams, N, sampleTime, ObstacleParams)
% Calculate the minimum distance from the vehicle to the object(s) in the
% body frame. 
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
%       ObstacleParams - struct
%          Contains: centroids - position of centroids in global frame
%                    bounds - boundaries of the obstacle surrounding the
%                              centroid in the local obstacle frame       
%
% OUTPUT:
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

minDistance = zeros(size(ObstacleParams,1));
minObstaclePoint = zeros(size(ObstacleParams,1),2);
% create a Direction Cosine Matrix (DCM) describing rotation of the body frame from the global frame
Body_DCM_Global = [cos(zOpt(4,1)), (-1)*sin(zOpt(4,1));
                   sin(zOpt(4,1)), cos(zOpt(4,1))];            

% loop through each obstacle
for p = 1:size(ObstacleParams,1)
    % loop through each vertex
    obstacleBounds = zeros(length(ObstacleParams(p).bounds),1);
    % min x position
    obstacleBounds(1,1) = ObstacleParams(p).bounds(1) + ObstacleParams(p).centroids(1,1);
    % max x position
    obstacleBounds(1,2) = ObstacleParams(p).bounds(2) + ObstacleParams(p).centroids(1,1);
    % min y position
    obstacleBounds(1,3) = ObstacleParams(p).bounds(3) + ObstacleParams(p).centroids(2,1);
    % max y position
    obstacleBounds(1,4) = ObstacleParams(p).bounds(4) + ObstacleParams(p).centroids(2,1);
    
    % store all of the obstacle points (just the corners for now)
    obstaclePositions = zeros(length(ObstacleParams(p).bounds),2);
    obstaclePositions(1,:) = [obstacleBounds(1,1), obstacleBounds(1,3)];
    obstaclePositions(2,:) = [obstacleBounds(1,1), obstacleBounds(1,4)];
    obstaclePositions(3,:) = [obstacleBounds(1,2), obstacleBounds(1,3)];
    obstaclePositions(4,:) = [obstacleBounds(1,2), obstacleBounds(1,4)];
    
    % generate a 2x1 position vector representing the distance between the
    % object boundary and the CM of the vehicle
    posBody = zeros(length(obstaclePositions),2);
    for i = 1:length(obstaclePositions)
        % position vector of obstacle position with respect to body in
        % global frame
        posGlobal = [obstaclePositions(i,1)-zOpt(1,1); obstaclePositions(i,2)-zOpt(2,1)];
        % position vector of obstacle position with respect to body in the
        % body frame        
        posBody(i,:) = Body_DCM_Global*posGlobal;        
    end
    
    obstacleDist = zeros(length(obstaclePositions),1);
    for j = 1:length(obstaclePositions)
        if (abs(posBody(j,2)) <= VehicleParams.trackWidth/2) && (posBody(j,1) > VehicleParams.lf)
            obstacleDist(j,1) = posBody(j,1) - VehicleParams.lf;
        elseif (abs(posBody(j,2)) <= VehicleParams.trackWidth/2) && (posBody(j,1) >= -VehicleParams.lr) && (posBody(j,1) <= VehicleParams.lf)
            obstacleDist(j,1) = 0;
        else
            % arbitrarily high value
            obstacleDist(j,1) = 100000000;
        end
        
    end
    minDistance(p,1) = min(obstacleDist);
    % element of obstaclePositions that carries the closest obstacle point
    closestObstacleElement = find(obstacleDist(minDistance == min(obstacleDist)));
    minObstaclePoint(p,:) = obstaclePositions(closestObstacleElement,:);
end

end