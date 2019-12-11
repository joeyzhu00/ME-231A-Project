function avoid = avoidHeuristic(ObstacleParams, z0, sampleDist)

% Calculate the minimum distance from the vehicle to the object(s) in the
% body frame. 
%
% INPUTS: 
%       z0 - double (4x1)
%          State from controller: [x-pos; y-pos; speed; vehicle heading]
%       sampleDist - double
%          Sampling Distance [meter]
%       ObstacleParams - struct
%          Contains: centroids - position of centroids in global frame
%                    bounds - boundaries of the obstacle surrounding the
%                              centroid in the local obstacle frame       
%
% OUTPUT:
%       avoid.mode - Integer (Px1)
%           0 = ignore obstacle, 1 = avoid left, 2 = avoid right
%       avoid.s = Double (Px2)
%           [s_min of obstacle, s_max of obstacle] 
%           Parameterized in s (real distance = s*sampleDist)
%           Estimate Obs bound in s-space (round down for safety)
%           Both values are 0 if vehicle is past the obstacle
%       avoid.y = Double (Px2)
%           [y_min of obstacle, y_max of obstacle] in global frame


% Loop through each obstacles
avoid.mode = zeros(size(ObstacleParams,2),1);
avoid.s = zeros(size(ObstacleParams,2),2);
avoid.y = zeros(size(ObstacleParams,2),2);

% position vector of obstacle centroid with respect to body in the body frame
posBody = zeros(size(ObstacleParams,2),2); %[x, y]

% create a Direction Cosine Matrix (DCM) describing rotation of the global frame from the body frame
Body_DCM_Global = [cos(z0(4)), (-1)*sin(z0(4));
                   sin(z0(4)), cos(z0(4))];           
               
for j=1:size(ObstacleParams,2)
    % Estimate Obs bound in s-space (round down for safety)
    avoid.s(j,:) = [floor((ObstacleParams(j).centroids(1)+ObstacleParams(j).bounds(1)-z0(1))/sampleDist),...
        ceil((ObstacleParams(j).centroids(1)+ObstacleParams(j).bounds(2)-z0(1))/sampleDist)];
    % Obstacle bound in y measure away from lane center. ASSUMING STRAIGHT LINE PATH
    % Equation (and shapre of obstacle) needs to be projected to the path if not straight line
    avoid.y(j,:) = [ObstacleParams(j).centroids(2)+ObstacleParams(j).bounds(3), ...
        ObstacleParams(j).centroids(2)+ObstacleParams(j).bounds(4)];
    
    % position vector of obstacle position with respect to body in global
    posGlobal = [ObstacleParams(j).centroids(1)-z0(1); ObstacleParams(j).centroids(2)-z0(2)];
    posBody(j,:) = (inv(Body_DCM_Global)*posGlobal)';
    % set any negative s to 0 (that point is behind the vehicle)
    avoid.s(avoid.s<=1)=0;
end

for j=1:size(ObstacleParams,2)
    if all(avoid.s(j,:))==0 % obstacle entirely behind vehicle, ignore. 
        avoid.mode(j,1)=0; 
    elseif posBody(j,2)<=0 % obstacle right of vehicle heading, go left.
        avoid.mode(j,1)=1;
    elseif posBody(j,2)>0% obstacle left of vehicle heading, go right.
        avoid.mode(j,1)=2;        
    end
end

end



%{
% POSSIBLE IMPROVEMENT TO USE CORNERS OF OBSTACLE FOR HEURISTIC
minDistance = zeros(size(ObstacleParams,2),1);
minObstaclePoint = zeros(size(ObstacleParams,2),2);
% create a Direction Cosine Matrix (DCM) describing rotation of the body frame from the global frame
Body_DCM_Global = [cos(z0(4,1)), (-1)*sin(z0(4,1));
                   sin(z0(4,1)), cos(z0(4,1))];            

for p = 1:size(ObstacleParams,2)
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
        posGlobal = [obstaclePositions(i,1)-z0(1); obstaclePositions(i,2)-z0(2)];
        % position vector of obstacle position with respect to body in the
        % body frame        
        posBody(i,:) = (inv(Body_DCM_Global)*posGlobal)';        
    end
end
%}

%     obstacleDist = zeros(length(obstaclePositions),1);
%     for j = 1:length(obstaclePositions)
%         if (abs(posBody(j,2)) <= VehicleParams.trackWidth/2) && (posBody(j,1) > VehicleParams.lf)
%             obstacleDist(j,1) = posBody(j,1) - VehicleParams.lf;
%         elseif (abs(posBody(j,2)) <= VehicleParams.trackWidth/2) && (posBody(j,1) >= -VehicleParams.lr) && (posBody(j,1) <= VehicleParams.lf)
%             obstacleDist(j,1) = 0;
%         else
%             % arbitrarily high value
%             obstacleDist(j,1) = 100000000;
%         end
%         
%     end
%     minDistance(p,1) = min(obstacleDist);
%     % element of obstaclePositions that carries the closest obstacle point
%     closestObstacleElement = find(obstacleDist(minDistance(p,1) == min(obstacleDist)));
%     % if there is nothing in the closestObstacleElement then set it equal
%     % to the first vertex 
%     if isempty(closestObstacleElement)
%         closestObstacleElement = 1;
%     end
%     minObstaclePoint(p,:) = obstaclePositions(closestObstacleElement,:);
% end
        

