function avoid = avoidHeuristic(ObstacleParams, z0, sampleDist)

% Function to decide direction to dodge toward and to calculate the
% location of the obstacle in term of "s" (distance along path divided by
% sample distance) and "y" (distance away from the centerline of the path)
%
% !!! Currenly assume straight line path. To make this work for a curve, need
% to add rhoS in here and project the shape of the onstacle to the
% curvilinear coordinate.
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

% Initialize
avoid.mode = zeros(size(ObstacleParams,2),1);
avoid.s = zeros(size(ObstacleParams,2),2);
avoid.y = zeros(size(ObstacleParams,2),2);
posBody = zeros(size(ObstacleParams,2),2); %[x, y]

% create a Direction Cosine Matrix (DCM) describing rotation of the global frame from the body frame
Body_DCM_Global = [cos(z0(4)), (-1)*sin(z0(4));
                   sin(z0(4)), cos(z0(4))];           

% Loop through each obstacles
for j=1:size(ObstacleParams,2)
    % Estimate Obstacle bounds in s-space (round down and up for safety)
    avoid.s(j,:) = [floor((ObstacleParams(j).centroids(1)+ObstacleParams(j).bounds(1)-z0(1))/sampleDist),...
        ceil((ObstacleParams(j).centroids(1)+ObstacleParams(j).bounds(2)-z0(1))/sampleDist)];
    % Obstacle bound in y measure away from lane center. ASSUMING STRAIGHT LINE PATH
    % Equation (and shape of obstacle) needs to be projected to the path if not straight line
    avoid.y(j,:) = [ObstacleParams(j).centroids(2)+ObstacleParams(j).bounds(3), ...
        ObstacleParams(j).centroids(2)+ObstacleParams(j).bounds(4)];
    
    % Position vector of obstacle position with respect to body in global
    posGlobal = [ObstacleParams(j).centroids(1)-z0(1); ObstacleParams(j).centroids(2)-z0(2)];
    % Position vector of obstacle centroid with respect to body in the body frame
    posBody(j,:) = (inv(Body_DCM_Global)*posGlobal)';
    
    % set any negative s to 0 (that point is behind the vehicle)
    avoid.s(avoid.s<1)=0;
end

% Loop through each obstacles
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
