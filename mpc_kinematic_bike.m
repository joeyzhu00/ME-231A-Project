function [feas, zOpt, uOpt] = mpc_kinematic_bike(M, N, z0, vehiclePath, sampleTime, VehicleParams)
% Function to facilitate MPC for the kinematic bicycle to track a global
% path
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
%
% NOTE: Currently only allows the vehicle to track a global path,
% vehiclePath

% number of states
nz = length(z0, 2);
% number of inputs [longitudinal accel; steering angle]
nu = 2;

% lower state constraints
IneqConstraints.zMin = [vehiclePath(1); -500; 0; -2*pi];
% upper state constraints
IneqConstraints.zMax = [vehiclePath(end); 500; 80; 2*pi];
% lower input constraints
IneqConstraints.uMin = [-0.5; -30*pi/180];
% upper input constraints
IneqConstraints.uMax = [0.5; 30*pi/180];
% limit on difference b/t current and previous steering input
IneqConstraints.betaRange = 0.05; % [rad]
% limit on longitudinal acceleration
IneqConstraints.longAccelRange = 0.3; % [m/s^2]

zOpt = zeros(nz, M+1);
uOpt = zeros(nu, M);
feas = false([1, M]);

zOpt(:,1) = z0;

for i = 1:M
    % NOTE: need to add logic to sweep through the global list
    pursuitPoint = 0;
    % solve the cftoc problem for a kinematic bicycle model
    [feas(i), z, u, cost] = cftoc_kinematic_bike(N, zOpt(:,i), sampleTime, VehicleParams, IneqConstraints, pursuitPoint);
    
    if ~feas(i)
        zOpt = [];
        uOpt = [];
        disp('Infeasible region reached!');
        return
    end
    
    % closed loop predictions
    zOpt(:,i+1) = z(:,2);
    uOpt = [u(1,1); u(2,1)];
end

end