function zp = bike_model(z, u, sampleTime, VehicleParams)
% Function representing the kinematic bicycle model
%
% INPUT:
%       z - double (4x1)
%          State trajectory
%                   [x-pos; y-pos; speed; vehicle heading]
%                   [m; m; m/s; rad]
%
%       u - double (2x1)
%          Input trajectory
%                   [longitudinal accel; steering angle
%                   [m/s^2; rad]
%
%       sampleTime - double
%          Sampling Time [sec]
%
%       VehicleParams - struct 
%          Contains: lf - distance from CM to front wheel [m]
%                    lr - distance from CM to rear wheel [m]
%                    trackWidth - Axle width [m]
%
% OUTPUTS:
%      zp - double (4x1)
%          Updated state trajectory

zp = sdpvar(4,1);

% global frame kinematics/dynamics
yawRate = z(3)*sin(u(2))/VehicleParams.lr;
xVelocity = z(3)*cos(z(4)+u(2));
yVelocity = z(3)*sin(z(4)+u(2));

% slip angle calculations
% front tires
alphaF = z(4) + VehicleParams.lf*yawRate/z(3) - u(2);
% rear tires
alphaR = z(4) - VehicleParams.lr*yawRate/z(3);

% tire force calculation
Fyf = -VehicleParams.Cf*alphaF;
Fyr = -VehicleParams.Cr*alphaR;

xAccel = yawRate*yVelocity + u(1);
yAccel = -yawRate*xVelocity + (2/VehicleParams.mass)*(Fyf*cos(u(2)) + Fyr);
yawAccel = (2/VehicleParams.Izz)*(VehicleParams.lf*Fyf - VehicleParams.lr*Fyr);

zp(1) = z(1) + sampleTime*xVelocity + (sampleTime^2)*xAccel;
zp(2) = z(2) + sampleTime*yVelocity + (sampleTime^2)*yAccel;
zp(3) = z(3) + sampleTime*u(1);
zp(4) = z(4) + sampleTime*yawRate + (sampleTime^2)*yawAccel;


end