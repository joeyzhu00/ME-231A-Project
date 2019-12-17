function zp = bike_model(z, u, sampleTime, vehicleParams)
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

zp(1) = z(1)+sampleTime*z(3)*cos(z(4)+u(2));
zp(2) = z(2)+sampleTime*z(3)*sin(z(4)+u(2));
zp(3) = z(3)+sampleTime*u(1);
zp(4) = z(4)+sampleTime*z(3)*sin(u(2));
end