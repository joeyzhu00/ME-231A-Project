function zOut = vehicle_dynamics(z, u, sampleTime, VehicleParams)
% Function representing the dynamic bicycle model to update the state
% feedback
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
%          Contains: lfl - distance from CM to front wheel left [m]
%                    lfr - distance from CM to front wheel right [m]
%                    lrl - distance from CM to rear wheel left[m]
%                    lrr - distance from CM to rear wheel right[m]
%                    trackWidth - Axle width [m]
%                    Cfl - Front left tire cornering coefficient [N/rad]
%                    Cfr - Front tight tire cornering coefficient [N/rad]
%                    Crl - Rear left tire cornering coefficient [N/rad]
%                    Crr - Rear right tire cornering coefficient [N/rad]
%                    Izz - Yaw Inertia [kg*m^2]
%                    mass - Vehicle Mass [kg]
%                    Bf - Distance from rear axle to CM
%
% OUTPUTS:
%      zOut - double (4x1)
%            Updated state trajectory

% global frame kinematics/dynamics
yawRate = z(3)*sin(u(2))/(VehicleParams.bf);
xVelocity = z(3)*cos(z(4)+u(2));
yVelocity = z(3)*sin(z(4)+u(2));

% slip angle calculations
% front left tire
alphaFl = z(4) + VehicleParams.lfl*yawRate/z(3) - u(2);
% front right tire
alphaFr = z(4) + VehicleParams.lfr*yawRate/z(3) - u(2);
% rear left tires
alphaRl = z(4) - VehicleParams.lrl*yawRate/z(3);
% rear right tires
alphaRr = z(4) - VehicleParams.lrr*yawRate/z(3);

% tire force calculation
Fyfl = -VehicleParams.Cfl*alphaFl;
Fyfr = -VehicleParams.Cfr*alphaFr;
Fyrl = -VehicleParams.Crl*alphaRl;
Fyrr = -VehicleParams.Crr*alphaRr;

xAccel = yawRate*yVelocity + u(1);
yAccel = -yawRate*xVelocity + (2/VehicleParams.mass)*((Fyfl+Fyfr)*cos(u(2)) + (Fyrl+Fyrr));
yawAccel = (2/VehicleParams.Izz)*((VehicleParams.lfl+VehicleParams.lfr)*(Fyfl+Fyfr) - (VehicleParams.lrl+VehicleParams.lrr)*(Fyrl+Fyrr));

zOut(1) = z(1) + sampleTime*xVelocity + (sampleTime^2)*xAccel;
zOut(2) = z(2) + sampleTime*yVelocity + (sampleTime^2)*yAccel;
zOut(3) = z(3) + sampleTime*u(1);
zOut(4) = z(4) + sampleTime*yawRate + (sampleTime^2)*yawAccel;

end
