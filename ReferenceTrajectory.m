function [ xdesired ] = ReferenceTrajectory(t,path)
% This function generates reference signal for nonlinear MPC controller
% used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

switch path 
    case 'e' %... eight
        x = 8*sin(t/3);
        y = -8*sin(t/3).*cos(t/3);
        z = 8*cos(t/3);
    case 'c' %... circular
        x = 8*sin(t/3);
        y = 8*cos(t/3);
        z = 10*ones(1,length(t));
    case 'h' %... Helix
        x = 8*sin(0.3*t);
        y = 8*cos(0.3*t);
        z = 0.15*t;
    case 'H' %... Complex Helix
        x = 15*(sin(0.2*t)-(sin(0.2*t)).^3);
        y = 15*(cos(0.2*t)-(cos(0.2*t)).^3);
        z = 0.15*t;
    case 'p' %... P2P
        x = 18*ones(1,length(t));
        y = 0*ones(1,length(t));
        z = 5*ones(1,length(t));
end

phi = zeros(1,length(t));
theta = zeros(1,length(t));
psi = zeros(1,length(t));
xdot = zeros(1,length(t));
ydot = zeros(1,length(t));
zdot = zeros(1,length(t));
phidot = zeros(1,length(t));
thetadot = zeros(1,length(t));
psidot = zeros(1,length(t));

xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end

