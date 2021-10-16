%% ================ EoM of a Quadrotor (Euler angle) ======================
% system parameters
Ix = 1.2; Iy = 1.2; Iz = 2;
k = 1; l = 0.25; m = 2; b = 0.2; grav = 9.81;

% rotation matrix (inertia to body)
Rz = [cos(psi) -sin(psi)  0;
      sin(psi)  cos(psi)  0;
         0          0     1];
Ry = [cos(theta) 0 sin(theta);
          0      1      0    ;
     -sin(theta) 0 cos(theta)];
Rx = [ 1     0          0;
       0  cos(phi) -sin(phi);
       0  sin(phi)  cos(phi)];
R = Rz*Ry*Rx;

% Euler kinematics equations
W = [1      0           -sin(theta);
     0   cos(phi)  sin(phi)*cos(theta);
     0  -sin(phi)  cos(phi)*cos(theta)];

% Jacobian matrix J
I = [Ix 0 0; 0 Iy 0; 0 0 Iz];
J = W'*I*W;

%%... Definition of sin-cos variables
sP = sin(phi);   cP = cos(phi);
sT = sin(theta); cT = cos(theta);
sS = sin(psi);   cS = cos(psi);

% Coriolis term C containing the gyroscopic and centripetal term
C11 = 0;
C12 = (Iy-Iz)*(dtheta*cP*sP + dpsi*sP^2*cT) + (Iz-Iy)*dpsi*cP^2*cT - Ix*dpsi*cT;
C13 = (Iz-Iy)*dpsi*cP*sP*cT^2;

C21 = (Iz-Iy)*(dtheta*cP*sP + dpsi*sP*cT) + (Iy-Iz)*dpsi*cP^2*cT + Ix*dpsi*cT;
C22 = (Iz-Iy)*dphi*cP*sP;
C23 = -Ix*dpsi*sT*cT + Iy*dpsi*sP^2*sT*cT + Iz*dpsi*cP^2*sT*cT;

C31 = (Iy-Iz)*dpsi*cT^2*sP*cP - Ix*dtheta*cT;
C32 = (Iz-Iy)*(dtheta*cP*sP*sT + dphi*sP^2*cT) + (Iy-Iz)*dphi*cP^2*cT + ...
      Ix*dpsi*sT*cT - Iy*dpsi*sP^2*sT*cT - Iz*dpsi*cP^2*sT*cT;
C33 = (Iy-Iz)*dphi*cP*sP*cT^2 - Iy*dtheta*sP^2*cT*sT - Iz*dtheta*cP^2*cT*sT + Ix*dtheta*cT*sT; 

C = [C11 C12 C13;
     C21 C22 C23;
     C31 C32 C33];

% Define total thrust and torques
thrust = k*(u1+u2+u3+u4);
tau_b = [l*k*(-u2+u4); l*k*(-u1+u3); b*(-u1+u2-u3+u4)];