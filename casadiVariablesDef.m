%% ================= CaSAdi Symbolic Variables Definition =================
%%. CasADi v3.5.1
import casadi.*
% state = [x y z phi theta psi xdot ydot zdot phidot thetadot psidot]';
x = SX.sym('x'); 
y = SX.sym('y');
z = SX.sym('z');
phi = SX.sym('phi');
theta = SX.sym('theta');
psi = SX.sym('psi');
dx = SX.sym('dx');
dy = SX.sym('dy');
dz = SX.sym('dz');
dphi = SX.sym('phi');
dtheta = SX.sym('theta');
dpsi = SX.sym('psi');

% control input: propeller thrust fi = k*wi^2
u1 = SX.sym('u1'); 
u2 = SX.sym('u2'); 
u3 = SX.sym('u3');
u4 = SX.sym('u4'); 