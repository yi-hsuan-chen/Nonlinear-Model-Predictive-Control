% Quadrotor P2P flight control simulation
% point stabilization + Multiple shooting + Runge Kutta
% Edited by Yi-Hsuan Chen, 03/22/2021

clc; clear; close all;

% CasADi v3.5.1
casadipath = 'C:\Users\yihsu\Desktop\KAUST\2021 Spring courses\ECE 372 - Dynamic Programming and Optimal Control\Final Project\casadi-windows-matlabR2016a-v3.5.5';
addpath(casadipath);
import casadi.*

%% ======================= User Define Parameters =========================
%------------------------- Flight path Setting ----------------------------
uConstraint = 1;                % 1: constrained / 0: unconstrained
uUpper = 15;                    % upper limit of propeller thrust
uLower = 0;                     % lower limit of propeller thrust
recordVideo = 0;                % 1: on / 0: off 
videoName = 'NMPC_Quadrotor_PS';
%% ============= Define symbolic state and control variables ==============
casadiVariablesDef;

state = [x y z phi theta psi dx dy dz dphi dtheta dpsi]';
control = [u1;u2;u3;u4];

n_state = length(state);
n_control = length(control);

%% ================= Define equation of motion (rhs) ======================
QuadrotorEoM;

% system dynamics Xdot = f(x,u)
rhs = SX.sym('rhs',12,1);
rhs(1:3) = [dx; dy; dz];
rhs(4:6) = [dphi; dtheta; dpsi];
rhs(7:9) = [0;0;-grav] + R*[0;0;thrust]/m;         % translational dynamics
rhs(10:12) = inv(J)*(tau_b-C*[dphi;dtheta;dpsi]);  % rotational dynamics

% nonlinear mapping function f(x,u)
f = Function('f',{state,control},{rhs}); 

%% ================ Define MPC and reformulate into NLP ===================
dt = 0.2;                                          % sampling time
N = 10;                                            % prediction horizon
U = SX.sym('U',n_control,N);                       % Decision variables (controls)
P = SX.sym('P',n_state*2);                         % parameters (initial state + reference state)
X = SX.sym('X',n_state,(N+1));                     % state variables (current state + predicted states)

% initialize the objective function and constraint vector
obj = 0;
g = [];

% Define weighting matrices Q and R
Q = zeros(n_state,n_state);
Q(1:3,1:3) = 50*eye(3); Q(4:5,4:5) = eye(2); Q(6,6) = 20;
R = eye(n_control,n_control);


% initial state and condition constraint
st = X(:,1);
g = [g; st-P(1:n_state)];

% compute objective symbolically
for k = 1:N
    st = X(:,k); con = U(:,k);
    xref = P(n_state+1:end);
    obj = obj+(st-xref)'*Q*(st-xref)+con'*R*con;
    % calculate x(k+1) using numerical integration (RK4)
    st_next = X(:,k+1);
    k1 = f(st,con);
    k2 = f(st + dt/2*k1, con);
    k3 = f(st + dt/2*k2, con);
    k4 = f(st + dt*k3, con);
    st_next_rk4 = st + dt/6*(k1+2*k2+2*k3+k4);      % predicted states
    g = [g;st_next-st_next_rk4];
end

% reshape the decision variables into one column vector
OPT_variables = [reshape(X,n_state*(N+1),1); reshape(U,n_control*N,1)];

% Define NLP problem
nlp_prob = struct('f',obj,'x',OPT_variables,'g',g,'p',P);

% NLP solver settings
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level = 0; % 0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver','ipopt',nlp_prob,opts);

args = struct;
% Equality constraints (dynamics equation)
args.lbg(1:n_state*(N+1)) = 0;
args.ubg(1:n_state*(N+1)) = 0;

% state constraints (unconstrained)
args.lbx(1:n_state*(N+1)) = -inf;
args.ubx(1:n_state*(N+1)) = inf;

% control constraints
switch uConstraint 
    case 0 % unconstrained
        umax = inf*ones(4,1);
        umin = -inf*ones(4,1);
    case 1 % constrained
        umax = [uUpper;uUpper;uUpper;uUpper];
        umin = [uLower;uLower;uLower;uLower];
end

args.lbx(n_state*(N+1)+1:n_control:n_state*(N+1)+n_control*N) = umin(1);
args.ubx(n_state*(N+1)+1:n_control:n_state*(N+1)+n_control*N) = umax(1);
args.lbx(n_state*(N+1)+2:n_control:n_state*(N+1)+n_control*N) = umin(2);
args.ubx(n_state*(N+1)+2:n_control:n_state*(N+1)+n_control*N) = umax(2);
args.lbx(n_state*(N+1)+3:n_control:n_state*(N+1)+n_control*N) = umin(3);
args.ubx(n_state*(N+1)+3:n_control:n_state*(N+1)+n_control*N) = umax(3);
args.lbx(n_state*(N+1)+4:n_control:n_state*(N+1)+n_control*N) = umin(4);
args.ubx(n_state*(N+1)+4:n_control:n_state*(N+1)+n_control*N) = umax(4);

%% ================== The simulation loop start here ======================
t0 = 0; tf = 20;
x0 = [0;0;0;0;0;0;0;0;0;0;0;0];
xd = 3; yd = 0; zd = 5;
xs = [xd;yd;zd;0;0;0;0;0;0;0;0;0];

xx(:,1) = x0; % store the state history
t(1) = t0;    % time series

u0 = zeros(N,n_control); % four control inputs
X0 = repmat(x0,1,N+1)';  % initialization of the states decision variables

sim_time = tf+dt;        % maximum simulation time
tol = 1e-2;    % error tolerance

mpciter = 0;
xx1 = [];
u_cl = [];

% main simulation loop - it works as long as the error is greater than tol
% and the number of MPC steps is less than its maximum value
while ( norm((x0-xs),2) > tol && mpciter < sim_time / dt)
    args.p = [x0;xs];   % set the values of the parameters vector
    % initialize the optimization variables
    args.x0 = [reshape(X0',n_state*(N+1),1); reshape(u0',n_control*N,1)];
    sol = solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,...
          'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
    % get the control inputs from the solution
    u = reshape(full(sol.x(n_state*(N+1)+1:end))',n_control,N)';
    % get state trajectory
    xx1(:,1:n_state,mpciter+1) = reshape(full(sol.x(1:n_state*(N+1)))',n_state,N+1)';
    % apply only the first element of control sequence
    u_cl = [u_cl; u(1,:)];
    t(mpciter+1) = t0;
    % Update the control and shift the solution
    [t0,x0,u0] = shift(dt,t0,x0,u,f);
    xx(:,mpciter+2) = x0;
    % get solution trajectory
    X0 = reshape(full(sol.x(1:n_state*(N+1)))',n_state,N+1)';
    % shift trajectory to initializa the next step
    X0 = [X0(2:end,:); X0(end,:)];
    mpciter = mpciter + 1;
end
ss_error = norm((x0-xs),2);

%%======================= Draw Point Stabilization response ==========================
PlotSystemResponse;
%%========================== Animation ====================================
AnimationTrajectory;