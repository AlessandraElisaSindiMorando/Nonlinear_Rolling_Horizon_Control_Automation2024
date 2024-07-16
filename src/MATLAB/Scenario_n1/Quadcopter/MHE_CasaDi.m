function [X_sol] = MHE_CasaDi(y_measurements,u_cl,X0)
%MHE_RK_CASADI This function implement a Moving Horizon Estimator (MHE). In
%particular
% * the 4th order Runge-Kutta method is used to discretize the continous 
% time non-linear system 
% * the IPOPT solver is used to solve the non-linear optimization problem 
% (for furter details see: https://web.casadi.org/docs/#nonlinear-programming
%----------------------------------------------------------------------------
%Input:
%   y_measurements : batch of past measurements over the estimation horizon
%   u_cl: batch of past control inputs applied over the estimation horizon
%   X0: initial guess used by the solver (from the previous instant)
%Ouput:
%   X_sol: optimal state estimate sequence over the estimation window 
%============================================================================

% Import CasADi files. 
% ! Path **TO BE CHANGED** according to the folder structures and the
% distribution (see https://web.casadi.org/get/)
addpath('../../casadi-3.6.5-linux64-matlab2018b/');
import casadi.*

% Sampling time 
Ts = 0.1; %[s]

% Estimation horizon
N_MHE = 5;

% State variables
%==========================================================================
% In this case, the state variables are the its position and translational
% velocity w.r.t the global reference frame ...
x1 = SX.sym('x1'); x2 = SX.sym('x2');
y1 = SX.sym('y1'); y2 = SX.sym('y2');
z1 = SX.sym('z1'); z2 = SX.sym('z2');
% ...and its orientation expressed in terms of Euler angles and rotational
% velocities w.r.t the body frame {X_B, Y_B, Z_B}
theta1 = SX.sym('theta1'); theta2 = SX.sym('theta2');
phi1 = SX.sym('phi1'); phi2 = SX.sym('phi2');
psi1 = SX.sym('psi1'); psi2 = SX.sym('psi2');

% State vector
states = [x1; x2; y1; y2; z1; z2;...
    theta1; theta2; phi1; phi2; psi1; psi2];

% Number of state variables
n_states = length(states);

% Control inputs
%=========================================================================
u1 = SX.sym('u1');
u2 = SX.sym('u2');
u3 = SX.sym('u3');
u4 = SX.sym('u4');

% Control vector
controls = [u1;u2;u3;u4];

% Number of control inputs
n_controls = length(controls);

% State evolution function
%=========================================================================
%   x_{k+1} = f(x_k, u_k)

% drone mass
mass = 0.5;%[kg]

% gravitational acceleration
g = 9.81;%[ms^{-2}]

% System r.h.s, i.e., f(x_k, u_k)
rhs = [x2;
    -(u1/mass)*sin(theta1);
    y2;
    (u1/mass)*cos(theta1)*sin(phi1)
    z2;
    (u1/mass)*cos(theta1)*cos(phi1)-g;
    theta2;
    u2;
    phi2;
    u3;
    psi2;
    u4]; 

% Nonlinear mapping function f(x,u)
f = Function('f',{states,controls},{rhs});

% Measurement function
%=========================================================================
%   y_k = h(x_k, u_k)

% Measurement model r.h.s, i.e., h(x_k, u_k)
measurement_rhs = [x1;y1;z1;theta1;phi1;psi1];

% Number of measurements
n_measurement = length(measurement_rhs);

% Nonlinear mapping function h(x,u)
h = Function('h',{states},{measurement_rhs});

% Decision variables
%=========================================================================
% The decision variable is the estimated state over the estimation horizon
X = SX.sym('X',n_states,(N_MHE+1)); 

% Make the decision variable one column  vector
OPT_variables = [reshape(X,12*(N_MHE+1),1)];

% Optimization parameters
%=========================================================================
% The parameters are measurements and applied control (known).
% In this case we stack them, the control u(N_MHE+1) will not be used.
P = SX.sym('P', n_measurement+n_controls, N_MHE+1);

% Objective function
%=========================================================================
% It's assumed that the measures provided by the motion capture system are
% affected by zero-mean white noise with known variance matrix (the ones of
% the Optitrack)
meas_cov = diag([0.02e-3; 0.02e-3; 0.02e-3; ...
    0.1*pi/180; 0.1*pi/180; 0.1*pi/180].^2);

% Weighing matrices of the deviation from the past measured values and the
% predicted one
V = inv(sqrt(meas_cov)); 

% Construct the objective function
obj = 0; 
% For each instant k in the estimation window
for k = 1:N_MHE+1
    % Predicted state at instant k (which is a decision variable) x_{k}
    st = X(:,k);
    % Measurement corresponding to the predicted state h(x_{k})
    h_x = h(st);
    % Past measured ouput at instant k (which is known)
    y_tilde = P(1:n_measurement,k);
    % Add elemet for the current k to the sum
    obj = obj+ (y_tilde-h_x)' * V * (y_tilde-h_x);
end

% System dyanmic constraints
%=========================================================================
% The constraint vector is initialized as empty
g = []; 
% For each instant k in the estimation window
for k = 1:N_MHE
    % Predicted state at instant k (which is a decision variable) x_{k}
    st = X(:,k);  
    % Control applied at instant k (which is known) u_{k}
    con = P(n_measurement+1:end,k);
    % Predicted state at instant k+1 (which is a decision variable) x_{k+1}
    st_next = X(:,k+1);
    % Runge Kutta 4th order discretization method 
    h=Ts;
    k1 = f(st, con);  
    k2 = f(st + h/2*k1, con); 
    k3 = f(st + h/2*k2, con); 
    k4 = f(st + h*k3, con);
    st_next_RK4=st +h/6*(k1 +2*k2 +2*k3 +k4);
    % Add the constraint to the constraints vector
    % x_{k+1} = f(x_k, u_k) <=> x_{k+1} - f(x_k, u_k) = 0
    g = [g; st_next-st_next_RK4];
end

% IPOPT Options 
%=========================================================================
% The IPOPT solves a parametric NLPs of the following form 
%       min_{x} f(x,p)
%       subject to: x_{lb} <= x <= x_{ub}
%                   g_{lb} <= g(x,p) <= g_{ub}
% where x is the decision variable and p is a known parameter vector 
nlp_mhe = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level = 0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_mhe,opts);

% IPOPT solver arguments 
%=========================================================================
args = struct;

% System dynamics upper-lower bounds
%-------------------------------------------------------------------------
% The g family of constraints 
%   g_{lb} <= g(x,p) <= g_{ub}
% refers to the system dynamics which are
% equality constraints
%   x_{k+1} - f(x_k, u_k) = 0
args.lbg(1:n_states*(N_MHE)) = 0;
args.ubg(1:n_states*(N_MHE)) = 0;

% State margin upper-lower bounds
%-------------------------------------------------------------------------
%   x_{lb} <= x <= x_{ub}

% State x1 lower and upper bound
args.lbx(1:n_states:n_states*(N_MHE+1),1) = -inf;
args.ubx(1:n_states:n_states*(N_MHE+1),1) = inf;

% State x2 lower and upper bound
args.lbx(2:n_states:n_states*(N_MHE+1),1) = -inf;
args.ubx(2:n_states:n_states*(N_MHE+1),1) = inf;

% State y1 lower and upper bound
args.lbx(3:n_states:n_states*(N_MHE+1),1) = -inf;
args.ubx(3:n_states:n_states*(N_MHE+1),1) = inf;

% State y2 lower and upper bound
args.lbx(4:n_states:n_states*(N_MHE+1),1) = -inf;
args.ubx(4:n_states:n_states*(N_MHE+1),1) = inf;

% State z1 lower and upper bound
args.lbx(5:n_states:n_states*(N_MHE+1),1) = 0;
args.ubx(5:n_states:n_states*(N_MHE+1),1) = inf;

% State z2 lower and upper bound
max_vert_speed = 1;% [m/s]
args.lbx(6:n_states:n_states*(N_MHE+1),1) = -max_vert_speed;
args.ubx(6:n_states:n_states*(N_MHE+1),1) = max_vert_speed;

% State theta1 lower and upper bound
max_tilt_angle = 30*pi/180;% [rad]
args.lbx(7:n_states:n_states*(N_MHE+1),1) = -max_tilt_angle; 
args.ubx(7:n_states:n_states*(N_MHE+1),1) = max_tilt_angle;

% State theta2 lower and upper bound
args.lbx(8:n_states:n_states*(N_MHE+1),1) = -inf;
args.ubx(8:n_states:n_states*(N_MHE+1),1) = inf;

% State phi1 lower and upper bound
args.lbx(9:n_states:n_states*(N_MHE+1),1) = -max_tilt_angle; 
args.ubx(9:n_states:n_states*(N_MHE+1),1) = max_tilt_angle;

% State phi2 lower and upper bound
args.lbx(10:n_states:n_states*(N_MHE+1),1) = -inf;
args.ubx(10:n_states:n_states*(N_MHE+1),1) = inf;

% State psi1 lower and upper bound
args.lbx(11:n_states:n_states*(N_MHE+1),1) = -inf;
args.ubx(11:n_states:n_states*(N_MHE+1),1) = inf;

% State phi2 lower and upper bound
max_rot_speed = 100*pi/180;% [rad/s]
args.lbx(12:n_states:n_states*(N_MHE+1),1) = -max_rot_speed;
args.ubx(12:n_states:n_states*(N_MHE+1),1) = max_rot_speed;

% Parameter
%-------------------------------------------------------------------------
% Get the past measurements and control over the estimation window and put 
% them as parameters in p
args.p   = [y_measurements;u_cl];

% Initial guess
%-------------------------------------------------------------------------
args.x0  = [reshape(X0',n_states*(N_MHE+1),1)];

% Solve
%=========================================================================
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

% Return the optimal state estimate sequence over the estimation window 
X_sol = reshape(full(sol.x(1:n_states*(N_MHE+1)))',n_states,N_MHE+1)';

success = solver.stats.success;
if success ~= 1
    warning(solver.stats.return_status)
end

end