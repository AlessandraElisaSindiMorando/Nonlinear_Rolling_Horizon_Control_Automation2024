function [X_sol] = MHE_RK_CasaDi(y_measurements,u_cl,X0)
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

% Import CasADi v3.1.1 files. 
% ! Path **TO BE CHANGED** according to the folder structures and the
% distribution (see https://web.casadi.org/get/)
addpath('../../casadi-3.6.5-linux64-matlab2018b/')
import casadi.*

% Sampling time 
Ts = 0.1; %[s]

% Estimation horizon
N_MHE = 5;

% State variables
%==========================================================================
% In this case, the state variables are the ground vehicle pose (x,y,theta)
% and its forward velocity v
x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta'); v = SX.sym('v');

% State vector
states = [x;y;theta;v]; 

% Number of state variables
n_states = length(states);

% Control inputs
%=========================================================================
% In this case, the control variables are the steering angle w1 and the
% forward acceleration w2
w1 = SX.sym('w1'); w2 = SX.sym('w2');

% Control vector
controls = [w1;w2]; 

% Number of control inputs
n_controls = length(controls);

% State evolution function
%=========================================================================
%   x_{k+1} = f(x_k, u_k)

% Distance between the front and the rear wheels 
L = 0.14;% [m]

% System r.h.s, i.e., f(x_k, u_k)
rhs = [v*cos(theta);v*sin(theta);(v/L)*tan(w1);w2];

% Nonlinear mapping function f(x,u)
f = Function('f',{states,controls},{rhs});

% Measurement function
%=========================================================================
%   y_k = h(x_k, u_k)

% Measurement model r.h.s, i.e., h(x_k, u_k)
measurement_rhs = [x;y;theta];

% Number of measurements
n_measurement = length(measurement_rhs);

% Nonlinear mapping function h(x,u)
h = Function('h',{states},{measurement_rhs});

% Decision variables
%=========================================================================
% The decision variable is the estimated state over the estimation horizon
X = SX.sym('X',n_states,(N_MHE+1));
% Make the decision variable one column  vector
OPT_variables = [reshape(X,n_states*(N_MHE+1),1)];

% Optimization parameters
%=========================================================================
% The parameters are measurements and applied control (known).
% In this case we stack them, the control u(N_MHE+1) will not be used.
P = SX.sym('P', n_measurement+n_controls, N_MHE+1);

% Objective function
%=========================================================================
% It's assumed that the measures provided by the motion capture system are
% affected by zero-mean white noise with known variance matrix
meas_cov = diag([0.02e-3; 0.02e-3; 0.1*pi/180].^2);

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
opts.ipopt.print_level = 0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_mhe, opts);

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
% hence g_{lb} = g_{ub} = 0
args.lbg(1:n_states*(N_MHE)) = 0;
args.ubg(1:n_states*(N_MHE)) = 0;

% State margin upper-lower bounds
%-------------------------------------------------------------------------
%   x_{lb} <= x <= x_{ub}

% State x lower and upper bound
args.lbx(1:n_states:n_states*(N_MHE+1),1) = -inf; 
args.ubx(1:n_states:n_states*(N_MHE+1),1) = inf;

% State y lower and upper bound
args.lbx(2:n_states:n_states*(N_MHE+1),1) = -inf; 
args.ubx(2:n_states:n_states*(N_MHE+1),1) = inf; 

% State theta lower and upper bound
args.lbx(3:n_states:n_states*(N_MHE+1),1) = -inf; 
args.ubx(3:n_states:n_states*(N_MHE+1),1) = inf;

% State v lower and upper bound (actually is the only bounded quantity)
v_max = 0.3;% [m/s]
args.lbx(4:n_states:n_states*(N_MHE+1),1) = -v_max;
args.ubx(4:n_states:n_states*(N_MHE+1),1) = v_max;

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

end