function [x_seq, mv] = NonLinear_MPC_with_Obstacles_CasaDi(x0, xs, u0, ...
    detected, Xobs)
%NONLINEAR_MPC_WITH_OBSTACLES_CASADI This function implement a 
% Nonlinear Model Predictive Control (NMPC). In particular
% * the 4th order Runge-Kutta method is used to discretize the continous 
% time non-linear system 
% * the IPOPT solver is used to solve the non-linear optimization problem 
% (for furter details see:
% https://web.casadi.org/docs/#nonlinear-programming)
%----------------------------------------------------------------------------
%Input:
%   x0: the current state
%   xs: the time-varying reference over the prediction horizon
%   u0: the previous control action
%   detected: flag that is 1 if an obstacle is detected, 0 otherwise
%   Xobs: (x,y)-position of the detected obstacle, it should be passed
%**ALWAYS** a value, it is used only if detected is 1
%Ouput:
%   x_seq: the optimal state sequence over the prediction horizon
%   mv: the first value of the optimal control sequence 
%============================================================================

% Import CasADi files. 
% ! Path **TO BE CHANGED** according to the folder structures and the
% distribution (see https://web.casadi.org/get/)
addpath('../../casadi-3.6.5-linux64-matlab2018b/')
import casadi.*

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

% Prediction horizon
Hp = 10;

% Control horizon
Hu = 2;

% Decision variables
%=========================================================================
% The decision variable is the estimated state over the prediction horizon
X = SX.sym('X',n_states,(Hp+1));

% The control input over the control horizon
U = SX.sym('U',n_controls,Hu);

% Parameters
%=========================================================================
% The parameter vector contains : 
P = SX.sym('P',n_states + n_controls + n_states* (Hp+1),1);
% * the current state at instant k, x_k
x_k = P(1:n_states);
% * the previous control action, u_k
u_prev_k = P(n_states+1:n_states+n_controls);
% * the constant reference over the prediction horizon r
r = reshape(P(n_states+n_controls+1:end,1),[n_states, (Hp+1)]);

% Objective function
%=========================================================================
% State weighing matrices
%------------------------------------------------
Q = diag([2000*10 2000*10 1000 1000]);

% Control weight matrices
%------------------------------------------------
R = zeros(2,2); R(1,1) = 10; R(2,2) = 10;

% Control variations weight matrices
%------------------------------------------------
DeltaR = zeros(2,2); DeltaR(1,1) = 100; DeltaR(2,2) = 100;

% Construction of the objective function
obj = 0;
% * first sum of (27a)
for i = 1:Hp+1
    %     obj. state penalty term
   
    obj = obj + (X(:,i)-r(:,i))'*Q*(X(:,i)-r(:,i));
end
% * second sum of (27a)
for i = 1:Hu
    %     obj. control penalty term
    obj = obj + U(:,i)'*R*U(:,i);
end
% * third sum of (27a)
obj = obj + (U(:,1)-u_prev_k)'*DeltaR*(U(:,1)-u_prev_k);
for i = 1:(Hu-1)
    %     obj. control variations penalty term
    obj = obj + (U(:,i+1)-U(:,i))'*DeltaR*(U(:,i+1)-U(:,i));
end

% Constraints
%=========================================================================
% Initialize the constraints vector
g = [];

% Evolution constraints (27c)
%-------------------------------------------------------------------------
% Distance between the front and the rear wheels 
L = 0.14;% [m]

% System r.h.s, i.e., f(x_k, u_k)
rhs = [v*cos(theta);v*sin(theta);(v/L)*tan(w1);w2];

% Nonlinear mapping function f(x,u)
f = Function('f',{states,controls},{rhs});

% Sampling time 
Ts = 0.1; %[s]

% For each instant k in the estimation window
for k = 1:Hp
    % Predicted state at instant k+1 (which is a decision variable) x_{k+1}
    st_next = X(:,k+1); 
    % Control applied at instant k (which is known) u_{k}
    con = U(:,min(k,Hu)); 
    % Predicted state at instant k (which is a decision variable) x_{k}
    st = X(:,k);
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

% Initial State (27b)
%-------------------------------------------------------------------------
g = [g; X(:,1)-x_k];

% Terminal region (27d) and (31)
%-------------------------------------------------------------------------
g = [g; (X(1:2,Hp+1) - r(1:2,Hp+1))'*(X(1:2,Hp+1) - r(1:2,Hp+1))];

% Obstacle avoidance (32) and (33)
%-------------------------------------------------------------------------
% If a barrier is encountered, the controller is switched and a further
% family of constraints is added to the NMPC formulation
if detected > 0
    for k = 1:(Hp+1)
        g = [g; (X(1:2,k) - Xobs)'*(X(1:2,k) - Xobs)];
    end
end

% Decision variables
%=========================================================================
% Multiple shooting: both the state x and the control u are optimization
% variables. The IPOPT solver interface requires the decision variables to
% be a column vector.
OPT_variables = [reshape(X,n_states*(Hp+1),1);...
    reshape(U,n_controls*Hu,1)];

% IPOPT Options 
%=========================================================================
% The IPOPT solves a parametric NLPs of the following form 
%       min_{x} f(x,p)
%       subject to: x_{lb} <= x <= x_{ub}
%                   g_{lb} <= g(x,p) <= g_{ub}
% where x is the decision variable and p is a known parameter vector 
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level = 0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

% IPOPT solver arguments 
%=========================================================================
args = struct;

% State margin upper-lower bounds
%-------------------------------------------------------------------------
%   x_{lb} <= x <= x_{ub}

% State x lower and upper bound
args.lbx(1:n_states:n_states*(Hp+1),1) = -inf;
args.ubx(1:n_states:n_states*(Hp+1),1) = inf;

% State y lower and upper bound
args.lbx(2:n_states:n_states*(Hp+1),1) = -inf;
args.ubx(2:n_states:n_states*(Hp+1),1) = inf; 

% State theta lower and upper bound
args.lbx(3:n_states:n_states*(Hp+1),1) = -inf;
args.ubx(3:n_states:n_states*(Hp+1),1) = inf;

% State v lower and upper bound (actually is the only bounded quantity),
% (37)
v_max = 0.3;% [m/s]
args.lbx(4:n_states:n_states*(Hp+1),1,1) = -v_max;
args.ubx(4:n_states:n_states*(Hp+1),1,1) = v_max;

% Control upper-lower bounds
%-------------------------------------------------------------------------
%   u_{lb} <= u <= u_{ub}
w1_min = -0.30;% [rad]
w1_MAX = 0.80;% [rad]
w2_MAX = 1;% [ms^{-2}]

% Control input w1 lower and upper bound (39a)
args.lbx(n_states*(Hp+1)+1:n_controls:n_states*(Hp+1)+n_controls*Hu,1) =...
    w1_min;
args.ubx(n_states*(Hp+1)+1:n_controls:n_states*(Hp+1)+n_controls*Hu,1) = ...
    w1_MAX;

% Control input w2 lower and upper bound (39b)
args.lbx(n_states*(Hp+1)+2:n_controls:n_states*(Hp+1)+n_controls*Hu,1) = ....
    -w2_MAX;
args.ubx(n_states*(Hp+1)+2:n_controls:n_states*(Hp+1)+n_controls*Hu,1) = ...
    w2_MAX;

% Constraints g 'bounds'
%-------------------------------------------------------------------------
%   g_{lb} <= g(x,p) <= g_{ub}

% + Evolution constraints
%   ---------------------------------------------------------------------
% Evolution constraints are equality constraints
%   x_{k+1} - f(x_k, u_k) = 0
% hence g_{lb} = g_{ub} = 0
args.lbg(1:n_states*(Hp+1)) = 0;
args.ubg(1:n_states*(Hp+1)) = 0;

% + Terminal region 
%   ---------------------------------------------------------------------
% Lower and upper bounds of terminal region constraints (28d)
% The robot should keep a safety distance d_{Safe}
d_Safe = 0.25;% [m]
% Margin 
epsilon = 0.05;% [m]
args.lbg(n_states*(Hp+1)+1) = 0;
args.ubg(n_states*(Hp+1)+1) = (d_Safe+epsilon)^2;
args.lbg(n_states*(Hp+1)+1) = 0;
args.ubg(n_states*(Hp+1)+1) = (d_Safe+epsilon)^2;


% + Obstacle Avoidance
%   ---------------------------------------------------------------------
% If a barrier is encountered, the controller is switched and a further
% family of constraints is added to the NMPC formulation. Hence also the
% lower and upper bounds of (32) should be defined
if detected > 0
    args.lbg(n_states*(Hp+1)+1+1:...
        (1+n_states)*(Hp+1)+1) = (d_Safe+epsilon)^2;
    args.ubg(n_states*(Hp+1)+1+1:...
        (1+n_states)*(Hp+1)+1) = inf;
end

% Parameter
%-------------------------------------------------------------------------
% The IPOPT solver interface requires the parameters to be a column vector.
ref = reshape(xs,[n_states * (Hp+1),1]);
args.p   = [x0;u0;ref];

% Initial guess
%-------------------------------------------------------------------------
% The IPOPT solver interface requires the initial guess to be a column 
% vector.
X0 = repmat(x0,1,Hp+1)';
U0 = repmat(u0,1,Hu)';
args.x0  = [reshape(X0',n_states*(Hp+1),1);
    reshape(U0,n_controls*Hu,1)];

% Solve
%=========================================================================
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

% Check constraints satisfaction
constraints_at_solution = full(sol.g);
if all(constraints_at_solution > 0)
    warning('Constraints not satisfied. The solution may be infeasible.');
end

% Extract the optimal control sequence over the control horizon
u = reshape(full(sol.x(n_states*(Hp+1)+1:end))',n_controls,Hu)';

% Return values, i.e., the first value of the optimal control sequence mv
% and the optimal state sequence over the prediction horizon
mv = u(1,:)';
x_seq = reshape(full(sol.x(1:n_states*(Hp+1)))',n_states,Hp+1)';
end

