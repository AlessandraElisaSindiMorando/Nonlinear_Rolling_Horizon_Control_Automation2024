% n. state variables
nx = 12;
% n. output variables
ny = 12;
% n. control variables
nu = 4;
% Create the NMPC object
nlmpcobj = nlmpc(nx, ny, nu);

% NMPC State Evolution Model 
nlmpcobj.Model.StateFcn = "UAVStateFnc";
% (the Jacobian is passed to speed up computations)
nlmpcobj.Jacobian.StateFcn = @UAVStateJacobianFcn;
% NMPC Output Evolution Model 
nlmpcobj.Model.OutputFcn = 'UAVOutputFcn';
nlmpcobj.Jacobian.OutputFcn = @UAVOutputJacobianFcn;

% NMPC sampling time
Ts = 0.1;% [s]
% NMPC prediction horizon
Hp = 10;
% NMPC control horizon
Hu = 2;

% set object fields
nlmpcobj.Ts = Ts;
nlmpcobj.PredictionHorizon = Hp;
nlmpcobj.ControlHorizon = Hu;

% Structural Parrot Bebop 2.0 boundaries 
max_tilt_angle = 30*pi/180;% [rad]
max_vert_speed = 1;% [ms^{-1}]
max_rot_speed = 100*pi/180;% [rad]

% State Bounds
% z1 
nlmpcobj.OutputVariables(5).Min = 0;
% z2
nlmpcobj.OutputVariables(6).Min = -max_vert_speed;
nlmpcobj.OutputVariables(6).Max = max_vert_speed;
% theta1
nlmpcobj.OutputVariables(7).Min = -max_tilt_angle;
nlmpcobj.OutputVariables(7).Max = max_tilt_angle;
% phi1 
nlmpcobj.OutputVariables(9).Min = -max_tilt_angle;
nlmpcobj.OutputVariables(9).Max = max_tilt_angle;
%psi2
nlmpcobj.OutputVariables(12).Min = -max_rot_speed;
nlmpcobj.OutputVariables(12).Max = max_rot_speed;

% State's weigths Q_{UAV}
q_UAV = [100 0 100 0 100 0 0 0 0 0 100 0];
nlmpcobj.Weights.OutputVariables = q_UAV;
