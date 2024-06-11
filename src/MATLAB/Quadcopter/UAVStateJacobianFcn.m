function [A,B] = UAVStateJacobianFcn(x_k,u_k)
% Jacobian of the state function (expressing the state derivatives as a 
% function of current state and input) in continous time used by 
% the Nonlinear Model Predictive Controller object
theta1 = x_k(7,1);
phi1 = x_k(9,1);

u1 = u_k(1,1);

m = 0.5;
g = 9.81;

A = [0 1    0 0     0 0     0                         0       0                              0     0 0
     0 0    0 0     0 0 -(u1/m)*cos(theta1)           0       0                              0     0 0
     0 0    0 1     0 0     0                         0       0                              0     0 0
     0 0    0 0     0 0 -(u1/m)*sin(theta1)*sin(phi1) 0      +(u1/m)*cos(theta1)*cos(phi1)   0     0 0
     0 0    0 0     0 1     0                         0       0                              0     0 0
     0 0    0 0     0 0 -(u1/m)*sin(theta1)*cos(phi1) 0      -(u1/m)*cos(theta1)*sin(phi1)   0     0 0
     0 0    0 0     0 0     0                         1       0                              0     0 0
     0 0    0 0     0 0     0                         0       0                              0     0 0
     0 0    0 0     0 0     0                         0       0                              1     0 0
     0 0    0 0     0 0     0                         0       0                              0     0 0
     0 0    0 0     0 0     0                         0       0                              0     0 1
     0 0    0 0     0 0     0                         0       0                              0     0 0
     ];

B = [0                              0       0       0   
    -(1/m)*sin(theta1)              0       0       0   
    0                               0       0       0   
    +(1/m)*cos(theta1)*sin(phi1)    0       0       0   
    0                               0       0       0   
    +(1/m)*cos(theta1)*cos(phi1)    0       0       0   
    0                               0       0       0   
    0                               1       0       0   
    0                               0       0       0   
    0                               0       1       0   
    0                               0       0       0   
    0                               0       0       1   
    ];

end