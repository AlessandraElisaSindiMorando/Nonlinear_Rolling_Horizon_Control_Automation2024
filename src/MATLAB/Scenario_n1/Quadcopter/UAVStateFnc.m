function [x_dot_k] = UAVStateFnc(x_k,u_k)
% State function (expressing the state derivatives as a function of 
% current state and input) in continous time used by the the Nonlinear 
% Model Predictive Controller object

x2 = x_k(2,1);
y2 = x_k(4,1);
z2 = x_k(6,1);
theta1 = x_k(7,1);
theta2 = x_k(8,1);
phi1 = x_k(9,1);
phi2 = x_k(10,1);
% psi1 = x_k(11,1);
psi2 = x_k(12,1);

u1 = u_k(1,1);
u2 = u_k(2,1);
u3 = u_k(3,1);
u4 = u_k(4,1);

m = 0.5;
g = 9.81;


x_dot_k = [x2
           -(u1/m)*sin(theta1)
           y2
           (u1/m)*cos(theta1)*sin(phi1)
           z2
           (u1/m)*cos(theta1)*cos(phi1)-g
           theta2
           u2
           phi2
           u3
           psi2
           u4];

end

