function [C,D] = UAVOutputJacobianFcn(x_k,u_k)
% Jacobians of the ouput continous-time function used by the Nonlinear 
% Model Predictive Controller object
[nx,~] = size(x_k);
[nu,~] = size(u_k);

C = eye(nx);
D = zeros(nx,nu);
end

