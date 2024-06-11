function computeResultingErrors(XErr)
%COMPUTERESULTINGERRORS Compute the Root Mean Square Error (RMSE) for all state
% components starting from a matrix of errors XErr
RMSEx = sqrt(mean((XErr(1,:)).^2));
RMSEy = sqrt(mean((XErr(2,:)).^2));
RMSEtheta = sqrt(mean((XErr(3,:)).^2));
RMSEv = sqrt(mean((XErr(4,:)).^2));
fprintf('RMSEx = %s RMSEy = %s RMSEtheta = %s RMSEv = %s\n',...
    RMSEx,RMSEy,RMSEtheta,RMSEv);
end