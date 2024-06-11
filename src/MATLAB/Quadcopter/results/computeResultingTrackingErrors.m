function computeResultingTrackingErrors(XErr)
%COMPUTERESULTINGTRACKINGERRORS Compute the Root Mean Square Error (RMSE) 
% for all tracked state components (x,y,z,psi) starting from a matrix of 
% tracking errors XErr
%=========================================================================

RMSEx = sqrt(mean((XErr(:,1)).^2));
RMSEy = sqrt(mean((XErr(:,2)).^2));
RMSEz = sqrt(mean((XErr(:,3)).^2));
RMSEpsi = sqrt(mean((XErr(:,4)).^2));
fprintf('RMSEx = %s RMSEy = %s RMSEz = %s RMSEpsi = %s\n',...
    RMSEx,RMSEy,RMSEz,RMSEpsi);
end