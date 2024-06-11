function computeResultingEstimationErrors(XUAV,XUAVBar)
%COMPUTERESULTINGESTIMATIONERRORS Compute the Root Mean Square Error (RMSE)
% for all quadcopter's components of the estimation error which is the
% difference between the actual state XUAV and the estimated one XUAVBar
%=========================================================================
% Estimation error over the entire simulation
XErr = XUAVBar - XUAV;
RMSEx1 = sqrt(mean((XErr(:,1)).^2));
RMSEx2 = sqrt(mean((XErr(:,2)).^2));
RMSEy1 = sqrt(mean((XErr(:,3)).^2));
RMSEy2 = sqrt(mean((XErr(:,4)).^2));
RMSEz1 = sqrt(mean((XErr(:,5)).^2));
RMSEz2 = sqrt(mean((XErr(:,6)).^2));
RMSEtheta1 = sqrt(mean((XErr(:,7)).^2));
RMSEtheta2 = sqrt(mean((XErr(:,8)).^2));
RMSEphi1 = sqrt(mean((XErr(:,9)).^2));
RMSEphi2 = sqrt(mean((XErr(:,10)).^2));
RMSEpsi1 = sqrt(mean((XErr(:,11)).^2));
RMSEpsi2 = sqrt(mean((XErr(:,12)).^2));
fprintf('RMSEx1 = %s RMSEx2 = %s RMSEy1 = %s RMSEy2 = %s RMSEz1 = %s RMSEz2 = %s\n',...
    RMSEx1,RMSEx2, RMSEy1, RMSEy2 ,RMSEz1, RMSEz2);
fprintf('RMSEtheta1 = %s RMSEtheta2 = %s RMSEphi1 = %s RMSEphi2 = %s RMSEpsi1 = %s RMSEpsi2 = %s\n',...
    RMSEtheta1,RMSEtheta2, RMSEphi1, RMSEphi2 ,RMSEpsi1, RMSEpsi2);
end