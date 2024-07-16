function PlotResults(XYZPsi,Ref,UGVXY,...
    XYZPsiErrRef, XYZPsiErrPred, ...
    XUAV, XUAVBar)
%PLOTRESULTS This function plots in an image the trajectory of the UAV and
% the UGV compared to the (x,y)-reference trajectory and displays 
% the metrics defined in (45), (46), and (48).
%-------------------------------------------------------------------------
%Input
% XYZPsi : x,y,z, and psi state of the quadcopter over the simulation 
%sampled every 0.1 seconds
% Ref : x,y,z, and psi of the quadcopter reference over the 
%simulation sampled every 0.1 seconds
% UGVXY : UGV (x,y)-position over the ground robot simulation horizon 
%sampeld every 0.1 seconds
% XYZPsiErrRef : tracking error with reference defined in (44) r_{T,UAV}(k) 
%over the simulation horizon sampled every 0.1 seconds
% XYZPsiErrPred : tracking error with reference defined in (43) r_{UAV}(k) 
%over the simulation horizon sampled every 0.1 seconds
% XUAV : entire quadcopter's state over the simulation horizon sampled 
%every 0.1 seconds
% XUAVBar : entire quadcopter's estimated state over the simulation horizon
%sampled every 0.1 seconds
%=========================================================================

% Plot two robots trajectories
%-------------------------------------------------------------------------
% Open a new figure
figure1 = figure(1);

% Define two vectors representing the altitude of the ground z = 0 to be
% used in the plot of both the UGV and the reference (this last is sampled
% every 5 seconds)
ground_z = zeros(1,601);
%ground_z_5Sec = zeros(size(XYZPsiRef5Sec(:,1)));

% Plot the quadcopter trajectory
plot3(XYZPsi(:,1),XYZPsi(:,2),XYZPsi(:,3),'k-','linewidth',2,'Color',[0.6350 0.0780 0.1840]);

% Hold on to plot all the trajectories in the same plot
hold on;

% DRaw the obstacle in (-1,1) and the unsafety region which is a circle of
% radius d_{SAFE} = 0.25 m and as center the obstacle
% t1 = 24;
% scale = 2 / (3 - cos(2*t1*(2*pi/60)));
% x1 = 2 * scale * cos(t1*(2*pi/60));
% y1 = 2 * scale * sin(2*t1*(2*pi/60)) / 2;
% t2 = 54;
% scale = 2 / (3 - cos(2*t2*(2*pi/60)));
% x2 = 2 * scale * cos(t2*(2*pi/60));
% y2 = 2 * scale * sin(2*t2*(2*pi/60)) / 2;
% drawObstacle3D(x1,y1,0.25)
% drawObstacle3DNoShowed(x2,y2,0.25)


% Plot the XY-plane reference trajectory
plot3(Ref(1,:),Ref(2,:),ground_z,'k-');

% Plot the steering car trajectory
plot3(UGVXY(:,1),UGVXY(:,2),ground_z,'Color',[0.4660 0.6740 0.1880], 'LineWidth',2)

% Set the right names in the legend
lgd = legend('UAV Traj.','d_{Safe}','obstacle',...
    'XY Ref','UGV Traj.');
% Put the legend nside top-right of axes (default for 2-D axes)
lgd.Location = 'northeast';

% Add axis names labels
xlabel('X [m]','FontSize',11);
ylabel('Y [m]','FontSize',11);
zlabel('Z [m]','FontSize',11);

% Rotate around the z axis for a better view
view([-63.555233019005186,28.837854449239181])

xlim([-3,3]);ylim([-3,3]);
grid on;
legend('show');
fontsize(figure1,16,"points")

view(0, 90);

set(gcf,'position',[100,100,1000,1000])
set(gca, 'color', 'none');

% Display metrics
%-------------------------------------------------------------------------
% UAV's NMPC performances that are collected in Table II
disp('  TABLE II : UAV NMPC Performances');
% RMSE of the tracking error with respect to the reference used in the MPC 
%formulation, i.e., r_{UAV}(k) defined in (43)
disp('Tracking Pred. Err. (47)');
computeResultingTrackingErrors(XYZPsiErrPred);
% RMSE of the tracking error with respect to the actual reference, i.e., 
%r_{T,UAV}(k) defined in (44)
disp('Tracking Ref. Err. (48)');
computeResultingTrackingErrors(XYZPsiErrRef);

% UAV's MHE performances that are collected in Table III
disp('  TABLE III : UAV MHE Performances');
% RSME of the estimation error
disp('Estimation. Err.');
computeResultingEstimationErrors(XUAV,XUAVBar)
end

