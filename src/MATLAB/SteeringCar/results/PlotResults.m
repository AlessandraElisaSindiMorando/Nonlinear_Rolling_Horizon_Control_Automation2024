function PlotResults(Ref,XSim,TrackErr, PredErr, EstErr)
% PLOTRESULTS This function shows the simulation results, in particular, 
% the UGV trajectory and its reference are plotted in a dedicated figure 
% and the root means square errors for the tracking, estimation, and 
% prediction are displayed.

% Close all figures
close all

% Obstacle (x,y)-postion
xObs = -2;% [m]
yObs = 0;% [m]

% Safety distance d_{Safe}
dSafe = 0.25;% [m]

% Construct the unsafe region defined as a circle of radius d_{Safe} and
% center the (x,y)-position of the obstacle
theta = linspace(0, 2*pi, 100);
safetyRegion_x = xObs + dSafe * cos(theta);
safetyRegion_y = yObs + dSafe * sin(theta);

% Create figure plotting the results
createaxes(XSim(:,1),XSim(:,2),...
    Ref(1,:),Ref(2,:), ...
    safetyRegion_y, safetyRegion_x, ...
    xObs, yObs)

%% Compute metrics
disp('Tracking Err.');
computeResultingErrors(TrackErr);
disp('Prediction Err.')
computeResultingErrors(PredErr);
disp('Estimation Err.')
computeResultingErrors(EstErr);
end

