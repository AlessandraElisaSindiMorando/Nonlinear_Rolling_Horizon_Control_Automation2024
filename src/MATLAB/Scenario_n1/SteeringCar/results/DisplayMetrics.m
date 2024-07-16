function DisplayMetrics(TrackErr, PredErr, EstErr)
% PLOTRESULTS This function displays the root means square errors for the 
% tracking, estimation, and prediction.
%% Compute metrics
disp('Tracking Err.');
computeResultingErrors(TrackErr);
disp('Prediction Err.')
computeResultingErrors(PredErr);
disp('Estimation Err.')
computeResultingErrors(EstErr);
end

