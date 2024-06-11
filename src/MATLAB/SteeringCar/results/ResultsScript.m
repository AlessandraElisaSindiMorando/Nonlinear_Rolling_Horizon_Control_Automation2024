% To remove all from the Workspace and console
clear; clc;
% To close all figures
close all
%=========================================================================


% Retrieve all data from out variable in the .mat file
load('out.mat')

% Reference of the UGV over the entire simulation horizon sampled with sample 
% time Ts = 0.1s
Ref = reshape(out.Ref.Data,4,1201);

% State of the UGV over the entire simulation horizon sampled with sample 
% time Ts = 0.1s
XSim = out.XSim.Data;
% Tracking error over the entire simulation horizon sampled with sample 
% time Ts = 0.1
TrackErr = reshape(out.TrackErr.Data,4,1201);
% Prediction error over the entire simulation horizon sampled with sample 
% time Ts = 0.1
PredErr = reshape(out.PredErr.Data,4,1201);
% Estimation error over the entire simulation horizon sampled with sample 
% time Ts = 0.1
EstErr = reshape(out.EstErr.Data,4,1201);

% Plot all results in dedicated figures
PlotResults(Ref,XSim,TrackErr, PredErr, EstErr)