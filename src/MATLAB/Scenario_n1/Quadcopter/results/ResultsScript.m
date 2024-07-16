% To remove all from the Workspace and console
clear; clc;
% To close all figures
close all
%=========================================================================

% Load all the data of the two simulations
% + the results related to the quadcopter simulation
load('out_Scenario1.mat');
% + the resulting state related to the ground robot simulation
load('XSim_Scenario1.mat')

% UAV results
%-------------------------------------------------------------------------
% Extract data from the out variable

% x,y,z, and psi state of the quadcopter over the simulation sampled every
% 0.1 seconds
XYZPsi = out.XYZPsi.Data;

% x,y,z, and psi of the quadcopter reference over the simulation sampled 
% every 0.1 seconds
XYZPsiRef = out.XYZPsiRef.Data;

% x,y,z, and psi of the quadcopter reference over the simulation sampled
% every 0.1 seconds
Reference = reshape(out.Ref.Data,[3,1201]);

% Tracking error with reference defined in (46) r_{T,UAV}(k) over the 
% simulation horizon sampled every 0.1 seconds
XYZPsiErrRef = out.XYZPsiErrRef.Data;

% Tracking error with reference defined in (45) r_{UAV}(k) over the 
% simulation horizon sampled every 0.1 seconds
XYZPsiErrPred = out.XYZPsiErrPred.Data;

% Entire quadcopter's state over the simulation horizon sampled every 
% 0.1 seconds
XUAV = out.XUAVSim.Data;

% Entire quadcopter's estimated state over the simulation horizon sampled 
% every 0.1 seconds
XUAVBar = reshape(out.XUAVbar.Data,[12,1201])';

% Extract the UGV (x,y)-position over the ground robot simulation horizon
% sampeld every 0.1 seconds
UGVXY = XSim(:,1:2);

% Plot in an image the trajectory of the two agents compared to the
% (x,y)-reference trajectory and display the metrics defined in (45), (46),
% and (48).
PlotResults(XYZPsi,Reference,UGVXY,...
    XYZPsiErrRef, XYZPsiErrPred, ...
    XUAV, XUAVBar)