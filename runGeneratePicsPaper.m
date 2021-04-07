%% Script to generate pictures for paper
clc; clear; close all;

%% Set path to initialization script initWorkspace and run it
addpath('NonLinMdl');
initWorkspace;

%% Matlab analysis: Bode plots linearized FAST models (reference)
% Bode plots of two linearized non-linear wind turbine models with linearized 
% FAST models. 

% Create Bode plots for comparison
compareLinearModels;

%% Simulink simulations 
% Simulink models are compared with FAST (NREL) references.
% FAST simulation results obtained with FASTTool (Tu Delft) are provided as
% mat-files in dataIn folder. 

%Load data if available from previous simulation.
loadData = 0;

% Run Simulink models in closed loop w baseline controller( Torque controller
% k-omega-squared, Pitch controller: Gainscheduled Pi)
figNo = 1;
runCompareModels('Sweep',loadData,figNo);
figNo = 4;
runCompareModels('NTW18',loadData,figNo);

% Run models in closed loop with qLPV MPC
figNo1 = 7;
runCompareCtrl('Sweep',loadData,figNo1);
figNo1 = 8;
runCompareCtrl('NTW18',loadData,figNo1);

