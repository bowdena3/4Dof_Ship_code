% ship4dofSim.m     mail     30/06/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of a roro ship in 4 degrees of 
% freedom under simple PID control.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean up:
clear;
close all;

%% Initialization:
% Load the previously generated parameters (see setUpDynamicModel3dof.m):
load('C:\Users\arthu\OneDrive\Documents\MSC Individual\code\code 4dof\data\roro4dof.mat');

% Define the time step and total simulation time:
tStep = 0.01;       % time step length (s)
tEnd  = 100;        % end time (s)

% Set the initial conditions:
ics = zeros(8,1);

%% Control settings:

% LQR K matrix
K_gain = [1.98420785910168e-05,-3.34104942959699e-08,6.00097102863763e-07,-6.10674199385264e-05;...
         0.0532741124749190,1.11137705265764,0.187182518765656,-382.434921145794;...
         -0.469842150842932,-0.0212473843851555,0.0410847712700009,7.54923264942698;...
         -0.532732282193882,-0.0261933549022078,0.0788207513290464,1.75238151746293]
% load('C:\Users\arthu\OneDrive\Documents\MSC Individual\code\code 4dof\data\rorocontrol.mat');
% K_gain = LQR.K_gain;

% Define constant thrust and rudder angle: - change as desired
Wv = 10;                % Prevailing wind speed [m/s]
W_theta = 90;           % Prevailing wind angle [currently degrees]
W_delta = deg2rad(70);  % Wing angle [rad]
R_delta = deg2rad(0);  % Rudder angle [rad]
Thrust = 1e6;           % propeller thrust [N]

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'ship4dof';
% Load the Simulink file:vek
load_system(sfile);

%% Run the first shot:
sout = sim(sfile,'StopTime',num2str(tEnd));

%% Close the Simulink file:
close_system(sfile);
toc;

%% Post-processing:
% Extract the data to be plotted:
t = sout.tout;
x = sout.get('logsout').getElement('states').Values.Data;
v = sout.get('logsout').getElement('velocity').Values.Data;
% u = sout.get('logsout').getElement('input').Values.Data;
% p = sout.get('logsout').getElement('angle').Values.Data;

% Plot the AUV's motions:
plotMotions(t,x(:,1:4),v);
% Plot the AUV's control input:
% plotControl(t,u(:,1),rad2deg(u(:,2)),x(:,7));

