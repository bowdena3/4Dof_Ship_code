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
load('roro4dof.mat');

% Define the time step and total simulation time:
tStep = 0.01;       % time step length (s)
tEnd  = 100;        % end time (s)

% Set the initial conditions:
ics = zeros(8,1);

%% Control settings:
% % Desired paramters:
% u_d = 1.5;    % desired forward speed (m/s)
% z_d = 1;    % desired depth (m)
% 
% % PID control settings:
% kp_u = 200;
% ki_u = 5;
% kd_u = 10;
% kp_z = 50;
% ki_z = 3;
% kd_z = 6;
% kp_t = 20;
% ki_t = 2;
% kd_t = 4;
% 
% % Set the time step for the PID controller:
% dtPID = 0.1;
% 
% % Define the saturation limits:
% lim.torque = 100;
% lim.angle  = deg2rad(15);

% LQR K matrix
% K_gain = [-3.05987952929453e-21,-0.0342276115016287,-0.999998301935798,105.985826133624,0.00456884029400482,-0.00184111780549751,-1.30747787936369,58.1152174358923;...
%          1.38946463867309e-30,1.08439425840482e-11,2.03746441663701e-14,-2.04164103638497e-07,1.45855467513394e-11,-4.75336023103087e-14,2.15456212592738e-14,1.49489809672734e-09;...
%          -1.86836097544575e-21,18.4973282418139,-0.00184736911647252,68198.3673044216,-0.214589656603202,0.199266211091638,-0.00241445033372992,3.97304420160200];

load('rorocontrol.mat');
K_gain = LQR.K_gain;

% Define constant thrust and rudder angle: - change as desired
Wv = 10;                % Prevailing wind speed [m/s]
W_theta = 90;           % Prevailing wind angle [currently degrees]
W_delta = deg2rad(70);  % Wing angle [rad]
R_delta = deg2rad(0);  % Rudder angle [rad]
Thrust = 1e6;           % propeller thrust [N] 1e6

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

