% setupDynamicModel4dof.m     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is run to generate the parameters for the simulation of an
% ship in 4 DOF.
%
% The parameters of the model for the ship are taken from
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Input the parameters for the model:
% Propeller values:             %% Too remove?
prop.mf = 0.51865;     % [kg]
prop.Kn = 0.5;         % [kg m2 s-1]
prop.Jm = 1;           % [kg m2]
prop.wp = 0.2;
prop.tp = 0.1;
prop.l = 6.604;
prop.q = 16.51;

% Actuation:                    %%(not updated)
act.Tnn = 6.279e-4;   % [kg m rad-2]     % Thrust coefficient 
act.Qnn = -1.121e-5;  % [kg m2 rad-2]    % Torque coefficient 
act.Yuuds = -9.64;    % [kg m-1 rad-1]   % Sway force coefficient for rudder displacement       To change?                 
act.Muuds = -6.15;    % [kg rad-1]       % Pitch moment coefficient for elevator displacement      To change?

% Inertia values:               %
m_non = 750.81e-5;
Ixx_non = 1.32e-5;
Izz_non = 43.25e-5;
rho = 1000;
Lpp = 230.66;
Unom = 12.7;
ine.m = m_non*(2/(rho*Lpp^3));          % [kg]            % Mass of vehicle
ine.xg = -0.46e-5;     % [m]           % z-coordinate of vehicle centre of gravity
ine.zg = -3.54e-5;     % [m]           % z-coordinate of vehicle centre of gravity
ine.Ixx = Ixx_non*(2/(rho*Lpp^5));      % [kg m2]         % Inertia around axiss
ine.Izz = Izz_non*(2/(rho*Lpp^5));      % [kg m2]         % Inertia around axiss

% ** to Replace with KCS coefficnets taken from Mucha, Philipp. (2017)
% Surge cofficients:

surge.Xvv = (-1.0e-5)*(ine.m/Lpp);     % [kg m-1] *m/L
surge.Xuu = (-64.5e-5)*(ine.m/Lpp);    % [kg m-1] *m/L   Axial drag coefficient – resisting forward motion
surge.Xuuu = (-137.2e-5)*(ine.m/(Lpp*Unom));       % [kg s m-2] *m/U*L
surge.Xrr = (4.4e-5)*(ine.m*Lpp);      % [kg m rad-2] *m*L   Drag coefficient for pitching motion
surge.Xvr = (-24.0e-5)*(ine.m);        % [kg rad-1] *m
surge.Xud = (-124.4e-5)*(ine.m);       % [kg] *m       Axial added mass

% Sway coefficients:    
sway.Yv_v = (-5801.5e-5)*(ine.m/Lpp);   % [kg m-1] *m/L
sway.Yr_r = (0.0)*(ine.m*Lpp);         % [kg m rad-2] *m*L
sway.Yrrr = (-158.0e-5)*(ine.m*Lpp^2/Unom);  % [kg m s rad-3] *m*L2/u
sway.Yr_v = (-409.4e-5)*(ine.m);       % [kg rad-1] *m
sway.Yrvv = (-994.6e-5)*(ine.m/Unom);   % [kg s m-1 rad-1] *m/u
sway.Yv_r = (-1192.7e-5)*(ine.m);      % [kg rad-1] *m
sway.Yvrr = (-1107.9e-5)*(ine.m*Lpp/Unom);   % [kg s rad-2] *m*L/u
sway.Yvd = (-878.0e-5)*(ine.m);        % [kg] *m             Crossflow added mass
sway.Ypd = (23.3e-5)*(ine.m*Lpp);      % [kg m rad-1] *m*L
sway.Yrd = (-48.1e-5)*(ine.m*Lpp);     % [kg m rad-1] *m*L

% Yaw coefficients:    
yaw.Nv_v = (-712.9e-5)*(ine.m/Lpp);    % [kg m-1] *m/L
yaw.Nr_r = (0.0)*(ine.m*Lpp);          % [kg m rad-2] *m*L 
yaw.Nrrr = (-224.5e-5)*(ine.m*Lpp^2/Unom);  % [kg m s rad-3] *m*L2/u
yaw.Nr_v = (-778.8e-5)*(ine.m);        % [kg rad-1] *m
yaw.Nrvv = (-1287.2e-5)*(ine.m/Unom);  % [kg s m-1 rad-1] *m/u
yaw.Nv_r = (-174.7e-5)*(ine.m);        % [kg rad-1] *m
yaw.Nvrr = (36.8e-5)*(ine.m*Lpp/Unom); % [kg s rad-2] *m*L/u
yaw.Nvd = (42.3e-5)*(ine.m);           % [kg] *m
yaw.Nrd = (-30.0e-5)*(ine.m*Lpp);      % [kg m rad-1] *m*L

% Yaw coefficients:     
roll.Kv_v = (99.2e-5)*(ine.m/Lpp);     % [kg m-1] *m/L
roll.Kr_r = (-20.0e-5)*(ine.m*Lpp);    % [kg m rad-2] *m*L
roll.Krrr = (0.0e-5)*(ine.m*Lpp^2/Unom);   % [kg m s rad-3] *m*L2/u
roll.Kr_v = (-41.1e-5)*(ine.m);        % [kg rad-1] *m
roll.Krvv = (-34.6e-5)*(ine.m/Unom);   % [kg s m-1 rad-1] *m/u
roll.Kv_r = (10.4e-5)*(ine.m);         % [kg rad-1] *m
roll.Kvrr = (-22.2e-5)*(ine.m*Lpp/Unom);   % [kg s rad-2] *m*L/u
roll.Kp = (-3.0e-5)*(ine.m*Unom);      % [kg m s-1 rad-1]  *m*u
roll.Kppp = (0.00e-5)*(ine.m*Lpp^2/Unom);  % [kg m s rad-3] *m*L2/u
roll.Kpd = (0.7e-5)*(ine.m*Lpp);       % [kg m rad-1] *m*L
roll.Kvd = (0.0)*(ine.m);              % [kg] *m
roll.Krd = (-1.0e-5)*(ine.m*Lpp);      % [kg m rad-1] *m*L

%% Assemble the desired vectors:
propulsion = [prop.Jm,prop.Kn,prop.mf,prop.l,prop.q,prop.wp,prop.tp];       %% edit
actuators = [act.Tnn,act.Qnn,act.Yuuds,act.Muuds];                          %%
rigid_body = [ine.m,ine.xg,ine.zg,ine.Ixx,ine.Izz];
X = [surge.Xvv,surge.Xuu,surge.Xuuu,surge.Xrr,surge.Xvr,surge.Xud];
Y = [sway.Yv_v,sway.Yr_r,sway.Yrrr,sway.Yr_v,sway.Yrvv,sway.Yv_r,sway.Yvrr,sway.Yvd]; 
N = [yaw.Nv_v,yaw.Nr_r,yaw.Nrrr,yaw.Nr_v,yaw.Nrvv,yaw.Nv_r,yaw.Nvrr,yaw.Nvd];
K = [roll.Kv_v,roll.Kr_r,roll.Krrr,roll.Kr_v,roll.Krvv,roll.Kv_r,roll.Kvrr,roll.Kp,roll.Kppp,roll.Kvd];

%% Calculate the inverse of the two mass matrix subcomponents:      %%can ignore
% Inertia matrix
M = [ine.m-surge.Xud, 0, 0, 0; 0, ine.m-sway.Yvd, ine.m*ine.zg-sway.Ypd, ine.m*ine.xg-sway.Yrd; 0, -ine.m*ine.zg-roll.Kvd, ine.Ixx-roll.Kpd, 0; 0, ine.m*ine.xg-yaw.Nvd, 0, ine.Izz-yaw.Nrd];
% inverse
Mi = pinv(M);

%% Save the required data to file:
auv.propulsion = propulsion;
auv.actuators  = actuators;
auv.rigid_body = rigid_body;
auv.X = X;
auv.Y = Y;
auv.N = N;
auv.K = K;
auv.Minv = Mi;
save('C:/Users/arthu/OneDrive/Documents/MSC Individual/code/code 4dof/data/roro4dof.mat','auv');
