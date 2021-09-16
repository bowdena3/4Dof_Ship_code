% setupDynamicModel4dof.m     mail 30/06/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is run to generate the parameters for the simulation of an
% ship in 4 DOF.
%
% The parameters of the model for the ship are taken from Perez, T., & Blanke, M. (2002). Mathematical Ship Modeling for Control Applications. DTU library.
% https://backend.orbit.dtu.dk/ws/portalfiles/portal/137166308/Mathematical_Ship_Modeling_for_Control_Applications_Perez_Blanke.pdf 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Input the parameters for the model:

% Inertia values:
m_non = 750.81e-5;          
Ixx_non = 1.32e-5;
Izz_non = 43.25e-5;
rho = 1000;
Lpp = 230.66;
Unom = 12.7;
ine.m = m_non*((rho*Lpp^3)/2);         % [kg]            % Mass of vehicle
ine.xg = -0.46;                     % [m]             % x-coordinate of vehicle centre of gravity (-0.46e-5;)
ine.zg = -4.54;                     % [m]             % z-coordinate of vehicle centre of gravity
ine.Ixx = Ixx_non*((rho*Lpp^5)/2);     % [kg m2]         % Inertia around axiss %%tofix
ine.Izz = Izz_non*((rho*Lpp^5)/2);     % [kg m2]         % Inertia around axiss %%tofix

% KCS coefficnets taken from Mucha, Philipp. (2017)
% Surge cofficients:
surge.Xu = (-226.2e-5)*(ine.m*Unom/Lpp);                % [kg s-1] *m*m/s*m
surge.Xvv = (-1.0e-5)*(ine.m/Lpp);     % [kg m-1] *m/L
surge.Xuu = (-64.5e-5)*(ine.m/Lpp);    % [kg m-1] *m/L   Axial drag coefficient – resisting forward motion
surge.Xuuu = (-137.2e-5)*(ine.m/(Lpp*Unom));       % [kg s m-2] *m/U*L
surge.Xrr = (4.4e-5)*(ine.m*Lpp);      % [kg m rad-2] *m*L   Drag coefficient for pitching motion
surge.Xvr = (-24.0e-5)*(ine.m);        % [kg rad-1] *m
surge.Xud = (-124.4e-5)*(ine.m);       % [kg] *m       Axial added mass

% Sway coefficients:    
sway.Yv_v = (-5801.5e-5)*(ine.m/Lpp);  % [kg m-1] *m/L
sway.Yr_r = (0.0)*(ine.m*Lpp);         % [kg m rad-2] *m*L
sway.Yrrr = (-158.0e-5)*(ine.m*Lpp^2/Unom);  % [kg m s rad-3] *m*L2/u
sway.Yr_v = (-409.4e-5)*(ine.m);       % [kg rad-1] *m
sway.Yrvv = (-994.6e-5)*(ine.m/Unom);  % [kg s m-1 rad-1] *m/u
sway.Yv_r = (-1192.7e-5)*(ine.m);      % [kg rad-1] *m
sway.Yvrr = (-1107.9e-5)*(ine.m*Lpp/Unom);   % [kg s rad-2] *m*L/u
sway.Yvd = (-878.0e-5)*(ine.m);        % [kg] *m             Crossflow added mass
sway.Ypd = (23.3e-5)*(ine.m*Lpp);      % [kg m rad-1] *m*L
sway.Yrd = (-48.1e-5)*(ine.m*Lpp);     % [kg m rad-1] *m*L

% Yaw coefficients:             % tourque [kg m2 s-2]
yaw.Nv_v = (-712.9e-5)*(ine.m);    % [kg] *m
yaw.Nr_r = (0.0)*(ine.m*Lpp^2);          % [kg m2 rad-2] *m*L2
yaw.Nrrr = (-224.5e-5)*(ine.m*Lpp^3/Unom);  % [kg m2 s rad-3] *m*L3/u
yaw.Nr_v = (-778.8e-5)*(ine.m*Lpp);        % [kg m rad-1] *m*L
yaw.Nrvv = (-1287.2e-5)*(ine.m*Lpp/Unom);  % [kg s rad-1] *m*L/u
yaw.Nv_r = (-174.7e-5)*(ine.m*Lpp);        % [kg m rad-1] *m*L
yaw.Nvrr = (36.8e-5)*(ine.m*Lpp^2/Unom); % [kg s m rad-2] *m*L2/u
yaw.Nvd = (42.3e-5)*(ine.m*Lpp);           % [kg m] *m*L
yaw.Nrd = (-30.0e-5)*(ine.m*Lpp^2);      % [kg m2 rad-1] *m*L2

% Roll coefficients:             % tourque [kg m2 s-2]     
roll.Kv_v = (99.2e-5)*(ine.m);    % [kg] *m
roll.Kr_r = (-20.0e-5)*(ine.m*Lpp^2);          % [kg m2 rad-2] *m*L2
roll.Krrr = (0.0e-5)*(ine.m*Lpp^3/Unom);  % [kg m2 s rad-3] *m*L3/u
roll.Kr_v = (41.1e-5)*(ine.m*Lpp);        % [kg m rad-1] *m*L
roll.Krvv = (-34.6e-5)*(ine.m*Lpp/Unom);  % [kg s rad-1] *m*L/u
roll.Kv_r = (10.4e-5)*(ine.m*Lpp);        % [kg m rad-1] *m*L
roll.Kvrr = (22.2e-5)*(ine.m*Lpp^2/Unom); % [kg s m rad-2] *m*L2/u
roll.Kp = (-3.0e-5)*(ine.m*Lpp*Unom);      % [kg m2 s-1 rad-1]  *m*u*L
roll.Kppp = (0.00e-5)*(ine.m*Lpp^3/Unom);  % [kg m2 s rad-3] *m*L3/u
roll.Kpd = (-0.7e-5)*(ine.m*Lpp^2);       % [kg m2 rad-1] *m*L2
roll.Kvd = (0.0)*(ine.m*Lpp);           % [kg m] *m*L
roll.Krd = (-1.0e-5)*(ine.m*Lpp^2);      % [kg m2 rad-1] *m*L2

% Wing parameters: 
wing.rho = 1.225;                        % [kg/m3] density air
wing.area = 1920;                           % [m2] Sail area - (80*24)
height = 80;
wing.height = height;                         % [m] height of wing - move to setup?
% d_bow = 0.5*2.5*wing.height;                 % [m] Bow sail distance from COG - 0.5*2.5*height?
% d_stern = -0.5*2.5*wing.height;                 % [m] Stern sail distance from COG - 0.5*2.5*height?

% Rudder parameters:
rud.rho = 1025;                        % [kg/m3] density water
span = 11.11;                           % [m] rudder span
chord = 3.78;                           % [m] rudder chord
rud.area = span*chord;                  % [m2] rudder area
rud.depth = span;                         % [m] span of rudder 

%% Assemble the desired vectors:
rigid_body = [ine.m,ine.xg,ine.zg,ine.Ixx,ine.Izz];
X = [surge.Xu,surge.Xvv,surge.Xuu,surge.Xuuu,surge.Xrr,surge.Xvr,surge.Xud];
Y = [sway.Yv_v,sway.Yr_r,sway.Yrrr,sway.Yr_v,sway.Yrvv,sway.Yv_r,sway.Yvrr,sway.Yvd]; 
N = [yaw.Nv_v,yaw.Nr_r,yaw.Nrrr,yaw.Nr_v,yaw.Nrvv,yaw.Nv_r,yaw.Nvrr,yaw.Nvd];
K = [roll.Kv_v,roll.Kr_r,roll.Krrr,roll.Kr_v,roll.Krvv,roll.Kv_r,roll.Kvrr,roll.Kp,roll.Kppp,roll.Kvd];

wing = [wing.rho,wing.area,wing.height];
d_bow = 0.5*2.5*(24);                 % [m] Bow sail distance from COG - 0.5*2.5*height?
d_stern = -0.5*2.5*(24);                 % [m] Stern sail distance from COG - 0.5*2.5*height?

rud = [rud.rho,rud.area,rud.depth];
d_rud = 115;                 % [m] rudder distance from COG - 230m boat

% Sail lift data
Cl_angle = [-180.0,-175.8,-170.1,-165.2,-160.0,-150.9,-140.3,-129.6,-120.2,-110.1,-99.7,-90.3,-79.9,-69.2,-59.7,-50.3,-40.2,-30.2,-25.7,-17.5,-15.0,-12.6,-10.5,-8.0,-6.8,-5.6,-0.2,0.2,5.6,6.8,8.0,10.5,12.6,15.0,17.5,25.7,30.2,40.2,50.3,59.7,69.2,79.9,90.3,99.7,110.1,120.2,129.6,140.3,150.9,160.0,165.2,170.1,175.8,180.0];
Cl_data = [0.464,1.023,1.235,1.046,1.122,1.410,1.471,1.290,0.867,0.565,0.279,-0.038,-0.332,-0.657,-1.034,-1.412,-1.509,-1.426,-1.259,-1.010,-0.987,-1.040,-1.100,-1.085,-1.032,-0.896,-0.185,0.185,0.896,1.032,1.085,1.100,1.040,0.987,1.010,1.259,1.426,1.509,1.412,1.034,0.657,0.332,0.038,-0.279,-0.565,-0.867,-1.290,-1.471,-1.410,-1.122,-1.046,-1.235,-1.023,-0.464];
% Sail drag data
Cd_angle = [-180,-175.5,-170.1,-160.6,-165.4,-150.5,-140.1,-130.3,-120.1,-110.3,-100.4,-90.0,-70.2,-60.3,-50.2,-39.7,-30.0,-24.9,-20.1,-17.4,-15.0,-12.6,-10.2,-7.8,-5.6,-1.9,1.9,5.6,7.8,10.2,12.6,15.0,17.4,20.1,24.9,30.0,39.7,50.2,60.3,70.2,90.0,100.4,110.3,120.1,130.3,140.1,150.5,160.6,165.4,170.1,175.5,180];
Cd_data = [0.025,0.039,0.179,0.407,0.300,0.836,1.311,1.652,1.613,1.760,1.881,1.915,1.721,1.735,1.602,1.201,0.800,0.566,0.439,0.372,0.326,0.272,0.172,0.078,0.032,0.025,0.025,0.032,0.078,0.172,0.272,0.326,0.372,0.439,0.566,0.800,1.201,1.602,1.735,1.721,1.915,1.881,1.760,1.613,1.652,1.311,0.836,0.407,0.300,0.179,0.039,0.025];
% 2 sail coeffient scale factors
wind_oriantation = [20,30,35,45,60,75,90,105,120,135,150,165,180];
Cl_ratio_bow = [1.26,1.41,1.38,1.38,1.30,1.26,1.20,1.18,1.13,0.93,0.97,-9.66,0.00];
Cd_ratio_bow = [1.32,1.37,1.43,1.55,1.53,1.49,1.43,1.42,1.32,0.97,1.04,1.00,-0.15];
Cl_ratio_stern = [0.85,1.00,1.10,1.08,1.10,1.13,1.15,1.21,1.14,1.17,1.11,1.07,0.00];
Cd_ratio_stern = [1.40,1.24,1.14,1.25,1.34,1.31,1.20,1.13,1.04,1.18,1.13,1.06,1.06];

AOA_rud = [-50,-45,-40,-35,-30,-25,-20,-15,-12.5,-10,-7.5,-5,-2.5,0,2.5,5,7.5,10,12.5,15,20,25,30,35,40,45,50];
Cl_rud = [-0.6315,-1.3158,-1.3222,-1.3055,-1.3523,-1.5600,-1.7260,-1.5092,-1.3084,-1.0953,-0.8384,-0.5664,-0.2855,0,0.2855,0.5664,0.8384,1.0953,1.3084,1.5092,1.7260,1.5600,1.3523,1.3055,1.3222,1.3158,0.6315];
Cd_rud = [0.41048,0.39972,0.34865,0.26262,0.21667,0.11866,0.04076,0.01607,0.01191,0.00961,0.00801,0.00693,0.00627,0.00609,0.00627,0.00693,0.00801,0.00961,0.01191,0.01607,0.04076,0.11866,0.21667,0.26262,0.34865,0.39972,0.41048];

%% Calculate the inverse of the two mass matrix subcomponents:      %% can ignore
% Inertia matrix
M = [ine.m-surge.Xud, 0, 0, 0; 0, ine.m-sway.Yvd, -ine.m*ine.zg-sway.Ypd, ine.m*ine.xg-sway.Yrd; 0, -ine.m*ine.zg-roll.Kvd, ine.Ixx-roll.Kpd, 0; 0, ine.m*ine.xg-yaw.Nvd, 0, ine.Izz-yaw.Nrd];
% inverse
Mi = pinv(M);

%% Save the required data to file:
ship.rigid_body = rigid_body;
ship.X = X;
ship.Y = Y;
ship.N = N;
ship.K = K;

ship.wing = wing;
ship.d_bow = d_bow;
ship.d_stern = d_stern;

ship.rud = rud;
ship.d_rud = d_rud ;

ship.Cl_angle = Cl_angle;
ship.Cl_data = Cl_data;
ship.Cd_angle = Cd_angle;
ship.Cd_data = Cd_data;
ship.wind_oriantation = wind_oriantation;
ship.Cl_ratio_bow = Cl_ratio_bow;
ship.Cd_ratio_bow = Cd_ratio_bow;
ship.Cl_ratio_stern = Cl_ratio_stern;
ship.Cd_ratio_stern = Cd_ratio_stern;

ship.AOA_rud = AOA_rud;
ship.Cl_rud = Cl_rud;
ship.Cd_rud = Cd_rud;

ship.Minv = Mi;

save('C:/Users/arthu/OneDrive/Documents/MSC Individual/code/code 4dof/data/roro4dof.mat','ship');
