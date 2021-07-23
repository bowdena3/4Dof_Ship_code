clc;
clear;
load('C:/Users/arthu/OneDrive/Documents/MSC Individual/code/code 4dof/data/roro4dof.mat','ship');

%% rudder

    syms delta Thrust W_delta 
    syms x y phi psi u v p r
    
    % Extract the input parameters:
    rudder  = ship.rud;     % rudder data
    d  = ship.d_rud;          % distance to COG
    AOA_rud  = ship.AOA_rud;         % rudder AOA data
    Cl_rud  = ship.Cl_rud;          % rudder Cl data
    Cd_rud  = ship.Cd_rud;          % rudder Cd data
    
%     % Extract the input vector:
%     delta = block.InputPort(1).Data;            % rudder angle [rads] - local (oriantated to apparent wind angle)
%     ship_velocity = block.InputPort(2).Data;          % ship velocity
%     Thrust  = block.InputPort(3).Data;                % ship thrust
    
%     % To make the code clearer, copy all desired parameters and states:
%     % Input vectors:
%     u = ship_velocity(1);           % ship velocity x [m/s]
%     v = ship_velocity(2);           % ship velocity y [m/s]
%     psi = ship_velocity(3);         % ship yaw angle [rads]
    
    % Parameters:       % to change
    rho   = rudder(1);                 % density water  
    area  = rudder(2);                 % rudder area
    height  = rudder(3);               % height of rudder         
    
    % conpute the 4dof force generated by the wing
    % Apparant and acutal wind   - (Independant of ship angle, dependent on course path) is it global coordinates?
    Vs = sqrt(u^2 + v^2);                                        % Ship speed [m/s]
%     % Prevents u = 0 causing infinite angles due to v/u
%     sign = (0.5-isreal(sqrt(-u)))*2;           % calculate u sighn
%     u = sign*(abs(u)+0.0001);                  % add 0.001 to absolute valie of u
    S_theta = atand(v/u);                               % Ship course angle [degrees] (atan for rads) - global +0.0001 so that inital velocity is not 0
    
    AOA = rad2deg(delta) + rad2deg(psi) - S_theta;                  % angle of attack on rudder

%     Cl = interp1(AOA_rud,Cl_rud,AOA);     % Coeffent of lift
%     Cd = interp1(AOA_rud,Cd_rud,AOA);     % Coeffent of drag
    Cl = (0.000000013069)*AOA^5+(-2.4578E-22)*AOA^4+(-0.000068501)*AOA^3+(3.0792E-19)*AOA^2+(0.1062)*AOA+(4.2687E-18);
    Cd = (-1.7593E-24)*AOA^5+(-0.000000038661)*AOA^4+(4.9449E-21)*AOA^3+(0.00027641)*AOA^2+(-2.6965E-18)*AOA+(-0.0184);

%     % Prevents Cl = 0 causing infinite angles due to Cd/Cl
%     sign = (0.5-isreal(sqrt(-Cd)))*2;           % calculate Cl sign
%     Cd = sign*(abs(Cd)+0.0001);                 % add 0.001 to absolute value of Cl    
    F_net = 0.5*sqrt(Cl^2 + Cd^2)*rho*area*Vs^2;   % force on rudder [N]
    F_theta = atand(Cl/Cd);                      % angle on rudder [degrees] (atan for rads) - local (oriantated to 90deg of coursw angle)
    
    
    F_surge = -F_net*cosd(F_theta-S_theta);                    % surge force on ship [N] - global (x co-ordinates)
    F_sway = F_net*sind(F_theta-S_theta);                     % sway force on ship [N] - global (y co-ordinates)
    
%     Fhs = F_net*cosd(F_theta+rad2deg(psi)-S_theta);          % hydrodynamic resisting force [N]
    Fhs = F_net*sind(F_theta+rad2deg(psi)-S_theta);          % hydrodynamic side force [N]
    
    % compute Yaw and Roll tourque
    T_roll = -Fhs*0.5*height;                          % roll torque on ship [Nm]
    T_yaw = -Fhs*d;                                    % yaw torque on ship [Nm]
        
    % Trust propeller
    % Approximate thrust generated by propeller
%     Thrust = 1e6;              % [N]
    Thrust_surge = Thrust*cos(psi);      % [N]
    Thrust_sway = Thrust*sin(psi);       % [N]
    control_r = [(F_surge+Thrust_surge);(F_sway+Thrust_sway);T_roll;T_yaw];            % [surge,sway,roll,yaw]   


%% sail

%     syms W_delta 
%     syms x y phi psi u v p r

    % Extract the input parameters:      % to change
    wing  = ship.wing;     % wing data
    d_bow  = ship.d_bow;     % distance to COG
%     Cl_angle = Cl_angle;     
%     Cl_data = Cl_data;      
%     Cd_angle = Cd_angle;     
%     Cd_data = Cd_data;      
    wind_oriantation = ship.wind_oriantation;     
    Cl_ratio_bow = ship.Cl_ratio_bow;      
    Cd_ratio_bow = ship.Cd_ratio_bow;   
    Cl_ratio_stern = ship.Cl_ratio_stern;      
    Cd_ratio_stern = ship.Cd_ratio_stern;       
    
    % Extract the input vector:
%     in = block.InputPort(1).Data;
%     W_delta = block.InputPort(1).Data;          % wing angle [degrees] - local
%     ship_velocity = block.InputPort(2).Data;          % ship velocity
%     wind_vector = block.InputPort(3).Data;           % Prevailing wind vector
    
%     u = ship_velocity(1);           % ship velocity x [m/s]
%     v = ship_velocity(2);           % ship velocity y [m/s]
%     psi = ship_velocity(3);         % ship yaw angle [rads]
    
    Vt = 1;           % Prevailing wind speed [m/s]
    W_theta = 50;      % Prevailing wind angle [degrees]
    
    % Parameters:       % to change
    rho   = wing(1);                 % density air  
    area  = wing(2);                 % Sail area
    height  = wing(3);               % height of wing         
    
    % conpute the 4dof force generated by the wing
    % Apparant and acutal wind   - (Independant of ship angle, dependent on course path) is it global coordinates?
    Vs = sqrt(u^2 + v^2);                                      % Ship speed [m/s]
    % Prevents u = 0 causing infinite angles due to v/u
%     sign = (0.5-isreal(sqrt(-u)))*2;           % calculate u sighn
%     u = sign*(abs(u)+0.0001);                % add 0.001 to absolute valie of u    
    S_theta = atand(v/u);                               % Ship course angle [degrees] (atan for rads) - global +0.0001 so that inital velocity is not 0
    
    sign = (0.5-isreal(sqrt(-W_theta)))*2;           % calculate u sighn
    TWA = 180*sign + S_theta - W_theta;                         % True wind angle [degrees] - local (oriantated to ship course)
    
    Va = sqrt(Vs^2+Vt^2-2*Vs*Vt*cosd(TWA));        % Apparant wind speed [m/s]
    AWA = asind((Vt/Va)*sind(TWA));               % Apparant wind angle (check) [degrees] - local (oriantated to ship course)
    
    AOA = AWA + S_theta - rad2deg(psi) - rad2deg(W_delta);
%     Cl1 = interp1(Cl_angle,Cl_data,AOA);     % Coeffent of lift
%     Cd1 = interp1(Cd_angle,Cd_data,AOA);     % Coeffent of drag
    Cl1 = (-0.0000000000000049833)*AOA^7+(0.00000000036694)*AOA^5+(-0.0000082927)*AOA^3+(0.0483)*AOA+(4.0547E-16);
    Cd1 = (0.00000000000047384)*AOA^6+(-0.000000028273)*AOA^4+(0.00041811)*AOA^2+(0.2626);    
    
    % Scale coeffents of drag based on data from Bordogna, G et al., (2018)
    W_O = AWA + S_theta - rad2deg(psi);    % wind oriantation angle - angle between wind and ship angle
    
%     Cl_ratio = interp1(wind_oriantation,Cl_ratio_bow,abs(W_O));
%     Cd_ratio = interp1(wind_oriantation,Cd_ratio_bow,abs(W_O));
    Cl_ratio_bow = (0.00000093886)*W_O^3+(-0.00065532)*W_O^2+(0.0696)*W_O+(-0.2385);
    Cd_ratio_bow = (-0.00000077607)*W_O^3+(0.00012007)*W_O^2+(-0.0038)*W_O+(1.4261);    
    Cl_ratio_stern = (-0.0000012734)*W_O^3+(0.00028272)*W_O^2+(-0.015)*W_O+(1.2056);
    Cd_ratio_stern = (0.000000048452)*W_O^3+(-0.000014812)*W_O^2+(-0.00027892)*W_O+(1.3083);
        
    Cl2_bow = Cl_ratio_bow*Cl1;                       % calculating 2 sail Cl
    Cd2_bow = Cd_ratio_bow*Cd1;                       % calculating 2 sail Cd
    Cl2_stern = Cl_ratio_stern*Cl1;                       % calculating 2 sail Cl
    Cd2_stern = Cd_ratio_stern*Cd1;                       % calculating 2 sail Cd
    
    F_net_bow = 0.5*sqrt(Cl2_bow^2 + Cd2_bow^2)*rho*area*Va^2;   % force on sail [N]
    F_theta_bow = atand(Cd2_bow/Cl2_bow);                      % angle on sail [degrees] (atan for rads) - local (oriantated to 90deg of AWA)
    F_net_stern = 0.5*sqrt(Cl2_stern^2 + Cd2_stern^2)*rho*area*Va^2;   % force on sail [N]
    F_theta_stern = atand(Cd2_stern/Cl2_stern);                      % angle on sail [degrees] (atan for rads) - local (oriantated to 90deg of AWA)

    
    F_surge_bow = F_net_bow*cosd(AWA+S_theta-90*sign-F_theta_bow);                    % surge force on ship [N] - global (x co-ordinates)
    F_sway_bow = F_net_bow*sind(AWA+S_theta-90*sign-F_theta_bow);                     % sway force on ship [N] - global (y co-ordinates)
%     Far_bow = F_net_bow*sind(AWA+S_theta-90-F_theta_bow-rad2deg(psi));               % aerodynamic driving force [N] (atan for rads) - local (oriantated to course)
    Fas_bow = F_net_bow*cosd(AWA+S_theta-90-F_theta_bow-rad2deg(psi));               % aerodynamic side force [N] (atan for rads) - local (oriantated to course)
    
    F_surge_stern = F_net_stern*cosd(AWA+S_theta-90*sign-F_theta_stern);                    % surge force on ship [N] - global (x co-ordinates)
    F_sway_stern = F_net_stern*sind(AWA+S_theta-90*sign-F_theta_stern);                     % sway force on ship [N] - global (y co-ordinates)
%     Far_stern = F_net_stern*sind(AWA+S_theta-90-F_theta_stern-rad2deg(psi));               % aerodynamic driving force [N] (atan for rads) - local (oriantated to course)
    Fas_stern = F_net_stern*cosd(AWA+S_theta-90-F_theta_stern-rad2deg(psi));               % aerodynamic side force [N] (atan for rads) - local (oriantated to course)

    % compute Yaw and Roll tourque
    T_roll_bow = Fas_bow*0.5*height;                          % roll torque on ship [Nm]
    T_yaw_bow = Fas_bow*d_bow;                                    % yaw torque on ship [Nm]    
    
    T_roll_stern = Fas_stern*0.5*height;                      % roll torque on ship [Nm]
    T_yaw_stern = Fas_stern*d_bow;                                % yaw torque on ship [Nm]
            
    control_w_bow = [F_surge_bow;F_sway_bow;T_roll_bow;T_yaw_bow];                  % [surge,sway,roll,yaw]
    control_w_stern = [F_surge_stern;F_sway_stern;T_roll_stern;T_yaw_stern];        % [surge,sway,roll,yaw]

%% ship 4dof

%     syms x y phi psi u v p r
    
    % Extract the input parameters:
    act  = ship.actuators;     %actuator data
    rbd  = ship.rigid_body;     %ridgied body data
    X    = ship.X;
    Y    = ship.Y;
    N    = ship.N;
    K    = ship.K;
    Minv = ship.Minv;
    
%     % To make the code clearer, copy all desired parameters and states:
%     % Input vector:    
%     F_surge = 1;           % Control surge force [N]
%     F_sway = 0;           % Control sway force force [N]
%     T_roll = 0;           % Control roll force [N]    
%     T_yaw = 0;           % Control yaw force force [N]
    
    % Parameters:
    m    = rbd(1);
    xg   = rbd(3);
    zg   = rbd(3);
    
    Xvv = X(1);
    Xuu = X(2);
    Xuuu = X(3);
    Xrr = X(4);
    Xvr = X(5);
    Xud = X(6);    
    
    Yv_v = Y(1);
    Yr_r = Y(2);
    Yrrr = Y(3);
    Yr_v = Y(4);
    Yrvv = Y(5);
    Yv_r = Y(6);
    Yvrr = Y(7);
    Yvd = Y(8);
    
    Kv_v = K(1);
    Kr_r = K(2);
    Krrr = K(3);
    Kr_v = K(4);
    Krvv = K(5);
    Kv_r = K(6);
    Kvrr = K(7);
    Kp = K(8);
    Kppp = K(9);
    Kvd = K(10);
    
    Nv_v = N(1);
    Nr_r = N(2);
    Nrrr = N(3);
    Nr_v = N(4);
    Nrvv = N(5);
    Nv_r = N(6);
    Nvrr = N(7);
    Nvd = N(8);
    
    % Speed up the calculations: pre-compute recurring terms:
    u2 = u^2;
    v2 = v^2;
    r2 = r^2;
    abs_v = abs(v);
    abs_r = abs(r);
    p2 = p^2;
    cos_psi = cos(psi);
    cos_phi = cos(phi);
    sin_psi = sin(psi);

    % Compute the right-hand side of the position derivative vector: (u,v,r,psi,p,phi)
    % Translational transformation matrix:
    R = [cos_psi, -cos_phi*sin_psi; sin_psi, cos_phi*cos_psi]; % correct?
    % Rotational transformation matrix:
    T = [1, 0; 0, cos_phi]; % correct?
    J = [R,zeros(2,2);zeros(2,2),T];       % equations of motion- to change
    dxdt1 = J*[u;v;p;r];                                              % in fossen book (psi = Jfunction)
    
    % Compute the right-hand side of the velocity derivative vector: 
    % [du/dt;dw/dt;dq/dt] = f_2(t);                                 % in fossen book pg 180
    % Coriolis matrix
    Crb = [0, 0, m*zg, -m*(xg*r+v); 0, 0, 0, m*u; -m*zg, 0, 0, 0; m*(xg*r+v), -m*u, 0, 0];      % Ridged body coriolis
    Ca = [0, 0, 0, Yvd*v; 0, 0, 0, -Xud*u; 0, 0, 0, Yvd*v; -Yvd*v, Xud*u, -Yvd*v, 0];           % Added mass coriolis
    % Gravity? matrix
    G = sym(zeros(4,4));
%     G(3,3) = -Kphi;           % Kphi zero with current parameters
    % Damping matrix
    D = [-Xuu*u-Xuuu*u2, -Xvv*v-Xvr*r, 0, -Xrr*r;...        % with missing terms removed
        0, -Yvrr*r2-Yv_v*abs_v-Yv_r*abs_r, 0, -Yrrr*r2-Yrvv*v2-Yv_r*abs_v-Yr_r*abs_r;...
        0, -Kvrr*r2-Kv_v*abs_v-Kv_r*abs_r, -Kp-Kppp*p2, -Krrr*r2-Krvv*v2-Kv_r*abs_v-Kr_r*abs_r;...
        0, -Nvrr*r2-Nv_v*abs_v-Nv_r*abs_r, 0, -Nrrr*r2-Nrvv*v2-Nv_r*abs_v-Nr_r*abs_r];
    eta = [x, y, phi, psi]';            % should be eta = [x, y, phi, psi]';
    nu =  [u, v, p, r]';

%     control = [F_surge;F_sway;T_roll;T_yaw];  % [surge,sway,roll,yaw]
    control = control_r+control_w_bow+control_w_stern;         % [surge,sway,roll,yaw]

    % 4dof equation of motion
    f = -Crb*nu-Ca*nu-D*nu-G*eta + control;  % check sign f=control-...
    dxdt2 = Minv*f;
    
    derivitives = [dxdt1;dxdt2];
    
    
%% Jacobian A and B matrix

A = jacobian(derivitives, [x, y, phi, psi, u, v, p, r]);
B = jacobian(derivitives, [delta, Thrust, W_delta]);

% A(1,:)*[x; y; phi; psi; u; v; p; r]+B(1,:)*[delta; Thrust; W_delta]

%% Calculate K gain (u = -kx)


Q = zeros(8,8);
Q(1,1) = 1;
Q(2,2) = 1;
Q(3,3) = 1;
Q(4,4) = 1;
Q(5,5) = 1;
Q(6,6) = 1;
Q(7,7) = 1;
Q(8,8) = 1;
R = [1, 0, 0; 0, 1, 0; 0, 0, 1];
[K,S,e] = lqr(A,B,Q,R);


