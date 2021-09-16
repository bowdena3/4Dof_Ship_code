clc;
clear;
load('C:/Users/arthu/OneDrive/Documents/MSC Individual/code/code 4dof/data/roro4dof.mat','ship');

%% rudder

    syms delta Thrust 
    syms x y phi psi u v p r
    
    % Extract the input parameters:
    rudder  = ship.rud;     % rudder data
    d  = ship.d_rud;          % distance to COG
    AOA_rud  = ship.AOA_rud;        % rudder AOA data
    Cl_rud  = ship.Cl_rud;          % rudder Cl data
    Cd_rud  = ship.Cd_rud;          % rudder Cd data
    
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
    
    AOA = rad2deg(delta) - S_theta;                  % angle of attack on rudder

%     Cl = (0.000000013069)*AOA^5+(-2.4578E-22)*AOA^4+(-0.000068501)*AOA^3+(3.0792E-19)*AOA^2+(0.1062)*AOA+(4.2687E-18);
%     Cd = (-1.7593E-24)*AOA^5+(-0.000000038661)*AOA^4+(4.9449E-21)*AOA^3+(0.00027641)*AOA^2+(-2.6965E-18)*AOA+(-0.0184);
    Cl = 1.5*sind(2*AOA);
    Cd = -cos(2*AOA)+1.03;

%     % Prevents Cl = 0 causing infinite angles due to Cd/Cl
%     sign = (0.5-isreal(sqrt(-Cd)))*2;           % calculate Cl sign
%     Cd = sign*(abs(Cd)+0.0001);                 % add 0.001 to absolute value of Cl    
    F_net = 0.5*sqrt(Cl^2 + Cd^2)*rho*area*Vs^2;   % force on rudder [N]
    F_theta = atand(Cl/Cd);                        % angle on rudder [degrees] (atan for rads) - local (oriantated to 90deg of coursw angle)
        
    Fhr = -F_net*cosd(F_theta-S_theta);                    % hydrodynamic resisting force [N]       % surge force on ship [N] - global (x co-ordinates)
    Fhs = F_net*sind(F_theta-S_theta);                     % hydrodynamic side force [N]         % sway force on ship [N] - global (y co-ordinates)
        
    % compute Yaw and Roll tourque
    T_roll = -Fhs*0.5*height;                          % roll torque on ship [Nm]
    T_yaw = -Fhs*d;                                    % yaw torque on ship [Nm]
    
    % Trust propeller
    % Approximate thrust generated by propeller
    control_r = [(Fhr+Thrust);Fhs;T_roll;T_yaw];            % [surge,sway,roll,yaw]   


%% sail

    syms W_delta_bow  W_delta_stern
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
    
    Vt = 10;           % Prevailing wind speed [m/s]
    W_theta = 90;      % Prevailing wind angle [degrees]
    
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
    TWA = 180*sign + rad2deg(psi) + S_theta - W_theta;                         % True wind angle [degrees] - local (oriantated to ship course)
    
    Va = sqrt(Vs^2+Vt^2-2*Vs*Vt*cosd(TWA));        % Apparant wind speed [m/s]
    AWA = asind((Vt/Va)*sind(TWA));                % Apparant wind angle (check) [degrees] - local (oriantated to ship course)
    
    Va_t = Va*sin(0.5*pi()-abs(phi));                % Apparant wind speed horisontally across wing (accounting for ship roll angle)
    
    AOA_bow = AWA + S_theta - rad2deg(W_delta_bow);         %  - rad2deg(psi)
    Cl1_bow = (-0.0000000000000049833)*AOA_bow^7+(0.00000000036694)*AOA_bow^5+(-0.0000082927)*AOA_bow^3+(0.0483)*AOA_bow+(4.0547E-16);      % Cl1 = interp1(Cl_angle,Cl_data,AOA);     % Coeffent of lift
    Cd1_bow = (0.00000000000047384)*AOA_bow^6+(-0.000000028273)*AOA_bow^4+(0.00041811)*AOA_bow^2+(0.2626);      % Cd1 = interp1(Cd_angle,Cd_data,AOA);     % Coeffent of drag
    
    AOA_stern = AWA + S_theta - rad2deg(W_delta_stern);         %  - rad2deg(psi)
    Cl1_stern = (-0.0000000000000049833)*AOA_stern^7+(0.00000000036694)*AOA_stern^5+(-0.0000082927)*AOA_stern^3+(0.0483)*AOA_stern+(4.0547E-16);      % Cl1 = interp1(Cl_angle,Cl_data,AOA);     % Coeffent of lift
    Cd1_stern = (0.00000000000047384)*AOA_stern^6+(-0.000000028273)*AOA_stern^4+(0.00041811)*AOA_stern^2+(0.2626);      % Cd1 = interp1(Cd_angle,Cd_data,AOA);     % Coeffent of drag
 
    % Scale coeffents of drag based on data from Bordogna, G et al., (2018)
    W_O = abs(AWA + S_theta);        % wind oriantation angle - angle between wind and ship angle   % - rad2deg(psi)
    Cl_ratio_bow = (0.00000093886)*W_O^3+(-0.00065532)*W_O^2+(0.0696)*W_O+(-0.2385);
    Cd_ratio_bow = (-0.00000077607)*W_O^3+(0.00012007)*W_O^2+(-0.0038)*W_O+(1.4261);    
    Cl_ratio_stern = (-0.0000012734)*W_O^3+(0.00028272)*W_O^2+(-0.015)*W_O+(1.2056);
    Cd_ratio_stern = (0.000000048452)*W_O^3+(-0.000014812)*W_O^2+(-0.00027892)*W_O+(1.3083);
    
    Cl2_bow = Cl_ratio_bow*Cl1_bow;                       % calculating 2 sail Cl
    Cd2_bow = Cd_ratio_bow*Cd1_bow;                       % calculating 2 sail Cd
    Cl2_stern = Cl_ratio_stern*Cl1_stern;                       % calculating 2 sail Cl
    Cd2_stern = Cd_ratio_stern*Cd1_stern;                       % calculating 2 sail Cd
    
    F_net_bow = 0.5*sqrt(Cl2_bow^2 + Cd2_bow^2)*rho*area*Va_t^2;    % force on sail [N]
    F_theta_bow = atand(Cd2_bow/Cl2_bow);                           % angle on sail [degrees] (atan for rads) - local (oriantated to 90deg of AWA)
    F_net_stern = 0.5*sqrt(Cl2_stern^2 + Cd2_stern^2)*rho*area*Va_t^2;   % force on sail [N]
    F_theta_stern = atand(Cd2_stern/Cl2_stern);                          % angle on sail [degrees] (atan for rads) - local (oriantated to 90deg of AWA)
        
    sign = (0.5-isreal(sqrt(-Cl2_bow)))*2;           % calculate Cl2 sign
    Far_bow = F_net_bow*cosd(AWA+S_theta-90*sign-F_theta_bow);                    % aerodynamic driving force [N] % surge force on ship [N]  Far_bow = F_net_bow*cosd(AWA+S_theta-90*sign-F_theta_bow); 
    Fas_bow = F_net_bow*sind(AWA+S_theta-90*sign-F_theta_bow);                     % aerodynamic side force [N] % sway force on ship [N] 
    
    sign = (0.5-isreal(sqrt(-Cl2_stern)))*2;           % calculate Cl2 sign
    Far_stern = F_net_stern*cosd(AWA+S_theta-90*sign-F_theta_stern);                    % aerodynamic driving force [N] % surge force on ship [N] 
    Fas_stern = F_net_stern*sind(AWA+S_theta-90*sign-F_theta_stern);                     % aerodynamic side force [N] % sway force on ship [N] 
    
    % compute Yaw and Roll tourque
    T_roll_bow = Fas_bow*0.5*height;                          % roll torque on ship [Nm]
    T_yaw_bow = Fas_bow*d_bow;                                    % yaw torque on ship [Nm]
    
    T_roll_stern = Fas_stern*0.5*height;                      % roll torque on ship [Nm]
    T_yaw_stern = Fas_stern*d_bow;                                % yaw torque on ship [Nm]
    
    control_w_bow = [Far_bow;Fas_bow;T_roll_bow;T_yaw_bow];                  % [surge,sway,roll,yaw]
    control_w_stern = [Far_stern;Fas_stern;T_roll_stern;T_yaw_stern];        % [surge,sway,roll,yaw]

%% ship 4dof

%     syms x y phi psi u v p r
    
    % Extract the input parameters:
    rbd  = ship.rigid_body;     %ridgied body data
    X    = ship.X;
    Y    = ship.Y;
    N    = ship.N;
    K    = ship.K;
    Minv = ship.Minv;
    
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
    sin_phi = sin(phi);

    % Compute the right-hand side of the position derivative vector: (u,v,r,psi,p,phi)
    % Translational transformation matrix:
%     R = [cos_psi, -cos_phi*sin_psi; sin_psi, cos_phi*cos_psi]; % wrong
    R = [cos_psi, sin_psi*sin_phi; 0, cos_phi]; 
    % Rotational transformation matrix:
    T = [1, 0; 0, cos_phi]; % correct?
    J = [R,zeros(2,2);zeros(2,2),T];       % equations of motion- to change
    dxdt1 = J*[u;v;p;r];                                              % in fossen book (psi = Jfunction)
    
    % Compute the right-hand side of the velocity derivative vector: 
    % [du/dt;dw/dt;dq/dt] = f_2(t);                                 % in fossen book pg 180
    % Coriolis matrix
    Crb = [0, 0, m*zg*r, -m*(xg*r+v); 0, 0, 0, m*u; -m*zg*r, 0, 0, 0; m*(xg*r+v), -m*u, 0, 0];      % Ridged body coriolis
    Ca = [0, 0, 0, Yvd*v; 0, 0, 0, -Xud*u; 0, 0, 0, Yvd*v; -Yvd*v, Xud*u, -Yvd*v, 0];           % Added mass coriolis
    % Gravity? matrix
    g = [0; 0; 1000*9.81*0.83*sin_phi*cos_phi; 0];
    % Damping matrix
    D = [-Xuu*u-Xuuu*u2, -Xvv*v-Xvr*r, 0, -Xrr*r;...        % with missing terms removed
        0, -Yvrr*r2-Yv_v*abs_v-Yv_r*abs_r, 0, -Yrrr*r2-Yrvv*v2-Yv_r*abs_v-Yr_r*abs_r;...
        0, -Kvrr*r2-Kv_v*abs_v-Kv_r*abs_r, -Kp-Kppp*p2, -Krrr*r2-Krvv*v2-Kv_r*abs_v-Kr_r*abs_r;...
        0, -Nvrr*r2-Nv_v*abs_v-Nv_r*abs_r, 0, -Nrrr*r2-Nrvv*v2-Nv_r*abs_v-Nr_r*abs_r];
%     eta = [x, y, phi, psi].';            % should be eta = [x, y, phi, psi]';
    nu =  [u, v, p, r].';

%     control = [F_surge;F_sway;T_roll;T_yaw];  % [surge,sway,roll,yaw]
    control = control_r+control_w_bow+control_w_stern;         % [surge,sway,roll,yaw]

    % 4dof equation of motion
    f = -Crb*nu-Ca*nu-D*nu-g + control;  % check sign f=control-...
    dxdt2 = Minv*f;
    
%     derivitives = [dxdt1;dxdt2];
    derivitives = [dxdt2];

%% 

    delta = 0;
    Thrust = 1e6;
    W_delta_bow = 80;
    W_delta_stern = 80;
    x = 50;
    y = 0;
    phi = 0;
    psi = 0;
%     u = 8;
    v = 0;
    p = 0;
    r = 0;
    
derivitives_2 = subs(derivitives);
% double()

velocity_u = 0 == derivitives_2(1);

uv = -solve(velocity_u,u);
% solve(velocity_u,u,'IgnoreAnalyticConstraints',1)

objective = @(W_delta_bow,W_delta_stern) uv;
disp(objective)

x0 = [1.396 1.396];
% disp(['Initial Objective: ' num2str(objective(x0))])
% disp(['Initial Objective: ' num2str(objective(1.396,1.396))])
objective(1.396,1.396);
% num2str(objective(1.396,1.396))

lb = -pi()*ones(2);         % -pi() -3.141
ub = pi()*ones(2);          % pi() 3.141
A = [];
b = [];
Aeq = [];
beq = [];

x = fmincon(objective,x0,A,b,Aeq,beq,lb,ub);

%%

syms t1 t2 ut
test1 = 0 == -ut+t1*t2-t2;
test2 = solve(test1,ut);
% objective = @(t1,t2) test2;
% objective = @(t1,t2) t1*t2-t2;
objective = @(a) a(1)-2*a(2);
disp(objective);
x0 = [1 2];
lb = [0,0];         % -pi() -3.141
ub = [5,5];          % pi() 3.141
A = [];
b = [];
Aeq = [];
beq = [];

x = fmincon(objective,x0,A,b,Aeq,beq,lb,ub);

%%
% 
% fun = @(x)100*(x(2)-x(1)^2)^2 + (1-x(1))^2;
% 
% x0 = [-1,2];
% A = [1,2];
% b = 1;
% x = fmincon(fun,x0,A,b)
