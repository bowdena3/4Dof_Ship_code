function ship_4dof(block)
% remus_dynamics.m     mail     30/06/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the modelling of the dynamics of
% an ship in 4 DOF.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 2;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the input ports correctly:
%     block.InputPort(1).Dimensions    = 2;    % control input vector (thrust, rudder angle)
    block.InputPort(1).Dimensions    = 3;    % control input vector (Wind speed, wind angle, rudder angle)
    % Specify whether there is direct feedthrough:
    block.InputPort(1).DirectFeedthrough = false;
    block.InputPort(1).SamplingMode  = 'Sample';
    
    % Size the output port correctly:
%     block.OutputPort(1).Dimensions   = 8;        % continuous states (return state vecotrs) ((X,Y,theta),(u,w,q),(,) )
    block.OutputPort(1).Dimensions   = 8;        % continuous states (return state vectors) ((X,Y,phi,psi),(u,v,p,r),(,) )
    % Set the output ports' sampling mode:
    block.OutputPort(1).SamplingMode = 'Sample';
    % Size the output port correctly:
    block.OutputPort(2).Dimensions   = 4;        % inertial velocity (return inertial velovity, _fixed frame)(maybe not needed)
    % Set the output ports' sampling mode:
    block.OutputPort(2).SamplingMode = 'Sample';
    
    % Register the continuous states:
    block.NumContStates = 8;                     % 8 continous states - pos change to 3
    
    % Define the number of parameters:
    block.NumDialogPrms = 9;                     % number of parameters the s fucntion accepts

    % Set block sample time to continuous:
    block.SampleTimes = [0,0];
    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
    block.RegBlockMethod('PostPropagationSetup', @PostPropagationSetup);
    block.RegBlockMethod('InitializeConditions', @InitialConditions);
    block.RegBlockMethod('Outputs',              @Output);    
    block.RegBlockMethod('Derivatives',          @Derivative);
end

%% Set up the dynamic work vector:
function PostPropagationSetup(block)                % setting up memory - does work without
    % Setup Dwork:
    block.NumDworks                = 1;
    block.Dwork(1).Name            = 'inertial_velocity'; 
    block.Dwork(1).Dimensions      = 4;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
end

%% Initialisitation:
function InitialConditions(block)                       % inital  conditions
    % Initialise the continuous states:
    block.ContStates.Data = block.DialogPrm(9).Data;    % initial conditions
    % Initialise the dynamic work vector:
    cos_psi = block.DialogPrm(9).Data(3);
    sin_psi = block.DialogPrm(9).Data(3);
    cos_phi = block.DialogPrm(9).Data(3);
%     J = [cos_psi,sin_psi,0;-sin_psi,cos_psi,0;0,0,1];       %%
    R = [cos_psi, -cos_phi*sin_psi; sin_psi, cos_phi*cos_psi]; %phi & psi (2.18), theta = 0;
    % R dimensions 2x2, T dimensions 2x2 - x, y, phi, psi
    % Rotational transformation matrix:
    T = [1, 0; 0, cos_phi];
    J = [R,zeros(2,2);zeros(2,2),T];
    block.Dwork(1).Data = J*block.DialogPrm(9).Data(1:4); %initial conditions
end

%% Output the continuous states:
function Output(block)   
    % Output the continuous states:
    block.OutputPort(1).Data = block.ContStates.Data;
    % Output the inertial velocity:
    block.OutputPort(2).Data = block.Dwork(1).Data;
end

%% Compute the derivative of the continuous states:
% dx/dt = f(x);
function Derivative(block)
    % Extract the input parameters:
    prp  = block.DialogPrm(1).Data;     %propeller data
    act  = block.DialogPrm(2).Data;     %actuator data
    rbd  = block.DialogPrm(3).Data;     %ridgied body data
    X    = block.DialogPrm(4).Data;
    Y    = block.DialogPrm(5).Data;
    N    = block.DialogPrm(6).Data;
    K    = block.DialogPrm(7).Data;
    Minv = block.DialogPrm(8).Data;
    
    % Extract the input vector:
    in = block.InputPort(1).Data;
    
    % Extract the continuous states:
    x = block.ContStates.Data;
    
    % To make the code clearer, copy all desired parameters and states:
    % Input vector:
%     thrust = in(1);      % thrust [N]
    Wv = in(1);           % Prevailing wind speed [m/s]
    W_theta = in(2);      % Prevailing wind angle [degrees]
    delta  = in(3);      % rudder angle [rad]
    
    % State vector:
    phi   = x(3);
    psi   = x(4);
    u     = x(5);
    v     = x(6);
    p     = x(7);
    r     = x(8);
    %add delta, phi etc
    
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
    eta = [0, 0, phi, psi]';            %% should be eta = [x, y, phi, psi]';
    nu =  [u, v, p, r]';
    
    %% Control
    % Contol calculation % example Bow 2 sail output lift - move out of equations of motion
    % Apparant and acutal wind   - (Independant of ship angle, dependent on course path) is it global coordinates?
    Vs = u^2 + v^2;                                      % Ship speed [m/s]
    S_theta = atand(v/u);                               % Ship course angle [degrees] (atan for rads) - global
    TWA = 180-S_theta-W_theta;                         % True wind angle [degrees] - local (oriantated to ship course)
    Va = sqrt(Vs^2+Wv^2-2*Vs*Wv*cos(TWA));        % Apparant wind speed [m/s]
    AWA = asind((Wv/Va)*sind(TWA));               % Apparant wind angle (check) [degrees] - local (oriantated to ship course)
        
    AOA = 20;       % example sail angle of attack - local (oriantated to apparent wind angle)
    Cl_angle = [0.2,5.6,6.8,8.0,10.5,12.6,15.0,17.5,25.7,30.2,40.2,50.3,59.7,69.2,79.9,90.3,99.7,110.1,120.2,129.6,140.3,150.9,160.0,165.2,170.1,175.8,180.0];
    Cl_data = [0.185,0.896,1.032,1.085,1.100,1.040,0.987,1.010,1.259,1.426,1.509,1.412,1.034,0.657,0.332,0.038,-0.279,-0.565,-0.867,-1.290,-1.471,-1.410,-1.122,-1.046,-1.235,-1.023,-0.464];
    tic;
    Cl = interp1(Cl_angle,Cl_data,AOA);     % Coeffent of lift
    Cd_angle = [1.9,5.6,7.8,10.2,12.6,15.0,17.4,20.1,24.9,30.0,39.7,50.2,60.3,70.2,90.0,100.4,110.3,120.1,130.3,140.1,150.5,160.6,165.4,170.1,175.5];
    Cd_data = [0.025,0.032,0.078,0.172,0.272,0.326,0.372,0.439,0.566,0.800,1.201,1.602,1.735,1.721,1.915,1.881,1.760,1.613,1.652,1.311,0.836,0.407,0.300,0.179,0.039];
    Cd = interp1(Cd_angle,Cd_data,AOA);     % Coeffent of drag
    toc
    
    rho = 1.225;        % Move to setup?
    area = 1;           % Sail area move to setup?
%     lift = 0.5*Cl*rho*area*Va^2;  % lift [N]
%     drag = 0.5*Cd*rho*area*Va^2;  % drag [N]
    F_net = 0.5*sqrt(Cl^2+Cd^2)*rho*area*Va^2;  % force on sail [N]
    F_theta_l = atand(Cd/Cl);                     % angle on sail [degrees] (atan for rads) - local (oriantated to 90deg of AWA)
%     Far = F_net*sind(AWA-F_theta_l);               % aerodynamic driving force [N] (atan for rads) - local (oriantated to course)
%     Fas = F_net*cosd(AWA-F_theta_l);               % aerodynamic side force [N] (atan for rads) - local (oriantated to course)
    F_theta = (90+F_theta_l-AWA-S_theta);            % angle on sail [degrees] - global (oriantated to x and y)
    F_surge = F_net*cos(F_theta);                    % surge force on ship [N] - global (x co-ordinates)
    F_sway = F_net*sin(F_theta);                     % sway force on ship [N] - global (y co-ordinates)
    
    % To do - compute differences due to 2 sails
    % Surge and sway force is just the sum of both sails
    % Yaw force will be generated due to two sail interaction
    
    rudder_coefficient = 0;         % maybe move to setUpDynamic?
%     control = [thrust;0;0;delta*rudder_coefficient*u2];  % [surge,sway,roll,yaw]
    control = [F_surge;F_sway;0;delta*rudder_coefficient*u2];  % [surge,sway,roll,yaw]     (may want to comute differences due to 2 sails)
    
    %% equations of motion
    % 4dof equation of motion
    f = -Crb*nu-Ca*nu-D*nu-G*eta + control;  % check sign f=control-...
    dxdt2 = Minv*f;
    
    % Compute the right-hand side of the propulsion vector:
    % [dn/dt;du_p/dt] = f3(t);  %%
%     dxdt3 = [(Q-Kn*n-Qnn*n_n)/Jm;...                                
%         (Tnn*n_n-CDl*up-CDq*abs(up)*(u-(1-wp)*u))/mf];
    
    % Store the inertial velocity as a work vector:
    block.Dwork(1).Data = dxdt1;
    
    % Compute the derivative vector dx/dt:
    block.Derivatives.Data = double([dxdt1;dxdt2]); %;dxdt3];
end