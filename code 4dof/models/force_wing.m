function force_wing(block)
% ship_dynamics.m     mail     08/07/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the modelling of the dynamics of
% an sail in 4 DOF.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the input ports correctly:
    block.InputPort(1).Dimensions    = 3;    % control input vector (ship vector)
    % Specify whether there is direct feedthrough:
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).SamplingMode  = 'Sample';
    % Size the input ports correctly:
    block.InputPort(2).Dimensions    = 2;    % control input vector (wind vector)
    % Specify whether there is direct feedthrough:
    block.InputPort(2).DirectFeedthrough = false;
    block.InputPort(2).SamplingMode  = 'Sample';
    
    % Size the output port correctly:
    block.OutputPort(1).Dimensions   = 4;        % Sail forces (return sail forces) (Fwing_x,Fwing_y,Fwing_yaw,Fwing_roll)
    % Set the output ports' sampling mode:
    block.OutputPort(1).SamplingMode = 'Sample';
    % Size the output port correctly:

    % Register the continuous states:
%     block.NumContStates = 8;                     % 8 continous states - to change
    block.NumContStates = 8;                     % 8 continous states - to change
    
    % Define the number of parameters:
    block.NumDialogPrms = 9;                     % number of parameters the s fucntion accepts - change

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
%     cos_psi = block.DialogPrm(9).Data(3);
%     sin_psi = block.DialogPrm(9).Data(3);
%     cos_phi = block.DialogPrm(9).Data(3);
%     R = [cos_psi, -cos_phi*sin_psi; sin_psi, cos_phi*cos_psi]; %phi & psi (2.18), theta = 0;
%     % R dimensions 2x2, T dimensions 2x2 - x, y, phi, psi
%     % Rotational transformation matrix:
%     T = [1, 0; 0, cos_phi];
%     J = [R,zeros(2,2);zeros(2,2),T];
%     block.Dwork(1).Data = J*block.DialogPrm(9).Data(1:4); %initial conditions
    block.Dwork(1).Data = block.DialogPrm(9).Data(1:4); %initial conditions
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
    % Extract the input parameters:      % to change
    wing  = block.DialogPrm(1).Data;     % wing data
    d  = block.DialogPrm(2).Data;     % distance to COG
    Cl_angle = block.DialogPrm(3).Data;     
    Cl_data = block.DialogPrm(4).Data;      
    Cd_angle = block.DialogPrm(5).Data;     
    Cd_data = block.DialogPrm(6).Data;      
    wind_oriantation = block.DialogPrm(7).Data;     
    Cl_ratio = block.DialogPrm(8).Data;      
    Cd_ratio = block.DialogPrm(9).Data;   
    % ^add with lift draf coeffents
    
    % Extract the input vector:
    in = block.InputPort(1).Data;
    
    % Extract the continuous states:      % to change maybe
    x = block.ContStates.Data;
    
    % To make the code clearer, copy all desired parameters and states:
    % Input vectors:
    ship_velocity = in(1)          % ship velocity
    u = ship_velocity(1)           % ship velocity x [m/s]
    v = ship_velocity(2)           % ship velocity y [m/s]
    phi = ship_velocity(3)         % ship yaw angle [rads]
    
    wind_vector = in(2);           % Prevailing wind vector
    Wv = wind_vector(1);           % Prevailing wind speed [m/s]
    W_theta = wind_vector(2);      % Prevailing wind angle [degrees]
    
    % State vector:       % to change
    phi   = x(3);
    psi   = x(4);
    u     = x(5);
    v     = x(6);
    p     = x(7);
    r     = x(8);
    
    % Parameters:       % to change
    rho   = wing(1);                 % density air  
    area  = wing(2);                 % Sail area
    height  = wing(3);               % height of wing         
    
    % conpute the 4dof force generated by the wing 
    % Apparant and acutal wind   - (Independant of ship angle, dependent on course path) is it global coordinates?
    Vs = u^2 + v^2;                                      % Ship speed [m/s]
    S_theta = atand(v/u);                               % Ship course angle [degrees] (atan for rads) - global
    TWA = 180-S_theta-W_theta;                         % True wind angle [degrees] - local (oriantated to ship course)
    Va = sqrt(Vs^2+Wv^2-2*Vs*Wv*cos(TWA));        % Apparant wind speed [m/s]
    AWA = asind((Wv/Va)*sind(TWA));               % Apparant wind angle (check) [degrees] - local (oriantated to ship course)
        
    AOA = 20;       % example sail angle of attack - local (oriantated to apparent wind angle) data taken from J. Robert and B. G. Newman (1979)
%     Cl_angle = [0.2,5.6,6.8,8.0,10.5,12.6,15.0,17.5,25.7,30.2,40.2,50.3,59.7,69.2,79.9,90.3,99.7,110.1,120.2,129.6,140.3,150.9,160.0,165.2,170.1,175.8,180.0];
%     Cl_data = [0.185,0.896,1.032,1.085,1.100,1.040,0.987,1.010,1.259,1.426,1.509,1.412,1.034,0.657,0.332,0.038,-0.279,-0.565,-0.867,-1.290,-1.471,-1.410,-1.122,-1.046,-1.235,-1.023,-0.464];
    Cl1 = interp1(Cl_angle,Cl_data,AOA);     % Coeffent of lift
%     Cd_angle = [1.9,5.6,7.8,10.2,12.6,15.0,17.4,20.1,24.9,30.0,39.7,50.2,60.3,70.2,90.0,100.4,110.3,120.1,130.3,140.1,150.5,160.6,165.4,170.1,175.5];
%     Cd_data = [0.025,0.032,0.078,0.172,0.272,0.326,0.372,0.439,0.566,0.800,1.201,1.602,1.735,1.721,1.915,1.881,1.760,1.613,1.652,1.311,0.836,0.407,0.300,0.179,0.039];
    Cd1 = interp1(Cd_angle,Cd_data,AOA);     % Coeffent of drag
    
    % Scale coeffents of drag based on data from Bordogna, G et al., (2018)
    % - currently using bow data
    W_O = AWA + S_theta - phi;    % wind oriantation angle - angle between wind and ship angle
%     wind_oriantation = [20,30,35,45,60,75,90,105,120,135,150,165,180];
%     Cl_ratio_bow = [1.26,1.41,1.38,1.38,1.30,1.26,1.20,1.18,1.13,0.93,0.97,-9.66,0.00];
%     Cd_ratio_bow = [1.32,1.37,1.43,1.55,1.53,1.49,1.43,1.42,1.32,0.97,1.04,1.00,-0.15];
    Cl_ratio = interp1(wind_oriantation,Cl_ratio,W_O);
    Cd_ratio = interp1(wind_oriantation,Cd_ratio,W_O);
    Cl2 = Cl_ratio*Cl1;                       % calculating 2 sail Cl
    Cd2 = Cd_ratio*Cd1;                       % calculating 2sail Cd
       
%     lift = 0.5*Cl*rho*area*Va^2;  % lift [N]
%     drag = 0.5*Cd*rho*area*Va^2;  % drag [N]
    F_net = 0.5*sqrt(Cl2^2 + Cd2^2)*rho*area*Va^2;   % force on sail [N]
    F_theta_l = atand(Cd2/Cl2);                      % angle on sail [degrees] (atan for rads) - local (oriantated to 90deg of AWA)
%     Far = F_net*sind(AWA-F_theta_l);               % aerodynamic driving force [N] (atan for rads) - local (oriantated to course)
%     Fas = F_net*cosd(AWA-F_theta_l);               % aerodynamic side force [N] (atan for rads) - local (oriantated to course)
    F_theta = (90 + F_theta_l - AWA - S_theta);      % angle on sail [degrees] - global (oriantated to x and y)
    F_surge = F_net*cosd(F_theta);                    % surge force on ship [N] - global (x co-ordinates)
    F_sway = F_net*sind(F_theta);                     % sway force on ship [N] - global (y co-ordinates)
    
    % compute Yaw and Roll tourque
    Fas = F_surge*cosd(phi) + F_sway*cosd(phi);       % aerodynamic side force [N]
    T_yaw = Fas*d;                                    % yaw torque on ship [Nm]
    T_roll = Fas*height;                              % roll torque on ship [Nm]
            
    control = [F_surge;F_sway;T_yaw;T_roll];            % [surge,sway,roll,yaw]
        
    % Store the inertial velocity as a work vector:
    block.Dwork(1).Data = control;
    
%     % Compute the derivative vector dx/dt:
%     block.Derivatives.Data = double(control);
end