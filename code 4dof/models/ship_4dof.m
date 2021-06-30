function ship_4dof(block)
% remus_dynamics.m     e.anderlini@ucl.ac.uk     07/05/2019
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
    block.InputPort(1).Dimensions    = 2;    % control input vector (thrust, rudder angle)
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
    thrust = in(1);      % thrust [N]
    delta  = in(2);      % rudder angle [rad]
    
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
    % 4dof equation of motion
%     control = [thrust;0;0;delta*rudder_coefficient*u2]; % ???? [surge,sway,roll,yaw]
    
%     f = Crb*nu+Ca*nu+D*nu+G*eta + control;  % check sign f=control-...
    f = Crb*nu+Ca*nu+D*nu+G*eta;
    dxdt2 = Minv*f;
    
    % Compute the right-hand side of the propulsion vector:
    % [dn/dt;du_p/dt] = f3(t);  %%
%     dxdt3 = [(Q-Kn*n-Qnn*n_n)/Jm;...                                % will change - (just equation got propultion thrust?)?
%         (Tnn*n_n-CDl*up-CDq*abs(up)*(u-(1-wp)*u))/mf];
    
    % Store the inertial velocity as a work vector:
    block.Dwork(1).Data = dxdt1;
    
    % Compute the derivative vector dx/dt:
    block.Derivatives.Data = double([dxdt1;dxdt2]); %;dxdt3];
end