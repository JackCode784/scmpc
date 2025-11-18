%% ARX model example
% Input/output size is assumed to be 1 ("SISO" system).

clc;
clear;
close all;

%% System model as ARX
% MPC is applied for a generic ARX-modeled system.
% y(k) = a_1 * y(k-1) + ... + a_na * y(k-na) + b_1 * u(k-nd) + ... + 
%        b_nb * u(k-nd-nb+1)
%      = Phi * theta

% Define the sampling time
Ts = 1e-3;

% Define system parameter vector
na = 3;                     % number of output coefficients
nb = 2;                     % number of input coefficients
nd = 2;                     % number of system delay samples
n = na + nb;                % number of theta parameters
thetaNominal = rand(n, 1) * 2 - 1; % random parameter vector in [-1, 1]^n
nyref = 1;                  % number of output reference

% Create an ARX object
arxModel = arxSys(na, nb, nd, thetaNominal, Ts);

%% MPC controller design

Nhor = 5;  % Prediction horizon
NhorU = 3; % Control horizon

% MPC weight matrices (random choices)
P = 4;  % ny * ny
Q = 2;  % ny * ny
R = 10; % nu * nu

% Constraints
ymin = -8;  % minimum output constraint
ymax = 8;   % maximum output constraint
umin = -0.5; % minimum input constraint
umax = 0.5; % maximum input constraint

% Constraint matrices
% Ay*y <= by
Ay = [1; -1];
by = [ymax; -ymin];
Au = [1; -1];
bu = [umax; -umin];

%% Robust MPC

nscenarios = 10;
sysVec = cell(nscenarios+1,1);
sysVec{1} = arxModel;

% Generation of scenarios
Z0c = thetaNominal;
Z0Gen = eye(n);
for i=1:nscenarios
    coeffs = rand(n, 1) * 2 - 1;    % coeffs in [-1, 1]
    thetaScen = thetaNominal + Z0Gen * coeffs;
    sysVec{i+1} = arxSys(na, nb, nd, thetaScen, Ts);
end

% rob_ctrl = scenarioMPCctrl(ctsys, Ts, constr, sysVec, options);

%% Choose if using normal or robust MPC
robust = 1;

%% Closed-loop simulation with MPC controller

% Define an initial condition 
yPast = zeros(1, na);
uPast = zeros(1, nb+nd-1);

% Define a reference output
yref = 5;

% Set a simulation time
Tsim = 30e-3;

% Associate controller to LTI system
% if robust
%     ctSysReg = ctsys.setController(rob_ctrl);
% else
%     ctSysReg = ctsys.setController(ctrl);
% end

% Closed-loop simulation
t = 0:Ts:Tsim; % every time instant in which control is given
uSim = zeros(1, length(t)); % record of resulting inputs
ySim = zeros(1, length(t)); % record of resulting outputs
for i=1:length(t) % for each simulation time instant
    uSim(i) = generateSCMPCControl(arxModel, yPast); % WIP...
    ySim(i) = arxModel.computeOutput(yPast, uPast); % WIP...
end

figure;
subplot(2,1,1);
% etc
subplot(2,1,2);
% etc

%% Simulate on different systems

figure
subplot(3,1,1)
hold on
subplot(3,1,2)
hold on
subplot(3,1,3)
hold on

for k = 1:nscenarios

    % Associate controller to LTI system
    if robust
        ctSys2Reg = sysVec{k+1}.setController(rob_ctrl);
    else
        ctSys2Reg = sysVec{k+1}.setController(ctrl);
    end

    % Closed-loop simulation (continuous time)
    signals = ctSys2Reg.sim(Tsim, x0, yref, opts);

    subplot(3,1,1)
    plot(signals.time,signals.state(:,1))
    subplot(3,1,2)
    plot(signals.time,signals.state(:,2))
    plot(signals.time,0*signals.state(:,2)+0.8,'--k')
    subplot(3,1,3)
    plot(signals.time,signals.input)
end

%% Generate Simulink model

% ctSysReg.generateSimulinkModel();

simulinkOptions = struct( ...
                'simVHDL', 1 ...
    );

% Set the signals range for circuit implementation
range.xmin = ymin;
range.xmax = ymax;
range.umin = umin;
range.umax = umax;
range.pmin = [];
range.pmax = [];
range.dmin = [];
range.dmax = [];
range.xrefmin = ymin(1);
range.xrefmax = ymax(1);
range.ymin = ymin(1);
range.ymax = ymax(1);

ADMMparameters.regPar = 2;   % Regularization parameter
    ADMMparameters.maxIter = 20; % Number of iterations

    vhdlOptions = struct( ...
               'architecture', 'fast',            ... % hardware architecture (can be 'fast' or 'small')
               'inputResolution', 16,             ... % resolution of the inputs
               'inputRepresentation', 'unsigned', ... % representation of the inputs
               'coeffResolution', 18,             ... % resolution of the coefficients
               'coeffIntResolution', 10,           ... % integer part of the coefficients (only for implicit MPC)
               'outputResolution', 16,            ... % resolution of the outputs
               'outputRepresentation', 'unsigned',... % representation of the outputs
               'frequency', 5e7,                  ... % FPGA working frequency
               'useADC', 1,                       ... % manage ADC scalings
               'useDAC', 1,                       ... % manage DAC scalings
               'range', range,                    ... % signals range
               'fpgaBoard', 'xc7z020clg484-1',    ... % target FPGA (only for implicit MPC)
               'ADMMparameters', ADMMparameters,  ... % parameters for ADMM algorithm (only for implicit MPC)
               'defaultOutput', 0.5               ... % output initial value
        );

ctSysReg.generateSimulinkModel(simulinkOptions, vhdlOptions);

%% Create VHDL files

% Set the signals range for circuit implementation
range.xmin = ymin;
range.xmax = ymax;
range.umin = umin;
range.umax = umax;
range.pmin = [];
range.pmax = [];
range.dmin = [];
range.dmax = [];
range.xrefmin = ymin(1);
range.xrefmax = ymax(1);
range.ymin = ymin(1);
range.ymax = ymax(1);

vhdlOptions = struct( ...
               'architecture', 'fast',            ... % hardware architecture (can be 'fast' or 'small')
               'inputResolution', 12,             ... % resolution of the inputs
               'inputRepresentation', 'unsigned', ... % representation of the inputs
               'coeffResolution', 18,             ... % resolution of the coefficients
               'coeffIntResolution', 5,           ...
               'outputResolution', 12,            ... % resolution of the outputs
               'outputRepresentation', 'unsigned',... % representation of the outputs
               'frequency', 1e5,                  ... % FPGA working frequency
               'fpgaBoard','zynq',              ...
               'range', range                     ... % signals range
        );

rob_ctrl.generateVHDL(vhdlOptions);
