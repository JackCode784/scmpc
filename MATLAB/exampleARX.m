%% ARX model example
% Input/output size is assumed to be 1 ("SISO" system).

clc;
clear;
close all;

%% System model as ARX
% MPC is applied for a generic ARX-modeled system.
% Assumption: there is no algebric dependence between input and output,
% hence nd > 0. We will optimize u(k) after measuring y(k).
% y(k) = a_1 * y(k-1) + ... + a_na * y(k-na) + 
%        b_1 * u(k-nd) + ... + b_nb * u(k-nd-nb+1)
%      = Phi * theta

Ts = 1e-3;  % sampling time

% Define system parameter vector
na = 2;                             % number of output coefficients
nb = 1;                             % number of input coefficients
nd = 2;                             % number of system delay samples
n = na + nb;                        % number of theta parameters
% thetaNominal = -ones(n, 1) / 5;  % random parameter vector in [-1, 1]^n
% thetaNominal = [0.792111637807507
% 0.152403919943540
% -0.166763123012609
% 0.0842386217714551
% 0.0441681636826590];
A = 1e3;
B = 1e3;
% thetaNominal = [2; -1; 0; 0; A*B*Ts^2];
thetaNominal = [2; -1; A*B*Ts^2];

realSys = arxSys(na, nb, nd, [2; -1; 1.1], Ts);

% Create an ARX object
arxModel = arxSys(na, nb, nd, thetaNominal, Ts);

%% MPC controller design

Nhor = 5;  % Prediction horizon
NhorU = 3; % Control horizon

% MPC weight matrices (random choices)
P = 1;  % ny * ny
Q = 1;  % ny * ny
R = 10; % nu * nu

% Constraints
ymin = 0;  % minimum output constraint
ymax = 8;   % maximum output constraint
umin = -0.3; % minimum input constraint
umax = 0.3; % maximum input constraint

%% Robust MPC

Nscen = 2;   % number of scenarios
sysVec = cell(Nscen+1,1); % scenarios + nominal system
sysVec{1} = arxModel;

% Generation of scenarios
Z0c = thetaNominal;     % zonotope (box) center
Z0Gen = eye(n) / 10;    % zonotope generators matrix (stored as columns)

%% Choose if using normal or robust MPC
% robust = 1;

%% Closed-loop simulation with MPC controller
% Define an initial condition
yPast = zeros(1, na); % output initial conditions for each scenario
uPast = zeros(1, nb+nd-2); % input initial condition, same for all scenarios

% Define a reference output
yref = 5;

% Set a simulation time
Tsim = 50e-3;

% Closed-loop simulation
t = 0:Ts:Tsim;                   % every time instant in which control is given, time axis
uSim = zeros(1, length(t));      % record of resulting input
ySim = zeros(1, length(t));  % record of resulting outputs

uSamples = [0, uPast]; % [u(k-1), ..., u(k-nb-nd+1)]
for k=1:length(t) % for each simulation time instant
    % First, measure current output y(k) from [u(k-1), ..., u(k-nb-nd+1)]
    ySim(k) = arxModel.computeOutput(yPast, uSamples);    

    % Then, compute optimal input u(k)
    % internally update initial conditions for each scenario
    [uOpt, yPast, uSamples] = generateSCMPCControl(arxModel, ySim(k), yPast, uSamples, yref, Nhor, NhorU, Q, P, R, umax, umin, ymax, ymin, Nscen);
    uSim(k) = uOpt(1);  % optimized input
end

% Plots for nominal system
figure;
subplot(2,1,1);
hold on;
plot(t, ySim(1,:), 'LineWidth',1.5);
plot(t, 0*ySim(1,:)+yref, '--r');
plot(t, 0*ySim(1,:)+ymax, '--k');
plot(t, 0*ySim(1,:)+ymin, '--k');
legend('y(k)', 'yref', 'ymax', 'ymin');
grid on;
hold off;
xlabel('Time instant');
ylabel('ARX output');
title('Nominal system output');

subplot(2,1,2);
hold on;
stairs(t, uSim, 'LineWidth',1.5);
plot(t, 0*uSim+umax, '--k');
plot(t, 0*uSim+umin, '--k');
legend('u(k)', 'umax', 'umin');
grid on;
hold off;
xlabel('Time instant');
ylabel('Optimized input at instant k');

%% Simulate on different systems
% Plots for all scenarios

% figure;
% hold on;
% plot(t, ySim(2:end, :), 'LineWidth',1.5);
% plot([t(1), t(end)], [ymin, ymin], '--k');
% plot([t(1), t(end)], [ymax, ymax], '--k');
% plot([t(1), t(end)], [yref, yref], '--r');
% hold off;
% grid on;
% xlabel('Time [s]');
% ylabel("Signal output");
% title(append(num2str(Nscen), " scenarios' outputs"));

%% Generate Simulink model

% % ctSysReg.generateSimulinkModel();
%
% simulinkOptions = struct( ...
%                 'simVHDL', 1 ...
%     );
%
% % Set the signals range for circuit implementation
% range.xmin = ymin;
% range.xmax = ymax;
% range.umin = umin;
% range.umax = umax;
% range.pmin = [];
% range.pmax = [];
% range.dmin = [];
% range.dmax = [];
% range.xrefmin = ymin(1);
% range.xrefmax = ymax(1);
% range.ymin = ymin(1);
% range.ymax = ymax(1);
%
% ADMMparameters.regPar = 2;   % Regularization parameter
%     ADMMparameters.maxIter = 20; % Number of iterations
%
%     vhdlOptions = struct( ...
%                'architecture', 'fast',            ... % hardware architecture (can be 'fast' or 'small')
%                'inputResolution', 16,             ... % resolution of the inputs
%                'inputRepresentation', 'unsigned', ... % representation of the inputs
%                'coeffResolution', 18,             ... % resolution of the coefficients
%                'coeffIntResolution', 10,           ... % integer part of the coefficients (only for implicit MPC)
%                'outputResolution', 16,            ... % resolution of the outputs
%                'outputRepresentation', 'unsigned',... % representation of the outputs
%                'frequency', 5e7,                  ... % FPGA working frequency
%                'useADC', 1,                       ... % manage ADC scalings
%                'useDAC', 1,                       ... % manage DAC scalings
%                'range', range,                    ... % signals range
%                'fpgaBoard', 'xc7z020clg484-1',    ... % target FPGA (only for implicit MPC)
%                'ADMMparameters', ADMMparameters,  ... % parameters for ADMM algorithm (only for implicit MPC)
%                'defaultOutput', 0.5               ... % output initial value
%         );

% ctSysReg.generateSimulinkModel(simulinkOptions, vhdlOptions);

%% Create VHDL files

% Set the signals range for circuit implementation
% range.xmin = ymin;
% range.xmax = ymax;
% range.umin = umin;
% range.umax = umax;
% range.pmin = [];
% range.pmax = [];
% range.dmin = [];
% range.dmax = [];
% range.xrefmin = ymin(1);
% range.xrefmax = ymax(1);
% range.ymin = ymin(1);
% range.ymax = ymax(1);
%
% vhdlOptions = struct( ...
%                'architecture', 'fast',            ... % hardware architecture (can be 'fast' or 'small')
%                'inputResolution', 12,             ... % resolution of the inputs
%                'inputRepresentation', 'unsigned', ... % representation of the inputs
%                'coeffResolution', 18,             ... % resolution of the coefficients
%                'coeffIntResolution', 5,           ...
%                'outputResolution', 12,            ... % resolution of the outputs
%                'outputRepresentation', 'unsigned',... % representation of the outputs
%                'frequency', 1e5,                  ... % FPGA working frequency
%                'fpgaBoard','zynq',              ...
%                'range', range                     ... % signals range
%         );
%
% rob_ctrl.generateVHDL(vhdlOptions);
