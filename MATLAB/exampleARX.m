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

Nsc = 2;   % number of scenarios
sysVec = cell(Nsc+1,1);
sysVec{1} = arxModel;

% Generation of scenarios
Z0c = thetaNominal;     % zonotope (box) center
Z0Gen = eye(n) / 10;    % zonotope generators matrix (stored as columns)
for k=1:Nsc
    eta = 0.5*rand(1);
    Bk = 1e3*(1+eta);
    % thetaScen = [2; -1; 0; 0; A*Bk*Ts^2];
    thetaScen = [2; -1; A*Bk*Ts^2];
    sysVec{k+1} = arxSys(na, nb, nd, thetaScen, Ts);
end

%% Choose if using normal or robust MPC
% robust = 1;

%% Closed-loop simulation with MPC controller
% Define an initial condition
yPast = zeros(Nsc+1, na); % output initial conditions for each scenario
uPast = zeros(Nsc+1, nb+nd-1); % input initial condition, same for all scenarios

% Define a reference output
yref = 5;

% Set a simulation time
Tsim = 50e-3;

% Closed-loop simulation
t = 0:Ts:Tsim;                   % every time instant in which control is given, time axis
uSim = zeros(Nsc+1, length(t));      % record of resulting inputs
ySim = zeros(Nsc+1, length(t));  % record of resulting outputs

for k=1:length(t) % for each simulation time instant
    % First, compute optimal input, then compute consequent output
    for l=1:Nsc+1
        uOpt_k = generateSCMPCControl(sysVec, yPast(l,:), uPast(l,:), yref, Nhor, NhorU, Q, P, R, umax, umin, ymax, ymin);
        uSim(l,k) = uOpt_k(1);              % effective input
        uSamples = [uSim(l,k) uPast(l,:)];  % initial conditions update

        % Compute every scenario's output
        ySim(l,k) = sysVec{l}.computeOutput(yPast(l,:), uSamples);

        % New initial conditions
        yPast(l,:) = [ySim(l,k) yPast(l, 1:end-1)];
        uPast(l,:) = uSamples(1:end-1);
    end
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
stairs(t, uSim(1,:), 'LineWidth',1.5);
plot(t, 0*uSim(1,:)+umax, '--k');
plot(t, 0*uSim(1,:)+umin, '--k');
legend('u(k)', 'umax', 'umin');
grid on;
hold off;
xlabel('Time instant');
ylabel('ARX input');

%% Simulate on different systems
% Plots for all scenarios

figure;
subplot(2,1,1);
hold on;
plot(t, ySim(2:end, :), 'LineWidth',1.5);
plot([t(1), t(end)], [ymin, ymin], '--k');
plot([t(1), t(end)], [ymax, ymax], '--k');
plot([t(1), t(end)], [yref, yref], '--r');
hold off;
grid on;
xlabel('Time [s]');
ylabel("Signal output");
title(append(num2str(Nsc), " scenarios' outputs"));

subplot(2,1,2);
hold on;
for i = 2:Nsc+1
    stairs(t, uSim(i,:), 'LineWidth',1.5);
end
plot([t(1), t(end)], [umin, umin], '--k');
plot([t(1), t(end)], [umax, umax], '--k');
xlabel('Time [s]');
ylabel('Input signal');
grid on;

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
