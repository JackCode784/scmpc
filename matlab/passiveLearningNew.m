%% Passive learning SCMPC
% 
clc;
clear;
close all;

rng("default"); % set random seed

allZonsPlot = false;    % plot all simulation zonotopes 
useSlack = true;        % use slack variables   
interruptBool = true;   % if optimization fails at some point, don't try again
scnConstr = true;       % include scenarios constraints
scnCost = true;         % include scenarios in cost

% Script doesn't work for every feasible value of these hyperparameters
nSim = 300;
tHzn = 10;
nhoru = 5;
nScen = 4;
sys = 'buckloss';
refstr = 'square2';

% Cost weight matrices
Q = 5;
R = 0.1;
noiseAmp = 0.02;

disp("ARX Passive learning scenario-based MPC:");
disp(['System: ', sys]);
disp(['Reference: ', refstr]);

[na, nb, nk, thetaTrue, Z0, uMax, uMin, yMax, yMin, yRef] = selectSys(sys, refstr, nSim, tHzn);

assert(tHzn >= nk, 'Prediction horizon too short!');
assert(tHzn - nk >= nhoru - 1, 'Inconsistent control horizon');
assert(noiseAmp > 0, 'There must be some noise.');

ZReds = cell(nSim+1,1);
ZReds{1} = Z0;
n = na + nb;

%% Yalmip setup
yalmip('clear');

% Optimization variables

% For the input vector, we have at instant k:
% 
% y(k) <-- u(k-nk), ..., u(k-nk-nb+1) for every k
% 
% Since we need y(k),...,y(k+tHzn-1), we'll need u(k-nk-nb+1),...,u(k-nk)
% to compute y(k), then u(k-nk+1),...,u(k-1),u(k),...,u(k-nk+tHzn-1). 
% u(k-1) is the first input sample to be optimized. So it results
% 
% Index: 1, ..., nk+nb-1, nk+nb,..., nb+nk+nhoru-2,..., nb+tHzn-1
% NhorU:                    1     2   ,...,     nhoru
% u = [u(k-nk-nb+1),..., u(k-1), u(k) ,..., u(k+NhorU-2),...,u(k+tHzn-1-nk)]
%     |_______________________| |__________________________________________|
%           nk+nb-1 samples                 tHzn-nk >= 0 samples
% Also, it must be true that tHzn-1-nk >= nhoru-2 <=> tHzn-nk >= nhoru-1,
% which is one of the asserts on top of the script.
% For a total of tHzn+nb-1 samples.
u = sdpvar(1,tHzn+nb-1);
uInit = sdpvar(1,nb+nk-2); % written in reverse order (recent -> old)

% As for the outputs, they need exactly na initial conditions (to compute
% the next output) and tHzn from the problem formulation.
% 
% At instant k-1:
% y = [y(k-na),...,y(k-1),y(k),...,y(k + tHzn - 1)]
% 
yNominal = sdpvar(1,tHzn+na);
yScenarios = sdpvar(nScen, tHzn+na);
yInit = sdpvar(1,na); % written in reverse order (recent -> old)
yRefVar = sdpvar;

thetaNominal = sdpvar(n,1);
thetaScenarios = sdpvar(n,nScen);

% Slack
slack = sdpvar;

%% Constraints definition
constr = [yNominal(1:na) == yInit(end:-1:1);
    u(1:nb+nk-2) == uInit(end:-1:1);
    u(nk+nb+nhoru-1:end) == u(nk+nb+nhoru-2); % input unchanged after control horizon
    uMin <= u(nb+nk-1:end);
    u(nb+nk-1:end) <= uMax;
    yMin - useSlack*slack <= yNominal(na+1:end);
    yNominal(na+1:end) <= yMax + useSlack*slack;
    useSlack*slack >= 0;
    ];

for t=1:tHzn
    constr = [constr; yNominal(t+na) == [yNominal(t+na-1:-1:t), u(t+nb-1:-1:t)] * thetaNominal];
end

if scnConstr
    constr = [constr; yScenarios(:,1:na) == ones(nScen, 1) * yInit(end:-1:1)];
    constr = [constr; yMin - useSlack*slack <= yScenarios(:,na+1:end);
    yScenarios(:,na+1:end) <= yMax + useSlack * slack];

    for l=1:nScen
        for t=1:tHzn
            constr = [constr; yScenarios(l,t+na) == [yScenarios(l,t+na-1:-1:t), u(t+nb-1:-1:t)] * thetaScenarios(:,l)];
        end
    end
end

%% Cost function
cost = 0;
cost = cost + (u(nb+nk-1:end) - u(nb+nk-2:end-1)) * R * (u(nb+nk-1:end) - u(nb+nk-2:end-1))';
cost = cost + (yNominal(na+1:end) - yRefVar) * Q * (yNominal(na+1:end) - yRefVar)';
cost = cost + useSlack*slack^2;

if scnCost && scnConstr
    for l=1:nScen
        cost = cost + (yScenarios(l,na+1:end) - yRefVar) * Q / nScen * (yScenarios(l,na+1:end) - yRefVar)';
    end
end

%% Optimizer object
inputs = {thetaScenarios, thetaNominal, yInit, uInit, yRefVar};
outputs = {u, yNominal, yScenarios, slack};
ops = sdpsettings('solver', 'quadprog', 'verbose', 1, 'usex0', 0);
scmpc = optimizer(constr, cost, ops, inputs, outputs);

%% Simulation
ySim = zeros(1,nSim);
uSim = zeros(1,nSim);
vols = zeros(1,nSim);
slackSeq = zeros(1,nSim);
dists = zeros(1,nSim);
thetaSeq = zeros(n,nSim);

yInit = zeros(1,na); % recent -> old

% uInit = [u(k-1),...,u(k-nk),...,u(k-nk-nb+1)]
%         |_____|
%       to be optimized
uInit = zeros(1,nb+nk-1); % recent -> old

% Simulation loop
for k=1:nSim
    % Output measurement
    noise = 2*noiseAmp*(0.5-rand);
    ySim(k) = [yInit, uInit(nk:nb+nk-1)] * thetaTrue + noise;

    % Various measurements
    vols(k) = volume(ZReds{k});
    dists(k) = norm(thetaTrue - ZReds{k}.c);
    thetaSeq(:, k) = ZReds{k}.c;

    % Zonotope update
    ZReds{k+1} = boundStripZonotopeInt(ZReds{k}, ySim(k), yInit, uInit, noiseAmp, nb, nk, 'new');
    yInit = [ySim(k), yInit(1:end-1)];

    % Optimization
    [thetaScenarios, thetaNominal] = stableScenary(ZReds{k+1}, nScen, na);
    inputs = {thetaScenarios, thetaNominal, yInit, uInit(1:nb+nk-2), yRef(k+1)};
    [optimalVars, errCode] = scmpc(inputs);

    if errCode == 1
        warning(['Unfeasible... at k = ', num2str(k)]);
        if interruptBool
            disp('Run interrupted.');
            break;
        else
            while(errCode == 1)
                [thetaScenarios, thetaNominal] = scenary(ZReds{k+1}, nScen);
                inputs = {thetaScenarios, thetaNominal, yInit, uInit(1:nb+nk-2), yRef(k+1:k+tHzn)};
                [optimalVars, errCode] = scmpc(inputs);
            end
            disp('Feasible optimization found!!!');
        end
    end

    slackSeq(k) = optimalVars{4};      % retrieve slack for this k
    uSim(k) = optimalVars{1}(nb+nk-1); % retrieve first optimized input
    uInit = [uSim(k), uInit(1:end-1)]; % input initial conditions update
end

%% Error computation 
err = ySim - yRef(1:nSim);
errRmse = sqrt(sum(err.^2)/length(err));

disp(['Slack mean: ', num2str(mean(slackSeq))]);

%% Zonotope plots
lastNotEmptyIdx = find(~cellfun('isempty', ZReds), 1, 'last');

figure;
hold on;
if allZonsPlot
    cmap = turbo(lastNotEmptyIdx);
    for l=1:lastNotEmptyIdx
        plot(ZReds{l}, 1:3, 'LineWidth',1.5, 'Color', cmap(l,:), 'HandleVisibility','off');
    end
    colorbar;
    colormap(cmap);
    clim([1, lastNotEmptyIdx]);
else
    plot(ZReds{1}, 1:3, 'LineWidth',1.5, 'DisplayName','\Theta_0');
    % plot(ZNonReds{lastNotEmptyIdx}, 1:3, 'LineWidth',1.5,'DisplayName',append('Last NR k=', num2str(k)));
    plot(ZReds{lastNotEmptyIdx}, 1:3, 'LineWidth',1.5,'DisplayName',append('\Theta_r(k=', num2str(lastNotEmptyIdx), ')'));
    grid on;
end
% plot3(ZNonReds{1}.c(1), ZNonReds{1}.c(2), ZNonReds{1}.c(3),'.','LineWidth',1.5, 'MarkerSize',10,'DisplayName','\theta_n^{NR}(1)');
plot3(ZReds{1}.c(1), ZReds{1}.c(2), ZReds{1}.c(3),'.','LineWidth',1.5, 'MarkerSize',20, 'DisplayName','\theta_n^{R}(1)');
% plot3(ZNonReds{lastNotEmptyIdx}.c(1), ZNonReds{lastNotEmptyIdx}.c(2), ZNonReds{lastNotEmptyIdx}.c(3),'.','LineWidth',1.5, 'MarkerSize',10, 'DisplayName','\theta_n^{NR}(k)');
plot3(ZReds{lastNotEmptyIdx}.c(1), ZReds{lastNotEmptyIdx}.c(2), ZReds{lastNotEmptyIdx}.c(3),'.','LineWidth',1.5, 'MarkerSize',20, 'DisplayName','\theta_n^{R}(k)');
plot3(thetaTrue(1), thetaTrue(2), thetaTrue(3), '*', 'LineWidth',1.5,'MarkerSize',10, 'DisplayName','\theta_t');
grid on;
xlabel('\theta_1');
ylabel('\theta_2');
zlabel('\theta_3');
title('Zonotopes');
legend('-DynamicLegend');
legend('boxoff');

%% Simpler plots
figure;
subplot(2,1,1);
hold on;
plot(ySim, 'LineWidth', 1.5);
plot(yRef(1:nSim), 'LineWidth', 1.5, 'LineStyle', '-.');
plot(0*ySim+yMax, 'k--');
plot(0*ySim+yMin, 'k--');
hold off;
grid on;
legend('Simulated output','Power reference', 'max', 'min');
xlabel('Time step');
title('System true output');

subplot(2,1,2);
hold on;
plot(err, 'LineWidth', 1.5);
plot([1, nSim], [errRmse, errRmse], 'LineWidth', 1.5, 'LineStyle','--','Color','r');
plot([1 nSim], mean(err)*[1, 1], 'LineWidth',1.5,'Color', 'k');
hold off;
grid on;
legend('Error', 'RMSE', 'Mean');
ylabel('Error');
xlabel('Time step');

figure;
hold on;
plot(uSim, 'LineWidth',1.5);
plot(0*uSim + uMin, 'k--');
plot(0*uSim + uMax, 'k--');
grid on;
title('Optimal control');
xlabel('Time step');

figure;
subplot(3,1,1);
plot(vols / volume(Z0), 'LineWidth',1.5);
grid on;
title('Uncertainty region volumes (reduced)');
xlabel('Time step');
ylabel('vol(Zr(t))/vol(Z0)');

subplot(3,1,2);
plot(vols(2:end) ./ vols(1:end-1), 'LineWidth',1.5);
grid on;
xlabel('Time instant');
ylabel('vol(Zr(t))/vol(Zr(t-1))');

subplot(3,1,3);
plot(dists / norm(thetaTrue), 'LineWidth',1.5);
xlabel('Time step');
ylabel('||\theta_t-\theta_n|| / ||\theta_t||');
grid on;

% figure;
% for i=1:n
%     subplot(n,1,i);
%     hold on;
%     plot(thetaSeq(i,:), 'LineWidth',1.5);
%     plot(ones(1,nSim) * thetaTrue(i), '--k');
%     hold off;
%     legend('', '\theta_t');
%     legend('boxoff');
%     grid on;
%     xlabel('Time instant');
%     ylabel(append('\theta_', num2str(i)));
% end
