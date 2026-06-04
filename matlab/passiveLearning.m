%% Passive learning SCMPC
% 
clc;
clear;
close all;

rng("default"); % set random seed
allZonsPlot = false; % plot all simulation zonotopes 
scnConstr = true; % include scenarios constraints
scnCost = true; % include scenarios in cost
interruptBool = true;

nSim = 250;
tHzn = 10;
nScen = 5;

% Cost weight matrices
Q = 5;
R = 0.1;
noiseAmp = 0.1;

ZNonReds = cell(nSim+1,1);
ZReds = cell(nSim+1,1);

[na, nb, nk, thetaTrue, Z0, sigma, uMax, uMin, yMax, yMin, yRef] = selectSys('buck', 'square2', nSim, tHzn);

ZNonReds{1} = Z0;
ZReds{1} = Z0;

n = na + nb;

%% Yalmip setup
yalmip('clear');

% Optimization variables
u = sdpvar(1,tHzn+nb+nk-2);
uInit = sdpvar(1,nb+nk-2);
yNominal = sdpvar(1,tHzn+na);
yScenarios = sdpvar(nScen, tHzn+na);
yInit = sdpvar(1,na);
yRefVar = sdpvar(1,tHzn);

thetaNominal = sdpvar(n,1);
thetaScenarios = sdpvar(n,nScen);

%% Constraints definition
constr = [yNominal(1:na) == yInit(end:-1:1);
    u(1:nb+nk-2) == uInit(end:-1:1);
    uMin <= u(nb+nk-1:end) <= uMax;
    yMin <= yNominal(na+1:end) <= yMax;
    ];

for t=1:tHzn
    constr = [constr; yNominal(t+na) == [yNominal(t+na-1:-1:t), u(t+nb-1:-1:t)] * thetaNominal];
end

if scnConstr
    constr = [constr; yScenarios(:,1:na) == ones(nScen, 1) * yInit(end:-1:1)];
    constr = [constr; yMin <= yScenarios(:,na+1:end) <= yMax];

    for t=1:tHzn
        constr = [constr; yScenarios(:,t+na) == diag([yScenarios(:,t+na-1:-1:t), ones(nScen,1) * u(t+nb-1:-1:t)] * thetaScenarios)];
    end
end

%% Cost function
cost = 0;
cost = cost + (u(nb+nk-1:end) - u(nb+nk-2:end-1))* R * (u(nb+nk-1:end) - u(nb+nk-2:end-1))';
cost = cost + (yNominal(na+1:end) - yRefVar) * Q * (yNominal(na+1:end) - yRefVar)';

if scnCost && scnConstr
    for l=1:nScen
        cost = cost + (yScenarios(l,na+1:end) - yRefVar) * Q / nScen * (yScenarios(l,na+1:end) - yRefVar)';
    end
end

%% Optimizer object
inputs = {thetaScenarios, thetaNominal, yInit, uInit, yRefVar};
outputs = {u, yNominal, yScenarios};
ops = sdpsettings('solver', 'quadprog', 'verbose', 1, 'usex0', 0);
scmpc = optimizer(constr, cost, ops, inputs, outputs);

%% Simulation
ySim = zeros(1,nSim);
uSim = zeros(1,nSim);
vols = zeros(1,nSim);
dists = zeros(1,nSim);
thetaSeq = zeros(n,nSim);

yInit = zeros(1,na); % recent -> old
uInit = zeros(1,nb+nk-1); % recent -> old

% Simulation loop
for k=1:nSim
    % Output measurement
    noise = 2*noiseAmp*(0.5-rand(1));
    ySim(k) = [yInit, uInit(nk:nb+nk-1)] * thetaTrue + noise;

    % Various measurements
    vols(k) = volume(ZReds{k});
    dists(k) = norm(thetaTrue - ZReds{k}.c);
    thetaSeq(:, k) = ZReds{k}.c;

    % Zonotope update
    ZNonReds{k+1} = boundStripZonotopeInt(ZNonReds{k}, ySim(k), yInit, uInit, noiseAmp, nb, nk, 'nr');
    ZReds{k+1} = reduceOrder(ZNonReds{k+1}, 'pca');

    yInit = [ySim(k), yInit(1:end-1)];

    % Optimization
    [thetaScenarios, thetaNominal] = scenary(ZReds{k+1}, nScen);
    inputs = {thetaScenarios, thetaNominal, yInit, uInit(1:nb+nk-2), yRef(k+1:k+tHzn)};
    [optimalVars, errCode] = scmpc(inputs);

    if errCode == 1
        disp(['Unfeasible scenarios found!!! k = ', num2str(k)]);
        if interruptBool
            disp('Run interrupted.');
            return;
        else
            while(errCode == 1)
                [thetaScenarios, thetaNominal] = scenary(ZReds{k+1}, nScen);
                inputs = {thetaScenarios, thetaNominal, yInit, uInit(1:nb+nk-2), yRef(k+1:k+tHzn)};
                [optimalVars, errCode] = scmpc(inputs);
            end
            disp('Feasible optimization found!!!');
        end
    end

    uSim(k) = optimalVars{1}(nb+nk-1);
    uInit = [uSim(k), uInit(1:end-1)];
end

%% Error computation 
err = ySim - yRef(1:nSim);
errRmse = sqrt(sum(err.^2)/length(err));

%% Zonotope plots
lastNotEmptyIdx = find(~cellfun('isempty', ZNonReds), 1, 'last');

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
    plot(ZNonReds{1}, 1:3, 'LineWidth',1.5, 'DisplayName','\Theta_0');
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

subplot(2,1,2);
hold on;
plot(err, 'LineWidth', 1.5);
plot([1, nSim], [errRmse, errRmse], 'LineWidth', 1.5, 'LineStyle','-','Color','r');
hold off;
grid on;
legend('Error', 'RMSE');
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

figure;
for k=1:n
    subplot(n,1,k);
    hold on;
    plot(thetaSeq(k,:), 'LineWidth',1.5);
    plot(ones(1,nSim) * thetaTrue(k), '--k');
    hold off;
    legend('', '\theta_t');
    legend('boxoff');
    grid on;
    xlabel('Time instant');
    ylabel(append('\theta_', num2str(k)));
end
