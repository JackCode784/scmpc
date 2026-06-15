%% Passive learning SCMPC with normalization
% This script compares the results from performing zonotope update and
% SCMPC optimization with and without normalizations applied.
% 
% Two distinct normalizations are applied:
% - one is such that the input and output samples are all within a known
% interval
% - one is such that the initial zonotope (and hopefully the next ones) are
% all within the box [-1, 1]^n, with n n.o. parameters.
% 
clc;
clear;
close all;

rng("default"); % set random seed

allZonsPlot = false;    % plot all simulation zonotopes 
useSlack = true;        % use slack variables   
interruptBool = true;   % if optimization fails at some point, don't try again
useScnConstr = true;    % include scenarios constraints
useScnCost = true;      % include scenarios in cost

% Script doesn't work for every feasible value of these hyperparameters
nSim = 300;
tHzn = 10;
tHznU = 5;
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

[na, nb, nk, thetaTrue, Z0, uMax, uMin, yMax, yMin, yRef] = ...
    selectSys(sys, refstr, nSim, tHzn);

assert(tHzn >= nk, 'Prediction horizon too short for current system delay!');
assert(tHzn - nk >= tHznU - 1, 'Inconsistent control horizon');
assert(noiseAmp > 0, 'There must be some noise, otherwise strip becomes line');

n = na + nb;

% Normalization parameters
c0 = Z0.c;
Dg = diag(sum(abs(Z0.G),2));
thetaTrueNorm = Dg \ (thetaTrue - c0);

ZNotNorms = cell(nSim+1,2); % first column for not normalized, second column for normalized
ZNorms = cell(nSim+1,1);
ZNotNorms{1,1} = Z0;
ZNorms{1} = normalizeZonotope(Z0, c0, Dg);
ZNotNorms{1,2} = normalizeZonotope(Z0, c0, Dg);

% Normalized variables constraints
yNormMax = 1;
yNormMin = -1;
uNormMax = 1;
uNormMin = -1;

my = (yNormMax - yNormMin) ./ (yMax - yMin);
qy = (yNormMin .* yMax - yNormMax .* yMin) ./ (yMax - yMin);
mu = (uNormMax- uNormMin) ./ (uMax - uMin);
qu = (uNormMin .* uMax - uNormMax .* uMin) ./ (uMax - uMin);

Dm = blkdiag(my * eye(na), mu * eye(nb));
q = [qy * ones(na, 1); qu * ones(nb, 1)];

%% Yalmip setup
yalmip('clear');

sysParams = struct('na', na, 'nb', nb, 'nd', nk, 'n', n);

horizParams = struct('tHzn', tHzn, ...
    'tHznU', tHznU, ...
    'nScen', nScen);

costParams = struct('Q', Q, ...
    'R', R, ...
    'useSlack', useSlack, ...
    'useScnConstr', useScnConstr, ...
    'useScnCost', useScnCost);

normParams = struct('useIONorm', true, ...
    'useZonNorm', true, ...
    'my', my, 'mu', mu, ...
    'qy', qy, 'qu', qu, ...
    'Dm', Dm, 'q', q, ...
    'Dg', Dg, 'c0', c0);

solverOps = [];

scmpc = buildscmpcoptimizer(sysParams, horizParams, costParams);
scmpcNorm = buildscmpcoptimizer(sysParams, horizParams, costParams, normParams, solverOps);

%% Simulation
ySim = zeros(1,nSim);
yNormSim = zeros(1, nSim);
uSim = zeros(1,nSim);
uNormSim = zeros(1,nSim);
vols = zeros(1,nSim);
volsNorm = zeros(1,nSim);
dists = zeros(1,nSim);
distsNorm = zeros(1,nSim);
thetaSeq = zeros(n,nSim);
thetaNormSeq = zeros(n,nSim);
noise = 2*noiseAmp*(0.5-rand(1, nSim));

yInit = zeros(1,na); % recent -> old
yInitNorm = my * yInit + qy;
% uInit = [u(k-1),...,u(k-nk),...,u(k-nk-nb+1)]
%         |_____|
%       last that has been optimized
uInit = zeros(1,nb+nk-1); % recent -> old
uInitNorm = mu * uInit + qu;

% Simulation loop
for k=1:nSim
    phi_k = [yInit, uInit(nk:nb+nk-1)]';
    phiNorm_k = [yInitNorm, uInitNorm(nk:nb+nk-1)]';

    % True system's output measurement
    ySim(k) = phi_k' * thetaTrue + noise(k);
    yNormSim(k) = (phiNorm_k - q)' * (Dm \ thetaTrue) + noise(k);
    yNormSim(k) = my * yNormSim(k) + qy; 

    % Various measurements
    vols(k) = volume(ZNotNorms{k,1});
    volsNorm(k) = volume(ZNorms{k});
    thetaSeq(:, k) = ZNotNorms{k,1}.c;
    thetaNormSeq(:, k) = ZNorms{k}.c;
    dists(k) = norm(thetaTrue - ZNotNorms{k,1}.c);
    distsNorm(k) = norm(thetaTrueNorm - ZNorms{k}.c);

    % Zonotope update (not normalized)
    ZNotNorms{k+1,1} = boundStripZonotopeInt(ZNotNorms{k,1}, ...
        ySim(k), yInit, uInit, noiseAmp, nb, nk, 'new');

    ZNotNorms{k+1,2} = normalizeZonotope(ZNotNorms{k+1,1},c0,Dg);
    
    % Zonotope update (normalized)
    ZNorms{k+1} = boundStripZonotopeInt(ZNorms{k}, ...
        (yNormSim(k) - qy) / my - (phiNorm_k - q)' * (Dm \ c0), ...
        (yInitNorm - qy) / my * Dg(1:na, 1:na), ...
        (uInitNorm - qu) / mu * Dg(na+1:end, na+1:end), ...
        noiseAmp, nb, nk, 'new');
    
    yInit = [ySim(k), yInit(1:end-1)];
    yInitNorm = [yNormSim(k), yInitNorm(1:end-1)];

    % Both optimizations
    [thetaScenariosNorm, thetaNominalNorm] = stableScenary(ZNorms{k+1}, nScen, na);
    [thetaScenarios, thetaNominal] = stableScenary(ZNotNorms{k+1,1}, nScen, na);
    yRefNorm = my * yRef(k+1) + qy;

    inputsNorm = {thetaScenariosNorm, thetaNominalNorm, ...
        yInitNorm, uInitNorm(1:nb+nk-2), ...
        yNormMin, yNormMax, uNormMin, uNormMax, ...
        yRefNorm};

    inputs = {thetaScenarios, thetaNominal, ...
        yInit, uInit(1:nb+nk-2), ...
        yMin, yMax, uMin, uMax, ...
        yRef(k+1)};

    [optimalVars, errCode] = scmpc(inputs);
    [optimalVarsNorm, errCodeNorm] = scmpcNorm(inputsNorm);

    if errCode ~= 0 || errCodeNorm ~= 0
        warning(['Unfeasible... at k = ', num2str(k)]);
        disp(['errCode = ', num2str(errCode)]);
        disp(['errCodeNorm = ', num2str(errCodeNorm)]);
        if interruptBool
            disp('Run interrupted.');
            break;
        else
            while(errCode ~= 0 || errCodeNorm ~= 0)
                [thetaScenariosNorm, thetaNominalNorm] = stableScenary(ZNorms{k+1}, nScen, na);
                [thetaScenarios, thetaNominal] = stableScenary(ZNotNorms{k+1}, nScen, na);
                inputs = {thetaScenarios, thetaNominal, ...
                    yInit, uInit(1:nb+nk-2), ...
                    yMin, yMax, uMin, uMax, ...
                    yRef(k+1)};
                inputsNorm = {thetaScenariosNorm, thetaNominalNorm, ...
                    yInitNorm, uInitNorm(1:nb+nk-2), ...
                    yNormMin, yNormMax, uNormMin, uNormMax,...
                    yRefNorm};
                [optimalVars, errCode] = scmpc(inputs);
                [optimalVarsNorm, errCodeNorm] = scmpcNorm(inputsNorm);
            end
            disp('Feasible optimization found!!!');
        end
    end

    uNormSim(k) = optimalVarsNorm{1}(nb+nk-1);
    uSim(k) = optimalVars{1}(nb+nk-1);
    uInit = [uSim(k), uInit(1:end-1)];
    uInitNorm = [uNormSim(k), uInitNorm(1:end-1)];
end

%% Error computation 
lastIdx = find(~cellfun('isempty', ZNorms), 1, 'last');

uDiscrepancy = uSim - (uNormSim - qu) / mu;
yDiscrepancy = ySim - (yNormSim - qy) / my;

uRMSE = sqrt(mean(uDiscrepancy .^ 2));
yRMSE = sqrt(mean(yDiscrepancy .^ 2));

maxUDiscrepancy = max(abs(uDiscrepancy));
maxYDiscrepancy = max(abs(yDiscrepancy));

disp(['Input RMSE = ', num2str(uRMSE), '[unnormalized]']);
disp(['Output RMSE = ', num2str(yRMSE), '[unnormalized]']);
disp(['Max Input discrepancy deltaU = ', num2str(maxUDiscrepancy), '[actual units]']);
disp(['Max Output discrepancy deltaY = ', num2str(maxYDiscrepancy), '[actual units]']);

unYRefError = ySim - yRef(1:nSim);
unYRefRMSE = sqrt(mean(unYRefError .^ 2));

nYRefError = (yNormSim - qy) / my - yRef(1:nSim);
nYRefRMSE = sqrt(mean(nYRefError .^ 2));

%% Zonotope comparison
% Compare normalized zonotopes
centerdiffs = cellfun(@(z1, z2) z1.c - z2.c, ZNotNorms(1:lastIdx,2), ZNorms(1:lastIdx), 'UniformOutput',false);
generatorsdiff = cellfun(@(z1, z2) z1.G - z2.G, ZNotNorms(1:lastIdx,2), ZNorms(1:lastIdx), 'UniformOutput',false);

maxCenterDist = max(cellfun(@(diff) norm(diff), centerdiffs));
maxGeneratorFrobNorm = max(cellfun(@(gendiff) norm(gendiff, 'fro'), generatorsdiff));

disp(['Max center discrepancy: ', num2str(maxCenterDist)]);
disp(['Max generator discrepancy: ', num2str(maxGeneratorFrobNorm)]);

%% Zonotope plots
figure;
hold on;
if allZonsPlot
    cmap = turbo(lastIdx);
    for l=1:lastIdx
        plot(ZNorms{l}, 1:3, 'LineWidth',1.5, 'Color', cmap(l,:), 'HandleVisibility','off');
    end
    colorbar;
    colormap(cmap);
    clim([1, lastIdx]);
else
    plot(ZNorms{1}, 1:3, 'LineWidth',1.5, 'DisplayName','\Theta_0^N');
    plot(ZNotNorms{1,1}, 1:3, 'LineWidth',1.5, 'DisplayName','\Theta_0');
    plot(ZNorms{lastIdx}, 1:3, 'LineWidth',1.5,'DisplayName',append('\Theta^N(k=', num2str(lastIdx), ')'));
    plot(ZNotNorms{lastIdx,1}, 1:3, 'LineWidth',1.5,'DisplayName',append('\Theta(k=', num2str(lastIdx), ')'));
    grid on;
end
plot3([-1 1 1 -1 -1 NaN -1 1 1 -1 -1 NaN -1 -1 NaN 1 1 NaN 1 1 NaN -1 -1], ...
    [-1 -1 1 1 -1 NaN -1 -1 1 1 -1 NaN -1 -1 NaN -1 -1 NaN 1 1 NaN 1 1], ...
    [-1 -1 -1 -1 -1 NaN 1 1 1 1 1 NaN -1 1 NaN -1 1 NaN -1 1 NaN -1 1], ...
    '--', 'LineWidth',1.5, 'DisplayName','Unit cube');
plot3(ZNorms{1}.c(1), ZNorms{1}.c(2), ZNorms{1}.c(3),'.','LineWidth',1.5, 'MarkerSize',20, 'DisplayName','c^N(1)');
plot3(ZNotNorms{1,1}.c(1), ZNotNorms{1,1}.c(2), ZNotNorms{1,1}.c(3),'.','LineWidth',1.5, 'MarkerSize',20, 'DisplayName','c^{UN}(1)');
plot3(ZNorms{lastIdx}.c(1), ZNorms{lastIdx}.c(2), ZNorms{lastIdx}.c(3),'.','LineWidth',1.5, 'MarkerSize',20, 'DisplayName','c^N(k)');
plot3(ZNotNorms{lastIdx,1}.c(1), ZNotNorms{lastIdx,1}.c(2), ZNotNorms{lastIdx,1}.c(3),'.','LineWidth',1.5, 'MarkerSize',20, 'DisplayName','c^{UN}(k)');
plot3(thetaTrueNorm(1), thetaTrueNorm(2), thetaTrueNorm(3), '*', 'LineWidth',1.5,'MarkerSize',10, 'DisplayName','\theta^N_t');
plot3(thetaTrue(1), thetaTrue(2), thetaTrue(3), '*', 'LineWidth',1.5,'MarkerSize',10, 'DisplayName','\theta^{UN}_t');
grid on;
xlabel('\theta_1');
ylabel('\theta_2');
zlabel('\theta_3');
axis equal;
title('Zonotopes');
legend('-DynamicLegend');
legend('boxoff');

%% Simpler plots
% Simulation plots
figure;
subplot(4,1,1);
hold on;
plot(ySim, 'LineWidth', 1.5);
plot((yNormSim - qy) / my, 'LineWidth',1.5);
plot(yRef(1:nSim), 'LineWidth', 1.5, 'LineStyle', '-.');
yline(yMax, 'k--');
yline(yMin, 'k--');
hold off;
grid on;
legend('UN y_k', 'N y_k','y_r', 'y_M', 'y_m');
legend('boxoff');
xlabel('k');
title('Measured output');

subplot(4,1,2);
hold on;
plot(yNormSim, 'LineWidth',1.5);
plot(my*yRef(1:nSim)+qy, 'LineWidth', 1.5, 'LineStyle', '-.');
yline(yNormMax, 'k--');
yline(yNormMin, 'k--');
hold off;
grid on;
legend('y^N_k', 'y^N_r', 'y^N_M', 'y^N_m');
legend('boxoff');

subplot(4,1,3);
hold on;
plot(unYRefError, 'LineWidth', 1.5);
plot(nYRefError, 'LineWidth', 1.5);
yline(unYRefRMSE, 'LineWidth', 1.5, 'LineStyle','--');
yline(nYRefRMSE, 'LineWidth', 1.5, 'LineStyle','--');
yline(mean(unYRefError),'LineWidth',1.5);
yline(mean(nYRefError),'LineWidth',1.5);
hold off;
grid on;
legend('UN y_k-y_r', 'N y_k-y_r', 'UN RMSE', 'N RMSE', 'UN mean', 'N mean');
legend('boxoff');
ylabel('Error[actual units]');
xlabel('k');

subplot(4,1,4);
hold on;
plot(uSim, 'LineWidth',1.5);
plot((uNormSim - qu) / mu, 'LineWidth',1.5);
yline(uMin, 'k--');
yline(uMax, 'k--');
hold off;
grid on;
legend('UN u_k', 'N u_k', 'u_m', 'u_M');
legend('boxoff');
title('Optimal control');
ylabel('u_k');
xlabel('k');

% Discrepancy
figure;
subplot(2,1,1);
plot(abs(uDiscrepancy), LineWidth=1.5);
grid on;
ylabel('|u_{UN} - u_N|');
xlabel('k');
title('u_k and y_k discrepancies');

subplot(2,1,2);
plot(abs(yDiscrepancy),LineWidth=1.5);
grid on;
ylabel('|y_{UN} - y_N|');
xlabel('k');

% Zonotope diagnostic plots
figure;
subplot(3,1,1);
hold on;
plot(vols / volume(Z0), 'LineWidth',1.5);
plot(volsNorm / volume(ZNorms{1}), 'LineWidth',1.5);
hold off;
grid on;
title('Uncertainty region volumes');
legend('UN', 'N');
legend('boxoff');
xlabel('k');
ylabel('vol(Z_k)/vol(Z_0)');

subplot(3,1,2);
hold on;
plot(vols(2:end) ./ vols(1:end-1), 'LineWidth',1.5);
plot(volsNorm(2:end) ./ volsNorm(1:end-1), 'LineWidth',1.5);
hold off;
grid on;
legend('UN', 'N');
legend('boxoff');
xlabel('k');
ylabel('vol(Z_k))/vol(Z_{k-1})');

subplot(3,1,3);
hold on;
plot(dists / norm(thetaTrue), 'LineWidth',1.5);
plot(distsNorm / norm(thetaTrueNorm), 'LineWidth',1.5);
hold off;
xlabel('k');
legend('UN', 'N');
legend('boxoff');
ylabel('||\theta_t-c_k|| / ||\theta_t||');
grid on;