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
slackSeq = zeros(1,nSim);
dists = zeros(1,nSim);
thetaSeq = zeros(n,nSim);
noise = 2*noiseAmp*(0.5-rand(1, nSim));

yInit = zeros(1,na); % recent -> old
% uInit = [u(k-1),...,u(k-nk),...,u(k-nk-nb+1)]
%         |_____|
%       to be optimized
uInit = zeros(1,nb+nk-1); % recent -> old

% Simulation loop
for k=1:nSim
    phi_k = [yInit, uInit(nk:nb+nk-1)]';
    % True system's output measurement
    ySim(k) = phi_k' * thetaTrue + noise(k);

    % Various measurements
    vols(k) = volume(ZNorms{k});
    thetaSeq(:, k) = ZNorms{k}.c;
    dists(k) = norm(thetaTrueNorm - ZNorms{k}.c);

    % Zonotope update (not normalized)
    ZNotNorms{k+1,1} = boundStripZonotopeInt(ZNotNorms{k,1}, ...
        ySim(k), yInit, uInit, noiseAmp, nb, nk, 'new');

    ZNotNorms{k+1,2} = normalizeZonotope(ZNotNorms{k+1,1},c0,Dg);
    
    % Zonotope update (normalized)
    ZNorms{k+1} = boundStripZonotopeInt(ZNorms{k}, ...
        ySim(k) - phi_k'*c0, ...
        yInit * Dg(1:na, 1:na), ...
        uInit * Dg(na+1:end, na+1:end), ...
        noiseAmp, nb, nk, 'new');
    yInit = [ySim(k), yInit(1:end-1)];

    % Optimization
    [thetaScenariosNorm, thetaNominalNorm] = stableScenary(ZNorms{k+1}, nScen, na);
    yInitNorm = my * yInit + qy;
    uInitNorm = mu * uInit + qu;
    yRefNorm = my * yRef(k+1) + qy;
    inputs = {thetaScenariosNorm, thetaNominalNorm, ...
        yInitNorm, uInitNorm(1:nb+nk-2), ...
        yNormMin, yNormMax, uNormMin, uNormMax, ...
        yRefNorm};
    [optimalVars, errCode] = scmpcNorm(inputs);

    if errCode ~= 0
        warning(['Unfeasible... at k = ', num2str(k)]);
        if interruptBool
            disp('Run interrupted.');
            break;
        else
            while(errCode ~= 0)
                [thetaScenariosNorm, thetaNominalNorm] = stableScenary(ZNorms{k+1}, nScen, na);
                inputs = {thetaScenariosNorm, thetaNominalNorm, ...
                    yInitNorm, uInitNorm(1:nb+nk-2), ...
                    yNormMin, yNormMax, uNormMin, uNormMax,...
                    yRefNorm};
                [optimalVars, errCode] = scmpcNorm(inputs);
            end
            disp('Feasible optimization found!!!');
        end
    end

    slackSeq(k) = optimalVars{4};      % retrieve slack for this k
    uNormSim(k) = optimalVars{1}(nb+nk-1); % retrieve first optimized input
    uSim(k) = (uNormSim(k) - qu) / mu;
    yNormSim(k) = my * ySim(k) + qy;
    uInit = [uSim(k), uInit(1:end-1)]; % input initial conditions update
end

%% Error computation 
err = ySim - yRef(1:nSim);
rmse = sqrt(sum(err.^2)/length(err));

disp(['RMSE: ', num2str(rmse)]);
disp(['Slack mean: ', num2str(mean(slackSeq))]);

%% Zonotope comparison
lastNotEmptyIdx = find(~cellfun('isempty', ZNorms), 1, 'last');

% Compare normalized zonotopes
centerdiffs = cellfun(@(znotnorm, znorm) znotnorm.c - znorm.c, ZNotNorms(1:lastNotEmptyIdx,2), ZNorms(1:lastNotEmptyIdx), 'UniformOutput',false);
generatorsdiff = cellfun(@(znotnorm, znorm) znotnorm.G - znorm.G, ZNotNorms(1:lastNotEmptyIdx,2), ZNorms(1:lastNotEmptyIdx), 'UniformOutput',false);

maxCenterDist = max(cellfun(@(diff) norm(diff), centerdiffs));
maxGeneratorFrobNorm = max(cellfun(@(gendiff) norm(gendiff, 'fro'), generatorsdiff));
disp(['Max center discrepancy: ', num2str(maxCenterDist)]);
disp(['Max generator discrepancy: ', num2str(maxGeneratorFrobNorm)]);

%% Zonotope plots
figure;
hold on;
if allZonsPlot
    cmap = turbo(lastNotEmptyIdx);
    for l=1:lastNotEmptyIdx
        plot(ZNorms{l}, 1:3, 'LineWidth',1.5, 'Color', cmap(l,:), 'HandleVisibility','off');
    end
    colorbar;
    colormap(cmap);
    clim([1, lastNotEmptyIdx]);
else
    plot(ZNorms{1}, 1:3, 'LineWidth',1.5, 'DisplayName','\tilde{\Theta}_0');
    % plot(ZNonReds{lastNotEmptyIdx}, 1:3, 'LineWidth',1.5,'DisplayName',append('Last NR k=', num2str(k)));
    plot(ZNorms{lastNotEmptyIdx}, 1:3, 'LineWidth',1.5,'DisplayName',append('\tilde{\Theta}_r(k=', num2str(lastNotEmptyIdx), ')'));
    grid on;
end
plot3([-1 1 1 -1 -1 NaN -1 1 1 -1 -1 NaN -1 -1 NaN 1 1 NaN 1 1 NaN -1 -1], ...
    [-1 -1 1 1 -1 NaN -1 -1 1 1 -1 NaN -1 -1 NaN -1 -1 NaN 1 1 NaN 1 1], ...
    [-1 -1 -1 -1 -1 NaN 1 1 1 1 1 NaN -1 1 NaN -1 1 NaN -1 1 NaN -1 1], ...
    '--', 'LineWidth',1.5, 'DisplayName','Unit cube');
plot3(ZNorms{1}.c(1), ZNorms{1}.c(2), ZNorms{1}.c(3),'.','LineWidth',1.5, 'MarkerSize',20, 'DisplayName','\theta_n^{R}(1)');
plot3(ZNorms{lastNotEmptyIdx}.c(1), ZNorms{lastNotEmptyIdx}.c(2), ZNorms{lastNotEmptyIdx}.c(3),'.','LineWidth',1.5, 'MarkerSize',20, 'DisplayName','\theta_n^{R}(k)');
plot3(thetaTrueNorm(1), thetaTrueNorm(2), thetaTrueNorm(3), '*', 'LineWidth',1.5,'MarkerSize',10, 'DisplayName','\tilde{\theta}_t');
grid on;
xlabel('\theta_1');
ylabel('\theta_2');
zlabel('\theta_3');
title('Zonotopes');
legend('-DynamicLegend');
legend('boxoff');

%% Simpler plots
% Simulation plots
figure;
subplot(3,1,1);
hold on;
plot(ySim, 'LineWidth', 1.5);
plot(yRef(1:nSim), 'LineWidth', 1.5, 'LineStyle', '-.');
plot(0*ySim+yMax, 'k--');
plot(0*ySim+yMin, 'k--');
hold off;
grid on;
legend('Simulated output','Power reference', 'y_M', 'y_m');
xlabel('Time step');
title('System true output');

subplot(3,1,2);
hold on;
plot(err, 'LineWidth', 1.5);
plot([1, nSim], [rmse, rmse], 'LineWidth', 1.5, 'LineStyle','--','Color','r');
plot([1 nSim], mean(err)*[1, 1], 'LineWidth',1.5,'Color', 'k');
hold off;
grid on;
legend('Error', 'RMSE', 'Mean');
ylabel('Error');
xlabel('Time step');

subplot(3,1,3);
hold on;
plot(uSim, 'LineWidth',1.5);
plot(0*uSim + uMin, 'k--');
plot(0*uSim + uMax, 'k--');
grid on;
legend('u(k)', 'u_m', 'u_M');
ylabel('Optimal control');
xlabel('Time step');

% Zonotope diagnostic plots
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
