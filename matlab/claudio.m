%% Passive learning SCMPC — FIXED VERSION
%
clc; clear; close all;

allZonsPlot = false;
scnConstr   = false;
scnCost     = false;
rng("default");

nSim  = 300;
tHzn  = 10;
nScen = 5;

Q = 5;
R = 0.1;

ZReds = cell(nSim+1, 1);

[na, nb, nk, thetaTrue, Z0, ~, uMax, uMin, yMax, yMin, yRef] = ...
    selectSys('milano', 'default', nSim, tHzn);

% =========================================================
%  FIX 1: sigma must upper-bound the ACTUAL noise amplitude
%  Noise = 2*noiseAmp*(0.5 - rand) ∈ (-noiseAmp, noiseAmp)
%  => sigma >= noiseAmp  for guaranteed strip containment
% =========================================================
noiseAmp = 2;
sigma    = noiseAmp;   % <-- was selectSys's small value; now correct

ZReds{1} = Z0;
n = na + nb;

%% Yalmip setup
yalmip('clear');

u            = sdpvar(1, tHzn+nb+nk-2);
uInit        = sdpvar(1, nb+nk-2);
yNominal     = sdpvar(1, tHzn+na);
yScenarios   = sdpvar(nScen, tHzn+na);
yInit        = sdpvar(1, na);
yRefVar      = sdpvar(1, tHzn);
thetaNominal = sdpvar(n, 1);
thetaScenarios = sdpvar(n, nScen);

%% Constraints
constr = [yNominal(1:na) == yInit(end:-1:1);
          u(1:nb+nk-2) == uInit(end:-1:1);
          uMin <= u(nb+nk-1:end);
          u(nb+nk-1:end) <= uMax;
          yMin <= yNominal(na+1:end);
          yNominal(na+1:end) <= yMax];

for t = 1:tHzn
    constr = [constr; yNominal(t+na) == ...
        [yNominal(t+na-1:-1:t), u(t+nb-1:-1:t)] * thetaNominal];
end

if scnConstr
    constr = [constr; yScenarios(:,1:na) == ones(nScen,1) * yInit(end:-1:1)];
    constr = [constr; yMin <= yScenarios(:,na+1:end);
                      yScenarios(:,na+1:end) <= yMax];
    for t = 1:tHzn
        constr = [constr; yScenarios(:,t+na) == ...
            diag([yScenarios(:,t+na-1:-1:t), ones(nScen,1)*u(t+nb-1:-1:t)] * thetaScenarios)];
    end
end

%% Cost
cost = 0;
cost = cost + (u(nb+nk-1:end) - u(nb+nk-2:end-1)) * R * ...
              (u(nb+nk-1:end) - u(nb+nk-2:end-1))';
cost = cost + (yNominal(na+1:end) - yRefVar) * Q * ...
              (yNominal(na+1:end) - yRefVar)';

if scnCost && scnConstr
    for l = 1:nScen
        cost = cost + (yScenarios(l,na+1:end) - yRefVar) * Q / nScen * ...
                      (yScenarios(l,na+1:end) - yRefVar)';
    end
end

%% Optimizer object
inputs  = {thetaScenarios, thetaNominal, yInit, uInit, yRefVar};
outputs = {u, yNominal, yScenarios};
ops     = sdpsettings('solver', 'quadprog', 'verbose', 0, 'usex0', 0);
scmpc   = optimizer(constr, cost, ops, inputs, outputs);

%% Simulation
ySim     = zeros(1, nSim);
uSim     = zeros(1, nSim);
vols     = zeros(1, nSim);
radi     = zeros(1, nSim);
dists    = zeros(1, nSim);
sigmas   = zeros(1, nSim);
thetaSeq = zeros(n, nSim);

yInitSim = zeros(1, na);
uInitSim = zeros(1, nb+nk-1);

% =========================================================
%  FIX 2: stable scenario sampler with stability rejection
%  A scenario theta is admissible only if ALL roots of its
%  characteristic polynomial satisfy |z| < stabMargin < 1.
%  We bound attempts to avoid infinite loops.
% =========================================================
stabMargin  = 0.9999;   % pole modulus threshold
maxAttempts = 200;      % hard cap on resampling attempts

for k = 1:nSim
    % --- Output measurement ---
    noise  = 2*noiseAmp*(0.5 - rand(1));
    ySim(k) = [yInitSim, uInitSim(nk:nb+nk-1)] * thetaTrue + noise;

    % --- Diagnostics ---
    vols(k)      = volume(ZReds{k});
    radi(k)      = zonRadius(ZReds{k});
    dists(k)     = norm(thetaTrue - ZReds{k}.c);
    thetaSeq(:,k)= ZReds{k}.c;
    sigmas(k)    = sigma;

    % --- Zonotope update ---
    ZReds{k+1} = boundStripZonotopeInt(ZReds{k}, ySim(k), yInitSim, uInitSim, ...
                                        sigma, nb, nk, 'new');
    yInitSim = [ySim(k), yInitSim(1:end-1)];

    % --- Scenario sampling with stability filter ---
    [thetaScens, thetaNom] = sampleStableScenarios( ...
        ZReds{k+1}, nScen, na, stabMargin, maxAttempts);

    % --- Solve MPC ---
    inp = {thetaScens, thetaNom, yInitSim, uInitSim(1:nb+nk-2), yRef(k+1:k+tHzn)};
    [optVars, errCode] = scmpc(inp);

    % =====================================================
    %  FIX 3: bounded fallback — never loop indefinitely.
    %  If still infeasible after resampling, fall back to
    %  nominal-only (zero scenario influence); if that also
    %  fails, hold the previous control input.
    % =====================================================
    fallbackAttempts = 0;
    while errCode ~= 0 && fallbackAttempts < 10
        fallbackAttempts = fallbackAttempts + 1;
        [thetaScens, thetaNom] = sampleStableScenarios( ...
            ZReds{k+1}, nScen, na, stabMargin, maxAttempts);
        inp     = {thetaScens, thetaNom, yInitSim, uInitSim(1:nb+nk-2), yRef(k+1:k+tHzn)};
        [optVars, errCode] = scmpc(inp);
    end

    if errCode ~= 0
        % Hard fallback: nominal model only (replicate center for all scenarios)
        thetaNomOnly = ZReds{k+1}.c;
        thetaScensFallback = repmat(thetaNomOnly, 1, nScen);
        inp = {thetaScensFallback, thetaNomOnly, yInitSim, ...
               uInitSim(1:nb+nk-2), yRef(k+1:k+tHzn)};
        [optVars, errCode] = scmpc(inp);

        if errCode ~= 0
            % Ultimate fallback: hold previous input
            warning('k=%d: all fallbacks failed, holding previous u.', k);
            if k > 1
                uSim(k) = uSim(k-1);
            else
                uSim(k) = 0;
            end
            uInitSim = [uSim(k), uInitSim(1:end-1)];
            continue;
        end
    end

    uSim(k)  = optVars{1}(nb+nk-1);
    uInitSim = [uSim(k), uInitSim(1:end-1)];
end

%% Error computation 
err = ySim - yRef(1:nSim);
errRmse = sqrt(sum(err.^2)/length(err));

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


%% =========================================================
%  HELPER — stable scenario sampler
%  Returns nScen column vectors from Z, each giving a stable
%  ARX system (poles inside unit disk with margin stabMargin).
%  Falls back to the zonotope center if quota not met.
% =========================================================
function [thetaScens, thetaNom] = sampleStableScenarios(Z, nScen, na, stabMargin, maxAttempts)
    ng        = size(Z.G, 2);
    thetaScens = zeros(size(Z.c, 1), nScen);
    thetaNom  = Z.c;           % nominal = center (always used)
    filled    = 0;
    attempts  = 0;

    while filled < nScen && attempts < maxAttempts
        attempts = attempts + 1;
        xi       = 2*rand(ng,1) - 1;          % xi uniform in [-1,1]^ng
        theta_s  = Z.c + Z.G * xi;

        % Stability check: characteristic poly z^na - theta_1*z^(na-1) - ... - theta_na
        charCoeffs = [1; -theta_s(1:na)];      % descending powers
        poles      = roots(charCoeffs);

        if all(abs(poles) < stabMargin)
            filled = filled + 1;
            thetaScens(:, filled) = theta_s;
        end
    end

    % Fill remaining slots with the center (stable by assumption)
    if filled < nScen
        for j = filled+1:nScen
            thetaScens(:, j) = Z.c;
        end
    end
end