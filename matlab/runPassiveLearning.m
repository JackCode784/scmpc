function results = runPassiveLearning(cfg)
% Closed-loop passive-learning SCMPC simulation.
%
% Runs the full Bravo et al. zonotope-update + Scenario MPC loop and
% returns structured results.  Normalization is optionally enabled via
% cfg.useNorm; the algorithm and code structure are identical in both
% modes - only the coordinate system changes.
%
% USAGE
%   cfg = defaultPassiveLearningCfg();   % get a fully populated default
%   cfg.sysName = 'mySystem';            % override individual fields
%   results = runPassiveLearning(cfg);
%
% CFG FIELDS (all optional; defaults from defaultPassiveLearningCfg)
%   Simulation
%     .nSim          number of closed-loop steps          (300)
%     .rngSeed       RNG seed for reproducibility   ('default')
%   System
%     .sysName       system identifier string        ('buckloss')
%     .refstr        reference signal identifier     ('square2')
%     .noiseAmp      noise half-amplitude               (0.02)
%   Horizons / scenarios
%     .tHzn          prediction horizon                    (10)
%     .tHznU         control horizon                        (5)
%     .nScen         number of uncertainty scenarios        (4)
%   Cost
%     .Q             output tracking weight                 (5)
%     .R             input variation weight               (0.1)
%   Flags
%     .useSlack      soft output constraints           (true)
%     .useScnConstr  per-scenario output constraints   (true)
%     .useScnCost    per-scenario cost term            (true)
%     .interruptBool stop on first infeasibility       (true)
%     .useNorm       I/O + zonotope normalisation      (false)
%   Display
%     .plotResults   show figures                      (true)
%     .allZonsPlot   plot every zonotope step          (false)
%     .verbose       solver verbosity flag             (true)
%
% RESULTS FIELDS
%   .ySim        [1 x nSim]   simulated system output
%   .uSim        [1 x nSim]   applied control input  (actual units always)
%   .yRef        [1 x nSim]   reference signal
%   .vols        [1 x nSim]   zonotope volumes
%   .slackSeq    [1 x nSim]   slack variable sequence
%   .dists       [1 x nSim]   distance from zonotope center to true theta
%   .thetaSeq    [n x nSim]   zonotope center sequence
%   .Zones       {nSim+1 x 1} zonotope cell array (normalized if useNorm)
%   .errRmse     scalar       output tracking RMSE
%   .lastIdx     scalar       last valid simulation step
%   .cfg         struct       cfg echo (with defaults filled in)
%
% NOTE ON YALMIP INSIDE A FUNCTION
%   yalmip('clear') clears the global YALMIP state.  Each call to
%   runPassiveLearning rebuilds the optimizer (~seconds).  If you need to
%   run many parameter sweeps, pre-build the optimizer with
%   buildSCMPCOptimizer and pass it in separately.

cfg = mergeDefaults(cfg, defaultPassiveLearningCfg());

rng(cfg.rngSeed);

fprintf('Passive learning SCMPC | System: %s | Ref: %s | useNorm: %d\n', ...
    cfg.sysName, cfg.refstr, cfg.useNorm);

[na, nb, nk, thetaTrue, Z0, cfg.uMax, cfg.uMin, cfg.yMax, cfg.yMin, yRef] = ...
    selectSys(cfg.sysName, cfg.refstr, cfg.nSim, cfg.tHzn);
yMax = cfg.yMax;
yMin = cfg.yMin;
uMax = cfg.uMax;
uMin = cfg.uMin;

assert(cfg.tHzn >= nk,                'Prediction horizon too short.');
assert(cfg.tHzn - nk >= cfg.tHznU-1,  'Inconsistent control horizon.');
assert(cfg.noiseAmp > 0,               'Noise amplitude must be positive.');

n = na + nb;

c0 = Z0.c;
Dg = diag(sum(abs(Z0.G), 2));

yNormMin = -1;  yNormMax = 1;
uNormMin = -1;  uNormMax = 1;

my = (yNormMax - yNormMin) / (yMax - yMin);
qy = (yNormMin * yMax - yNormMax * yMin) / (yMax - yMin);
mu = (uNormMax - uNormMin) / (uMax - uMin);
qu = (uNormMin * uMax - uNormMax * uMin) / (uMax - uMin);

Dm = blkdiag(my * eye(na), mu * eye(nb));
q  = [qy * ones(na,1); qu * ones(nb,1)];

thetaTrueRef = thetaTrue;   % reference for distance metric (actual space)
if cfg.useNorm
    thetaTrueRef = Dg \ (thetaTrue - c0);   % in normalized theta-space
end

Zones    = cell(cfg.nSim+1, 1);
Zones{1} = Z0;
if cfg.useNorm
    Zones{1} = normalizeZonotope(Z0, c0, Dg);
end

yalmip('clear');

sysParams   = struct('na',na, 'nb',nb, 'nd',nk, 'n',n);
horizParams = struct('tHzn',cfg.tHzn, 'tHznU',cfg.tHznU, 'nScen',cfg.nScen);
costParams  = struct('Q',cfg.Q, 'R',cfg.R, ...
    'useSlack',    cfg.useSlack,    ...
    'useScnConstr',cfg.useScnConstr, ...
    'useScnCost',  cfg.useScnCost);

if cfg.useNorm
    normParams = struct('useIONorm',true, 'useZonNorm',true, ...
        'my',my, 'mu',mu, 'qy',qy, 'qu',qu, ...
        'Dm',Dm, 'q',q, 'Dg',Dg, 'c0',c0);
else
    normParams = struct();   % no normalization; both flags default to false
end

solverOps = sdpsettings('solver','quadprog', ...
    'verbose', cfg.verbose, 'usex0',0);
scmpc = buildSCMPCOptimizer(sysParams, horizParams, costParams, ...
    normParams, solverOps);

ySim     = zeros(1, cfg.nSim);
uSim     = zeros(1, cfg.nSim);   % always in actual units
vols     = zeros(1, cfg.nSim);
slackSeq = zeros(1, cfg.nSim);
dists    = zeros(1, cfg.nSim);
thetaSeq = zeros(n, cfg.nSim);
noise    = 2*cfg.noiseAmp*(0.5 - rand(1, cfg.nSim));

yInit = zeros(1, na);
uInit = zeros(1, nb+nk-1);

lastIdx = cfg.nSim;   % updated on early break

for k = 1:cfg.nSim

    % Measurement
    phi_k  = [yInit, uInit(nk:nb+nk-1)]';
    ySim(k) = phi_k' * thetaTrue + noise(k);

    % Diagnostics
    vols(k)        = volume(Zones{k});
    thetaSeq(:,k)  = Zones{k}.c;
    dists(k)       = norm(thetaTrueRef - Zones{k}.c);

    % Bravo strip update
    if cfg.useNorm
        % Strip in normalized theta-space: |(Dg*phi)^T*tilde_theta - tilde_d| <= sigma.
        % I/O normalization cancels algebraically (see derivation), so
        % actual yInit/uInit scaled by Dg is the correct argument.
        Zones{k+1} = boundStripZonotopeInt(Zones{k}, ...
            ySim(k) - phi_k' * c0,            ...  % tilde_d
            yInit  * Dg(1:na,    1:na),         ...  % Dg_y * y-lags
            uInit  * Dg(na+1:end, na+1:end),    ...  % Dg_u * u-lags
            cfg.noiseAmp, nb, nk, 'new');
    else
        Zones{k+1} = boundStripZonotopeInt(Zones{k}, ...
            ySim(k), yInit, uInit, cfg.noiseAmp, nb, nk, 'new');
    end

    % Advance output history
    yInit = [ySim(k), yInit(1:end-1)];

    % Scenarios
    % stableScenary returns scenarios in Zones{k+1}'s coordinate space:
    %   - actual theta    when useNorm = false
    %   - normalized tilde_theta when useNorm = true
    [thetaScenarios, thetaNominal] = stableScenary(Zones{k+1}, cfg.nScen, na);

    % Build optimizer inputs
    if cfg.useNorm
        % Convert histories and reference to I/O-normalized space.
        % Derived on-the-fly; no separate normalized history to maintain.
        yInitN = my * yInit              + qy;    % [1 x na]
        uInitN = mu * uInit(1:nb+nk-2)   + qu;    % [1 x nb+nk-2]
        yRefN  = my * yRef(k+1)          + qy;    % scalar
        inputs = {thetaScenarios, thetaNominal, ...
            yInitN, uInitN, ...
            yNormMin, yNormMax, uNormMin, uNormMax, yRefN};
    else
        inputs = {thetaScenarios, thetaNominal, ...
            yInit, uInit(1:nb+nk-2), ...
            yMin, yMax, uMin, uMax, yRef(k+1)};
    end

    % Solve
    [optimalVars, errCode] = scmpc(inputs);

    if errCode ~= 0
        warning('Solver failure at k = %d (errCode = %d).', k, errCode);
        if cfg.interruptBool
            disp('Simulation interrupted.'); 
            lastIdx = k; 
            break;
        else
            while errCode ~= 0
                [thetaScenarios, thetaNominal] = stableScenary(Zones{k+1}, cfg.nScen, na);
                if cfg.useNorm
                    inputs = {thetaScenarios, thetaNominal, ...
                        yInitN, uInitN, ...
                        yNormMin, yNormMax, uNormMin, uNormMax, yRefN};
                else
                    inputs = {thetaScenarios, thetaNominal, ...
                        yInit, uInit(1:nb+nk-2), ...
                        yMin, yMax, uMin, uMax, yRef(k+1)};
                end
                [optimalVars, errCode] = scmpc(inputs);
            end
            disp('Feasible solution found after resampling.');
        end
    end

    slackSeq(k) = optimalVars{4};
    uOpt        = optimalVars{1}(nb+nk-1);    % first optimized input

    if cfg.useNorm
        uSim(k) = (uOpt - qu) / mu;            % denormalize to actual units
    else
        uSim(k) = uOpt;
    end

    uInit = [uSim(k), uInit(1:end-1)];        % advance actual history
end

K = 1:lastIdx;
err     = ySim(K) - yRef(K);
errRmse = sqrt(mean(err.^2));

fprintf('RMSE: %.4f | Slack mean: %.4f | Steps completed: %d/%d\n', ...
    errRmse, mean(slackSeq(K)), lastIdx, cfg.nSim);

results.ySim     = ySim;
results.uSim     = uSim;
results.yRef     = yRef(1:cfg.nSim);
results.vols     = vols;
results.slackSeq = slackSeq;
results.dists    = dists;
results.thetaSeq = thetaSeq;
results.Zones    = Zones;
results.errRmse  = errRmse;
results.lastIdx  = lastIdx;
results.cfg      = cfg;
results.thetaTrue = thetaTrue;   % useful for plotting
results.Z0        = Z0;
if cfg.useNorm
    results.thetaTrueNorm = Dg \ (thetaTrue - c0);
end

if cfg.plotResults
    plotPassiveLearningResults(results);
end

end % runPassiveLearning


function plotPassiveLearningResults(res)
%PLOTPASSIVELEARNINGRESULTS  All diagnostic plots for a simulation run.
cfg     = res.cfg;
K       = 1:res.lastIdx;
Zones   = res.Zones;
lastIdx = res.lastIdx;

% — Zonotope evolution
figure('Name','Zonotopes');
hold on;
if cfg.allZonsPlot
    cmap = turbo(lastIdx);
    for l = 1:lastIdx
        plot(Zones{l}, 1:3, 'LineWidth',1.5, ...
            'Color',cmap(l,:),'HandleVisibility','off');
    end
    colorbar; 
    colormap(cmap); 
    clim([1, lastIdx]);
else
    plot(Zones{1},       1:3, 'LineWidth',1.5,'DisplayName','\Theta_0');
    plot(Zones{lastIdx}, 1:3, 'LineWidth',1.5, ...
        'DisplayName',['\Theta_',num2str(lastIdx)]);
    grid on;
end
if cfg.useNorm && isfield(res,'thetaTrueNorm')
    tRef = res.thetaTrueNorm;
    lbl  = '\tilde{\theta}_t';
else
    tRef = res.thetaTrue;
    lbl  = '\theta_t';
end
plot3(Zones{1}.c(1),       Zones{1}.c(2),       Zones{1}.c(3),       '.','MarkerSize',20,'DisplayName','\theta_n(1)');
plot3(Zones{lastIdx}.c(1), Zones{lastIdx}.c(2), Zones{lastIdx}.c(3), '.','MarkerSize',20,'DisplayName',['\theta_n(k=',num2str(lastIdx),')']);
plot3(tRef(1), tRef(2), tRef(3), '*','MarkerSize',10,'LineWidth',1.5,'DisplayName',lbl);
if cfg.useNorm   % unit cube reference
    cubePts = [-1 1 1 -1 -1 NaN -1  1  1 -1 -1 NaN -1 -1 NaN  1  1 NaN  1  1 NaN -1 -1; ...
        -1 -1 1  1 -1 NaN -1 -1  1  1 -1 NaN -1 -1 NaN -1 -1 NaN  1  1 NaN  1  1; ...
        -1 -1 -1 -1 -1 NaN  1  1  1  1  1 NaN -1  1 NaN -1  1 NaN -1  1 NaN -1  1];
    plot3(cubePts(1,:),cubePts(2,:),cubePts(3,:),'k--','LineWidth',1.5,'DisplayName','Unit cube');
end
xlabel('\theta_1'); ylabel('\theta_2'); zlabel('\theta_3');
title('Parameter zonotopes'); legend('-DynamicLegend'); legend('boxoff');

% — Output and error
figure('Name','Output');
subplot(2,1,1);
hold on;
plot(K, res.ySim(K),   'LineWidth',1.5,'DisplayName','Output');
plot(K, res.yRef(K),   'LineWidth',1.5,'LineStyle','-.','DisplayName','Reference');
yline(res.cfg.yMax,'k--','HandleVisibility','off');   % requires yMax in cfg if stored
yline(res.cfg.yMin,'k--','HandleVisibility','off');
hold off; grid on; legend; xlabel('k'); title('System output');

subplot(2,1,2);
err = res.ySim(K) - res.yRef(K);
hold on;
plot(K, err,                     'LineWidth',1.5,'DisplayName','Error');
yline(res.errRmse,'r--','LineWidth',1.5,'DisplayName','RMSE');
yline(mean(err),  'k',  'LineWidth',1.5,'DisplayName','Mean');
hold off; grid on; legend; ylabel('Error'); xlabel('k');

% — Control input
figure('Name','Control');
hold on;
plot(K, res.uSim(K), 'LineWidth',1.5,'DisplayName','u^*');
yline(res.cfg.uMin,'k--','HandleVisibility','off');
yline(res.cfg.uMax,'k--','HandleVisibility','off');
hold off; grid on; title('Optimal control input'); xlabel('k'); legend;

% — Uncertainty metrics
vol0 = volume(Zones{1});
figure('Name','Uncertainty');
subplot(3,1,1);
plot(K, res.vols(K)/vol0,'LineWidth',1.5);
grid on; ylabel('vol(Z(k))/vol(Z_0)'); xlabel('k'); title('Volume ratio');

subplot(3,1,2);
plot(K(2:end), res.vols(K(2:end))./res.vols(K(1:end-1)),'LineWidth',1.5);
grid on; ylabel('vol(k)/vol(k-1)'); xlabel('k');

subplot(3,1,3);
normFactor = norm(tRef);
plot(K, res.dists(K)/normFactor,'LineWidth',1.5);
grid on; ylabel('dist/||\theta_t||'); xlabel('k');
title('Distance from center to true parameter');
end

function cfg = defaultPassiveLearningCfg()
% Return a cfg struct with all defaults populated.
cfg.nSim         = 300;
cfg.tHzn         = 10;
cfg.tHznU        = 5;
cfg.nScen        = 4;
cfg.sysName      = 'buckloss';
cfg.refstr       = 'square2';
cfg.Q            = 5;
cfg.R            = 0.1;
cfg.noiseAmp     = 0.02;
cfg.useSlack     = true;
cfg.useScnConstr = true;
cfg.useScnCost   = true;
cfg.interruptBool = true;
cfg.useNorm      = false;
cfg.allZonsPlot  = false;
cfg.rngSeed      = 'default';
cfg.plotResults  = true;
cfg.verbose      = true;
end

function cfg = mergeDefaults(cfg, defaults)
% Copy missing fields from defaults into cfg.
fields = fieldnames(defaults);
for i = 1:numel(fields)
    f = fields{i};
    if ~isfield(cfg, f)
        cfg.(f) = defaults.(f);
    end
end
% Always populate system bounds from selectSys output if not already set.
% (They are set inside runPassiveLearning after selectSys returns.)
end