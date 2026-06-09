%% scmpc = buildscmpcoptimizer(sysParams, horizParams, costParams, normParams, solverOps)
% This returns an optimizer object for the ARX SCMPC optimization problem.
% All params structs must have the following fields.
% 
% Inputs:
% - sysParams
%       .na
%       .nb
%       .nd
%       .n
% - horizParams
%       .tHzn
%       .tHznU
%       .nScen
% - costParams
%       .Q
%       .R
%       .useSlack
%       .useScnConstr
%       .useScnCost
% - normParams
%       .useIONorm
%       .useZonNorm
%       Required when useIONorm == true
%           .my, .mu
%           .qy, .qu
%           .Dm
%           .q
%       Required when useZonNorm == true
%           .Dg
%           .c0
% - solverOps: 
%       default: sdpsettings('solver','quadprog','verbose',1,'usex0',0)
% 
% Outputs:
% - scmpc: yalmip optimizer object with inputs
%          scmpc(thetaScenarios, thetaNominal, yInit, uInit, yMinVar, yMaxVar, uMinVar, uMaxVar, yRefVar)
% 

function scmpc = buildscmpcoptimizer(sysParams, horizParams, costParams, normParams, solverOps)

if nargin < 5 || isempty(solverOps)
    solverOps = sdpsettings('solver','quadprog','verbose',1,'usex0',0);
end
if nargin < 4 || isempty(normParams)
    normParams = struct();
end

useIONorm  = isfield(normParams,'useIONorm')  && normParams.useIONorm;
useZonNorm = isfield(normParams,'useZonNorm') && normParams.useZonNorm;

% Unpack parameters
na    = sysParams.na;
nb    = sysParams.nb;
nd    = sysParams.nd;
n     = sysParams.n;

tHzn  = horizParams.tHzn;
tHznU = horizParams.tHznU;
nScen = horizParams.nScen;

Q            = costParams.Q;
R            = costParams.R;
useSlack     = costParams.useSlack;
useScnConstr = costParams.useScnConstr;
useScnCost   = costParams.useScnCost;

DmInv = [];
if useIONorm
    R = R * (normParams.my / normParams.mu)^2;
    DmInv = diag(1 ./ diag(normParams.Dm));   % n*n numeric diagonal
end

% Optimization variables:
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
uInit = sdpvar(1,nb+nd-2); % written in reverse order (recent -> old)
uMaxVar = sdpvar;
uMinVar = sdpvar;

% As for the outputs, they need exactly na initial conditions (to compute
% the next output) and tHzn from the problem formulation.
%
% At instant k-1:
% y = [y(k-na),...,y(k-1),y(k),...,y(k + tHzn - 1)]
%
yNominal = sdpvar(1,tHzn+na);
yScenarios = sdpvar(nScen, tHzn+na);
yInit = sdpvar(1,na); % written in reverse order (recent -> old)
yMaxVar = sdpvar;
yMinVar = sdpvar;
yRefVar = sdpvar;

thetaNominal = sdpvar(n,1);
thetaScenarios = sdpvar(n,nScen);

% Slack
slack = sdpvar;

% Constraints definition
constr = [...
    % Initial conditions
    yNominal(1:na) == yInit(end:-1:1);
    u(1:nb+nd-2) == uInit(end:-1:1);
    % input unchanged after control horizon
    u(nd+nb+tHznU-1:end) == u(nd+nb+tHznU-2);
    % Input hard constraints
    uMinVar <= u(nb+nd-1:end);
    u(nb+nd-1:end) <= uMaxVar;
    % Output soft constraints
    yMinVar - useSlack*slack <= yNominal(na+1:end);
    yNominal(na+1:end) <= yMaxVar + useSlack*slack;
    % Slack variable must be non-negative
    useSlack*slack >= 0 ];

% Nominal prediction constraint
for t=1:tHzn
    phi = [yNominal(t+na-1:-1:t), u(t+nb-1:-1:t)];
    constr = [constr; yNominal(t+na) == predarx(phi, thetaNominal, normParams, DmInv, useIONorm, useZonNorm)];
end

% Scenario constraints
if useScnConstr
    constr = [constr; yScenarios(:,1:na) == ones(nScen, 1) * yInit(end:-1:1)];
    constr = [constr; yMinVar - useSlack * slack <= yScenarios(:,na+1:end);
        yScenarios(:,na+1:end) <= yMaxVar + useSlack * slack
        ];
    
    % Scenario predictions
    for l=1:nScen
        for t=1:tHzn
            phi = [yScenarios(l,t+na-1:-1:t), u(t+nb-1:-1:t)];
            constr = [constr; yScenarios(l,t+na) == predarx(phi, thetaScenarios(:,l), normParams, DmInv, useIONorm, useZonNorm)];
        end
    end
end

% Cost function
deltaU = u(nb+nd-1:end) - u(nb+nd-2:end-1);
cost = deltaU * R * deltaU';
cost = cost + (yNominal(na+1:end) - yRefVar) * Q * (yNominal(na+1:end) - yRefVar)';
cost = cost + useSlack*slack^2;

if useScnCost && useScnConstr
    for l=1:nScen
        cost = cost + (yScenarios(l,na+1:end) - yRefVar) * Q / nScen * (yScenarios(l,na+1:end) - yRefVar)';
    end
end

% Optimizer object
inputs = {thetaScenarios, thetaNominal, yInit, uInit, yMinVar, yMaxVar, uMinVar, uMaxVar, yRefVar};
outputs = {u, yNominal, yScenarios, slack};
scmpc = optimizer(constr, cost, solverOps, inputs, outputs);

end

% --------------------------------
% ARX prediction local function
% --------------------------------
function ypred = predarx(phi, theta, normParams, DmInv, useIONorm, useZonNorm)

if useZonNorm
    theta = normParams.Dg * theta + normParams.c0;
end

if useIONorm
    ypred = normParams.my * ((phi - normParams.q') * (DmInv * theta)) + normParams.qy;
else
    ypred = phi * theta;
end

end
