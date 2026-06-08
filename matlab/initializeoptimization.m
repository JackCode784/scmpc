%% scmpc = initializeoptimization(tHzn,nb,nk,na,nScen,nhoru,uMin,uMax,yMin,yMax,useSlack,useScnConstr,R,Q,useScnCost)
% Defines the yalmip optimization problem and returns the optimizer object
% 
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
% 

function scmpc = initializeoptimization(tHzn,nb,nd,na,nScen,nhoru,uMin,uMax,yMin,yMax,useSlack,useScnConstr,R,Q,useScnCost,isNormalized)
n = na + nb;

% Optimization variables
u = sdpvar(1,tHzn+nb-1);
uInit = sdpvar(1,nb+nd-2); % written in reverse order (recent -> old)

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

% Constraints definition
constr = [yNominal(1:na) == yInit(end:-1:1);
    u(1:nb+nd-2) == uInit(end:-1:1);
    u(nd+nb+nhoru-1:end) == u(nd+nb+nhoru-2); % input unchanged after control horizon
    uMin <= u(nb+nd-1:end); % hard constraints
    u(nb+nd-1:end) <= uMax; % hard constraints
    yMin - useSlack*slack <= yNominal(na+1:end); % soft constraints (slack)
    yNominal(na+1:end) <= yMax + useSlack*slack; % soft constraints (slack)
    useSlack*slack >= 0;    % slack variable must be non-negative
    ];

for t=1:tHzn
    constr = [constr; yNominal(t+na) == [yNominal(t+na-1:-1:t), u(t+nb-1:-1:t)] * thetaNominal];
end

if useScnConstr
    constr = [constr; yScenarios(:,1:na) == ones(nScen, 1) * yInit(end:-1:1)];
    constr = [constr; yMin - useSlack * slack <= yScenarios(:,na+1:end);
                      yScenarios(:,na+1:end) <= yMax + useSlack * slack
                      ];

    for l=1:nScen
        for t=1:tHzn
            constr = [constr; yScenarios(l,t+na) == [yScenarios(l,t+na-1:-1:t), u(t+nb-1:-1:t)] * thetaScenarios(:,l)];
        end
    end
end

% Cost function
cost = (u(nb+nd-1:end) - u(nb+nd-2:end-1)) * R * (u(nb+nd-1:end) - u(nb+nd-2:end-1))';
cost = cost + (yNominal(na+1:end) - yRefVar) * Q * (yNominal(na+1:end) - yRefVar)';
cost = cost + useSlack*slack^2;

if useScnCost && useScnConstr
    for l=1:nScen
        cost = cost + (yScenarios(l,na+1:end) - yRefVar) * Q / nScen * (yScenarios(l,na+1:end) - yRefVar)';
    end
end

% Optimizer object
inputs = {thetaScenarios, thetaNominal, yInit, uInit, yRefVar};
outputs = {u, yNominal, yScenarios, slack};
ops = sdpsettings('solver', 'quadprog', 'verbose', 1, 'usex0', 0);
scmpc = optimizer(constr, cost, ops, inputs, outputs);

end