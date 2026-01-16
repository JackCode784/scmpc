function [uOpt, yPast, uSamples] = generateSCMPCControl(nominalSys, yCurr, yPast, uSamples, yref, Nhor, NhorU, Q, P, R, umax, umin, ymax, ymin, Nscen)
% generateSCMPCControl      Evaluates the MPC control function
%   yPast = [y(k) ... y(k-na+1)]
% refers to the past output samples from the true system. Instead
%   uSamples = [u(k-1) ... u(k-nd) ... u(k-nd-nb+1)]
%   u = [u(k), ..., u(k+NHorU-1)]
% uPast contains the past input samples (the same applied to every
% scenario) from k-1 to k-nb-nd+1. u contains the optimization variables,
% namely the input to be fed to the system for a total of NhorU samples,
% from time instant k-1 to k+NhorU-2.
% 
% This calls patternsearch to solve the MPC optimization problem, while
% also ensuring saturation constraints on input and output are satisfied.
% While the cost function is computed on the nominal system only, the
% constraints will be computed for all scenarios.

% Scenario generation
sysVec = generateScenarios(Nscen,nominalSys);

% Update initial conditions
yPast = [yCurr, yPast(1:end-1)]; % [y(k), ..., y(k-na+1)]
uPast = uSamples(1:end-1);  % uPast = [u(k-1), ..., u(k-nb-nd+2)]

% Default (and only) algorithm is patternsearch
% Optimize uOpt to find u(k) from yPast
% Cost function: only nominal system used
costfcnArx = @(uOpt) costfunctionArx(nominalSys, yPast(1,:), uOpt, uPast, yref, Nhor, NhorU, Q, P, R);

% (Non)linear constraints function: all scenarios included
nlconArx = @(u) nonlinconfunctionArx(sysVec, u, ones(Nscen+1,1)*yPast, uPast, Nhor, NhorU, Nscen, umax, umin, ymax, ymin);

psopts = optimoptions('patternsearch','Algorithm','nups-mads','InitialMeshSize',0.15,'ConstraintTolerance',1e-6);
% fmopts = optimoptions('fmincon','algorithm','interior-point');

% uOpt is [uOpt(k+NhorU-1) ... uOpt(k)]
[uOpt, J] = patternsearch(costfcnArx,repmat(uPast(1),1,NhorU),[],[],[],[],repmat(umin,1,NhorU),repmat(umax,1,NhorU),nlconArx,psopts);
% [uOpt, J] = fmincon(costfcnArx,zeros(1, NhorU),[],[],[],[],[],[],nlconArx,fmopts);

uOpt = reshape(uOpt,1,NhorU);
uSamples = [uOpt(1), uSamples(1:end-1)];    % [u*(k), u(k-1), ..., u(k-nb-nd+2)]

end