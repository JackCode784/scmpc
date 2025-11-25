function uOpt = generateSCMPCControl(sysVec, yPast, uPast, yref, Nhor, NhorU, Q, P, R, umax, umin, ymax, ymin)
% generateSCMPCControl      Evaluates the MPC control function
%   yPast = [y1(k-1) ... y1(k-na)
%           y2(k-1) ... y2(k-na)
%                   ...         
%           y3(k-1) ... y(k-na)] 
% refers to the past output samples for each scenario. Instead
%   uPast = [u(k-1) ... u(k-nd) ... u(k-nd-nb+1)]
%   u = [u(k+NHorU-1) ... u(k)]
% uPast contains the past input samples (the same applied to every
% scenario) from k-1 to k-nb-nd+1. u contains the optimization variables,
% namely the input to be fed to the system for a total of NhorU samples,
% from time instant k to k+NhorU-1.
% 
% This calls patternsearch to solve the MPC optimization problem, while
% also ensuring saturation constraints on input and output are satisfied.
% While the cost function is computed on the nominal system only, the
% constraints will be computed for all scenarios.

% Define old optimal input
% persistent uOptOld;
% if isempty(uOptOld)
%     uOptOld = 0;
% end

Nscen = length(sysVec); % number of scenarios (nominal system included)

% Default (and only) algorithm is patternsearch
% Cost function: only nominal system used
costfcnArx = @(uOpt) costfunctionArx(sysVec{1}, yPast, uOpt, uPast, yref, Nhor, NhorU, Q, P, R);

% (Non)linear constraints function: all scenarios included
nlconArx = @(u) nonlinconfunctionArx(sysVec, u, yPast, uPast, Nhor, NhorU, Nscen, umax, umin, ymax, ymin);

psopts = optimoptions('patternsearch','Algorithm','nups-mads','InitialMeshSize',0.15,'ConstraintTolerance',1e-6);
% fmopts = optimoptions('fmincon','algorithm','interior-point');

% uOpt is [uOpt(k+NhorU-1) ... uOpt(k)]
[uOpt, J] = patternsearch(costfcnArx,repmat(uPast(1),1,NhorU),[],[],[],[],repmat(umin,1,NhorU),repmat(umax,1,NhorU),nlconArx,psopts);
% [uOpt, J] = fmincon(costfcnArx,zeros(1, NhorU),[],[],[],[],[],[],nlconArx,fmopts);

uOpt = reshape(uOpt,1,NhorU);

end