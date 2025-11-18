function u = generateSCMPCControl(arxObject, ySeq, uSeq, yref, Nhor, NhorU, Q, P, R)
% generateSCMPCControl      Evaluates the MPC control function
%   ySeq = [y(k-1) ... y(k-na)]
%   uSeq = [u(k+NHorU-1) ... u(k) u(k-1) ... u(k-nd) ... u(k-nd-nb+1)]
%   takes into account the control horizon
% 
% u = generateSCMPCControl(OBJ, y0, u0, Nhor, NhorU)
% 
% u = generateSCMPCControl(OBJ, y0, u0, Nhor, NhorU, yref)
% 

% if ~ismember(nargin, [5, 6])
%     error('Wrong number of inputs.');
% end

% if nargin == 4
%     yref = varargin{4};
% end

na = arxObject.na;
nb = arxObject.nb;
nd = arxObject.nd;
theta = arxObject.theta;

% Define old optimal input
persistent uOptOld;
if isempty(uOptOld)
    uOptOld = 0;
end

% Default (and only) algorithm is patternsearch
costfcnArx = @(uOpt) costfunctionArx(arxObject, ySeq, uOpt, uSeq, yref, Nhor, NhorU, Q, P, R);
nlcon = @(uu) nonlinconfunctionArx();

psopts = optimoptions('patternsearch','Algorithm','nups-mads','InitialMeshSize',0.3);

uOpt = patternsearch(costfcnArx,zeros(1, NhorU),[],[],[],[],[],[],nlcon,psopts);
uOpt = reshape(uOpt,1,NhorU);




end