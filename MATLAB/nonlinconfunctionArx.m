function [c, ceq] = nonlinconfunctionArx(arxObject, u, ySeq, uSeq, yref, NhorU, Nscen)
%   ySeq = [y(k-1) ... y(k-na)] initial conditions for all scenarios
%   uSeq = [u(k-1) ... u(k-nd) ... u(k-nd-nb+1)] initial conditions for all scenarios
%   u = [u(k+NHorU-1) ... u(k)] optimization variable(s)

% nu = 1; % implicit assumption
na = arxObject.na;
nb = arxObject.nb;
nd = arxObject.nd;

% Control input, optimization variable
u = reshape(u, 1, NhorU);

c = [];
ceq = [];

ucur = u(:, end);

H = blkdiag()

end
