function J = costfunctionArx(arxObject, yPast, u, uPast, yref, Nhor, NhorU, Q, P, R)
% costfunctionArx       Returns the cost value for the given input
% sequence u = [u(k), ..., u(k+NhorU-1)].
%   yPast = [y(k) ... y(k-na+1)] are the past output samples (k-1, ...,
%   k-na).
%   uPast = [u(k-1) ... u(k-nd) ... u(k-nd-nb+2)] are the past input
%   samples (k-1, ..., k-nb-nd+2).

u = reshape(u, 1, NhorU);
J = 0; % cost init

% Vector approach
% R = repmat({R}, 1, NhorU);
% R = blkdiag(R{:});
% J = J + u'*R*u;

% C-like approach
for k=1:NhorU
    uSamples = [u(:,k), uPast];                   % [u(k), u(k-1), ..., u(k-nb-nd+2)]...
    y = arxObject.computeOutput(yPast, uSamples); % ...to compute y(k+1)
    J = J + (y-yref)' * Q * (y-yref) + uSamples(1)'*R*uSamples(1);
    yPast = [y, yPast(1:end-1)];   % update past output samples [y(k+1), ..., y(k-na+2)]
    uPast = [uSamples(1), uPast(1:end-1)];  % becomes uPast = [u(k), ..., u(k-nb-nd+3)]
end

for k=NhorU+1:Nhor-1
    uSamples = [uPast(1), uPast];
    y = arxObject.computeOutput(yPast, uSamples);
    J = J + (y-yref)' * Q * (y-yref);
    yPast = [y, yPast(1:end-1)];
    uPast = [uSamples(1), uPast(1:end-1)];
end

uSamples = [uPast(1), uPast];
y = arxObject.computeOutput(yPast, uSamples);
J = J + (y-yref)' * P * (y-yref);

end