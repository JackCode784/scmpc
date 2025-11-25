function J = costfunctionArx(arxObject, yPast, u, uPast, yref, Nhor, NhorU, Q, P, R)
% costfunctionArx       Returns the cost value for the given input
% sequence u = [u(k+NhorU-1), ..., u(k)].
%   yPast = [y(k-1) ... y(k-na)] are the past output samples (k-1, ...,
%   k-na).
%   uPast = [u(k-1) ... u(k-nd) ... u(k-nd-nb+1)] are the past input
%   samples (k-1, ..., k-nb-nd+1).

u = reshape(u, 1, NhorU);
J = 0; % cost init

% Vector approach
% R = repmat({R}, 1, NhorU);
% R = blkdiag(R{:});
% J = J + u'*R*u;

% C-like approach
for k=1:NhorU
    uSamples = [u(:,k), uPast];
    y = arxObject.computeOutput(yPast, uSamples);
    J = J + (y-yref)' * Q * (y-yref) + uSamples(1)'*R*uSamples(1);
    yPast = [y, yPast(1:end-1)];   % update past output samples
    uPast = [uSamples(1), uPast(1:end-1)];
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