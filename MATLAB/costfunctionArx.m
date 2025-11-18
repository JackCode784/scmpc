function J = costfunctionArx(arxObject, ySeq, u, uSeq, yref, Nhor, NhorU, Q, P, R)
%   ySeq = [y(k-1) ... y(k-na)]
%   uSeq = [u(k-1) ... u(k-nd) ... u(k-nd-nb+1)]
%   u = [u(k+NHorU-1) ... u(k)]

na = arxObject.na;
nb = arxObject.nb;
nd = arxObject.nd;

u = reshape(u, 1, NhorU);

% ucur = uSeq(NhorU); % current input sample u(k)
% J = ucur' * R * ucur;

% No input after control horizon
% uSeq = [zeros(1, Nhor-NhorU), uSeq];
uSeq = [u, uSeq];

% Optimizable?
for k=1:NhorU
    uSamples = uSeq(NhorU-k+1:NhorU-k+nb+nd);
    y = arxObject.computeOutput(arxObject, ySeq, uSamples);
    J = J + (y-yref)' * Q * (y-yref);
    ucur = uSamples(1);         % input at this time instant
    J = J + ucur' * R * ucur;
    ySeq = [y, ySeq(1:end-1)];   % update past output samples
end

for k=NhorU+1:Nhor-1
    uSamples = [0, uSamples(1:end-1)];
    y = arxObject.computeOutput(arxObject, ySeq, uSamples);
    J = J + (y-yref)' * Q * (y-yref);
    ySeq = [y, ySeq(1:end-1)];
end

uSamples = [0, uSamples(1:end-1)];
y = arxObject.computeOutput(arxObject, ySeq, uSamples);
J = J + (y-yref)' * P * (y-yref);

end