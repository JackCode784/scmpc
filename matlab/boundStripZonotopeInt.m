%% boundStripZonotopeInt
% Bounds the intersection of given zonotope and strip by means of a bigger
% (and higher-order) zonotope, the volume of which is minimized by setting
% a lambda parameter minimizing its generators' matrix
% Froebenius norm.
%
% Steps:
% - Receives current zonotope of parameters, current and past input/output
%   samples, etc.;
% - Creates strip of possible outputs given a tolerance (measurement
%   error);
% - if the strip contains the zonotope, their intersection is trivial
% - else, if they intersect, compute new zonotope containing their
%   intersection (adding one generator)
% - return such zonotope
%
% Inputs:
% - oldZon: current zonotope of parameters
% - yCurr: current true system's output
% - yPast: vector of past output samples [y(k-1),...,y(k-na)]
% - uSamples: vector of input samples [u(k-nk),...,u(k-nk-nb+1)]
% - noiseAmp: amplitude of measurement noise
% - nb: number of input samples needed to compute an output sample
% - nk: system delay
% - method: string to choose the update method
%
% Outputs:
% - newZon: newly computed zonotope containing strip-oldZon intersection
%

function newZon = boundStripZonotopeInt(oldZon, yCurr, yPast, uSamples, noiseAmp, nb, nk, method)

Phi = [yPast, uSamples(nk:nk+nb-1)];
C = [Phi; -Phi];    % strip matrix
d = [yCurr + noiseAmp; -yCurr + noiseAmp];  % output sample + measurement noise
strip = polytope(C, d); % two halfspaces intersection (equality => hyperplanes)

if contains(strip, oldZon)
    newZon = oldZon;
    return;
end

switch method
    case 'nr'
        newZon = plusOneGeneratorBound(oldZon,Phi,noiseAmp,yCurr);
    case 'new'
        newZon = newBoundStripZonInt(oldZon, strip);
    otherwise
        disp("Unrecognized bounding method!");
        newZon = oldZon;
end

end

% Local function for new zonotope-strip intersection bounding
% Ref: Bounded Error Identification of Systems With Time-Varying Parameters
% Bravo et al. (2006)
function newZon = newBoundStripZonInt(oldZon, strip)

ng = size(oldZon.G, 2); % n.o. generators
C = strip.A;
Phi = C(1,:);

% Support strip
d = C*oldZon.c + sum(abs(Phi*oldZon.G));
supportStrip = polytope(C, d);

% Tight strip explicit computation
d = [min(strip.b(1), supportStrip.b(1)); min(strip.b(2), supportStrip.b(2))]; % d = [qu, -ql]
tightStrip = polytope(C,d);
tsc = [1 -1] * tightStrip.b / 2;
tsr = ones(1,2) * tightStrip.b / 2;

allZons = cell(ng+1,1);
vols = zeros(ng+1,1);
for j=1:ng
    allZons{j} = oldZon;
    tol = 1e-8 * norm(Phi) * norm(oldZon.G(:,j));
    if abs(Phi * oldZon.G(:,j)) > tol
        allZons{j}.c = allZons{j}.c + (tsc - Phi * oldZon.c)/(Phi*oldZon.G(:,j)) * oldZon.G(:,j);
        allZons{j}.G(:,1:ng ~= j) = oldZon.G(:,1:ng ~= j) - (Phi * oldZon.G(:, 1:ng ~= j)) .* oldZon.G(:,j) / (Phi * oldZon.G(:,j));
        allZons{j}.G(:,j) = tsr / (Phi * oldZon.G(:,j)) * oldZon.G(:,j);
    end
    vols(j) = volume(allZons{j});
end
allZons{end} = oldZon;
vols(end) = volume(oldZon);

[~, jstar] = min(vols);
newZon = allZons{jstar};

end

function newZon = plusOneGeneratorBound(oldZon,Phi,noiseAmp,yCurr)
% if stripIntersectsZonotope(C, d, Zc, ZGens)
oldCenter = oldZon.c;
oldGens = oldZon.G;
sigma = 2*sum(abs(Phi*oldGens)) + noiseAmp; % new strip's sigma
lambdaStar = (oldGens * oldGens') * Phi' / (Phi * (oldGens * oldGens') * Phi' + sigma^2); % n-dimensional vector
% lambdaStar = lambdaStar / (Phi * lambdaStar + sigma^2);
newCenter = oldCenter + lambdaStar*(yCurr - Phi * oldCenter);
newGen = [(eye(length(newCenter))-lambdaStar*Phi)*oldGens, sigma*lambdaStar];
newZon = zonotope(newCenter, newGen);
% end
end