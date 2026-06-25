%% selectSy(sys, refStr, nSim, tHzn)
% Select system and reference output to perform passive learning.
% All contemplated systems and reference trajectories:
%   buck, buckloss
%       square1
%       square2
%       sin
%       const
%   milano, milanorand
%       default
%       sin
%       const
%   bm
%       prbs
%       sin
%       square
%       const
% 

function [na, nb, nk, thetaTrue, Z0, uMax, uMin, yMax, yMin, yRef] = selectSys(sys, refStr, nSim, tHzn)

moreArgs = {};

switch sys
    case {'buck', 'buckloss'}
        [na, nb, nk, thetaTrue, Z0, uMax, uMin, yMax, yMin, moreArgs] = localBuck(sys, moreArgs);
    case {'milano', 'milanorand'}
        [Z0,na,nb,nk,uMax,uMin,yMax,yMin,thetaTrue,moreArgs] = localMilano(sys, moreArgs);
    case 'bm'
        [na,nb,nk,thetaTrue,Z0,uMax,uMin,yMax,yMin,moreArgs] = localBenchmark(moreArgs);
    case 'rand'
        % WIP
    otherwise
        disp('System not recognized');
        return;
end

yRef = selectRef(sys, refStr, tHzn, nSim, moreArgs);

return;
end

function yRef = selectRef(sys, refStr, tHzn, nSim, moreArgs)

switch sys
    case {'buck', 'buckloss'}
        vIn = moreArgs{1};
        switch refStr
            case 'square1'
                yRef = idinput(nSim+tHzn,'prbs',[0, 0.005], [vIn / 4, 3*vIn/4]);
            case 'square2'
                yRef = [repmat(3,1,round(nSim/4)) repmat(4,1,round(nSim/4)) repmat(5,1,round(nSim/4)) repmat(7, 1, round(nSim/4+tHzn))]';
            case 'sin'
                ii = 1:nSim+tHzn;
                yRef = sin(0.1*ii);
            case 'const'
                yRef = zeros(1,nSim+tHzn)+7.5;
        end
    case {'milano','milanorand'}
        switch refStr
            case 'default'
                yRef = idinput(nSim+tHzn,'prbs',[0, 0.05], [-100,100]); % Power reference
            case 'sin'
                Ts = moreArgs{1};
                yRef = 100*sin(2*pi*3*(0:nSim+tHzn-1)*Ts)';
            case 'const'
                yRef = 5 + zeros(nSim + tHzn, 1);
        end
    case 'bm'
        switch refStr
            case 'prbs'
                yRef = idinput(nSim+tHzn, 'prbs', [0, 0.05], [-1.5, 1.5]);
            case 'sin'
                ii = (0:nSim+tHzn-1)';
                yRef = 1.5 * sin(0.05 * ii);
            case 'square'
                yRef = [repmat(-1.2, 1, round((nSim+tHzn)/2)), ...
                    repmat( 1.2, 1, ceil((nSim+tHzn)/2))]';
            case 'const'
                yRef = zeros(1,nSim) - 8;
            otherwise
                yRef = idinput(nSim+tHzn, 'prbs', [0, 0.05], [-1.5, 1.5]);
        end
    case 'rand'
        % WIP
    otherwise
        disp('Unrecognized requested yref');
end

yRef = yRef(:)';

return;
end

% Local function for selecting buck system
function [na, nb, nk, thetaTrue, Z0, uMax, uMin, yMax, yMin, moreArgs] = localBuck(sys, moreArgs)

na = 2;
nb = 1;
nk = 2;
vIn = 10;   % [V]
R = 6;      % [Ohm]
C = 15e-6;  % [F]
L = 150e-6; % [H]
Ts = 1e-5;  % [s]
unc = 0.2;  % [%]
r = 0;
if strcmp(sys, 'buckloss')
    r = 1;
end

% WIP
computeTheta = @(RLC) [2 - Ts/(RLC(1)*RLC(3) - Ts * r / RLC(2));
    Ts/RLC(3) * (1/RLC(1) - Ts/RLC(2)) - 1 + Ts * r / RLC(2) - Ts^2 * r / (RLC(1) * RLC(2) * RLC(3));
    Ts^2/(RLC(2)*RLC(3)) * vIn];

% fun = @(x) norm(computeTheta(x));
% xmin = fmincon(fun, [R;L;C], [], [], [], [], (1-unc)*[R;L;C], (1+unc)*[R;L;C]);
% fun = @(x) -norm(computeTheta(x));
% xmax = fmincon(fun, [R;L;C], [], [], [], [], (1-unc)*[R;L;C], (1+unc)*[R;L;C]);
% thetaMin = computeTheta(xmin);
% thetaMax = computeTheta(xmax);

% Alt
% thetaMin = [2-Ts/(1-unc)^2/R/C;
%     Ts/(1+unc)/C * (1/(1+unc)/R - Ts/(1-unc)/L);
%     Ts^2/((1+unc)^2*L*C)*vIn*(1-unc)];
% thetaMax = [2-Ts/(1+unc)^2/R/C;
%     Ts/(1-unc)/C * (1/(1-unc)/R - Ts/(1+unc)/L);
%     Ts^2/((1-unc)^2*L*C)*vIn*(1+unc)];

% thetaNominal = computeTheta([R,L,C]); % theta nominal

% diffs = abs([thetaMin, thetaMax] - thetaNominal);
% maxCoords = max(diffs, [], 2);
% G = diag(maxCoords);

load Z0buck.mat;

[thetaTrue, ~] = stableScenary(Z0, 1, na);

% R = 1.2*R;
% L = 1.2*L;
% C = 1.2*C;
% thetaTrue = [2-Ts/(R*C);
%     Ts / C * (1/R - Ts/L)-1;
%     Ts^2/(L*C)*vIn*1.2];
% Z0.G = diag(Z0.c)*0.2;

uMax = 1;
uMin = 0;
yMax = vIn;
yMin = 0;

moreArgs = [moreArgs, vIn];

end

% Local function for selecting Milano-related systems
function [Z0,na,nb,nk,uMax,uMin,yMax,yMin,thetaTrue,moreArgs] = localMilano(sys,moreArgs)
load("data_model_BESS.mat", "Jmin", "Z_", "Ts");
Z0 = Z_{end};
clear("Z_");
na = 3;
nb = 3;
nk = 1;
% sigma = Jmin * 1.1;
uMax = 200;
uMin = -uMax;
yMax = 110;
yMin = -yMax;

if strcmp(sys, 'milano')
    thetaTrue = Z0.c;
else % milanorand
    [thetaTrue, ~] = stableScenary(Z0, 1, na);
end
moreArgs = [moreArgs, Ts];
end

% Local function for benchmark system
function [na,nb,nk,thetaTrue,Z0,uMax,uMin,yMax,yMin,moreArgs] = localBenchmark(moreArgs)

% --- Model orders ---
na  = 2;    % number of past outputs in the regressor
nb  = 2;    % number of past inputs  in the regressor
nk  = 1;    % input delay (most recent input term is u(t-nk))
%
% ARX structure: y(t) = theta_1*y(t-1) + theta_2*y(t-2)
%                     + theta_3*u(t-1) + theta_4*u(t-2) + e(t)
% Parameter vector: theta = [theta_1; theta_2; theta_3; theta_4]

% ---------------------------------------------------------------
% Zonotope Z0 = { c + G*xi : xi in [-1,1]^4 }
% ---------------------------------------------------------------

% --- Center: nominal parameter vector ---
% AR part: nominal poles at 0.6 +/- 0.3j  (|poles| = sqrt(0.45) ~ 0.671)
%   characteristic polynomial: z^2 - 1.20*z + 0.45
% MA part: static gain ~ (0.60+0.25)/(1-1.20+0.45) = 0.85/0.25 = 3.4
Z0   = zonotope.empty;
Z0.c = [  1.20;   % theta_1 : AR coeff of y(t-1)
         -0.45;   % theta_2 : AR coeff of y(t-2)
          0.60;   % theta_3 : MA coeff of u(t-1)
          0.25];  % theta_4 : MA coeff of u(t-2)

% --- Square generator matrix (4x4) ---
% Design criteria:
%   (a) Strict diagonal dominance by rows -> G is nonsingular (Gershgorin)
%   (b) Off-diagonal entries model AR/MA cross-parameter correlations
%   (c) The entire zonotope lies in the Schur-stability region:
%         max_{Z0}(theta_1+theta_2) = 0.75 + ||G_row1+G_row2||_1
%                                   = 0.75 + 0.20 = 0.95 < 1   [C1]
%         min_{Z0}(1+theta_1-theta_2) = 2.53 > 0               [C2]
%         max_{Z0}|theta_2| = 0.45 + 0.08 = 0.53 < 1           [C3]
Z0.G = [ 0.09,  0.02,  0.00,  0.01;   % generators for theta_1
         0.02,  0.05,  0.01,  0.00;   % generators for theta_2
         0.00,  0.01,  0.08,  0.02;   % generators for theta_3
         0.01,  0.00,  0.02,  0.05];  % generators for theta_4

% --- True parameter vector: sampled from Z0 ---
% thetaTrue = Z0.c + Z0.G * xi_true, with xi_true in [-1,1]^4 drawn by scenary.
% By construction thetaTrue lies in Z0 and corresponds to a stable AR part
% (guaranteed by the Schur-Cohn verification above).
[thetaTrue, ~] = scenary(Z0, 1);

% --- Noise level ---
% sigma = 0.10;   % output measurement noise std dev

% --- Input / output bounds ---
uMax =  1.5;
uMin = -uMax;
yMax =  6.0;    % conservative bound: static gain * uMax ~ 3.4*1.5 = 5.1
yMin = -yMax;

end

% Generate random zonotope around thetaNominal
function [Z, thetaTrue] = randZon(thetaNominal, n, relUnc)

if isempty(thetaNominal)
    thetaNominal = 2*rand(n,1)-1;
end

Z = zonotope.empty;
Z.c = thetaNominal;
Z.G = diag(thetaNominal) .* relUnc;

[thetaTrue, ~] = scenary(Z, 1);
thetaTrue = thetaTrue(:);

end
