%% Test different zon-strip bounds
% Test new strip-zonotope intersection algorithm
% 
close all;
clc;

plotZons = true;
na = 1;
nb = 1;
n = na + nb;
nk = 1;
ng = n;
Zorig = zonotope(zeros(n,1), rand(n,ng)*2 - 1);
yCurr = rand * 2 - 1;
yInit = rand(1,na) * 2 - 1;
uInit = rand(1,nb+nk-1) * 2 - 1;
sigma = 0.1;
ZNR = boundStripZonotopeInt(Zorig, yCurr, yInit, uInit, sigma, nb, nk, 'nr');

% Gram-Schmidt
Zgs = reduceOrder(ZNR, 'gs');
minRad = zonRadius(Zorig);
vn = vecnorm(Zgs.G);
Zgs.G(:,vn > minRad) = Zgs.G(:,vn > minRad) ./ vn(vn > minRad) * minRad;
P = polytope(Zgs) & polytope(Zorig);

% Usual strip
Phi = [yInit, uInit(nk:end)];
C = [Phi; -Phi];
d = [yCurr + sigma; -yCurr + sigma];
strip = polytope(C,d);

% Support strip
d = C*Zorig.c + sum(abs(Phi*Zorig.G));
supportStrip = polytope(C, d);

% Tight strip explicit computation
d = [min(strip.b(1), supportStrip.b(1)); min(strip.b(2), supportStrip.b(2))]; % d = [qu, -ql]
tightStrip = polytope(C,d);

allZons = cell(ng,1);
qc = [1 -1] * tightStrip.b / 2;
qr = ones(1,2) * tightStrip.b / 2;
vol = zeros(ng,1);

for j=1:ng
    allZons{j} = zonotope.empty;
    allZons{j}.c = Zorig.c;
    allZons{j}.G = Zorig.G;
    if (Phi * Zorig.G(:,j) ~= 0)
        allZons{j}.c = allZons{j}.c + (qc - Phi * Zorig.c)/(Phi*Zorig.G(:,j)) * Zorig.G(:,j);
        allZons{j}.G(:,1:ng ~= j) = Zorig.G(:,1:ng ~= j) - (Phi * Zorig.G(:, 1:ng ~= j)) .* Zorig.G(:,j) / (Phi * Zorig.G(:,j));
        allZons{j}.G(:,j) = qr / (Phi * Zorig.G(:,j)) * Zorig.G(:,j);
    end
    vol(j) = volume(allZons{j});
    % plot(allZons{j}, 1:2, 'LineWidth',1.5,'DisplayName', ['j = ' num2str(j)]);
end

% Volumes comparison
[newVol, jstar] = min(vol);
classicVol = volume(ZNR);
% pcaVol = volume(reduce(ZNR, 'pca', 1));
gsVol = volume(Zgs);
% polyVol = volume(P);
origVol = volume(Zorig);

str = ["New", "ClassicNR", "GS", "ORIG"];
allVolsRel = [newVol, classicVol, gsVol, origVol] ./ origVol;
[allVolsRel, idx] = sort(allVolsRel, 'ascend');
str = str(idx);

if plotZons
    figure;
    hold on;
    plot(Zorig, 1:2, 'LineWidth',1.5, 'DisplayName','orig');
    plot(ZNR, 1:2, 'LineWidth',1.5, 'DisplayName', 'classicNR');
    plot(reduce(ZNR, 'pca', 1), 1:2, 'LineWidth', 1.5, 'DisplayName', 'PCA');
    plot(Zgs, 1:2, 'LineWidth', 1.5, 'DisplayName', 'gs');
    plot(P, 1:2, 'LineWidth', 1.5, 'DisplayName', 'gspoly');
    % plot(strip, 1:2, 'LineWidth',1.5, 'DisplayName','Strip');
    % plot(supportStrip, 1:2, 'LineWidth',1.5, 'DisplayName','SuppStrip');
    % plot(tightStrip, 1:2, 'LineWidth',1.5, 'DisplayName','TightStrip');
    axis square;
    grid on;
    plot(allZons{jstar}, 1:2, 'LineWidth',1.5, 'DisplayName', ['jstar = ', num2str(jstar)]);
    legend('boxoff');
    legend('-DynamicLegend');
    axis equal;
end

figure;
bar(str, allVolsRel);
grid on;
ylabel('Volume');
title(['n = ', num2str(n), ', n_g = ' num2str(ng)]);
