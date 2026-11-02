%% Attempt algorithm to find zonotope vertex
% If present, wrong.mat contains a case in which the algorithm failed (the
% distance between the found vertex is NOT the zonotope's radius because
% not maximum).
clc; clear; close all;
nG = 10;
Z = zonotope(zeros(2,1), rand(2,nG)*2-1);

allCoeffs = dec2bin(0:2^nG-1) - '0';
allCoeffs = allCoeffs * 2 - 1;
allCoeffs = allCoeffs';

allPoints = Z.G * allCoeffs;

figure;
hold on;
plot(Z, 1:2, 'LineWidth', 1.5);
plot(allPoints(1,:), allPoints(2,:), 'LineStyle','none', 'Marker','.', 'MarkerSize',10);
xlim([-10, 10]);
ylim([-10, 10]);
grid on;

% Algorithm starts here
[~, idxsort] = sort(vecnorm(Z.G), 'descend');
Z.G = Z.G(:, idxsort);

v = Z.G(:,1);
plot([0, v(1)], [0, v(2)], 'LineWidth',1.2);
for i=2:nG
    vOld = v;
    if v' * Z.G(:,i) >= 0
        v = v + Z.G(:,i);
    else
        v = v - Z.G(:,i);
    end
    plot([vOld(1), v(1)], [vOld(2), v(2)], 'LineWidth',2);
end

% Check if algorithm guessed right
[maxNorm, maxIdx] = max(vecnorm(allPoints));
vNorm = norm(v);

plot(allPoints(1,maxIdx), allPoints(2,maxIdx), Marker="*", Color="b", MarkerSize=10, LineWidth=2);

disp(['Norm found ' num2str(vNorm)]);
disp(['Max norm ', num2str(maxNorm)]);

if abs(vNorm - maxNorm) > 10*eps
    save vertexFail.mat;
end
