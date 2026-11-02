%% Test zonotope reduction
% Main results:
% - order 1 reduced zonotope volume with respect to non-reduced grows like
%   ~e^(x^2) with space dimension n.
% - while keeping reduction order high, deletion of more generators creates
%   bigger reduced zonotopes but now by much.
%
% Hence, it's important to keep distant from reducing zonotopes to order 1,
% especially in higher dimensional spaces.
%

clc;
close all;
clear;

n = 2:8; % test dimensions
meanSamples = 20; % n.o. mean samples
gensAdded = [1,10];
means = zeros(numel(n),1);
vols = zeros(2,meanSamples);

% order 1 reduction
i = 1;
for currDim = n
    for s=1:meanSamples
        znr = zonotope(zeros(currDim,1), randn(currDim,currDim+1));
        zr = reduce(znr, 'pca', 1);
        vols(:,s) = [volume(znr); volume(zr)];
    end
    means(i) = mean(vols(2,:) ./ vols(1,:));
    i = i + 1;
end

figure;
hold on;
plot(n, means, 'LineWidth',1.5);
plot(n, exp(0.13*n.^2 - 0.25*n), 'LineWidth',1.5);
hold off;
grid on;
xlabel('Space dimension');
ylabel('Vol(Z_r) / Vol(Z_{nr})');
title('Volume ratios for order=1 reduction');
legend('Volumes', 'e^{0.13 n^2 - 0.25 n}');

means = zeros(numel(n), numel(gensAdded));

% other results
i = 1;
n = 2:6;

for currDim = n
    j = 1;
    for currAdd = gensAdded
        ngensStart = currDim ^ 2 + currAdd;

        for s=1:meanSamples
            znr = zonotope(zeros(currDim,1), randn(currDim,ngensStart));
            zr = reduce(znr, 'pca', currDim); % final n.o. generators is currDim^2
            vols(:, s) = [volume(znr); volume(zr)];
        end
        means(i,j) = mean(vols(2,:) ./ vols(1,:));
        j = j + 1;
    end
    i = i + 1;
end

figure;
hold on;
for i=1:length(gensAdded)
    plot(n, means(1:length(n),i), 'LineWidth',1.5, 'DisplayName',append(num2str(gensAdded(i)), ' more gens'));
end
legend('-DynamicLegend');
legend('boxoff');
hold off;
grid on;
xlabel('Space dimension');
ylabel('Vol(Z_r) / Vol(Z_{nr})');
title('Volume ratios for order=n^2 reduction');
