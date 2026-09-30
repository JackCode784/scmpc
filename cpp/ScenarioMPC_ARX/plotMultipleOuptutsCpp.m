function plotMultipleOuptutsCpp(opts)

clc; 
% clear; 
% close all;

if opts.isVitisSim
    titleStr = 'Fixed point';
    d = "..\..\vitis\scmpc-arx\solution1\csim\build\";
else
    titleStr = 'Floating point';
    d = pwd;
end

outputFiles = dir(fullfile(d, '', '*.txt'));
nFiles = length({outputFiles.name});
allData = struct();

errMeans = zeros(nFiles,1);
errRMSEs = zeros(nFiles,1);

for i=1:nFiles
    curData = ['data', num2str(i)];
    allData.(curData) = readtable(append(d, '\', outputFiles(i).name), VariableNamingRule="preserve");

    % Tracking error
    allData.(curData).err = allData.(curData).yref - allData.(curData).ySim;
    errMeans(i) = mean(allData.(curData).err);
    errRMSEs(i) = sqrt(mean(allData.(curData).err.^2));
end

% Plots
figure;
subplot(3,1,1);
hold on;
for i=1:nFiles
    curData = ['data', num2str(i)];
    plot(allData.(curData).ySim, LineWidth=1.5); grid on; xlabel('k');
end
plot(allData.(curData).yref, 'r--', LineWidth=1.5);
yline(allData.(curData).yMin(1), 'k--', num2str(allData.(curData).yMin(1)), 'LineWidth',1.5);
yline(allData.(curData).yMax(1), 'k--', num2str(allData.(curData).yMax(1)), 'LineWidth',1.5);
hold off;
ylabel('Simulated analog output');
legend('yref');
title('Output simulation');

% y_k - y_{k-1} plot
subplot(3,1,2);
hold on;
for i=1:nFiles
    curData = ['data', num2str(i)];
    plot(diff(allData.(curData).ySim), LineWidth=1.5);
end
yline(allData.(curData).deltaY(1)*[-1, 1], 'k--', {num2str(-allData.(curData).deltaY(1)),num2str(allData.(curData).deltaY(1))}, LineWidth=1.5);
hold off;
grid on;
xlabel('k');
ylabel('y_k - y_{k-1}');
title('\Deltay');

% Optimal input
subplot(3,1,3);
hold on;
for i=1:nFiles
    curData = ['data', num2str(i)];
    plot(allData.(curData).uSim, LineWidth=1.5); grid on; xlabel('k');
end
plot(allData.(curData).uMax,'k--', 'LineWidth',1.5);
plot(allData.(curData).uMin,'k--', 'LineWidth',1.5);
hold off;
ylabel('u^*_k');

end