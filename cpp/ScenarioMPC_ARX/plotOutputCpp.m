%% Plot output data from CPP MADS ARX implementation
function plotOutputCpp(opts)
clc;
% close all;

if opts.isVitisSim
    titleStr = 'Fixed point';
    d = "..\..\vitis\scmpc-arx\solution1\csim\build\";
    % d = dir(fullfile(cd, '**', 'output.txt'));
    % d = d(1).folder;
else
    titleStr = 'Floating point';
    d = "";
end

% Save data as table instead of matrix
data = readtable(append(d, "output.txt"), VariableNamingRule="preserve");

% Tracking error
data.err = data.yref - data.ySim;
errMean = mean(data.err);
errRMSE = sqrt(mean(data.err.^2));

% Output "derivative" computation
yDer = diff(data.ySim);
data.("deltaY") = 0.1 * ones(height(data),1);

% Plots
% Output simulations
figure(Name=titleStr); subplot(3,1,1);
hold on;
yyaxis left;
plot(data.ySim, 'LineWidth', 1.5); grid on;xlabel('k');
plot(data.yref, 'r--', 'LineWidth', 1.5);
yline(data.yMin(1), 'k--', num2str(data.yMin(1)), 'LineWidth',1.5);
yline(data.yMax(1), 'k--', num2str(data.yMax(1)), 'LineWidth',1.5);
hold off;
ylabel('Simulated analog output');
legend('y_k', 'yref');
if ismember('ySimDig', data.Properties.VariableNames)
    % uSimDig = output(:,8);
    % ySimDig = output(:,9);

    % figure;
    % subplot(2,1,1);
    yyaxis right;
    hold on;
    stairs(data.ySimDig, 'LineWidth', 1.5); grid on; ylabel('Simulated digital output');
    hold off;
    legend('off');
end
title('Output simulation');

% y_k - y_{k-1} plot
subplot(3,1,2);
hold on;
plot(yDer, LineWidth=1.5);
yline(data.deltaY(1)*[-1, 1], 'k--', {num2str(-data.deltaY(1)),num2str(-data.deltaY(1))}, LineWidth=1.5);
hold off;
grid on;
xlabel('k');
ylabel('y_k - y_{k-1}');
title('\Deltay');

% Optimal input
subplot(3,1,3);
hold on;
plot(data.uSim, 'LineWidth', 1.5); grid on;xlabel('k');
plot(data.uMax,'k--', 'LineWidth',1.5);
plot(data.uMin,'k--', 'LineWidth',1.5);
hold off;
ylabel('u^*_k');
if ismember('uSimDig', data.Properties.VariableNames)
    yyaxis right;
    hold on;
    stairs(data.uSimDig, 'LineWidth', 1.5); grid on;xlabel('k');ylabel('Optimized digital input');title('Optimal input');
end

% Tracking error plot
figure;
subplot(2,1,1);
hold on;
% boxplot(err, Orientation="horizontal");
% xline(errMean, 'r--', LineWidth=1.5);
% xline(errRMSE, 'k:', LineWidth=1.5);
plot(data.err, LineWidth=1.5);
yline(errMean, 'r--', num2str(errMean), LineWidth=1.5);
yline(errRMSE, 'k:', num2str(errRMSE), LineWidth=1.5);
hold off;
grid on;
title('Tracking error');
xlabel('k');
legend('e_k = y^*_k - y_k', 'Mean', 'RMSE');

% Zonotope volume
% figure;
subplot(2,1,2);
plot(data.vol, 'LineWidth',1.5);
grid on;
xlabel('k');
ylabel('Vol(Z_k)');
title('Normalized zonotope volume');

% Latency/area occupation dependency on number of scenarios
% Keeping the same FPGA board, same #pragma directives

if opts.plotSpecs
    plotBoardSpecs();
end

end

function plotBoardSpecs()
nscenarios = [10, 25, 50, 100];
latencies = [18031, 40651, 16316, 27216];  % clock cycles
occupationPercs = [0, 27, 10, 36, 0;
    0, 27, 11, 39, 0;
    0, 38, 13, 38, 0;
    0, 38, 13, 38, 0]; % BRAM, DSP, FF, LUT, URAM

% Latencies
figure;
subplot(2,1,1);
stem(latencies, 'Marker','x', 'MarkerSize',10, 'LineWidth',1.5);
set(gca, Xtick=1:length(nscenarios), XTickLabel=nscenarios);
xlabel('Number of scenarios');
ylabel('Latency [clock cycles]');
grid on;

% Occupation percentages
subplot(2,1,2);
bar(occupationPercs);
set(gca, Xtick=1:length(nscenarios), XTickLabel=nscenarios);
set(gca, XTickLabel=nscenarios);
xlabel('Number of scenarios');
ylabel('Percentages');
legend('BRAM', 'DSP', 'FF', 'LUT', 'URAM');
grid on;
end