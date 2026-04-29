%% Plot output data from CPP MADS ARX implementation
function plotOutputCpp(vitisbool)
clc;
% close all;

if vitisbool
    path = "C:\Users\jackf\Documents\MPC\ARXforVITIS\VitisProj\ARX\solution1\csim\build\";
else
    path = "";
end

% load(append(path, "output.txt"));
% uSim = output(:,1);
% ySim = output(:,2);
% yref = output(:,3);
% uMin = output(:,4);
% uMax = output(:,5);
% yMin = output(:,6);
% yMax = output(:,7);

% Save data as table instead of matrix
data = readtable(append(path, "output.txt"), VariableNamingRule="preserve");

figure; subplot(2,1,1);
hold on;
plot(data.ySim, 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Simulated output');
plot(data.yref, 'r--', 'LineWidth', 1.5);
plot(data.yMin, 'k--', 'LineWidth',1.5);
plot(data.yMax, 'k--', 'LineWidth',1.5);
hold off;
legend('y(k)', 'yref');
subplot(2,1,2);
hold on;
stairs(data.uSim, 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Optimized input');
plot(data.uMax,'k--', 'LineWidth',1.5);
plot(data.uMin,'k--', 'LineWidth',1.5);

if(width(data) > 7)
    % uSimDig = output(:,8);
    % ySimDig = output(:,9);

    figure;
    subplot(2,1,1);
    hold on;
    plot(data.ySimDig, 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Simulated digital output');
    hold off;
    subplot(2,1,2);
    hold on;
    stairs(data.uSimDig, 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Optimized digital input');

    figure;
    plot(data.vol, 'LineWidth',1.5);
    grid on;
    xlabel('Time instant');
    ylabel('Volume (normalized)');
    title('Normalized zonotope volume');
end

%% Latency/area occupation dependency on number of scenarios
% Keeping the same FPGA board, same #pragma directives
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