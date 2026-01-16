%% Plot output data from CPP MADS ARX implementation

close all;
path = "";

% Comment this to print VSCode simulation, otherwise -> Vitis's results
% path = "C:\Users\jackf\Documents\MPC\ARXforsetVITIS\VitisProj\ARX\solution1\csim\build\";

load(append(path, "output.txt"));
figure; subplot(2,1,1);
hold on;
plot(output(:,2), 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Simulated output');
plot(output(:,2)*0+5, 'k--', 'LineWidth', 1.5);
hold off;
subplot(2,1,2);
hold on;
stairs(output(:,1), 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Optimized input');
plot(output(:,1)*0+0.3,'k--', 'LineWidth',1.5);
plot(output(:,1)*0-0.3,'k--', 'LineWidth',1.5);

if(size(output, 2) > 2)
    figure;
    subplot(2,1,1);
    hold on;
    plot(output(:,4), 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Simulated digital output');
    hold off;
    subplot(2,1,2);
    hold on;
    stairs(output(:,3), 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Optimized digital input');
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
