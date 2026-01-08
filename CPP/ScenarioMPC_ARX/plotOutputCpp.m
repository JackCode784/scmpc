%% Plot output data from CPP MADS ARX implementation

close all;
path = "";

% Comment this to print VSCode simulation, otherwise -> Vitis's results
path = "C:\Users\jackf\Documents\MPC\ARXforVITIS\VitisProj\ARX\solution1\csim\build\";

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
