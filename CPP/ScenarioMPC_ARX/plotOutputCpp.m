%% Plot output data from CPP MADS ARX implementation
% 
close all;
load output.txt;
figure; subplot(2,1,1);
hold on;
plot(output(:,2), 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Simualted output');
plot(output(:,2)*0+5, 'k--', 'LineWidth', 1.5);
hold off;
subplot(2,1,2);
hold on;
stairs(output(:,1), 'LineWidth', 1.5); grid on;xlabel('Sample');ylabel('Optimized input');
plot(output(:,1)*0+0.3,'k--', 'LineWidth',1.5);
plot(output(:,1)*0-0.3,'k--', 'LineWidth',1.5);
