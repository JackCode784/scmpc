%% BESS power regulation system identification
close all
clear all
clc

Ts=10e-3;%sample time

% Experimental data
load('EscoData_With1Delay.mat')

% Identification dataset
To=1;
Tend=9000;


In = Input(To:Tend);% Power request 
Output = P_act_out(To:Tend);% Measured power at the PCC

% Model estimation
data = iddata(Output,In,Ts);
sys=arx(data,[5 5 1]);
Sys_m=ss(sys);
% Model simulation
d=2*randn(length(Input),1); % Process disturbance-Gaussian process with 2kW std. deviation
y_sim=lsim(sys,Input+d)+4*(rand(length(Input),1)-0.5); % Measurement noise +/- 2kW

ui=Input;
yi=y_sim;

save system_data Sys_m ui yi Ts

%%
figure
hold on
plot(Input)
plot(P_act_out)
plot(y_sim)
legend('Power request','BESS output','Model output')



