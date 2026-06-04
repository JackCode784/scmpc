%% testBugs.m
% Script to test optimizer returning NaN issue
% 

clc;
clear;

load("data_model_BESS.mat", "Jmin", "Z_", "Ts");

na = 3;
nb = 3;
nk = 1;

n = na + nb;
Z0 = Z_{end};

%% Yalmip problem formulation
yalmip('clear');

N_scn = 9;
T_hzn = nk + 15;
scn_cost = 0;
scn_out = 1;

Q = 5;
R = 0.1;

Umax = 200;
Umin = -Umax;

% Decision variables
U = sdpvar(1, T_hzn+nb+nk-2);
Uini = sdpvar(1,nb+nk-2);    % initial condition for ARX model

Y_scenarios = sdpvar(N_scn, T_hzn+na);
Y_nominal = sdpvar(1,T_hzn+na);
Ysp = sdpvar(1, T_hzn);
Ymax = sdpvar(1,1);              % bound on output power |Y| <= Ymax
Yini = sdpvar(1,na);            % initial condition for ARX model
Dy = sdpvar(1,1);                % Bound on output power variation

theta_scenarios = sdpvar(n,N_scn);      % ARX model parameters for random scenarios; commento scambiato con quello sotto
theta_nominal = sdpvar(n,1);          % ARX nominal model parameter; commento scambiato con quello sopra

%% Constraints
freeConstr = [];
Constraints = [];

Constraints = [Constraints; Y_nominal(1:na) == Yini(end:-1:1)];
Constraints = [Constraints; U(1:nb+nk-2) == Uini(end:-1:1)];
Constraints = [Constraints; Y_scenarios(:, 1:na) == ones(N_scn, 1) * Yini(end:-1:1)];
freeConstr = [freeConstr; Constraints];

Constraints = [Constraints; Umin <= U(nb+nk-1:end) <= Umax];
Constraints = [Constraints; -Ymax <= Y_nominal(na+1:end) <= Ymax];
Constraints = [Constraints; -Ymax <= Y_scenarios(:, na+1:end) <= Ymax];

for t=1:T_hzn
    Constraints = [Constraints; Y_nominal(t+na) == [Y_nominal(t+na-1:-1:t), U(t+nb-1:-1:t)] * theta_nominal];
    freeConstr = [freeConstr; Y_nominal(t+na) == [Y_nominal(t+na-1:-1:t), U(t+nb-1:-1:t)] * theta_nominal];
end

if scn_out == 1
    for t=1:T_hzn
        for z=1:N_scn  % Dynamic cosntraints for N_scn scenarios
            Constraints = [Constraints ; Y_scenarios(z,t+na) == [Y_scenarios(z,t+na-1:-1:t), U(t+nb-1:-1:t)]*theta_scenarios(:,z)]; % equality constraint
            freeConstr = [freeConstr; Y_scenarios(z,t+na) == [Y_scenarios(z,t+na-1:-1:t), U(t+nb-1:-1:t)]*theta_scenarios(:,z)];
        end
    end
end

%% Cost function
cost = 0;

for t=1:T_hzn
    cost = cost + R*U(t+nk+nb-2)^2 + Q*(Y_nominal(t+na)-Ysp(t))^2; % Tracking cost for nominal model (Zonotope center)

    if scn_cost == 1
        for t_mpc=1:N_scn
            cost = cost + Q / N_scn * (Y_scenarios(t_mpc, t+na) - Ysp(t))^2;
        end
    end
end

%% Optimizer
parameters_in = {theta_scenarios, theta_nominal, Yini, Uini, Ymax, Ysp};  % quantità al variare delle quali cambia la soluzione?
solutions_out = {U, Y_scenarios, Y_nominal};   % valori ottimi di queste quantità
ops = sdpsettings('solver','quadprog','verbose',1,'usex0',0);

constrMPC = optimizer(Constraints, cost, ops, parameters_in, solutions_out);
freeMPC = optimizer(freeConstr, cost, ops, parameters_in, solutions_out);

%% Simulation
N_sim = 250;

yMax = 110;
noiseAmp = 2;
Epsilon = Jmin * 1.1;

Y_sim = zeros(2, N_sim);
U_opt = zeros(2, N_sim);
U_vol = zeros(2, N_sim, 2); % non-reduced and reduced volumes
distances = zeros(2,N_sim); % distance between true theta and zonotope center
thetaVec = zeros(n, N_sim, 2);

Psp=idinput(N_sim+T_hzn,'prbs',[0, 0.05], [-100,100]); % Power reference

AA = 0;   % conta se MPC infattibile?
cont = 0; % conta se MPC fattibile?
fails = [];
Y_ini = zeros(2,na);    % output initial condition
U_ini = zeros(2,nb+nk-1);    % input initial condition

% True system
[thetaTrue, ~] = scenary(Z0, 1);
thetaTrue = thetaTrue';
Z0 = zonotope(thetaTrue .* (1 + 0.1*(2*rand(n,1)-1)), diag(thetaTrue)*0.1);

Z = cell(N_sim, 2);
Z_ = cell(N_sim, 2);
Z{1,1} = Z0;
Z{1,2} = Z0;
Z_{1,1} = Z0;
Z_{1,2} = Z0;

for t_mpc=1:N_sim
    Y_sim(:, t_mpc) = [Y_ini, U_ini(:, nk:nk+nb-1)] * thetaTrue + 2 * noiseAmp * (0.5 - rand(1));
    Z{t_mpc+1, 1} = boundStripZonotopeInt(Z{t_mpc, 1}, Y_sim(1, t_mpc), Y_ini(1,:), U_ini(1,:), Epsilon, nb, nk);
    Z{t_mpc+1, 2} = boundStripZonotopeInt(Z{t_mpc, 2}, Y_sim(2, t_mpc), Y_ini(1,:), U_ini(2,:), Epsilon, nb, nk);
    Z_{t_mpc+1, 1} = reduce(Z{t_mpc+1, 1}, 'pca', 1);
    Z_{t_mpc+1, 2} = reduce(Z{t_mpc+1, 2}, 'pca', 1);
        
    U_vol(:, t_mpc, 1) = [0; volume(Z_{t_mpc,1})];
    U_vol(:, t_mpc, 2) = [0; volume(Z_{t_mpc,2})];
    distances(1, t_mpc) = norm(thetaTrue - Z_{t_mpc,1}.c);
    distances(2, t_mpc) = norm(thetaTrue - Z_{t_mpc,2}.c);
    thetaVec(:, t_mpc, 1) = Z_{t_mpc, 1}.c;
    thetaVec(:, t_mpc, 2) = Z_{t_mpc, 2}.c;

    Y_ini = [Y_sim(:, t_mpc), Y_ini(:, 1:end-1)];

    [Theta_scenarios,Theta_nominal] = scenary(Z_{t_mpc+1, 1},N_scn);
    constrInputs = {Theta_scenarios(:,:)',Theta_nominal(:,:)',Y_ini(1,:), U_ini(1,1:nb+nk-2),yMax, Psp(t_mpc+1:t_mpc+T_hzn)'};
    [Theta_scenarios, Theta_nominal] = scenary(Z_{t_mpc+1, 2}, N_scn);
    freeInputs = {Theta_scenarios(:,:)', Theta_nominal(:,:)', Y_ini(2,:), U_ini(2, 1:nb+nk-2), yMax, Psp(t_mpc+1:t_mpc+T_hzn)'};

    [constrOut, constrError] = constrMPC(constrInputs);
    [freeOut, freeError] = freeMPC(freeInputs);

    U_opt(:, t_mpc) = [constrOut{1}(nb+nk-1); freeOut{1}(nb+nk-1)];

    if isnan(U_opt(1,t_mpc))
        fails = [fails, t_mpc];
        U_opt(1, t_mpc) = U_opt(2,t_mpc);
    end

    U_ini = [U_opt(:, t_mpc) U_ini(:,1:end-1)];

    t_mpc
end

%% Error computation and plots
err = Y_sim - Psp(1:N_sim)';
errRmse = sqrt(sum(err.^2, 2) / size(err, 2));

% Plot
close all;

figure;
subplot(2,1,1);
hold on;
plot(Y_sim', 'LineWidth',1.5);
plot(Psp(1:N_sim), 'LineWidth',1.5, 'LineStyle','-.');
grid on;
xlabel('Time step');
legend('constrOut', 'freeOut', 'ref');
ylim([-100, 100]);

subplot(2,1,2);
hold on;
plot(err', 'LineWidth',1.5);
plot([1, N_sim], [errRmse(1), errRmse(1)], 'k--');
plot([1, N_sim], [errRmse(2), errRmse(2)], 'k--');
hold off;
grid on;
legend('constrErr', 'freeErr', 'constrRMSE', 'freeRMSE');
ylabel('Error');
xlabel('Time sample');
ylim([-100,100]);

figure;
hold on;
plot(U_opt', 'LineWidth',1.5);
plot([1, N_sim], [Umax, Umax], 'k--');
plot([1, N_sim], [Umin, Umin], 'k--');
grid on;
title('Optimal control');
legend('Constr', 'Free');
xlabel('Time step');

figure;
subplot(3,1,1);
hold on;
plot(U_vol(2, :, 1) / volume(Z0), 'LineWidth',1.5);
plot(U_vol(2, :, 2) / volume(Z0), 'LineWidth',1.5);
legend('constr', 'free');
hold off;
grid on;
title('Uncertainty region volumes');
xlabel('Time step');
ylabel('vol(Zr(t))/vol(Z0)');

subplot(3,1,2);
hold on;
plot(U_vol(2,2:end,1) ./ U_vol(2,1:end-1,1), 'LineWidth',1.5);
plot(U_vol(2,2:end,2) ./ U_vol(2,1:end-1,2), 'LineWidth',1.5);
hold off;
grid on;
xlabel('Time instant');
ylabel('vol(Zr(t))/vol(Zr(t-1))');

subplot(3,1,3);
plot(distances' / norm(thetaTrue), 'LineWidth',1.5);
xlabel('Time step');
ylabel('||\theta_t-\theta_n|| / ||\theta_t||');
grid on;

figure;
for k=1:n
    subplot(n,1,k);
    hold on;
    plot(thetaVec(k,:,1), 'LineWidth',1.5);
    plot(thetaVec(k,:,2), 'LineWidth',1.5);
    plot(ones(1,N_sim) * thetaTrue(k), '--k');
    xline(fails, 'LineWidth',1.5);
    hold off;
    legend('constr', 'free', 'ref');
    grid on;
    xlabel('Time instant');
    ylabel(append('\theta_', num2str(k)));
end
