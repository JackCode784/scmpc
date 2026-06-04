%% Main_LMPC_BESS modification test
%
clc;
clear;

load("data_model_BESS.mat", "Jmin", "Z", "Z_", "Sys_m");

Z0 = Z_{end};     % prendo uno zonotopo (ridotto) inizale?

% Modifiche di coerenza col modello ARX
% y(k+1) = a_1 y(k) + a_2 y(k-1) + ... + a_na y(k-na+1) +
%           ... + b_1 u(k-nk) + b_1 u(k-nk-1) + ... + b_nb u(k-nk-nb+1)
%        = [Y, U]*theta

% ny = 1; % output dimension
% nu = 1; % input dimension

na = 3; % uguale
nb = 3; % modificato per coerenza
nk = 1; % uguale
l = max(na, nk+nb-1)+1; % primo slot temporale "libero"
n = na + nb; % numero di parametri da stimare

%% Setup
yalmip('clear')
N_decision = 1;                   % decision variables
Epsilon = 10/100;                 % state constraint violation rate at 10%
N_scn = N_decision/Epsilon - 1;   % number of required scenarios

T_hzn = nk+15;    % prediction horizon -> funzione costo; che è nk?
Scn_cost = 0;     % tracking cost of all scenarios if Scn_cost=1
scenario = 1;     % Robust constraint active if scenario=1, else nominal MPC

% Cost parameters (matrici di peso dello stato e controllo
% rispettivamente?)
Q = 5;    % power tracking error cost
R = 0.1;  % input effort cost

% Input Limits
Umax = 200;   % maximal value for Power command input
Umin = -200;  % minimal value for Power command input

% Decision variables (modified)
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

%% Constraints definition
% Power ramp constraint
Constraints = [];
% Constraints = -Dy <= diff(Yini) <= Dy;
% Constraints = [Constraints; -Dy <= diff(Y_nominal) <= Dy];

% Initial condition constraint  for nominal model
Constraints = [Constraints; Y_nominal(1:na) == Yini(end:-1:1)];
Constraints = [Constraints; U(1:nb+nk-2) == Uini(end:-1:1)];

% Per ogni scenario impongo i primi l valori delle uscite
for z=1:N_scn  % Initial condition constraint for N_scn scenarios
    Constraints = [Constraints; Y_scenarios(z, 1:na) == Yini(end:-1:1)];
end

% THIS CONSTRAINTS MAKES OPIMIZATION UNFEASIBLE
% Constraints = [Constraints; -Ymax*1000 <= Y_scenarios(:,na+1:end) <= Ymax*1000];

Constraints = [Constraints; Umin <= U(nb+nk-1:end) <= Umax];
Constraints = [Constraints; -Ymax <= Y_nominal(na+1:end) <= Ymax];

for t=1:T_hzn
    Constraints = [Constraints; Y_nominal(t+na) == [Y_nominal(t+na-1:-1:t), U(t+nb-1:-1:t)] * theta_nominal];
end

if scenario == 1
    for t=1:T_hzn
        for z=1:N_scn  % Dynamic cosntraints for N_scn scenarios
            Constraints = [Constraints ; Y_scenarios(z,t+na) == [Y_scenarios(z,t+na-1:-1:t), U(t+nb-1:-1:t)]*theta_scenarios(:,z)]; % equality constraint
        end
    end
end

%% Objective Function for nominal model
cost = 0;

for t=1:T_hzn
    cost = cost + R*U(t+nk+nb-2)^2 + Q*(Y_nominal(t+na)-Ysp(t))^2; % Tracking cost for nominal model (Zonotope center)

    if Scn_cost == 1
        for k=1:N_scn
            cost = cost + Q / N_scn * (Y_scenarios(k, t+na) - Ysp(t))^2;
        end
    end
end

Objective = cost;

%% Build Struct for optimizer
parameters_in = {theta_scenarios,theta_nominal,Yini,Uini,Ymax,Ysp,Dy};  % quantità al variare delle quali cambia la soluzione?
solutions_out = {U,Y_scenarios,Y_nominal};   % valori ottimi di queste quantità
% solutions_out = {U};
ops = sdpsettings('solver','quadprog','verbose',1,'usex0',0);

Struct_MPC = optimizer(Constraints,Objective,ops,parameters_in,solutions_out);

%% MPC and simulation setup
N_sim = 250;

% Values to (until now) symbolic constants
Dy = 100;
lomax1 = 110;

noise_on = 2;
Epsilon = Jmin * 1.1;

% sys = Sys_m;
% X = zeros(order(sys), N_sim);
Y_sim = zeros(1, N_sim);
U_opt = zeros(1, N_sim);
U_vol = zeros(2, N_sim); % non-reduced and reduced volumes
distances = zeros(1,N_sim); % distance between true theta and zonotope center
thetaVec = zeros(n,N_sim);

Psp=idinput(N_sim+T_hzn,'prbs',[0, 0.05], [-100,100]); % Power reference
% Psp = 100*sin(2*pi*3*(0:N_sim+T_hzn-1)*Ts)';
% Psp = 5 + zeros(N_sim+T_hzn, 1); % Power reference

AA = 0;   % conta se MPC infattibile?
cont = 0; % conta se MPC fattibile?
Y_ini = zeros(1,na);    % output initial condition
U_ini = zeros(1,nb+nk-1);    % input initial condition

% Control loop
% GENMAX = 10;

% True system
% [thetaTrue, ~] = scenary(Z0, 1);
% thetaTrue = thetaTrue';
% Z0 = zonotope(thetaTrue .* (1 + 0.1*(2*rand(n,1)-1)), diag(thetaTrue)*0.1);
thetaTrue = Z0.c;

% thetaTrue = [
%     0.862675492640401
%     0.067411715284747
%     -0.239852802612219
%     0.038680873615599
%     0.130048666369714
%     0.007374812192510];

Z{1} = Z0;  % condizione iniziale zonotopo
Z_{1} = Z0; % condizione iniziale zonotopo (ridotto)

for t_mpc=1:N_sim
    % Output measurement
    % Y_sim(t_mpc) = sys.C * X(:,t_mpc) + noise_on * 2 *(0.5 - rand(1));
    Y_sim(t_mpc) = [Y_ini, U_ini(nk:nk+nb-1)] * thetaTrue + 2 * noise_on * (0.5 - rand(1));    % ARX alternative

    % Aggiornamento zonotopo
    % Z{t_mpc+1} = boundStripZonotopeInt(Z{t_mpc}, Y_sim(t_mpc), Y_ini, U_ini, Epsilon, nb, nk);
    % Z_{t_mpc+1} = reduce(Z{t_mpc+1}, 'pca', 1);

    Fy = Y_ini;
    Fu = U_ini(nk:nk+nb-1);
    Ft = [Fy, Fu];      % [y(k-1), ..., y(k-na), u(k-nk),...,u(k-nk-nb+1)]
    Yt = Y_sim(t_mpc);  % y(k)

    C = [Ft; -Ft];
    d = [Yt + Epsilon; -Yt + Epsilon];
    strip = polytope(C, d);
    oldRedZon = Z_{t_mpc};
    oldNonRedZon = Z{t_mpc};
    % if t_mpc < 100
    %     Z{t_mpc+1} = Z{t_mpc};
    %     Z_{t_mpc+1} = Z_{t_mpc};
    % else
        if contains(strip, oldNonRedZon) == 1
            Z{t_mpc+1} = oldNonRedZon;
            Z_{t_mpc+1} = oldRedZon;
        elseif true%isIntersecting(strip, oldNonRedZon)
            H1 = Z{t_mpc}.G;
            c1 = Z{t_mpc}.c;
            lambdaStar = (H1*H1')*(Ft')/(Ft*(H1*H1')*(Ft')+Epsilon^2);
            centerStar = c1+lambdaStar*(Yt-Ft*c1);
            HStar = [(eye(n)-lambdaStar*Ft)*H1, Epsilon*lambdaStar];
            Z{t_mpc+1} = zonotope(centerStar, HStar);
            Z_{t_mpc+1} = reduce(Z{t_mpc+1}, 'pca', 1);

            % ngen = size(Z{t_mpc+1}.G, 2);
            % if ngen > GENMAX
            %     % Z_{t_mpc+1} = reduce(Z{t_mpc+1}, 'pca', GENMAX/n);
            %     Z_{t_mpc+1} = Z_{t_mpc};
            % else
            %     Z_{t_mpc+1} = Z{t_mpc+1};
            % end
        else
            warning('Empty FPS at iteration');
            t_mpc
        end
    % end

    U_vol(:, t_mpc)= [0; volume(Z_{t_mpc})];
    distances(t_mpc) = norm(thetaTrue - Z_{t_mpc}.c);
    thetaVec(:,t_mpc) = Z_{t_mpc}.c;

    [Theta_scenarios,Theta_nominal] = scenary(Z_{t_mpc+1},N_scn); % genera nuovi parametri nominali e scenari
    Y_ini = [Y_sim(t_mpc), Y_ini(1:end-1)];
    inputs = {Theta_scenarios(:,:)',Theta_nominal(:,:)',Y_ini,U_ini(1:nb+nk-2),lomax1, Psp(t_mpc+1:t_mpc+T_hzn)',Dy};
    [Output_Values,error] = Struct_MPC(inputs);

    % Check if MPC was succesful
    if(error==0)
        cont=cont+1; % 0 errors
    else
        AA=AA+1;
        warning('MPC unfeasible')
    end

    U_opt(t_mpc) = Output_Values{1}(nb+nk-1);
    % if isnan(U_opt(t_mpc))
    %     U_opt(t_mpc) = U_opt(t_mpc-1);
    % end
    U_ini = [U_opt(t_mpc), U_ini(1:end-1)]; % u(k), ..., u(k-nk-nb+2)

    % X(:,t_mpc+1)=sys.A*X(:,t_mpc)+sys.B*(U_opt(t_mpc)+2*randn(1));
    t_mpc
end

%% Error computation and plots
err = Y_sim - Psp(1:N_sim)';
errRmse = sqrt(sum(err.^2) / length(err));

% Plot
% close all;

figure;
subplot(2,1,1);
hold on;
plot(Y_sim, 'LineWidth',1.5);
plot(Psp(1:N_sim), 'LineWidth',1.5, 'LineStyle','-.');
grid on;
legend('Simulated output', 'Power reference');
xlabel('Time step');

subplot(2,1,2);
hold on;
plot(err, 'LineWidth', 1.5);
plot([1, N_sim], [errRmse, errRmse], 'k--');
hold off;
grid on;
legend('Error', 'RMSE');
ylabel('Error');
xlabel('Time sample');

figure;
hold on;
plot(U_opt);
plot(0*U_opt + Umin, 'k--');
plot(0*U_opt + Umax, 'k--');
grid on;
title('Optimal control');
xlabel('Time step');

figure
subplot(3,1,1);
hold on;
plot(U_vol' / volume(Z0), 'LineWidth',1.5);
legend('Non reduced', 'Reduced');
hold off;
grid on;
title('Uncertainty region volumes');
xlabel('Time step');
ylabel('vol(Zr(t))/vol(Z0)');

subplot(3,1,2);
plot(U_vol(2,2:end) ./ U_vol(2,1:end-1), 'LineWidth',1.5);
grid on;
xlabel('Time instant');
ylabel('vol(Zr(t))/vol(Zr(t-1))');

subplot(3,1,3);
plot(distances / norm(thetaTrue), 'LineWidth',1.5);
xlabel('Time step');
ylabel('||\theta_t-\theta_n|| / ||\theta_t||');
grid on;

figure;
for k=1:n
    subplot(n,1,k);
    hold on;
    plot(thetaVec(k,:), 'LineWidth',1.5);
    plot(ones(1,N_sim) * thetaTrue(k), '--k');
    hold off;
    legend('', 'ref');
    grid on;
    xlabel('Time instant');
    ylabel(append('\theta_', num2str(k)));
end