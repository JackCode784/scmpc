%% Main_LMPC_BESS.m
% This file runs a Scenario MPC loop, given an ARX zonotopic uncertainty model of a BESS
% It requires the CORA toolbox, YAlmip and Quadprog.
% Nominal model and Zonotopic uncertainty are contained in file data_model_BESS.mat

clc;
close all;
clear;
rng("default");
learning = 1;
plots = 1;

% Alcune variabili:
% - l = max(na, nb+nk)+1: istante temporale più "vecchio" che compare
% nell'espressione dell'uscita.
% - na: numero di coefficienti relativi ai precedenti valori dell'uscita
% nel modello ARX
% - nb: numero di coefficienti relativi ai precedenti valori dell'ingresso
% nel modello ARX
% - nk: ritardo del modello, i primi nk campioni dell'ingresso non
% influenzano l'uscita

% Load resistance
R = 6;
% Inductance
L = 150e-6;
% Capacitance
C = 15e-6;
f = 100e3;
tau = 1/f;
Vin = 10;

na = 2;
nb = 1;

thetaTrue = [(1-(tau-R*C)/(R*C))
    -(1+(tau^2*R-tau*L)/(R*L*C))
    tau^2*Vin/(L*C)];

unc = 1.2;
thetaTrue(1) = (1-(tau-unc*R*unc*C)/(unc*R*unc*C));
thetaTrue(2) = -(1+(tau^2*unc*R-tau*unc*L)/(unc*R*unc*L*unc*C));
thetaTrue(3) = tau^2*unc*Vin/(unc*L*unc*C);
% thetaTrue = thetaTrue+0.01*randn(size(thetaTrue));

c0 = [(1-(tau-R*C)/(R*C))
    -(1+(tau^2*R-tau*L)/(R*L*C))
    tau^2*Vin/(L*C)];

G0 = [diag(c0)*0.2];

Z0 = zonotope(c0,G0);     % prendo uno zonotopo (ridotto) inizale?
n=length(Z0.c); % dimensione dello zonotopo (e spazio dei parametri)

if plots
    figure(100)
    hold on
    plot(Z0,1:3)
    plot3(c0(1),c0(2),c0(3),'k.','MarkerSize',20)
    plot3(thetaTrue(1),thetaTrue(2),thetaTrue(3),'g.','MarkerSize',20)
end


%% MPC formulation - Yalmip

yalmip('clear')
N_decision=1;                   % decision variables
Epsilon=0.1;                 % state constraint violation rate at 10%
N_scn = 5;

T_hzn=10;    % prediction horizon -> funzione costo
Scn_cost=0;     % tracking cost of all scenarios if Scn_cost=1
scenario=1;     % Robust constraint active if scenario=1, else nominal MPC

% Cost parameters (matrici di peso dello stato e controllo
% rispettivamente?)
QQ = 1;    % power tracking error cost
RR = 10;  % input effort cost

% Input Limits
Umax=1;   % maximal value for Power command input
Umin=0;  % minimal value for Power command input

% Decision Variables
U = sdpvar(1,T_hzn-1) ;   % Power request input; crea vettore riga 16+4-2 elementi

Y   = sdpvar(N_scn,T_hzn); % Output power evolution for N scenarios (uscita vista anche per un passo in più)
Y0  = sdpvar(1,T_hzn);     % Output power evolution for nominal model
Ysp = sdpvar(1,T_hzn);     % Power set point
Ymax = sdpvar(1,1);             % bound on output power |Y| <= Ymax
Yini = sdpvar(1,na);           % initial condition for ARX model
Uini = sdpvar(1,nb);      % initial condition for ARX model
theta1 = sdpvar(n,N_scn);      % ARX model parameters for random scenarios; commento scambiato con quello sotto
theta0 = sdpvar(n,1);          % ARX nominal model parameter; commento scambiato con quello sopra

%% Constraints definition
Constraints=[];

% Initial condition constraint  for nominal model
% Primi l valori di Y nel modello nominale sono fissati
Constraints = [Constraints ; Y0(1:na)==[Yini]]; % Yini= y(k-na+1):y(k) (na values)

% Primi l-1 valori dell'ingresso sono fissati
Constraints = [Constraints ; U(1:nb)==[Uini]]; %Uini=u(k-nk-nb+1):u(k-1) (nb+nk-1 values)

% Per ogni scenario impongo i primi l valori delle uscite
for z=1:N_scn  % Initial condition constraint for N_scn scenarios
    Constraints = [Constraints ; Y(z,1:na)==[Yini]]; %equality constraint
end

% Vincoli per ogni istante di tempo a partire da l (regime?)
for t=na:T_hzn-1
    % Power request limits
    Constraints = [Constraints ;U(t)<=Umax];
    Constraints = [Constraints ;U(t)>=Umin];

    % Power output limits (sistema nominale)
    Constraints = [Constraints ; -Ymax<=Y0(t+1)<=Ymax];

    for z=1:N_scn  % Initial condition constraint for N_scn scenarios
        Constraints = [Constraints ; -Ymax <= Y(z,t+1) <= Ymax]; %equality constraint
    end

    % Dynamic contraints
    % y(k+1) = a_1 y(k) + a_2 y(k-1) + ... + a_na y(k-na+1) +
    % ...                    b_0 u(k-nk+1) + b_1 u(k-nk) + ... + b_nb u(k-nk-nb+1)
    %        = theta' * [Y; U]

    % Dynamic constraint  for nominal model
    % Impone che l'uscita segua il modello ARX per tutti gli istanti di
    % tempo {l+1, l+2, ..., T_hzn}
    Constraints = [Constraints ; Y0(t+1)==[Y0(t:-1:t-na+1),U(t-1:-1:t-1-nb+1)]*theta0]; %equality constraint

    % Se facciamo SC-MPC, tutti gli scenari devono rispettare la dinamica
    % (ognuno con i propri parametri, infatti c'è theta1)
    for z=1:N_scn  % Dynamic cosntraints for N_scn scenarios
        Constraints = [Constraints ; Y(z,t+1)==[Y(z,t:-1:t-na+1),U(t-1:-1:t-1-nb+1)]*theta1(:,z)]; %equality constraint
    end
end

%% Objective Function for nominal model
cost = 0;
for t=2:T_hzn-1
    cost=cost + RR*(U(t)-U(t-1))^2 + QQ*(Y0(t+1)-Ysp(t+1))^2; % Tracking cost for nominal model (Zonotope center)

    % Se teniamo conto del costo per tutti gli scenari
    % if Scn_cost== 1
    %     for k=1:N_scn
    %         cost = cost + (Q/N_scn)*(Y(k,t+1)-Ysp(t+1))^2; % Tracking cost for each scenario
    %     end
    % end
end

% Objective = cost+10*Dy^2;
Objective = cost;   % per l'ottimizzatore?

%% Build Struct for optimizer
parameters_in = {theta1,theta0,Yini,Uini,Ymax,Ysp};  % quantità al variare delle quali cambia la soluzione?
solutions_out = {U,Y,Y0};   % valori ottimi di queste quantità

ops = sdpsettings('solver','quadprog','verbose',1,'usex0',0);
% ops = sdpsettings('verbose',0,'usex0',0);
% ops = sdpsettings('verbose',0);

Struct_MPC = optimizer(Constraints,Objective,ops,parameters_in,solutions_out);

%% MPC and simulation setup
clc
N = 300;              % simulation length
lomax1 = 300;       % bound on output power
noise_on=0.0;         % Measurement noise amplitude - 2kW

% Variables
Y_real=zeros(N,1);
Y_sim=zeros(N,1);
U_opt=zeros(N,1);   % controllo ottimo
U_vol=zeros(N,1);   % volume dello spazio di incertezza

% Generate input signals to support system identification
% The idinput command generates an input signal with specified
% characteristics for your system.
% Psp=idinput(N+T_hzn,'prbs',[0, 0.05], [-100,100]); % Power reference
Psp = [repmat(3,1,round(N/3)) repmat(4,1,round(N/3)) repmat(5,1,round(N/3)+T_hzn-1)]';% output looks bad (noise too big)


AA=0;   % conta se MPC infattibile?
cont=0; % conta se MPC fattibile?
Y_ini = zeros(1,na);        % output initial condition
U_ini = zeros(1,nb+1-1);   % input initial condition; sbagliato il numero di campioni?
Z{2}=Z0;                    % initial uncertainty model
Z2{2} = Z0;
U_vol(2)=volume(Z{2});
P{2} = Z0;

%% Control loop
% Added
c1 = Z{2}.c;
H1 = Z{2}.G;

% "Raggio" del box
l = vecnorm(Z{2}.G);
diag0 = norm(l);

for t_mpc=3:N
    t_mpc
    Epsilon = 0.99*Epsilon;
    % Epsilon = sum(abs([Y_sim(t_mpc-1:-1:t_mpc-na), U_opt(t_mpc-1:-1:t_mpc-nb)]*Z2{t_mpc-1}.G)) + noiseAmp;

    % output measurement
    Y_sim(t_mpc) = Y_real(t_mpc) + noise_on*2*(0.5-rand(1)); % rumore in [-2, 2]

    %%%%%%% Information operator F %%%%%%%%%%%%%%
    Fy = flipud(Y_sim(t_mpc-na:t_mpc-1));       % precedenti na campioni ordinati da più recente a meno
    Fu = flipud(U_opt(t_mpc-nb-1:t_mpc-1-1));
    Ft = [Fy', Fu'];    % messi in riga
    Yt = Y_sim(t_mpc);  % campione appena calcolato dell'uscita

    %%% data strip %%%
    C=[Ft;-Ft];
    d=[Yt+Epsilon;-Yt+Epsilon];
    S1 = polytope(C,d);

    if plots
        figure(100)
        cla
        hold on
        % plot(S1,[1:3])
    end

    if stripContainsZonotope(S1.A, S1.b, Z2{t_mpc-1}.c, Z2{t_mpc-1}.G) %contains(S1,Z2{t_mpc-1})
        Z{t_mpc} = Z{t_mpc-1};
        Z2{t_mpc} = Z{t_mpc};
    else

        lamda1=H1*H1'*Ft'/(Ft*H1*H1'*Ft'+Epsilon^2);
        cc1=c1+lamda1*(Yt-Ft*c1);
        HH1=[(eye(n)-lamda1*Ft)*H1,Epsilon*lamda1];
        Z{t_mpc} = zonotope(cc1, HH1);

        Z{t_mpc} = reduceOrder(Z{t_mpc}, 'gs'); % gs

        Hprev = Z{t_mpc-1}.G;
        l = vecnorm(Hprev);
        diagprev = norm(l);
        diag = min([diag0 diagprev]);

        Hr = Z{t_mpc}.G;
        l = vecnorm(Hr);


        for i = 1:numel(l)
            if l(i) > diag
                Hr(:,i) = Hr(:,i)/l(i)*diag;
            end
        end

        Z2{t_mpc} = Z{t_mpc}; % gs
        Z2{t_mpc}.G = Hr; % gs ma generatori accorciati



    end

    % Interseco lo zonotopo con il box iniziale
    P = polytope(Z2{t_mpc}) & polytope(Z{2});

    if plots
        figure(100)
        hold on
        % Current
        plot(Z{t_mpc},[1:3],'r', 'DisplayName','no gs')
        % Rescaled
        plot(Z2{t_mpc},[1:3],'g', 'DisplayName','gs')
        plot(P,[1:3],'c', 'DisplayName', 'gs & Z_0')
        axis square
        legend('-DynamicLegend');

    end

    if learning == 0
        Z{t_mpc} = Z{t_mpc-1};
    else
        Z{t_mpc} = Z2{t_mpc}; % gs generatori accorciati
    end

    c1 = Z{t_mpc}.c;
    H1 = Z{t_mpc}.G;

    CC(:,t_mpc) = c1;

    U_vol(t_mpc) = volume(P);
U_vol2(t_mpc) = volume(Z2{t_mpc});

    % Scenario MPC
    Y_ini=[Y_ini(2:end), Y_sim(t_mpc)];     % nuovo valore iniziale dell'uscita
    U_ini=[U_ini(2:end), U_opt(t_mpc-1)];   % nuovo valore iniziale dell'ingresso
    % Z = zonotope(P{t_mpc});
    % figure(100)
    % plot(Z,[1:3],'c')
    % [Theta1,Theta0]= scenary(Z,N_scn); % genera nuovi parametri nominali e scenari

    Theta0 = Z{t_mpc}.c;
    Theta1 = P.randPoint(N_scn);
    Theta0 = Theta0';
    Theta1 = Theta1';

    %optimization problem inputs
    inputs = {Theta1(:,:)',Theta0(:,:)',Y_ini,U_ini,lomax1,[Psp(t_mpc:t_mpc+T_hzn-1)']};

    %optimization problem solution
    [Output_Values,error]=Struct_MPC(inputs);

    if(error==0)
        cont=cont+1;%0 errors
    else
        AA=AA+1;
        warning('MPC unfeasible')
    end

    U_opt(t_mpc) = Output_Values{1}(2);
    %
    % figure(200)
    % subplot(3,1,1)
    % cla
    % plot(Output_Values{1})
    % subplot(3,1,2)
    % cla
    % plot(Output_Values{2}')
    % subplot(3,1,3)
    % cla
    % plot(Output_Values{3}(:))

    %simulation of BESS dynamic model
    Y_real(t_mpc+1)=thetaTrue(1)*Y_real(t_mpc)+thetaTrue(2)*Y_real(t_mpc-1)+thetaTrue(3)*U_opt(t_mpc-1);
end

%% Error computation
err = Y_sim - Psp(1:N);
errRmse = sqrt(sum(err.^2) / length(err));

%  Results plots
figure
plot(Y_sim)
hold on
plot(Psp)
grid on;
legend('Simulated output', 'Power reference');
xlabel('Time step');

figure;
hold on;
plot(U_opt);
plot(0*U_opt + Umin, 'k--');
plot(0*U_opt + Umax, 'k--');
grid on;
title('Optimal control');
xlabel('Time step');

figure
plot(U_vol(2:end))
grid on;
title('Uncertainty region volume');
xlabel('Time step');

% Added error figure
figure;
hold on;
plot(err, 'LineWidth', 1.5);
plot([1, N], [errRmse, errRmse], 'k--');
hold off;
grid on;
legend('Error', 'RMSE');
ylabel('Error');
xlabel('Time sample');

figure
hold on
grid on
plot3(thetaTrue(1),thetaTrue(2),thetaTrue(3),'g.')
plot3(c0(1),c0(2),c0(3),'r.')
plot3(CC(1,3:end),CC(2,3:end),CC(3,3:end),'b')
legend('\theta_t', '\theta_n(0)', '\theta_n(k)');
