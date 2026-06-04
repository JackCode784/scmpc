%% Main_LMPC_BESS.m
% This file runs a Scenario MPC loop, given an ARX zonotopic uncertainty model of a BESS
% It requires the CORA toolbox, YAlmip and Quadprog.
% Nominal model and Zonotopic uncertainty are contained in file data_model_BESS.mat

clc;
close all;
clear;
load('data_model_BESS.mat', 'Jmin', 'Z', 'Z_', 'na', 'nb', 'nk', 'l', 'Sys_m');

rng("default");

% Alcune variabili:
% - l = max(na, nb+nk)+1: istante temporale più "vecchio" che compare
% nell'espressione dell'uscita.
% - na: numero di coefficienti relativi ai precedenti valori dell'uscita
% nel modello ARX
% - nb: numero di coefficienti relativi ai precedenti valori dell'ingresso
% nel modello ARX
% - nk: ritardo del modello, i primi nk campioni dell'ingresso non
% influenzano l'uscita

Z0=Z_{end};     % prendo uno zonotopo (ridotto) inizale?
n=length(Z0.c); % dimensione dello zonotopo (e spazio dei parametri)

%% MPC formulation - Yalmip

yalmip('clear')
N_decision=1;                   % decision variables
Epsilon=10/100;                 % state constraint violation rate at 10%
N_scn=(N_decision/Epsilon)-1;   % number of required scenarios

T_hzn=nk+15;    % prediction horizon -> funzione costo
Scn_cost=0;     % tracking cost of all scenarios if Scn_cost=1
scenario=1;     % Robust constraint active if scenario=1, else nominal MPC

% Cost parameters (matrici di peso dello stato e controllo
% rispettivamente?)
Q = 5;    % power tracking error cost
R = 0.1;  % input effort cost

% Input Limits
Umax=200;   % maximal value for Power command input
Umin=-200;  % minimal value for Power command input

% Decision Variables
U = sdpvar(1,T_hzn+l-2) ;   % Power request input; crea vettore riga 16+4-2 elementi
Dy= sdpvar(1,1) ;           % Bound on output power variation

Y   = sdpvar(N_scn,T_hzn+l-1); % Output power evolution for N scenarios (uscita vista anche per un passo in più)
Y0  = sdpvar(1,T_hzn+l-1);     % Output power evolution for nominal model
Ysp = sdpvar(1,T_hzn+l-1);     % Power set point
Ymax= sdpvar(1,1);             % bound on output power |Y| <= Ymax
Yini = sdpvar(1,na);           % initial condition for ARX model
Uini = sdpvar(1,nb+nk-1);      % initial condition for ARX model
theta1 = sdpvar(n,N_scn);      % ARX model parameters for random scenarios; commento scambiato con quello sotto
theta0 = sdpvar(n,1);          % ARX nominal model parameter; commento scambiato con quello sopra

%% Constraints definition
Constraints=[];

% Power ramp constraint
% l è una specie di istante di tempo per iniziare il regime?
Constraints=[-Dy <= diff([Y0(l-1:end)]) <=Dy];

% Initial condition constraint  for nominal model
% Primi l valori di Y nel modello nominale sono fissati
Constraints = [Constraints ; Y0(1:l)==[zeros(1,l-na),Yini]]; % Yini= y(k-na+1):y(k) (na values)

% Primi l-1 valori dell'ingresso sono fissati
Constraints = [Constraints ; U(1:l-1)==[zeros(1,l-nb-nk),Uini]]; %Uini=u(k-nk-nb+1):u(k-1) (nb+nk-1 values)

% Per ogni scenario impongo i primi l valori delle uscite
for z=1:N_scn  % Initial condition constraint for N_scn scenarios
    Constraints = [Constraints ; Y(z,1:l)==[zeros(1,l-na),Yini]]; %equality constraint
end

% Vincoli per ogni istante di tempo a partire da l (regime?)
for t=l:T_hzn-1
    % Power request limits
    Constraints = [Constraints ;U(t)<=Umax];
    Constraints = [Constraints ;U(t)>=Umin];

    % Power output limits (sistema nominale)
    Constraints = [Constraints ; -Ymax<=Y0(t)<=Ymax];

    % Dynamic contraints
    % y(k+1) = a_1 y(k) + a_2 y(k-1) + ... + a_na y(k-na+1) +
    % ...                    b_0 u(k-nk+1) + b_1 u(k-nk) + ... + b_nb u(k-nk-nb+1)
    %        = theta' * [Y; U]

    % Dynamic constraint  for nominal model
    % Impone che l'uscita segua il modello ARX per tutti gli istanti di
    % tempo {l+1, l+2, ..., T_hzn}
    Constraints = [Constraints ; Y0(t+1)==[Y0(t:-1:t-na+1),U(t+1-nk:-1:t+1-nk-nb)]*theta0]; %equality constraint

    % Se facciamo SC-MPC, tutti gli scenari devono rispettare la dinamica
    % (ognuno con i propri parametri, infatti c'è theta1)
    if scenario==1
        for z=1:N_scn  % Dynamic cosntraints for N_scn scenarios
            Constraints = [Constraints ; Y(z,t+1)==[Y(z,t:-1:t-na+1),U(t+1-nk:-1:t+1-nk-nb)]*theta1(:,z)]; %equality constraint
        end
    end
end

%% Objective Function for nominal model
cost = 0;
for t=l:T_hzn
    cost=cost + R*U(t)^2 + Q*(Y0(t+1)-Ysp(t+1))^2; % Tracking cost for nominal model (Zonotope center)

    % Se teniamo conto del costo per tutti gli scenari
    if Scn_cost==1
        for k=1:N_scn
            cost = cost + (Q/N_scn)*(Y(k,t+1)-Ysp(t+1))^2; % Tracking cost for each scenario
        end
    end
end

% Objective = cost+10*Dy^2;
Objective = cost;   % per l'ottimizzatore?

%% Build Struct for optimizer
parameters_in = {theta1,theta0,Yini,Uini,Ymax,Ysp,Dy};  % quantità al variare delle quali cambia la soluzione?
solutions_out = {U,Y,Y0};   % valori ottimi di queste quantità

ops = sdpsettings('solver','quadprog','verbose',1,'usex0',0);
% ops = sdpsettings('verbose',0,'usex0',0);
% ops = sdpsettings('verbose',0);

Struct_MPC = optimizer(Constraints,Objective,ops,parameters_in,solutions_out);

%% MPC and simulation setup
clc
N=500;              % simulation length
Dy=100;             % bound for power ramp constraint
lomax1 = 110;       % bound on output power
noise_on=2;         % Measurement noise amplitude - 2kW
Epsilon=Jmin*1.1;   % Noise bound for online update

% Variables
sys=Sys_m;
X=zeros(order(sys),N);
Y_sim=zeros(N,1);
U_opt=zeros(N,1);   % controllo ottimo
U_vol=zeros(N,1);   % volume dello spazio di incertezza

% Generate input signals to support system identification
% The idinput command generates an input signal with specified
% characteristics for your system.
Psp=idinput(N+T_hzn,'prbs',[0, 0.05], [-100,100]); % Power reference
% Psp = 5 + zeros(N+T_hzn, 1); % output looks bad (noise too big)

AA=0;   % conta se MPC infattibile?
cont=0; % conta se MPC fattibile?
Y_ini = zeros(1,na);        % output initial condition
U_ini = zeros(1,nb+nk-1);   % input initial condition; sbagliato il numero di campioni?
Z{1}=Z0;                    % initial uncertainty model
Z_{1}=Z0;                   % initial uncertainty model
U_vol(1)=volume(Z_{1});

%% Control loop
% Added
c1 = Z0.c;
H1 = Z0.G;
% Theta0 = Z0.c';

for t_mpc=2:N
    % output measurement
    Y_sim(t_mpc)=sys.C*X(:,t_mpc) + noise_on*2*(0.5-rand(1)); % rumore in [-2, 2]
    % Y_sim(t_mpc) = [Y_ini(end:-1:1), U_opt(t_mpc-1) U_ini(end:-1:1)] * Theta0' + noise_on*2*(0.5-rand(1));

    % Model learning
    if t_mpc <= l % minore stretto? No, va bene così perché parte da t_mpc = 2
        Z{t_mpc}=Z0;
        Z_{t_mpc}=Z0;
    else
        %%%%%%% Information operator F %%%%%%%%%%%%%%
        Fy = flipud(Y_sim(t_mpc-na:t_mpc-1));       % precedenti na campioni ordinati da più recente a meno
        Fu = flipud(U_opt(t_mpc-nk-nb:t_mpc-nk));   
        Ft = [Fy', Fu'];    % messi in riga
        Yt = Y_sim(t_mpc);  % campione appena calcolato dell'uscita

        %%% data strip %%%
        C=[Ft;-Ft];
        d=[Yt+Epsilon;-Yt+Epsilon];
        S1 = polytope(C,d);
        S2=Z_{t_mpc-1};
        S3=Z{t_mpc-1};
        %%% zonotope update %%%%
        if contains(S1,S3)==1
            Z{t_mpc} = S3;
            Z_{t_mpc}= reduce(Z{t_mpc},'pca',3);
        elseif true%isIntersecting(S1, S3) % myIntersect here!!!
            lamda1=H1*H1'*Ft'/(Ft*H1*H1'*Ft'+Epsilon^2);
            cc1=c1+lamda1*(Yt-Ft*c1);
            HH1=[(eye(n)-lamda1*Ft)*H1,Epsilon*lamda1];
            Z{t_mpc} = zonotope(cc1, HH1);
            Z_{t_mpc}=reduce(Z{t_mpc},'pca',3);
            c1 = Z{t_mpc}.c;
            H1 = Z{t_mpc}.G;
        else
            warning('Empty FPS at iteration');
            t_mpc
        end

    end
    U_vol(t_mpc,1)=volume(Z_{t_mpc});
    % U_vol(t_mpc,2)=volume(Z{t_mpc});

    % Scenario MPC
    Y_ini=[Y_ini(2:end), Y_sim(t_mpc)];     % nuovo valore iniziale dell'uscita
    U_ini=[U_ini(2:end), U_opt(t_mpc-1)];   % nuovo valore iniziale dell'ingresso
    [Theta1,Theta0]= scenary(Z_{t_mpc},N_scn); % genera nuovi parametri nominali e scenari

    %optimization problem inputs
    inputs = {Theta1(:,:),Theta0(:,:),Y_ini,U_ini,lomax1,[ zeros(1,l-1), Psp(t_mpc:t_mpc+T_hzn-1)'],Dy};

    %optimization problem solution
    [Output_Values,error]=Struct_MPC(inputs);

    if(error==0)
        cont=cont+1;%0 errors
    else
        AA=AA+1;
        warning('MPC unfeasible')
    end

    U_opt(t_mpc) = Output_Values{1}(l);

    %simulation of BESS dynamic model
    X(:,t_mpc+1)=sys.A*X(:,t_mpc)+sys.B*(U_opt(t_mpc)+2*randn(1));
end

%% Error computation
err = Y_sim - Psp(1:N);
errRmse = sqrt(sum(err.^2) / length(err));

%%  Results plots
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
plot(U_vol(:,1) / volume(Z0));
grid on;
title('Uncertainty region volume');
xlabel('Time step');
ylabel('Volume of reduced zonotopes / Z0');

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

figure;
hold on;
plot(Z0, 1:3, 'LineWidth',1.5);
% plot(Z{end}, 1:3, 'LineWidth',1.5);
plot(Z_{end}, 1:3, 'LineWidth',1.5);
hold off;
grid on;
legend('\Theta_0', '\Theta_R(500)');
