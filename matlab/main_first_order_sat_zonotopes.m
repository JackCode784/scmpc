%% Main_LMPC_BESS.m
% This file runs a Scenario MPC loop, given an ARX zonotopic uncertainty model of a BESS
% It requires the CORA toolbox, YAlmip and Quadprog.
% Nominal model and Zonotopic uncertainty are contained in file data_model_BESS.mat

clc;
close all;
clear;

learning = 1;

% Alcune variabili:
% - l = max(na, nb+nk)+1: istante temporale più "vecchio" che compare
% nell'espressione dell'uscita.
% - na: numero di coefficienti relativi ai precedenti valori dell'uscita
% nel modello ARX
% - nb: numero di coefficienti relativi ai precedenti valori dell'ingresso
% nel modello ARX
% - nk: ritardo del modello, i primi nk campioni dell'ingresso non
% influenzano l'uscita

na = 1;
nb = 1;
nk = 1;

c0 = [0
    1];

G0 = [1 0
    0   1];

Z0 = zonotope(c0,G0);     % prendo uno zonotopo (ridotto) inizale?
n=length(Z0.c); % dimensione dello zonotopo (e spazio dei parametri)
figure(100)
hold on
plot(Z0)

%% MPC formulation - Yalmip

yalmip('clear')
N_decision=1;                   % decision variables
Epsilon=10/100;                 % state constraint violation rate at 10%
N_scn=(N_decision/Epsilon)-1;   % number of required scenarios
% N_scn = 1;

T_hzn=nk+15;    % prediction horizon -> funzione costo
Scn_cost=0;     % tracking cost of all scenarios if Scn_cost=1
scenario=1;     % Robust constraint active if scenario=1, else nominal MPC

% Cost parameters (matrici di peso dello stato e controllo
% rispettivamente?)
Q = 500;    % power tracking error cost
R = 0;  % input effort cost

% Input Limits
Umax=10;   % maximal value for Power command input
Umin=-10;  % minimal value for Power command input

% Decision Variables
U = sdpvar(1,T_hzn-1) ;   % Power request input; crea vettore riga 16+4-2 elementi

Y   = sdpvar(N_scn,T_hzn); % Output power evolution for N scenarios (uscita vista anche per un passo in più)
Y0  = sdpvar(1,T_hzn);     % Output power evolution for nominal model
Ysp = sdpvar(1,T_hzn);     % Power set point
Ymax = sdpvar(1,1);             % bound on output power |Y| <= Ymax
Yini = sdpvar(1,na);           % initial condition for ARX model
Uini = sdpvar(1,nb+nk-1);      % initial condition for ARX model
theta1 = sdpvar(n,N_scn);      % ARX model parameters for random scenarios; commento scambiato con quello sotto
theta0 = sdpvar(n,1);          % ARX nominal model parameter; commento scambiato con quello sopra

%% Constraints definition
Constraints=[];

% Initial condition constraint  for nominal model
% Primi l valori di Y nel modello nominale sono fissati
Constraints = [Constraints ; Y0(1:1)==[Yini]]; % Yini= y(k-na+1):y(k) (na values)

% Primi l-1 valori dell'ingresso sono fissati
% Constraints = [Constraints ; U(1:1)==[Uini]]; %Uini=u(k-nk-nb+1):u(k-1) (nb+nk-1 values)

% Per ogni scenario impongo i primi l valori delle uscite
for z=1:N_scn  % Initial condition constraint for N_scn scenarios
    Constraints = [Constraints ; Y(z,1:1)==[Yini]]; %equality constraint
end

% Vincoli per ogni istante di tempo a partire da l (regime?)
for t=1:T_hzn-1
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
    Constraints = [Constraints ; Y0(t+1)==[Y0(t:-1:t-na+1),U(t:-1:t-nb+1)]*theta0]; %equality constraint

    % Se facciamo SC-MPC, tutti gli scenari devono rispettare la dinamica
    % (ognuno con i propri parametri, infatti c'è theta1)
        for z=1:N_scn  % Dynamic cosntraints for N_scn scenarios
            Constraints = [Constraints ; Y(z,t+1)==[Y(z,t:-1:t-na+1),U(t:-1:t-nb+1)]*theta1(:,z)]; %equality constraint
        end
end

%% Objective Function for nominal model
cost = 0;
for t=1:T_hzn-1
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
parameters_in = {theta1,theta0,Yini,Uini,Ymax,Ysp};  % quantità al variare delle quali cambia la soluzione?
solutions_out = {U,Y,Y0};   % valori ottimi di queste quantità

ops = sdpsettings('solver','quadprog','verbose',1,'usex0',0);
% ops = sdpsettings('verbose',0,'usex0',0);
% ops = sdpsettings('verbose',0);

Struct_MPC = optimizer(Constraints,Objective,ops,parameters_in,solutions_out);

%% MPC and simulation setup
clc
N = 200;              % simulation length
lomax1 = 6;       % bound on output power
noise_on=0.01;         % Measurement noise amplitude - 2kW

% SISTEMA VERO
a = 0.6;
tau = 1;
b = 1.4;

% Variables
Y_real=zeros(N,1);
Y_sim=zeros(N,1);
U_opt=zeros(N,1);   % controllo ottimo
U_vol=zeros(N,1);   % volume dello spazio di incertezza

% Generate input signals to support system identification
% The idinput command generates an input signal with specified
% characteristics for your system.
% Psp=idinput(N+T_hzn,'prbs',[0, 0.05], [-100,100]); % Power reference
ii = 1:N+T_hzn;
Psp = sin(0.1*ii);
Psp = Psp(:);
% Psp = 5 + zeros(N+T_hzn, 1); % output looks bad (noise too big)

AA=0;   % conta se MPC infattibile?
cont=0; % conta se MPC fattibile?
Y_ini = zeros(1,na);        % output initial condition
U_ini = zeros(1,nb+nk-1);   % input initial condition; sbagliato il numero di campioni?
Z{1} = Z0;                    % initial uncertainty model               
P{1} = Z0;
U_vol(1)=volume(Z{1});

%% Control loop
% Added
c1 = Z0.c;
H1 = Z0.G;
% Theta0 = Z0.c';

c1 = Z{1}.c;
H1 = Z{1}.G;

sz0 = sqrt(Z{1}.G(1,1)^2+Z{1}.G(2,2)^2);

for t_mpc=2:N

    t_mpc
    Epsilon = 0.99*Epsilon;
    % output measurement
    Y_sim(t_mpc) = Y_real(t_mpc) + noise_on*2*(0.5-rand(1)); % rumore in [-2, 2]

    %%%%%%% Information operator F %%%%%%%%%%%%%%
    Fy = flipud(Y_sim(t_mpc-na:t_mpc-1));       % precedenti na campioni ordinati da più recente a meno
    Fu = flipud(U_opt(t_mpc-nb:t_mpc-1));
    Ft = [Fy', Fu'];    % messi in riga
    Yt = Y_sim(t_mpc);  % campione appena calcolato dell'uscita

    %%% data strip %%%
    C=[Ft;-Ft];
    d=[Yt+Epsilon;-Yt+Epsilon];
    S1 = polytope(C,d);

    figure(100)
    cla
    plot(1-tau*a,tau*b,'.k','MarkerSize',10)
    hold on
    if C(1,2) ~= 0
        t1 = [-0.5 0.5];
        t2 = (d(1)-C(1,1)*t1)/C(1,2);        
        plot(t1,t2,'k')
    end
    if C(2,2) ~= 0
        t1 = [-0.5 0.5];
        t2 = (d(2)-C(2,1)*t1)/C(2,2);        
        plot(t1,t2,'k')
    end
          
    %%% zonotope update %%%%
        lamda1=H1*H1'*Ft'/(Ft*H1*H1'*Ft'+Epsilon^2);
        cc1=c1+lamda1*(Yt-Ft*c1);
        HH1=[(eye(n)-lamda1*Ft)*H1,Epsilon*lamda1];
        
        Z{t_mpc} = zonotope(cc1, HH1);
        Z{t_mpc} = reduce(Z{t_mpc},'pca',1); 

        Hr = Z{t_mpc}.G;

        % for j = 1:size(Hr,1)
        %     if sum(Hr(:,j) ~= 0)
        %         max_nrm = max(norm(Hr(:,j)));
        %         Hr(:,j) = 
        % 
        %         Hr(:,j) = Hr(:,j)./norm(Hr(:,j));
        %         % HH1(1,j) = HH1(1,j)*2;
        %         % HH1(2,j) = HH1(2,j)*2;
        %     end
        % end

        sz = norm(Z{t_mpc}.G(:,1)+Z{t_mpc}.G(:,2));
        sz = min(sz,sz0);
        
        max_norm = max(vecnorm(Hr));
        Hr = Hr./max_norm*sz;
        Z2{t_mpc} = Z{t_mpc};
        Z2{t_mpc}.G = Hr;
       


        figure(200)
        hold on
        plot(Z{1},[1 2],'k')
        plot(Z{t_mpc},[1 2],'r')
        plot(Z2{t_mpc},[1 2],'g')
        
        Z{t_mpc} = Z2{t_mpc};

        c1 = Z{t_mpc}.c;
        H1 = Z{t_mpc}.G;

        % Z{t_mpc} = zonotope(S1 & Z{t_mpc-1});
        
        % P1 = polytope(Z{t_mpc});
        % P2 = polytope(Z{1});
        % P{t_mpc} = P1&P2; 

        P{t_mpc} = Z{t_mpc};

        % P{t_mpc} = polytope(Z{t_mpc});


        % P{t_mpc} = zonotope(P{t_mpc});
        % P{t_mpc} = polytope(P{t_mpc});
        % P{t_mpc} = box(P{t_mpc});




    if learning == 0
        P{t_mpc} = P{t_mpc-1};
    end  

    figure(100)
    hold on
    plot(Z{1},[1 2],'m')
    plot(Z{t_mpc-1},[1 2],'b')
    plot(Z{t_mpc},[1 2],'c')
    plot(P{t_mpc},[1 2],'g')

    U_vol(t_mpc)=volume(P{t_mpc});
    if isa(P{t_mpc},'polytope')
        N_edges(t_mpc) = numel(P{t_mpc}.b);
    else
        N_edges(t_mpc) = 4;
    end

    % Scenario MPC
    Y_ini=[Y_ini(2:end), Y_sim(t_mpc)];     % nuovo valore iniziale dell'uscita
    U_ini=[U_ini(2:end), U_opt(t_mpc-1)];   % nuovo valore iniziale dell'ingresso

    % ZZ = zonotope(P{t_mpc});
    % [Theta1,Theta0]= scenary(ZZ,N_scn); % genera nuovi parametri nominali e scenari
    
    Theta0 = Z{t_mpc}.c;    
    Theta1 = P{t_mpc}.randPoint(N_scn);
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

    U_opt(t_mpc) = Output_Values{1}(1);

    %simulation of BESS dynamic model
    Y_real(t_mpc+1)=(1-tau*a)*Y_real(t_mpc)+tau*b*U_opt(t_mpc);
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
plot(U_vol)
grid on;
title('Uncertainty region volume');
xlabel('Time step');

figure
plot(N_edges)
grid on;
title('Number of edges');
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

[1-tau*a;tau*b]
