%% Initial zonotopic uncertainty set for ARX model
clc
close all
clear all
load("system_data.mat")

%% model parameters
N0=100;         % Length of initial dataset for bounding box estimation
Nend=500;       % Length of dataset for zonotopic update 
u=ui(1:N0,:);   % == Input(1:N0)
y=yi(1:N0,:);   % == y_sim(1:N0), uscita affetta da rumore addittivo di misura e sull'ingresso

%% SM-ARX model and bounding box
% Model structure?
% y(k) = a_1 y(k-1) + a_2 y(k-2) + ... +     a_na y(k-na) +
%        b_1 u(k-nk) + b_2 u(k-nk-1) + ... + b_nb u(k-nk-nb+1)
% ==> na+nb parameters
% or
% y(k) = a_1 y(k-1) + ... + a_na y(k-na) + 
%        b_1 u(k-nk) + ... + b_{nb+1} u(k-nk-nb)
% ==> na+nb+1 parameters
% 
 
clc
na=3;               % numero di coefficienti dei campioni passati dell'uscita
nb=2;               % numero di coefficienti dei valori dell'ingresso
nk=1;               % "Model delay"?
n=na+nb+1;          % dimensione dello zonotopo & numero totale di parametri
l=max(na,nb+nk)+1;  % max delay + 1 
delta=1.1;          % ?

%% Set membership model estimate
[theta_center,theta_max,theta_min,Jmin] = MIMOARX(y,u,na,nb,nk,delta);

% Jmin
% theta_max'
% theta_center'
% theta_min'

%% Zonotopic uncertainty refinement 

c1=theta_center(:,1);               % center of ARX parameters
thetamax1=theta_max(:,1);           % upper bound on ARX parameters
thetamin1=theta_min(:,1);           % Lower bound on ARX parameters
H1=diag((thetamax1-thetamin1)/2);   % zonotope=c+Hb; box bounding
Z{N0}=zonotope(c1,H1);              % zonotope generation
Z_{N0}=zonotope(c1,H1);             % reduced order zonotope generation


%%%%%%% Information operator F for updated dataset %%%%%%%%%%%%%%
% La prossima serie di operazioni crea delle matrici tali che:
% y_pred(l:end) = F1 * theta
% dove y_pred viene generata seguendo il modello ARX.
Fy1=toeplitz(yi(l-1:end-1,1), flipud(yi(l-na:l-1,1)));
Fu1=toeplitz(ui(l-nk:end-nk,1), flipud(ui(l-nk-nb:l-nk,1)));
Ft1 = [Fy1, Fu1];
Yt1= yi(l:end,1);   % uscite vere

for k=N0:Nend-l
    C=[Ft1(k,:);-Ft1(k,:)];
    d=[Yt1(k)+Jmin;-Yt1(k)+Jmin];
    % Politopo "striscia"
    S1 = polytope(C,d); % S1 = {theta | C*theta <= d <=> y(k)-Jmin <= y_pred(k) <= y(k)+Jmin }
    S2=Z_{k};   % tutti gli zonotopi ridotti
    S3=Z{k};    % tutti gli zonotopi non ridotti

    if contains(S1,S3)==1   % se S1 contiene S3
       Z{k+1} = S3;
       Z_{k+1}=reduce(Z{k+1},'pca',1);
    elseif isIntersecting(S1, S3)==1
        lamda1=H1*H1'*Ft1(k,:)'/(Ft1(k,:)*H1*H1'*Ft1(k,:)'+Jmin^2);
        cc1=c1+lamda1*(Yt1(k)-Ft1(k,:)*c1);
        HH1=[(eye(n)-lamda1*Ft1(k,:))*H1,Jmin*lamda1];
        Z{k+1} = zonotope(cc1, HH1);
        Z_{k+1}=reduce(Z{k+1},'pca',1);
        c1=cc1;
        H1=HH1;
    else 
        warning('Empty FPS at iteration');
        k
    end
        
end
     
Z_{end}

%%
close all
figure(1)
hold on
plot(Z{N0},[1 2 3])
% plot(Z{k+1},[1  2 3],'r')
plot(Z_{k+1},[1 2 3],'g')

figure(2)
hold on
plot(Z{N0},[4 5 6 ])
% plot(Z{k+1},[4 5 6],'r')
plot(Z_{k+1},[4 5 6],'g')


save('data_model_BESS.mat')


%%
k=Nend;
N_scn=25;
[theta1,theta0]= scenary(Z{end},N_scn); 
sys_c=filt(theta0(na+1:end),[1 -theta0(1:na)]);
sys_c.iodelay=nk;
y_z=lsim(sys_c,u);

figure
plot(y)
hold on
plot(y_z,'.-')

for i=1:N_scn
    sys_s=filt(theta1(i,na+1:end),[1 -theta1(i,1:na)]);
    sys_s.iodelay=nk;
    y_z=lsim(sys_s,u);
    plot(y_z,'.-')
end



