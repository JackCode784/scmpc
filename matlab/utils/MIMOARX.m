function [theta_center,theta_max,theta_min,Epsilon1] = MIMOARX(y,u,na,nb,nk,delta)

% Set membership estimate of an ARX model with parameters
% na: length of the output regressor
% nb: length of the input regressor
% nk: model delay
% model structure:y(k) = a_1 y(k-1) + a_2 y(k-2) + ... + a_na y(k-na) +
% ...                    b_1 u(k-nk) + b_2 u(k-nk-1) + ... + b_nb u(k-nk-nb+1)
% 
% => I campioni più recenti (i primi nk) dell'ingresso NON influenzano 
%    l'uscita

% Il modello di sopra è errato: il numero di parametri (dimensione del 
% problema) sono na+nb, così come l (che rappresenta il campione successivo
% a quello più indietro nel tempo sia per l'ingresso che per l'uscita) 
% dovrebbe essere definito, usando il modello di sopra, come 
% l = max(na, nk+nb-1)+1.
% 
% Il modello che è coerente col codice scritto quindi non è quello di sopra
% ma il seguente:
% 
% y(k) = a_1*y(k-1) + ... + a_na*y(k-na) + 
%        b_1*u(k-nk) + ... + b_(nb+1)*u(k-nk-nb)
% 

N=length(y)
l=max(na,nb+nk)+1    % max delay
m= na+nb+1          % problem dimension

%%%%%%% Information operator F %%%%%%%%%%%%%%
Fy1=toeplitz(y(l-1:end-1,1), flipud(y(l-na:l-1,1)));
Fu1=toeplitz(u(l-nk:end-nk,1), flipud(u(l-nk-nb:l-nk,1)));
F1 = [Fy1, Fu1];
Y1= y(l:end,1);

% feasability 
    cvx_begin
            variables theta1(m) Epsilon1;
            minimize( Epsilon1 ) 
            subject to
            norm(F1*theta1-Y1,inf) <= Epsilon1  
    cvx_end
 

 Jmin1=Epsilon1*delta;
 Epsilon1=Epsilon1*delta;
 
    % noise bound increment for robustness

%% bounding box and central parameters 


theta_center1=zeros(m,1);
theta_max1=zeros(m,1);
theta_min1=zeros(m,1);

for it=1:m
    cvx_begin
        variable theta1(m);
        minimize( theta1(it) ) 
        subject to
        norm(F1*theta1-Y1,inf) <= Epsilon1
    cvx_end

    cvx_begin
        variable thetam1(m);
        maximize( thetam1(it) ) 
        subject to
        norm(F1*thetam1-Y1,inf) <= Epsilon1
    cvx_end

    theta_center1(it)=(theta1(it)+thetam1(it))/2;
    theta_max1(it)=thetam1(it);
    theta_min1(it)=theta1(it);
end

theta_center=theta_center1;
theta_max=theta_max1;
theta_min=theta_min1;



