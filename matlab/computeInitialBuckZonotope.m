clc;clear;close all;

% Nominal values
r = 0;      % [Ohm]
R = 6;      % [Ohm]
C = 15e-6;  % [F]
L = 150e-6; % [H]
vIn = 10;   % [V]
Ts = 1e-5;  % [s]

[RR,LL,CC, rr] = ndgrid(linspace(0.8*R,1.2*R,30),linspace(0.8*L,1.2*L,30),linspace(0.8*C,1.2*C,30), linspace(0,2,30));

theta1 = 2 - Ts./(RR.*CC); % non cambia lungo l'asse 2
theta2 = Ts./CC .* (1./RR - Ts./LL) - 1; % cambia per ogni asse
theta3 = Ts^2./(LL.*CC) * vIn; % non cambia per l'asse 1

theta1l = theta1 - Ts .* rr ./ LL;
theta2l = theta2 + Ts .* rr ./ LL - Ts^2 .* rr ./ (RR .* LL .* CC);
theta3l = theta3;

theta1c = 2 - Ts./(R.*C);
theta2c = Ts./C .* (1./R - Ts./L) - 1;
theta3c = Ts^2./(L.*C) * vIn;

M = [theta1l(:) theta2l(:) theta3l(:)];
c = mean(M);
Gred = M-c;
Gred = Gred';

% Obtain matrix of points from generator matrix
V = [Gred,-Gred];               % has zero mean
C=cov(V');                      % compute the covariance matrix
[U,~,~] = svd(C);               % singular value decomposition
Gtrans = U'*Gred;               % map generators
Gbox = diag(max(abs(Gtrans'))); % box generators
Gred = U*Gbox;                  % transform generators back
Z0 = zonotope(c', Gred);        % build reduced zonotope

% Plots
figure
plot(Z0,1:3)
hold on
plot3(theta1l(:),theta2l(:),theta3l(:),'.')

figure;
hold on;
plot3(Gtrans(1,:), Gtrans(2,:),Gtrans(3,:),'.');
plot(zonotope(zeros(3,1), Gbox), 1:3, 'LineWidth',1.5);

save Z0buck.mat Z0;
