%% Test zonotopes normalizations
% Script for testing different ideas for normalizing zonotopes.
% 
close all; clc; clear;

ng = 4;
amp = 2;
zorig = zonotope(20*amp*(0.5-rand(2,1)),amp*rand(2,ng));
phi = 10*amp*(0.5 - rand(1,2));

Znew = boundStripZonotopeInt(zorig, 10*amp*(0.5-rand), phi(1), phi(2), 0.5 - rand, 1, 1, 'new');

ih = IntervalHull(zorig.c,zorig.G);
% zorig = ih;
dih = diag(ih.G);
% phinorm = (phi - zorig.c)./dih;
% znorm1 = zonotope(zeros(2,1),zorig.G / max(dih));
% znorm2 = zonotope(zeros(2,1),zorig.G ./ dih);
znorm3 = zonotope(zeros(2,1), zorig.G ./ dih);
% znorm4 = zonotope(zorig.c ./ (dih + abs(zorig.c)), zorig.G ./ (dih + abs(zorig.c)));

znewnorm = zonotope((Znew.c - zorig.c) ./ dih, Znew.G ./ dih);

figure;
hold on;
grid on;
axis equal;
plot([-1 1 1 -1 -1], [-1 -1 1 1 -1], '--', 'LineWidth',1.5);
plot(znorm3.c(1) + [-1 1 1 -1 -1], znorm3.c(2) + [-1 -1 1 1 -1], '--', 'LineWidth',1.5);
plot(zorig, 1:2, 'LineWidth',1.5);
plot(Znew, 1:2, 'LineWidth',1.5);
plot(znewnorm, 1:2, 'LineWidth',1.5);
plot(znorm3,1:2,'LineWidth',1.5);
% plot(znorm4,1:2,'LineWidth',1.5);

% Phi vector plot
% plot([zorig.c(1),phi(1)],[zorig.c(2),phi(2)],'LineWidth',1.5);
% plot([0,phinorm(1)],[0,phinorm(2)],'LineWidth',1.5);

legend('unit cube','unit cube in c', 'orig', 'new', 'new norm','d_i');
