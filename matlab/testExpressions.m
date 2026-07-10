%% testEquivalence
% Test computation equivalence in old algorithm
% 

% clc;
clear;
load("data_model_BESS.mat", "Jmin", "Z", "Z_");
Z0 = Z_{end};
clear("Z_", "Z_");

%%
na = 3; nb = 3; nk = 1;
n = na + nb;
nTest = 20;
u = rand(nTest, 1) * 400 - 200;
yPast = zeros(1, na);
uSamples = zeros(1, nk+nb-1);
sigma = 1.1 * Jmin;

Z1 = cell(nTest+1, 1); % my func
Z2 = cell(nTest+1, 1); % long method
centerErrs = true(1, nTest);
genErrs = true(1, nTest);
Z1{1} = Z0;
Z2{1} = Z0;

for i=1:nTest
    Phi = [yPast, uSamples(nk:nk+nb-1)];
    yCurr = Phi * Z0.c;
    Z1{i+1} = boundStripZonotopeInt(Z1{i}, yCurr, yPast, uSamples, sigma, nb, nk);

    Fy = yPast; % y(k-1), ..., y(k-na)
    Fu = uSamples(nk:nk+nb-1);
    Ft = [Fy, Fu];
    C = [Ft; -Ft];
    d = [yCurr + sigma; -yCurr + sigma];
    strip = polytope(C,d);
    if contains(strip, Z2{i})
        Z2{i+1} = Z2{i};
    else
        H1 = Z2{i}.G;
        c1 = Z2{i}.c;
        lambdaStar = H1*(H1')*(Ft')/(Ft*H1*(H1')*(Ft')+sigma^2);
        centerStar = c1+lambdaStar*(yCurr-Ft*c1);
        HStar = [(eye(n)-lambdaStar*Ft)*H1, sigma*lambdaStar];
        Z2{i+1} = zonotope(centerStar, HStar);
    end

    centerErrs(i) = any(Z1{i+1}.c ~= Z2{i+1}.c);
    genErrs(i) = any(Z1{i+1}.G ~= Z2{i+1}.G, "all");

    yPast = [yCurr, yPast(1:end-1)];
    uSamples = [u(i), uSamples(1:end-1)];
end

if any(centerErrs) || any(genErrs)
    disp(['First wrong iteration ', num2str(find(centerErrs, 1))]);
    disp(['First wrong iteration ', num2str(find(genErrs, 1))]);
else
    disp("All good!");
end
