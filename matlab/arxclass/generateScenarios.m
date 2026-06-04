function sysVec = generateScenarios(Nscen,nominalSys)

sysVec = cell(Nscen+1, 1);
sysVec{1} = nominalSys;
for k=2:Nscen+1
    eta = 0.5*rand(1);
    % Bk = 1e3*(1+eta);
    % thetaScen = [2; -1; 0; 0; A*Bk*Ts^2];
    % thetaScen = [2; -1; A*Bk*Ts^2];
    thetaScen = [2; -1; eta+1];
    sysVec{k} = arxSys(nominalSys.na, nominalSys.nb, nominalSys.nd, thetaScen, nominalSys.Ts);
end

end