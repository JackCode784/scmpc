%% scenary(Z, N_scn)
% Genera N_scn punti (scenari) all'interno dello zonotopo Z come 
% combinazione lineare a coefficienti randomici in [-1, 1]. Inoltre, 
% restituisce il centro dello zonotopo in quanto sistema nominale.
% 

function [thetaScen,thetaNominal]= scenary(Z,N_scn)

thetaScen = Z.c + Z.G * (2*(rand(size(Z.G, 2), N_scn)-0.5));
thetaNominal = Z.c;

end