%% Znorm = normalizeZonotope(Z, c0, Dg)
% This normalizes the input zonotope applying an affine transformation:
%       Znorm = Dg^-1 * (Z - c0)
% 

function Znorm = normalizeZonotope(Z, c0, Dg)
    invDg = diag(diag(Dg) .^ -1);
    Znorm = zonotope(invDg * (Z.c - c0), invDg * Z.G);
end
