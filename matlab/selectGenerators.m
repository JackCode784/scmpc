function [Gred, Gunred] = selectGenerators(G, order, metric)

Gred = [];
Gunred = [];

if isempty(G)
    return;
end

G = nonzeroFilter(G);

[n, p] = size(G);

% if number of desired generators is less than current one
if p > order * n    
    nNotToRed = n*(order - 1);
    nToRed = p - nNotToRed;

    [~, idxToRed] = mink(metric, nToRed);
    Gred = G(:, idxToRed);
    Gunred = G(:, setdiff(1:p, idxToRed));
else
    Gunred = G;
end

end