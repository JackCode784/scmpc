%% myVolume.m
% Computes the volume of a given zonotope. For such task, only the matrix
% of generators is given to the function.
%
% Inputs:
% - G: generators matrix
%
% Outputs:
% - vol: volume of the given zonotope's generators matrix
%

function vol = volumeZonotope(G)

if isempty(G)
    vol = NaN;
    return;
end

[n, p] = size(G);

if p < n
    val = 0;
    return;
end

submatricesIdx = nchoosek(1:p, n);
submatricesIdx = submatricesIdx';

G = G(:, submatricesIdx);

determinants = zeros(1,size(submatricesIdx, 2));

for i=1:size(submatricesIdx, 2)
    determinants(i) = det(G(:, n*(i-1)+1:n*i));
end
vol = 2^n * sum(determinants);

end