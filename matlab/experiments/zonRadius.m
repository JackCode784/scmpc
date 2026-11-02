%% zonRadius(Z)
% Computes zonotope radius for a order 1 zonotope with orthogonal
% generators.
% 

function r = zonRadius(Z)

r = sqrt(sum(Z.G .^ 2, 'all'));

end