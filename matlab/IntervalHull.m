%% Compute interval hull of given zonotope
% This function computes the interval hull of the given zonotope and
% returns it. It only reduces to n generators the least important ones from
% the original based on the metric vector. It then substitutes the
% to-be-reduced portion of the original matrix with a diagonal matrix which
% contains the span of the to-be-reduced generators.
% 
% Inputs:
% - center: center of the given zonotope
% - G: generators of the given zonotope (each column is a generator)
% - order: desired reduction order for the given zonotope
% - metric: vector of values representing a certain metric for each column
%           in G (i.e. for each generator)
% 
% Outputs:
% - newCenter: center of the computed zonotope (equal to old center)
% - newGens: generators matrix of the computed zonotope
% 

function newZ = IntervalHull(center, G)

% [Gred, Gunred] = selectGenerators(G, order, metric);

newCenter = center;
delta = sum(abs(G), 2);
newGens = diag(delta);

newZ = zonotope(newCenter, newGens);

end
