function val = computeMetric(G, metric)

if strcmp(metric, 'l2')
    val = vecnorm(G,2,1);
elseif strcmp(metric, 'l1-inf')
    val = vecnorm(G, 1, 1) - vecnorm(G, Inf, 1);
end

end