%% [xNorm, m, q] = map(x, xmin, xmax, xnormmin, xnormmax, isinteger)
% Normalize vector xmin <= x <= xmax in range xnormmin <= xnorm <=
% xnormmax.
%       map : [xmin, xmax] -> [xnormmin, xnormmax]
% 

function [xnorm, m , q] = map(x, xmin, xmax, xnormmin, xnormmax, isinteger)
    % Compute gain and bias to apply for each coordinate
    m = (xnormmax - xnormmin) ./ (xmax - xmin);
    q = (xnormmin .* xmax - xnormmax .* xmin) ./ (xmax - xmin);

    % Apply gain and bias
    xnorm = m .* x + q;

    % Saturate in given interval
    % This does something only if there's some noise in the input
    xnorm = max(xnorm, xnormmin);
    xnorm = min(xnorm, xnormmax);

    % Round to integer if necessary
    if isinteger
        xnorm = round(xnorm);
    end
end
