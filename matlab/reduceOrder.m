%% Reduce zonotope order
%
function newZon = reduceOrder(oldZon, method)

newZon = zonotope.empty;
center = oldZon.c;
G = oldZon.G;

switch method
    case 'none'
        newCenter = center;
        newGens = G;
    case 'ih'
        newZ = IntervalHull(center,G);
        newCenter = newZ.c;
        newGens = newZ.G;
    case 'pca'
        X = [G, -G];
        [U, ~, ~] = svd(X * X'); % svd of covariance
        newZ = IntervalHull(U'*center, U'*G);
        newCenter = center;
        newGens = U*newZ.G;
    case 'gs'
        n = size(G,1);
        [~, idx] = sort(vecnorm(G), 'descend');
        G = G(:,idx);

        % Gram-Schmidt
        U = zeros(n);
        U(:,1) = G(:,1);
        for i=2:n
            projs = dot(U(:,1:i-1), G(:,i) * ones(1,i-1)) ./ dot(U(:,1:i-1), U(:,1:i-1));
            projs = U(:,1:i-1) * projs';
            U(:,i) = G(:,i) - projs;
        end
        
        % (U*D)^-1 = D^-1 * U^-1 = (U*D)^T = D * U^T
        % U^-1 = D^2 * U^T
        % D = diag(1./vecnorm(U))
        Uinv = diag(vecnorm(U) .^ -2) * U';
        Gproj = Uinv * G;
        Gbox = diag(sum(abs(Gproj')));
        Gproj = U*Gbox;
        newCenter = center;
        newGens = Gproj;
    otherwise
        disp("Unrecognized reduction method.");
        newCenter = center;
        newGens = G;
end

newZon.c = newCenter;
newZon.G = newGens;

return;
end