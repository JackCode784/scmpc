function [thetaScens, thetaNom] = sampleStableScenarios(Z, nScen, na, stabMargin, maxAttempts)
    ng        = size(Z.G, 2);
    thetaScens = zeros(size(Z.c, 1), nScen);
    thetaNom  = Z.c;           % nominal = center (always used)
    filled    = 0;
    attempts  = 0;

    while filled < nScen && attempts < maxAttempts
        attempts = attempts + 1;
        xi       = 2*rand(ng,1) - 1;          % xi uniform in [-1,1]^ng
        theta_s  = Z.c + Z.G * xi;

        % Stability check: characteristic poly z^na - theta_1*z^(na-1) - ... - theta_na
        charCoeffs = [1; -theta_s(1:na)];      % descending powers
        poles      = roots(charCoeffs);

        if all(abs(poles) < stabMargin)
            filled = filled + 1;
            thetaScens(:, filled) = theta_s;
        end
    end

    % Fill remaining slots with the center (stable by assumption)
    if filled < nScen
        for j = filled+1:nScen
            thetaScens(:, j) = Z.c;
        end
    end
end