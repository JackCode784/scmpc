%% stableScenary
% Generate stable scenarios

function [thetaScen, thetaNom] = stableScenary(Z, nScen, na)

thetaNom = Z.c;
[n, ngen] = size(Z.G);
thetaScen = zeros(n, nScen);
maxAttempts = 30;

for i=1:nScen
    for attempt=1:maxAttempts
        thetaScen(:,i) = Z.c + Z.G * (2 * rand(ngen,1) - 1);
        rabs = abs(roots([1; -thetaScen(1:na, i)]));
        if rabs < 1
            break;
        elseif attempt == maxAttempts
            % Go back to center
            thetaScen(:,i) = Z.c;
        end 
    end
end

end
