#include "setup.h"

// Generalized way of creating new scenarios
// Returns one theta row vector for each scenario. First row is nominal value for theta
void generateScenarios(theta_type thetaScenarios[Nscen + 1][nTheta])
{
    theta_type thetaRange[nTheta];

    for (int i = 0; i < nTheta; i++)
    {
        thetaScenarios[Nscen][i] = thetaNominal[i];
        thetaRange[i] = thetaMax[i] - thetaMin[i];
    }
    for(int i = 0; i < Nscen; i++)
    {
        pseudoRandArx(thetaScenarios[i], thetaRange);
    }
}
