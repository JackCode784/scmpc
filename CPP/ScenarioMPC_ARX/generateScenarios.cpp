#include "setup.h"

#ifndef CMPLSYS
// Generalized way of creating new scenarios
// Returns one theta row vector for each scenario. First row is nominal value for theta
void generateScenarios(theta_type thetaScenarios[Nscen][nTheta])
{
    theta_type thetaRange[nTheta];

    for (int i = 0; i < nTheta; i++)
    {
        thetaRange[i] = thetaMax[i] - thetaMin[i];
    }
    for(int i = 0; i < Nscen; i++)
    {
        pseudoRandArx(thetaScenarios[i], thetaRange);
    }
}
#else
// This is only used with passive learning
// generators points to the generator matrix, the columns of which are the generators
void generateScenarios(theta_type thetaScenarios[Nscen][nTheta])
{
    rand_type coeffs[nTheta];

    for(int i = 0; i < Nscen; i++)
    {
        pseudoRandArx(coeffs, nTheta);   // nGens numbers in [-1, 1]

        for(int j = 0; j < nTheta; j++)
        {
            thetaScenarios[i][j] = thetaNominal[j];
            for(int k = 0; k < nTheta; k++)
            {
                thetaScenarios[i][j] += generators[j][k] * coeffs[k];
            }
        }
    }
}
#endif