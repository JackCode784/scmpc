#include "setup.h"
#include <cstdlib> // rand()
#include <ctime>   // time()

// Generalized way of creating new scenarios
// Returns one theta row vector for each scenario. First row is nominal value for theta
void generateScenarios(theta_type thetaScenarios[Nscen + 1][nTheta])
{
    theta_type thetaRange[nTheta];

    // srand(time(NULL)); // different random number each time

    for (int i = 0; i < nTheta; i++)
        thetaScenarios[Nscen][i] = thetaNominal[i];

    // for (int i = 1; i < Nscen + 1; i++)
    // {
    //     for (int j = 0; j < nTheta; j++)
    //     {
    //         thetaRange[j] = thetaMax[j] - thetaMin[j];
    //         thetaScenarios[i][j] = static_cast<theta_type>(rand()) / (static_cast<theta_type>(RAND_MAX)); // random number in [0, 1]
    //         thetaScenarios[i][j] *= thetaRange[j];                                                        // random vector in [0, thetaMax-thetaMin]
    //         thetaScenarios[i][j] += thetaMin[j];                                                          // random vector in [thetaMin, thetaMax]
    //     }
    // }

    for (int i = 0; i < nTheta; i++)
    {
        thetaRange[i] = thetaMax[i] - thetaMin[i];
        for (int j = 0; j < Nscen ; j++)
        {
            thetaScenarios[j][i] = static_cast<theta_type>(rand()) / (static_cast<theta_type>(RAND_MAX)); // random number in [0, 1]
            thetaScenarios[j][i] *= thetaRange[i];                                                        // random vector in [0, thetaMax-thetaMin]
            thetaScenarios[j][i] += thetaMin[i];                                                          // random vector in [thetaMin, thetaMax]
        }
    }
}
