#include "setup.h"
#ifdef DEBUG_MODE
#include <cstdlib> // rand()
#include <ctime>   // time()
#endif

// Generalized way of creating new scenarios
// Returns one theta row vector for each scenario. First row is nominal value for theta
void generateScenarios(theta_type thetaScenarios[Nscen + 1][nTheta])
{
    theta_type thetaRange[nTheta];

    for (int i = 0; i < nTheta; i++)
        thetaScenarios[Nscen][i] = thetaNominal[i];

    #ifdef DEBUG_MODE
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
    #else 
    // WIP
    #endif
}
