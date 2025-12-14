#include "setup.h"

// Calls MADS alg to compute optimal input sequence for the ARX system
void generateSCMPCControl(input_type uOpt[NhorU], theta_type thetaScenarios[Nscen+1][nTheta], output_type yInit[na], input_type uInit[nb+nd-1], output_type yref[ny], hzn_type predictionHzn, hzn_type controlHzn)
{
    // Internal copies of initial conditions
    output_type yInit_int[na];
    input_type uInit_int[nb+nd-1];
    
    for(int i = 0; i < na; i++)
        yInit_int[i] = yInit[i];

    for(int i = 0; i < nb + nd - 1; i++)
        uInit_int[i] = uInit[i];

    // Generate scenarios for constraints violation
    generateScenarios(thetaScenarios);

    // Algorithm used for computing the optimal input during the control horizon is MADS
    MADSARX(uOpt, uInit_int, yInit_int, yref, thetaScenarios, predictionHzn, controlHzn);
}
