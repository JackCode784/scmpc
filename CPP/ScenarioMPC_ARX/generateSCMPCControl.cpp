#include "setup.h"

// Calls MADS alg to compute optimal input sequence for the ARX system
void generateSCMPCControl(input_type uOpt[NhorU], theta_type thetaScenarios[Nscen+1][nTheta], output_type yInit[na], input_type uInit[nb+nd-1], output_type yref[ny], hzn_type predictionHzn, hzn_type controlHzn)
{
    // Generate scenarios for constraints violation
    generateScenarios(thetaScenarios);

    // Algorithm used for computing the optimal input during the control horizon is MADS
    MADSARX(uOpt, uInit, yInit, yref, thetaScenarios, predictionHzn, controlHzn);
}
