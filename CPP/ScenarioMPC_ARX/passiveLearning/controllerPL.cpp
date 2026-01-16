#include "../setup.h"

void controllerPL(digital_input_type uOptDig[NhorU], const digital_output_type yCurrDig, digital_output_type yrefDig)
{
    input_type uOpt[NhorU];
    input_type uPast[nb + nd - 2];
    output_type yref;
    output_type yCurr;
    theta_type thetaScenarios[Nscen+1][nTheta];

    /*
     * Optimal control generation
     */
    // Scenario generation
    generateScenarios(thetaScenarios);
    
    /*
     *   DA conversions
     */
    for (int i = 0; i < NhorU; i++)
        uOpt[i] = DAConvertU(uOptDig[i]);

    yref = DAConvertY(yrefDig);
    yCurr = DAConvertY(yCurrDig);


    // Run MADS optimization algorithm
    MADSARX(uOpt, uPast, yInit, yref, thetaScenarios);

    // Convert uOpt back to digital
    for (int i = 0; i < NhorU; i++)
        uOptDig[i] = ADConvertU(uOpt[i]);

    /*
     *   Uncertainty zonotope update and reduction
     */
    input_type uSamples[nb + nd];
    theta_type newCenter[nTheta];
    theta_type newGens[nTheta][nTheta+1];

    uSamples[0] = uOpt[0];
    for(int i = 1; i < nb + nd; i++)
        uSamples[i] = uPast[i-1];

    // Generate new zonotope
    boundStripZonotopeIntersection(yCurr, yInit, uSamples, thetaNominal, generators, newCenter, newGens);

    // Reduce order of new zonotope
    intervalHull(thetaNominal, generators, newCenter, newGens);

    return;
}
