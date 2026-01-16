#include "setup.h"

// Take digital inputs, convert them to fixed point, find optimal input, return it as digital
// At instant k measure y(k), find uOpt = [u(k), u(k+1), ..., u(k+NhorU-1)]
void controller(digital_input_type uOptDig[NhorU], const digital_output_type yCurrDig, const digital_output_type yrefDig)
{
    // Convert digital inputs to fixed point/float
    input_type uOpt[NhorU];
    input_type uPast[nb + nd - 2];
    output_type yref;
    output_type yCurr;
    theta_type thetaScenarios[Nscen + 1][nTheta];

    #ifdef PL
    /*
     *   Uncertainty zonotope update and reduction
     */
    theta_type newCenter[nTheta];
    theta_type newGens[nTheta][nTheta+1];
    
    // Writes in newCenter, newGens the new zonotope center and generators
    boundStripZonotopeIntersection(yCurr, yInit, uSamples, thetaNominal, generators, newCenter, newGens);

    intervalHull(thetaNominal, generators, newCenter, newGens);
    #endif

    /* 
     *  Input optimization 
     */
    // Generate scenarios for cost function computations
    generateScenarios(thetaScenarios);

    for (int i = 0; i < NhorU; i++)
        uOpt[i] = DAConvertU(uOptDig[i]);

    yref = DAConvertY(yrefDig);
    yCurr = DAConvertY(yCurrDig); // if this is y(k)...

    // Update inital conditions
    for (int i = na - 1; i > 0; i--)
        yInit[i] = yInit[i - 1]; // ...this becomes yInit = [y(k), y(k-1), ..., y(k-na+1)] ...
    yInit[0] = yCurr;            // ...necessary to compute y(k+1) together with u(k)

    for (int i = 0; i < nb + nd - 2; i++)
        uPast[i] = uSamples[i];

    // Run MADS optimization algorithm
    // computing u(k)
    MADSARX(uOpt, uPast, yInit, yref, thetaScenarios);

    // Convert uOpt back to digital
    for (int i = 0; i < NhorU; i++)
        uOptDig[i] = ADConvertU(uOpt[i]);

    // Update input initial conditions
    for (int i = nb + nd - 2; i > 0; i--)
        uSamples[i] = uSamples[i - 1];
    uSamples[0] = uOpt[0];

    return;
}