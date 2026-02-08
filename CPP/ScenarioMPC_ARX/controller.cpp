#include "setup.h"
#include <stdio.h>

#ifdef CMPLSYS
theta_type thetaNominal[nTheta] = {0.7921, 0.1524, -0.1668, 0.0842, 0.0442, 0.0860}; // extern in setup.h
theta_type generators[nTheta][nTheta] = {
    {-0.8959, -0.4594, -0.0026, -0.0027, -0.0187, 0.0080},
    {1.3452, -0.1401, 0.0030, 0.0111, -0.0283, 0.0079},
    {-0.5326, 0.4275, -0.0051, 0.0289, -0.0362, 0.0077},
    {-0.0082, 0.0028, 0.0524, 0.0788, 0.0367, 0.0085},
    {0.0859, 0.0502, -0.1015, -0.0126, 0.0271, 0.0086},
    {0.0021, 0.1180, 0.0538, -0.0985, 0.0126, 0.0086}
};  // extern in setup.h
#endif

output_type yInit[na] = {0};    // extern in setup.h
input_type uSamples[nb + nd - 1] = {0}; // extern in setup.h

// Take digital inputs, convert them to fixed point, find optimal input, return it as digital
// At instant k measure y(k), find uOpt = [u(k), u(k+1), ..., u(k+NhorU-1)]
void controller(digital_input_type uOptDig[NhorU], const digital_output_type yCurrDig, const digital_output_type yrefDig)
{
    // Convert digital inputs to fixed point/float
    input_type uOpt[NhorU];
    input_type uPast[nb + nd - 2];
    output_type yref;
    output_type yCurr;
    theta_type thetaScenarios[Nscen][nTheta];

    yref = DAConvertY(yrefDig);
    yCurr = DAConvertY(yCurrDig); // if this is y(k)...
    
    for (int i = 0; i < NhorU; i++)
        uOpt[i] = DAConvertU(uOptDig[i]);   // u(k), u(k+1), ..., u(k+NhorU-1)

#ifdef PL
    /*
     *   Uncertainty zonotope update and reduction
     */
    theta_type newCenter[nTheta];
    theta_type newGens[nTheta][nTheta + 1];

    // Writes in newCenter, newGens the new zonotope center and generators
    boundStripZonotopeIntersection(yCurr, yInit, uSamples, thetaNominal, generators, newCenter, newGens);

    printf("\n\n");
    for(int i = 0; i < nTheta; i++)
        printf("%f ", newCenter[i]);
    printf("\n\n");

    for(int i = 0; i < nTheta; i++)
    {
        for(int j =0; j < nTheta + 1; j++)
            printf("%f\t", newGens[i][j]);
        printf("\n");
    }

    intervalHull(thetaNominal, generators, newCenter, newGens);

    printf("\n\n");
    for(int i = 0; i < nTheta; i++)
    {
        for(int j =0; j < nTheta; j++)
            printf("%f ", generators[i][j]);
        printf("\n");
    }
    for(int i = 0; i < nTheta; i++)
        (((thetaNominal[i] < 0) ? -thetaNominal[i] : thetaNominal[i]) < generators[i][i]) ? printf("Yes ") : printf("No ");
#endif

    /*
     *  Input optimization
     */
    // Generate scenarios for cost function computations
    generateScenarios(thetaScenarios);

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
