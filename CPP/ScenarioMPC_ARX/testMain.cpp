#include "setup.h"
#include <stdio.h>
#ifdef DEBUG_MODE
#include <cstdlib>
#include <time.h>
#endif

int main(void)
{
    printf("\nTest run ARX with MADS\n\n");

#ifdef DEBUG_MODE
    srand(time(NULL)); // different random number each run
#endif

#ifdef FIXED
    rand_type v[nOpt];
    for (int i = 0; i < 100; i++)
    {
        pseudoRandArx(v);
        for (int k = 0; k < nOpt; k++)
            printf("%f ", v[k].to_float());
        printf("\n");
    }
#endif

    int nSim = 50;
    output_type ySim[nSim]; // output simulation samples
    input_type uSim[nSim];  // input simulation samples
    input_type uOpt[NhorU]; // input optimal values at each time instant [u(k), u(k+1), ..., u(k+NhorU-1)]
    output_type yCurr;      // output temporary sample
    output_type yref = 5;   // output reference signal

    input_type uSamples[nb + nd - 1]; // [u(k-1), ..., u(k-nb-nd+1)]
    output_type yInit[na];            // [y(k-1), ..., y(k-na)]

    for (int i = 0; i < nb + nd - 1; i++)
        uSamples[i] = 0;

    for (int i = 0; i < na; i++)
        yInit[i] = 0;

    for (int i = 0; i < NhorU; i++)
        uOpt[i] = 0;

#ifdef FIXED
    for (int i = 0; i < 30; i++)
    {
        generateScenarios(thetaScenarios);
        for (int j = 0; j < Nscen + 1; j++)
        {
            for (int k = 0; k < nTheta; k++)
                printf("%f ", thetaScenarios[j][k].to_float());
            printf("\n");
        }
        printf("\n");
    }
#endif

#if defined(FIXED) || defined(CONVERSIONS_MODE)
    // From analog to digital signals conversions
    digital_input_type uSimDig[nSim];
    digital_output_type ySimDig[nSim];
    digital_output_type yInitDig[na];
    digital_output_type yrefDig;
    digital_input_type uInitDig[nb + nd - 1];
    digital_input_type uOptDig[NhorU];

    for (int i = 0; i < na; i++)
        yInitDig[i] = ADConvertY(yInit[i]);

    for (int i = 0; i < nb + nd - 1; i++)
        uInitDig[i] = ADConvertU(uSamples[i]);

    for (int i = 0; i < NhorU; i++)
        uOptDig[i] = ADConvertU(uOpt[i]);

    yrefDig = ADConvertY(yref);
#endif

    /* Simulation */
    // For each simulation instant, generate new scenarios and
    // compute optimal input for nominal system, while ensuring
    // constraint satisfaction for the newly-generated scenarios
    for (int k = 0; k < nSim; k++)
    {
        /* Simulate ARX */
        yCurr = computeArxOutput(yInit, uSamples, thetaNominal);
        ySim[k] = yCurr;

        // Update inital conditions
        for (int i = na - 1; i > 0; i--)
            yInit[i] = yInit[i - 1]; // ...this becomes yInit = [y(k), y(k-1), ..., y(k-na+1)] ...
        yInit[0] = yCurr;            // ...necessary to compute y(k+1) together with u(k)

#if defined(CONVERSIONS_MODE) || defined(FIXED)
        // Memorize digital conversion of current output
        ySimDig[k] = ADConvertY(yCurr);
#endif

        controller(uOptDig, ySimDig[k], yrefDig);
        uSimDig[k] = uOptDig[0]; // receding horizon implementation

        for (int i = 0; i < NhorU; i++)
            uOpt[i] = DAConvertU(uOptDig[i]);
        uSim[k] = uOpt[0]; // receding horizon implementation

        // Update input initial conditions
        for (int i = nb + nd - 2; i > 0; i--)
            uSamples[i] = uSamples[i - 1];
        uSamples[0] = uOpt[0];
    }

    /* Output file creation and writing */
    FILE *fp;
    fp = fopen("output.txt", "w");
    for (int k = 0; k < nSim; k++)
    {
#ifdef FIXED
        fprintf(fp, "%f %f %f %f", uSim[k].to_float(), ySim[k].to_float(), uSimDig[k].to_float(), ySimDig[k].to_float());
#endif
#if !defined(FIXED) && defined(CONVERSIONS_MODE)
        fprintf(fp, "%f %f %d %d", uSim[k], ySim[k], uSimDig[k], ySimDig[k]);
#endif
#if !defined(FIXED) && !defined(CONVERSIONS_MODE)
        fprintf(fp, "%f %f", uSim[k], ySim[k]);
#endif
        fprintf(fp, "\n");
    }
    fclose(fp);
    return 0;
}
