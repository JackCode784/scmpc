#include "setup.h"
#include <stdio.h>
#ifdef DEBUG_MODE
#include <cstdlib>
#include <time.h>
#endif

int main(void)
{
    #ifdef DEBUG_MODE
    printf("\nTest run ARX with MADS\n\n");

    srand(time(NULL)); // different random number each time
    #endif

    #ifdef DEBUG_MODE
    rand_type v[nOpt];
    for(int i = 0; i < 100; i++)
    {
        pseudoRandArx(v);
        for(int k = 0; k < nOpt; k++)
            printf("%f ", v[k]);
        printf("\n");
    }
    #endif

    short nSim = 50;
    output_type yInit[na] = {0};         // output past samples
    output_type yref = 5;          // output reference signal
    output_type ySim[nSim] = {0};        // output simulation samples
    output_type yCurr;               // output temporary sample
    input_type uInit[nb + nd - 1] = {0}; // input past samples
    input_type uSim[nSim] = {0};         // input simulation samples
    input_type uOpt[NhorU] = {0};        // input optimal values at each time instant
    input_type uSamples[nb + nd];        // past and current input samples
    theta_type thetaScenarios[Nscen + 1][nTheta];

    #ifdef DEBUG_MODE
    for(int i = 0; i < 30; i++)
    {
        generateScenarios(thetaScenarios);
        for(int j = 0; j < Nscen+1; j++)
        {
            for(int k = 0; k < nTheta; k++)
                printf("%f ", thetaScenarios[j][k]);
            printf("\n");
        }
        printf("\n");
    }
    #endif

    #if defined(CONVERSIONS_MODE) || !defined(DEBUG_MODE)
    // From analog to digital signals conversions
    digital_input_type uSimDig[nSim] = {0};
    digital_output_type ySimDig[nSim] = {0};
    digital_output_type yInitDig[na];
    digital_output_type yrefDig;
    digital_input_type uInitDig[nb+nd-1];
    digital_input_type uOptDig[NhorU];

    for(int i = 0; i < na; i++)
        yInitDig[i] = ADConvertY(yInit[i]);

    for(int i = 0; i < nb + nd - 1; i++)
        uInitDig[i] = ADConvertU(uInit[i]);

    for(int i = 0; i < NhorU; i++)
        uOptDig[i] = ADConvertU(uOpt[i]);

    yrefDig = ADConvertY(yref);
    #endif

    /* Simulation */
    // For each simulation instant, generate new scenarios and
    // compute optimal input for nominal system, while ensuring
    // constraint satisfaction for the newly-generated scenarios
    for (int k = 0; k < nSim; k++)
    {
        #if defined(CONVERSIONS_MODE) || !defined(DEBUG_MODE)
        controller(uOptDig, thetaScenarios, yInitDig, uInitDig, yrefDig);
        uSimDig[k] = uOptDig[0];    // receding horizon implementation
        
        for(int i = 0; i < NhorU; i++)
            uOpt[i] = DAConvertU(uOptDig[i]);
        #else
        // Don't use ADC and DAC
        generateSCMPCControl(uOpt, thetaScenarios, yInit, uInit, yref);
        #endif
        // DEBUG_MODE
        uSim[k] = uOpt[0];  // receding horizon implementation

        /* Simulate ARX */
        // Fill uSamples
        uSamples[0] = uOpt[0];
        for (int i = 0; i < nb + nd - 1; i++)
            uSamples[i + 1] = uInit[i];

        yCurr = computeArxOutput(yInit, uSamples, thetaNominal);
        ySim[k] = yCurr;

        // update initial conditions
        for (int i = 1; i < na; i++)
            yInit[i] = yInit[i - 1];
        yInit[0] = ySim[k];

        for (int i = 1; i < nb + nd - 1; i++)
            uInit[i] = uInit[i - 1];
        uInit[0] = uSamples[0]; // uSamples[0] == uSim[k]

        #if defined(CONVERSIONS_MODE) || !defined(DEBUG_MODE)
        // Memorize digital conversion of current output
        ySimDig[k] = ADConvertY(yCurr);
        
        // update initial conditions
        for (int i = 1; i < na; i++)
            yInitDig[i] = yInitDig[i - 1];
        yInitDig[0] = ySimDig[k];

        for (int i = 1; i < nb + nd - 1; i++)
            uInitDig[i] = uInitDig[i - 1];
        uInitDig[0] = uSimDig[k]; // uSamples[0] == uSim[k]
        #endif
    }

    /* Output file creation and writing */
    FILE *fp;
    fp = fopen("output.txt", "w");
    for (int k = 0; k < nSim; k++)
    {
        #if defined(DEBUG_MODE) && !defined(CONVERSIONS_MODE)
        fprintf(fp, "%lf %lf", uSim[k], ySim[k]);
        #endif 
        #if !defined(DEBUG_MODE) || defined(CONVERSIONS_MODE)
        fprintf(fp, "%f %f %d %d", uSim[k], ySim[k], uSimDig[k], ySimDig[k]);
        #else
        fprintf(fp, "%f %f %f %f", uSim[k].to_float(), ySim[k].to_float(), uSimDig[k].to_float(), ySimDig[k].to_float());
        #endif
        
        fprintf(fp, "\n");
    }
    fclose(fp);
    return 0;
}
