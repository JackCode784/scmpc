#include "setup.h"
#include <stdio.h>

int main(void)
{
    printf("Test run ARX with MADS\n\n");

    int nSim = 50;
    output_type yInit[na] = {0};         // output past samples
    output_type yref[ny] = {5};          // output reference signal
    output_type ySim[nSim] = {0};        // output simulation samples
    output_type yCurr[ny];               // output temporary sample
    input_type uInit[nb + nd - 1] = {0}; // input past samples
    input_type uSim[nSim] = {0};         // input simulation samples
    input_type uOpt[NhorU] = {0};        // input optimal values at each time instant
    input_type uSamples[nb + nd];        // past and current input samples

    theta_type thetaScenarios[Nscen + 1][nTheta];
    theta_type currTheta[nTheta];

    /* Simulation */
    // For each simulation instant, generate new scenarios and
    // compute optimal input for nominal system, while ensuring
    // constraint satisfaction for the newly-generated scenarios
    for (int k = 0; k < nSim; k++)
    {
        generateSCMPCControl(uOpt, thetaScenarios, yInit, uInit, yref, Nhor, NhorU);
        uSim[k] = uOpt[0]; // receding horizon: only first input is considered

        /* Simulate ARX */
        // Fill uSamples
        uSamples[0] = uOpt[0];
        for (int i = 0; i < nb + nd - 1; i++)
            uSamples[i + 1] = uInit[i];

        computeArxOutput(yCurr, yInit, uSamples, thetaNominal);
        ySim[k] = yCurr[0];
        
        // update initial conditions
        for (int i = 1; i < na; i++)
            yInit[i] = yInit[i - 1];
        yInit[0] = ySim[k];

        for (int i = 1; i < nb + nd - 1; i++)
            uInit[i] = uInit[i - 1];
        uInit[0] = uSamples[0]; // uSamples[0] == uSim[k]
    }

    /* Output file creation and writing */
    FILE *fp;
    fp = fopen("output.txt", "w");
    for (int k = 0; k < nSim; k++)
    {
        // for (int idxScen = 0; idxScen < Nscen + 1; idxScen++)
        fprintf(fp, "%f %f ", uSim[k], ySim[k]);
        fprintf(fp, "\n");
    }
    fclose(fp);

    return 0;
}
