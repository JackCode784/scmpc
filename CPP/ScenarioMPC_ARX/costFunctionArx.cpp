#include "setup.h"
// #include "hls_math.h"

// costFunctionArx computes the cost for the current point currU.
// ARX system assumption: system is SISO (single input single output), i.e. nu = ny = 1
void costFunctionArx(cost_type cost[2], const output_type yPast[na], const input_type currU[nOpt], const input_type uPast[nb + nd - 1], const output_type yref[ny], hzn_type predictionHzn, hzn_type controlHzn, const theta_type thetaScenarios[Nscen + 1][nTheta])
{
    // Following MATLAB implementation
    cost[0] = 0; // cost init
    cost[1] = 0;

    input_type uSamples[nb + nd];
    input_type uPastCurr[nb+nd-1];
    output_type currY[ny];
    output_type err[ny];
    output_type yPastCurr[na];

    // initialize internal copies of yPast, uPast
    for(int i = 0; i < na; i++)
        yPastCurr[i] = yPast[i];

    for(int i = 0; i < nb + nd - 1; i++)
        uPastCurr[i] = uPast[i];

    // Generic indexes
    int i, j;

    // Input constraint violation
    for (i = 0; i < nOpt; i++)
    {
        if (currU[i] < umin[i] || currU[i] > umax[i])
        {
            cost[1] = 500;
            break;
        }
    }
    // for (i = 0; i < nu; i++)
    // {
    //     for (j = 0; j < NhorU; j++)
    //         if (currU[i][j] < umin || currU[i][j] > umax)
    //         {
    //             cost[1] = 500;
    //             break;
    //         }
    //     if(cost[1] != 0)
    //         break;
    // }

    // Compute cost during control horizon
    for (int k = 0; k < controlHzn; k++)
    {
        // Fill uSamples
        uSamples[0] = currU[k];
        for (i = 1; i < nb + nd; i++)
            uSamples[i] = uPastCurr[i - 1];

        //// Update cost
        // Assumption: SISO system
        // Current input contribution
        cost[0] += uSamples[0] * R[0][0] * uSamples[0];
        // for(i = 0; i < nu; i++)
        //     for(j = 0; j < nu; j++)
        //         cost += uSamples[i][0] * R[i][j] * uSamples[j][0];

        // For each sceanario compute output sequence and constraints violation
        for (int l = 0; l < Nscen + 1; l++)
        {
            // Compute system's current output
            computeArxOutput(currY, yPastCurr, uSamples, thetaScenarios[l]);

            // for (i = 0; i < ny; i++)
            //     err[i] = currY[i] - yref[i];

            // for (i = 0; i < ny; i++)
            //     for (j = 0; j < ny; j++)
            //         cost[0] += err[i] * Q[i][j] * err[j];

            // Update constraint violation cost
            updateConstraintViolation(cost, currY);

            // Update past output and input samples
            // Assumption: SISO system
            // for (i = 1; i < na; i++)
            //     yPast[l][i] = yPast[l][i - 1];
            // yPast[l][0] = currY[0];
        }

        // Update cost (only nominal system contribution)
        for (i = 0; i < ny; i++)
                err[i] = currY[i] - yref[i];

            for (i = 0; i < ny; i++)
                for (j = 0; j < ny; j++)
                    cost[0] += err[i] * Q[i][j] * err[j];

        for (i = 1; i < na; i++)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = currY[0];

        for (i = 1; i < nb + nd - 1; i++)
            uPastCurr[i] = uPastCurr[i - 1];
        uPastCurr[0] = uSamples[0]; // currU[k]
    }

    // Compute cost after control horizon
    for (int k = controlHzn; k < predictionHzn - 1; k++)
    {
        // Fill uSamples
        uSamples[0] = uPastCurr[0];
        for (i = 1; i < nb + nd; i++)
            uSamples[i] = uPastCurr[i - 1];

        //// Update cost
        // Assumption: SISO system
        // Current input contribution
        cost[0] += uSamples[0] * R[0][0] * uSamples[0];
        // for(i = 0; i < nu; i++)
        //     for(j = 0; j < nu; j++)
        //         cost += uSamples[i][0] * R[i][j] * uSamples[j][0];

        for (int l = 0; l < Nscen + 1; l++)
        {
            // Compute system's current output
            computeArxOutput(currY, yPastCurr, uSamples, thetaScenarios[l]);

            for (i = 0; i < ny; i++)
                err[i] = currY[i] - yref[i];

            for (i = 0; i < ny; i++)
                for (j = 0; j < ny; j++)
                    cost[0] += err[i] * Q[i][j] * err[j];

            // Update constraint violation cost
            updateConstraintViolation(cost, currY);

            // Update past output and input samples
            // Assumption: SISO system
            // for (i = 1; i < na; i++)
            //     yPast[l][i] = yPast[l][i - 1];
            // yPast[l][0] = currY[0];
        }

        for (i = 1; i < na; i++)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = currY[0];

        for (i = 1; i < nb + nd - 1; i++)
            uPastCurr[i] = uPastCurr[i - 1];
        uPastCurr[0] = uSamples[0];
    }

    // Last time instant (use P matrix)
    uSamples[0] = uPastCurr[0];
    for (i = 1; i < nb + nd; i++)
        uSamples[i] = uPastCurr[i - 1];

    for (int l = 0; l < Nscen + 1; l++)
    {
        computeArxOutput(currY, yPastCurr, uSamples, thetaScenarios[l]);

        for (i = 0; i < ny; i++)
            err[i] = currY[i] - yref[i];

        for (i = 0; i < ny; i++)
            for (j = 0; j < ny; j++)
                cost[0] += err[i] * Q[i][j] * err[j];

        // Update constraint violation cost
        updateConstraintViolation(cost, currY);
    }
}
