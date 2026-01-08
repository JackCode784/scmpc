#include "setup.h"
#ifdef FIXED
#include "hls_math.h"
#endif

// costFunctionArx computes the cost for the current point currU.
// ARX system assumption: system is SISO (single input single output), i.e. nu = ny = 1
void costFunctionArx(cost_type cost[2], const output_type yPast[na], const input_type currU[nOpt], const input_type uPast[nb + nd - 1], const output_type yref, const theta_type thetaScenarios[Nscen + 1][nTheta])
{
    cost[0] = 0; // cost init
    cost[1] = 0; // cost init

    input_type uSamples[nb + nd];
    input_type uPastCurr[nb + nd - 1];
    output_type currY;
    err_type err;
    output_type yPastCurr[na];

    // initialize internal copies of yPast, uPast
    for (int i = 0; i < na; i++)
        yPastCurr[i] = yPast[i];

    for (int i = 0; i < nb + nd - 1; i++)
        uPastCurr[i] = uPast[i];

    // Input constraint violation
    for (int i = 0; i < nOpt; i++)
    {
        if (currU[i] < UMIN || currU[i] > UMAX)
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
    for (int k = 0; k < NhorU; k++)
    {
        // Fill uSamples
        uSamples[0] = currU[k];
        for (int i = 1; i < nb + nd; i++)
            uSamples[i] = uPastCurr[i - 1];

        // Cost contribution (current input)
        cost[0] += uSamples[0] * R * uSamples[0];

        // For each scenario compute output sequence and constraints violation
        for (int l = 0; l < Nscen + 1; l++)
        {
            // Compute system's current output
            currY = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);

            // Cost contribution (all sceanrios)
            // err = currY - yref;
            // cost[0] += err * Q * err;

            // Update constraint violation cost
            updateConstraintViolation(cost, currY);
        }
#ifndef __SYNTHESIS__
		// float currYf = currY.to_float();
#endif

        // Update cost (only nominal system contribution)
        err = currY - yref;

        cost[0] += err * Q * err;

        // Update initial conditions for next output
        for (int i = 1; i < na; i++)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = currY;

        for (int i = 1; i < nb + nd - 1; i++)
            uPastCurr[i] = uPastCurr[i - 1];
        uPastCurr[0] = uSamples[0]; // currU[k]
    }

    // Compute cost after control horizon
    for (int k = NhorU; k < Nhor - 1; k++)
    {
        // Fill uSamples
        uSamples[0] = uPastCurr[0];
        for (int i = 1; i < nb + nd; i++)
            uSamples[i] = uPastCurr[i - 1];

        // Cost contribution (current input)
        cost[0] += uSamples[0] * R * uSamples[0];

        for (int l = 0; l < Nscen + 1; l++)
        {
            // Compute system's current output
            currY = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);

            // Cost contribution (all scenarios)
            // err = currY - yref;
            // cost[0] += err * Q * err;

            // Update constraint violation cost
            updateConstraintViolation(cost, currY);
        }

        // Cost contribution (only nominal system)
        err = currY - yref;

        cost[0] += err * Q * err;

        // Update initial conditions for next output
        for (int i = 1; i < na; i++)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = currY;

        for (int i = 1; i < nb + nd - 1; i++)
            uPastCurr[i] = uPastCurr[i - 1];
        uPastCurr[0] = uSamples[0];
    }

    /*
     *   Last time instant (use P matrix)
     */
    uSamples[0] = uPastCurr[0];
    for (int i = 1; i < nb + nd; i++)
        uSamples[i] = uPastCurr[i - 1];

    // Cost contribution (current input)
    // cost[0] += uSamples[0] * R * uSamples[0];

    for (int l = 0; l < Nscen + 1; l++)
    {
        // Compute output for each scenario
        currY = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);

        // Cost contribution (all scenarios)
        // err = currY - yref;
        // cost[0] += err * Q * err;

        // Update constraint violation cost
        updateConstraintViolation(cost, currY);
    }

    // Cost contribution (only nominal system)
    err = currY - yref;
    cost[0] += err * P * err;
}
