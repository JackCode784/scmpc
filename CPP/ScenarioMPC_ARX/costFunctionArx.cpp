#include "setup.h"
#ifdef FIXED
#include "hls_math.h"
#endif

// costFunctionArx computes the cost for the current point currU.
// ARX model assumption: system is SISO (single input single output), i.e. nu = ny = 1
void costFunctionArx(cost_type cost[2], const output_type yPast[na], const input_type currU[nOpt], const input_type uPast[nb + nd - 2], const output_type yref, const theta_type thetaScenarios[Nscen + 1][nTheta])
{
    #ifdef PRAGMAS
    // #pragma HLS INLINE
    #endif
    
    cost[0] = 0; // cost init
    cost[1] = 0; // cost init

    input_type uPastCurr[nb + nd - 2];  // local copy of uPast
    output_type yPastCurr[na];          // local copy of yPast
    input_type uSamples[nb + nd - 1];
    output_type yNext;
    err_type err;

    #ifdef PRAGMAS
    // #pragma HLS ARRAY_PARTITION variable=currU type=complete
    #endif

    // initialize internal copies of yPast, uPast
    for (int i = 0; i < na; i++)
    {
        #ifdef PRAGMAS
        // #pragma HLS UNROLL
        #endif
        yPastCurr[i] = yPast[i];
    }

    for (int i = 0; i < nb + nd - 2; i++)
    {
        #ifdef PRAGMAS
        // #pragma HLS UNROLL
        #endif
        uPastCurr[i] = uPast[i];
    }

    // Input constraint violation
    for (int i = 0; i < nOpt; i++)
    {
        #ifdef PRAGMAS
        // #pragma HLS dependence variable=cost type=inter false
        #endif
        if (currU[i] < UMIN || currU[i] > UMAX)
        {
            cost[1] = 500;
            break;
        }
    }

    // Compute cost during control horizon
    for (int k = 0; k < NhorU; k++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif

        // Fill uSamples
        uSamples[0] = currU[k]; // u(k)
        for (int i = 1; i < nb + nd - 1; i++)
            uSamples[i] = uPastCurr[i - 1];

        // Cost contribution (current input)
        cost[0] += uSamples[0] * R * uSamples[0];

        // For each scenario compute output sequence and constraints violation
        for (int l = 0; l < Nscen + 1; l++)
        {
            #ifdef PRAGMAS
            // #pragma HLS UNROLL
            #endif

            // Compute system's next output y(k+1) from y(k), ..., y(k-na+1), u(k), ..., u(k-nb-nd+2)
            yNext = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);

            #ifdef PL
            // Cost contribution (all scenarios)
            err = yNext - yref;
            cost[0] += (err * Q * err) / Nscen;
            #endif

            // Update constraint violation cost
           updateConstraintViolation(cost, yNext);
        }

        // Update cost (only nominal system contribution)
        err = yNext - yref;
        cost[0] += err * Q * err;

        // Update initial conditions for next output
        for (int i = na - 1; i > 0; i--)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = yNext;

        for (int i = nb + nd - 2; i > 0; i--)
            uPastCurr[i] = uPastCurr[i - 1];
        uPastCurr[0] = uSamples[0]; // currU[k]
    }

    // Compute cost after control horizon
    for (int k = NhorU; k < Nhor - 1; k++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        // Fill uSamples
        uSamples[0] = uPastCurr[0];
        for (int i = 1; i < nb + nd - 1; i++)
            uSamples[i] = uPastCurr[i - 1];

        // Cost contribution (current input)
        cost[0] += uSamples[0] * R * uSamples[0];

        for (int l = 0; l < Nscen + 1; l++)
        {
            #ifdef PRAGMAS
            // #pragma HLS UNROLL
            #endif

            // Compute system's current output
            yNext = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);
            
            #ifdef PL
            // Cost contribution (all scenarios)
            err = yNext - yref;        
            cost[0] += (err * Q * err) / Nscen;
            #endif

            // Update constraint violation cost
           updateConstraintViolation(cost, yNext);
        }

        // Cost contribution (only nominal system)
        err = yNext - yref;
        cost[0] += err * Q * err;

        // Update initial conditions for next output
        for (int i = na - 1; i > 0; i--)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = yNext;

        for (int i = nb + nd - 2; i > 0; i--)
            uPastCurr[i] = uPastCurr[i - 1];
        uPastCurr[0] = uSamples[0];
    }

    /*
     *   Last time instant (use P matrix)
     */
    uSamples[0] = uPastCurr[0];
    for (int i = 1; i < nb + nd - 1; i++)
        uSamples[i] = uPastCurr[i - 1];

    // Cost contribution (current input)
    // cost[0] += uSamples[0] * R * uSamples[0];

    for (int l = 0; l < Nscen + 1; l++)
    {
        #ifdef PRAGMAS
        // #pragma HLS UNROLL
        #endif

        // Compute system's current output
        yNext = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);

        #ifdef PL
        // Cost contribution (all scenarios)
        err = yNext - yref;
        cost[0] += (err * Q * err) / Nscen;
        #endif

        // Update constraint violation cost
       updateConstraintViolation(cost, yNext);
    }

    // Cost contribution (only nominal system)
    err = yNext - yref;
    cost[0] += err * P * err;

    return;
}
