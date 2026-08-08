#include "setup.h"
#ifdef FIXED
#include "hls_math.h"
#endif

// costFunctionArx computes the cost for the current point currU.
// ARX model assumption: system is SISO (single input single output), i.e. nu = ny = 1
void costFunctionArxOptimized(cost_type cost[2], const output_type yPast[na], const input_type currU[nOpt], const input_type uPast[nb + nk - 2], const output_type yref, const theta_type thetaScenarios[Nscen][nTheta])
{
    #ifdef PRAGMAS
    // #pragma HLS INLINE
    #endif
    
    cost[0] = 0; // cost init
    cost[1] = 0; // cost init

    // input_type uPastCurr[nb + nk - 2];  // local copy of uPast
    output_type yPastCurr[na];          // local copy of yPast
    input_type uSamples[nb + nk - 1];
    output_type yNext;
    err_type err;
    cost_type scenariosContrib;

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

    for (int i = 0; i < nb + nk - 2; i++)
    {
        #ifdef PRAGMAS
        // #pragma HLS UNROLL
        #endif
        uSamples[i + 1] = uPast[i];
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

    // Input contribution to cost function
    for(int k = 0; k < NhorU; k++)
        cost[0] += currU[k] * currU[k];
    
    for(int k = 0; k < Nhor - NhorU - 1; k++)
        cost[0] += currU[nOpt-1] * currU[nOpt-1];
    
    cost[0] *= R;


    /*
        Idea: scenariosContrib accumulates all scenarios + nominal system contributions for all k=0,..., Nhor-1, then the scenarios strictly should be divided by Nscen (or equivalently shifted by LOG2NSCEN) and then everything (nominal system too) should be multiplied by outputWeight (or terminalOutputWeight). This could cause overflow if error is large or Nscen is high (=> saturation?). The loops from k=0,...,NhorU-1 and k=NhorU,...,Nhor-1 cannot be parallelized since the latter depends on the former (output initial conditions must be computed and updated at every step). Also, since the contribution of the nominal system and the scenarios differ by a division by Nscen (but share the multiplication by outputWeight), they should be computed separately and then summed. 
    */
   
    // Compute cost during control horizon
    for (int k = 0; k < NhorU; k++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif

        // Fill uSamples
        uSamples[0] = currU[k]; // u(k)
        scenariosContrib = 0;

        // For each scenario compute output sequence and constraints violation
        for (int l = 0; l < Nscen; l++)
        {
            #ifdef PRAGMAS
            // #pragma HLS UNROLL
            #endif

            // Compute system's next output y(k+1) from y(k), ..., y(k-na+1), u(k), ..., u(k-nb-nk+2)
            yNext = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);

            #ifdef PL
            // Cost contribution (all scenarios)
            err = yNext - yref;
            scenariosContrib += (err * err);
            #endif

            // Update constraint violation cost
           updateConstraintViolation(cost, yNext);
        }

        #ifdef PL
        #ifndef FIXED
        scenariosContrib /= Nscen;
        #else
        scenariosContrib >> LOG2NSCEN;
        #endif
        #endif

        // Update cost (only nominal system contribution)
        yNext = computeArxOutput(yPastCurr, uSamples, thetaCenter);
        err = yNext - yref;
        cost[0] += outputWeight * (err * err + scenariosContrib);

        // Update initial conditions for next output
        for (int i = na - 1; i > 0; i--)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = yNext;

        for (int i = nb + nk - 2; i > 0; i--)
            uSamples[i] = uSamples[i - 1];
    }

    // Compute cost after control horizon
    for (int k = NhorU; k < Nhor - 1; k++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif

        // Fill uSamples
        // uSamples[0] = uSamples[1];

        // Cost contribution (current input)
        cost[0] += uSamples[0] * R * uSamples[0];
        scenariosContrib = 0;

        for (int l = 0; l < Nscen; l++)
        {
            #ifdef PRAGMAS
            // #pragma HLS UNROLL
            #endif

            // Compute system's current output
            yNext = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);
            
            #ifdef PL
            // Cost contribution (all scenarios)
            err = yNext - yref;        
            scenariosContrib += (err * err); // sum of err^2 for all scenarios except nominal
            #endif

            // Update constraint violation cost
           updateConstraintViolation(cost, yNext);
        }

        #ifdef PL
        #ifndef FIXED
        scenariosContrib /= Nscen; // all scenarios err^2 / Nscen
        #else
        scenariosContrib >> LOG2NSCEN;
        #endif
        #endif

        // Update cost (only nominal system contribution)
        yNext = computeArxOutput(yPastCurr, uSamples, thetaCenter);
        err = yNext - yref;
        cost[0] += outputWeight * (err * err + scenariosContrib);

        // Update initial conditions for next output
        for (int i = na - 1; i > 0; i--)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = yNext;

        for (int i = nb + nk - 2; i > 0; i--)
            uSamples[i] = uSamples[i - 1];
    }

    /*
     *   Last time instant (use terminalOutputWeight matrix)
     */
    // uSamples[0] = uPastCurr[0];

    // Cost contribution (current input)
    // cost[0] += uSamples[0] * R * uSamples[0];
    scenariosContrib = 0;

    for (int l = 0; l < Nscen; l++)
    {
        #ifdef PRAGMAS
        // #pragma HLS UNROLL
        #endif

        // Compute system's current output
        yNext = computeArxOutput(yPastCurr, uSamples, thetaScenarios[l]);

        #ifdef PL
        // Cost contribution (all scenarios)
        err = yNext - yref;
        scenariosContrib += (err * err);
        #endif

        // Update constraint violation cost
       updateConstraintViolation(cost, yNext);
    }

    #ifdef PL
    #ifndef FIXED
    scenariosContrib /= Nscen;
    #else
    scenariosContrib >> LOG2NSCEN;
    #endif
    #endif

    // Update cost (only nominal system contribution)
    yNext = computeArxOutput(yPastCurr, uSamples, thetaCenter);
    err = yNext - yref;
    cost[0] += err * terminalOutputWeight * err + outputWeight * scenariosContrib;

    return;
}
