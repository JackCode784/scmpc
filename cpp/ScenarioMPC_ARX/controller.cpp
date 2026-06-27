/**
 * @file  controller.cpp
 * @brief SCMPC controller for an ARX system - HLS top-level function.
 *
 * SHARED MUTABLE STATE
 * ---------------------
 * The variables below are defined here WITHOUT the "static" keyword, giving
 * them EXTERNAL linkage.  Any other .cpp file that includes setup.h sees
 * the extern declarations and refers to the same physical storage.
 *
 * "static" at file scope would give INTERNAL linkage: every translation
 * unit that included setup.h would get its own private copy, and those
 * copies would silently diverge as soon as one was updated.  That is the
 * wrong behaviour for shared controller state.
 *
 *   thetaCenter[nTheta]              - current zonotope centre c(k)
 *   thetaGens  [nTheta][nGens]- current generator matrix G(k)
 *   nGens                       - number of active generator columns
 *   yHist      [na]                  - output history y(k−1), ..., y(k−na)
 *   uHist      [nb+nk−1]             - input  history u(k−1), ..., u(k−nb−nk+1)
 *
 * ZONOTOPE LIFECYCLE
 * ------------------
 * boundStripZonotopeIntersection() - produces newCenter/newGens with the
 *                             same nGens columns; result is copied back
 *                             into thetaCenter/thetaGens.
 * intervalHull is NOT used - the strip-intersection algorithm used here
 *                             already keeps the generator count constant.
 *
 * ORDERING OF OPERATIONS INSIDE controller()
 * -------------------------------------------
 * The order is critical and must not be changed without careful thought
 * about which data belongs to time k vs k−1.
 *
 *  1. Convert digital inputs -> algorithm types.
 *  2. [PL] Update zonotope using y(k), OLD yHist, OLD uHist.
 *          The strip S(k) = {theta : |y(k)−phi(k)^T*theta|<=epsilon} uses the regressor
 *          phi(k) = [y(k−1),..., u(k−1),...] - the OLD histories.
 *  3. [AL] WIP stub.
 *  4. Generate scenarios from the (now updated) thetaCenter / thetaGens.
 *  5. Shift yHist <- [y(k), y(k−1), ...].
 *  6. Snapshot uHist into uPast (before it is updated in step 9).
 *  7. Run MADS -> uOpt.
 *  8. Convert uOpt -> digital.
 *  9. Shift uHist <- [uOpt[0], u(k−1), ...].
 */

#include "setup.h"
#include <stdio.h>

/* ======================================================================
   DEFINITION of the shared mutable controller state.
   "extern" declarations for these live in setup.h (included above).
   No "static" - external linkage is required so all translation units
   share the same storage.
   ====================================================================== */
theta_type  thetaCenter[nTheta]              = {    THETA_NOMINAL_INIT  };
theta_type  thetaGens  [nTheta][nGens]       = {    GENERATORS_INIT     };
output_type yHist[na]                         = { Y_HIST_INIT };
input_type  uHist[nb + nk - 1]               = { U_HIST_INIT };

/* ======================================================================
   controller()
   ====================================================================== */
digital_input_type controller(const digital_output_type yCurrDig,
                              const digital_output_type yrefDig)
{
	#ifdef PRAGMAS
	#pragma HLS INTERFACE ap_none port=yCurrDig
	#pragma HLS INTERFACE ap_none port=yrefDig
	#pragma HLS INTERFACE ap_ctrl_hs port=return
	#endif
    /* ------------------------------------------------------------------ */
    /*  Step 1 - Convert digital inputs to algorithm types                */
    /* ------------------------------------------------------------------ */
    #ifdef CONVERSIONS_MODE
    output_type yCurr = DAConvertY(yCurrDig);
    output_type yref  = DAConvertY(yrefDig);
    #else 
    output_type yCurr = yCurrDig;
    output_type yref = yrefDig;
    #endif
    
    input_type uOpt[NhorU];
    for (int i = 0; i < NhorU; i++)
        uOpt[i] = uHist[0]; // warm start

    /* ------------------------------------------------------------------ */
    /*  Step 2 - [PL mode] Zonotope update                                */
    /* ------------------------------------------------------------------ */
#if CTRL_MODE == CTRL_MODE_PL
    {
        /*
         * Intersect the current zonotope with the strip S(k).
         * The strip uses the OLD yHist and OLD uHist - the regressor
         * phi(k) = [y(k−1),..., u(k−1),...] refers to measurements taken
         * BEFORE this time step.
         *
         * newCenter and newGens are temporaries with the same dimensions
         * as thetaCenter and thetaGens.  We cannot pass thetaCenter/
         * thetaGens as both input and output to the same call because
         * the function reads the old values while writing the new ones;
         * aliasing would corrupt the computation.
         * After the call we copy the result back into the globals.
         */
        theta_type newCenter[nTheta];
        theta_type newGens  [nTheta][nGens];

        boundStripZonotopeIntersectionNew(yCurr,
                                       yHist, uHist,
                                       thetaCenter, thetaGens,
                                       newCenter, newGens);

        /* Copy result back into the shared global zonotope state. */
        for (int i = 0; i < nTheta; i++) {
            thetaCenter[i] = newCenter[i];
            for (int j = 0; j < nGens; j++)
                thetaGens[i][j] = newGens[i][j];
        }

#ifdef DEBUG_PRINT
        printf("\n--- PL: updated zonotope ---\n");
        printf("thetaCenter: ");
        for (int i = 0; i < nTheta; i++) printf("%f ", (double)thetaCenter[i]);
        printf("\nthetaGens:\n");
        for (int i = 0; i < nTheta; i++) {
            for (int j = 0; j < nGens; j++)
                printf("%8.4f ", (double)thetaGens[i][j]);
            printf("\n");
        }
#endif
    }
#endif  /* CTRL_MODE == CTRL_MODE_PL */

    /* ------------------------------------------------------------------ */
    /*  Step 3 - [AL mode] Active learning stub (WIP)                     */
    /* ------------------------------------------------------------------ */
#if CTRL_MODE == CTRL_MODE_AL
    {
        /* TODO: dual-control probing perturbation on uOpt. */
    }
#endif

    /* ------------------------------------------------------------------ */
    /*  Step 4 - Generate uncertainty scenarios                            */
    /* ------------------------------------------------------------------ */
    /*
     * generateScenarios reads thetaCenter and thetaGens via the extern
     * globals, but we pass them as explicit arguments to keep the data
     * flow visible to the HLS scheduler.  The values are the most recent
     * ones: updated by step 2 in PL mode, or fixed at init in SCMPC mode.
     */
    theta_type thetaScenarios[Nscen][nTheta];
    generateScenarios(thetaScenarios, thetaCenter, thetaGens);

    /* ------------------------------------------------------------------ */
    /*  Step 5 - Update output history: push y(k) into yHist             */
    /* ------------------------------------------------------------------ */
    for (int i = na - 1; i > 0; i--)
        yHist[i] = yHist[i - 1];
    yHist[0] = yCurr;   /* yHist = [y(k), y(k−1), ..., y(k−na+1)] */

    /* ------------------------------------------------------------------ */
    /*  Step 6 - Snapshot input history for MADS                          */
    /* ------------------------------------------------------------------ */
    /*
     * uPast = [u(k−1), ..., u(k−nb−nk+1)] - needed by costFunctionArx to
     * roll out predictions for y(k+1), ..., y(k+N) alongside uOpt.
     * Must be taken BEFORE uHist is updated with uOpt[0] in step 9.
     */
    input_type uPast[nb + nk - 2];
    for (int i = 0; i < nb + nk - 2; i++)
        uPast[i] = uHist[i];

    /* ------------------------------------------------------------------ */
    /*  Step 7 - Run MADS optimisation                                    */
    /* ------------------------------------------------------------------ */
    MADSARX(uOpt, uPast, yHist, yref, thetaScenarios);

    /* ------------------------------------------------------------------ */
    /*  Step 8 - Convert uOpt back to digital                             */
    /* ------------------------------------------------------------------ */
    digital_input_type uOptDig;
    #ifdef CONVERSIONS_MODE
    uOptDig = ADConvertU(uOpt[0]);
    #else
    uOptDig = uOpt[0];
    #endif

    /* ------------------------------------------------------------------ */
    /*  Step 9 - Update input history: push uOpt[0] into uHist           */
    /* ------------------------------------------------------------------ */
    for (int i = nb + nk - 2; i > 0; i--)
        uHist[i] = uHist[i - 1];
    uHist[0] = uOpt[0];   /* receding horizon: only u(k) is applied */

    return uOptDig;
}