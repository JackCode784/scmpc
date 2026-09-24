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
 *   uOptPrev   [NhorU]               - last step's MADS solution u*(k−1), ...,
 *                                      u*(k−1+NhorU−1), kept ONLY to seed the
 *                                      next warm start (see Step 1b/7b below) -
 *                                      not part of the ARX regressor, unlike uHist.
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
 *  1b. Warm-start uOptNorm by shifting uOptPrev (last call's MADS
 *          solution) by one step, repeating its last entry.
 *  2. [PL] Update zonotope using y(k), OLD yHist, OLD uHist.
 *          The strip S(k) = {theta : |y(k)−phi(k)^T*theta|<=epsilon} uses the regressor
 *          phi(k) = [y(k−1),..., u(k−1),...] - the OLD histories.
 *  3. [AL] WIP stub.
 *  4. Generate scenarios from the (now updated) thetaCenter / thetaGens.
 *  5. Shift yHist <- [y(k), y(k−1), ...].
 *  6. Snapshot uHist into uPast (before it is updated in step 9).
 *  7. Run MADS -> uOpt, warm-started from step 1b.
 *  7b. Save uOpt into uOptPrev, to seed step 1b on the NEXT call.
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
norm_output_type yHist[na]                   = { Y_HIST_INIT };
norm_input_type  uHist[nb + nk - 1]          = { U_HIST_INIT };
/*
 * MUST be tied to U_HIST_INIT's operating point, not zero: a first
 * MADS call warm-started from a value inconsistent with the assumed
 * equilibrium encoded in Y_HIST_INIT/U_HIST_INIT can return a poor
 * first solution, and - unlike the old flat "uOptNorm[i] = uHist[0]"
 * warm start, which re-anchored to the REAL applied input every single
 * call - the shifted warm start below carries that solution's quality
 * forward via uOptPrev with no automatic reset to reality in between.
 * A bad cold start can therefore seed a persistent closed-loop problem
 * rather than a one-step transient (confirmed empirically: an earlier
 * all-zero default here caused floating-point simulation to oscillate
 * and never settle for the whole run, not just its first few samples).
 */
norm_input_type  uOptPrev[NhorU]             = { U_PREV_INIT };

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

	/*
	 * thetaCenter/thetaGens/yHist/uHist/uOptPrev are FILE-SCOPE globals
	 * (defined above with external linkage) rather than local arrays, so
	 * their memory partitioning must be declared here, inside the
	 * top-level function, where HLS elaborates the design's static
	 * storage. A pragma attached to the point of declaration outside a
	 * function has no effect.
	 *
	 * All five are read/written elementwise by fully-unrolled loops in
	 * computeArxOutput, generateScenarios, matDet/zonotopeVolume,
	 * boundStripZonotopeIntersectionNew (PL/AL mode), and - for uOptPrev -
	 * the warm-start shift/save-back loops below (Step 1b/7b). Complete
	 * partitioning turns each into individual registers instead of a
	 * single-/dual-port BRAM, which is required for those unrolled
	 * accesses to happen in parallel and is affordable here because
	 * nTheta, nGens, na, nb+nk-1 and NhorU are all <= 6 for every
	 * ACTIVE_SYSTEM.
	 */
	#pragma HLS ARRAY_PARTITION variable=thetaCenter complete dim=1
	#pragma HLS ARRAY_PARTITION variable=thetaGens   complete dim=0
	#pragma HLS ARRAY_PARTITION variable=yHist       complete dim=1
	#pragma HLS ARRAY_PARTITION variable=uHist       complete dim=1
	#pragma HLS ARRAY_PARTITION variable=uOptPrev    complete dim=1
	#endif
    /* ------------------------------------------------------------------ */
    /*  Step 1 - Convert digital inputs to algorithm types                */
    /* ------------------------------------------------------------------ */
    norm_output_type yCurrNorm;
    norm_output_type yrefNorm;
    digital_input_type uOptDig;

    yCurrNorm = dig2ctrlY(yCurrDig);
    yrefNorm = dig2ctrlY(yrefDig);

    #ifdef DEBUG_PRINT
    double center_f[nTheta], gens_f[nTheta][nGens];
    double yCurrDig_f, yrefDig_f;
    double yCurrNorm_f, yrefNorm_f;
    for(int i = 0; i < nTheta; i++) {
        center_f[i] = thetaCenter[i].to_double();
        for(int j = 0; j < nGens; j++) gens_f[i][j] = thetaGens[i][j].to_double();
    }
    yCurrDig_f = yCurrDig.to_double();
    yrefDig_f = yrefDig.to_double();
    yCurrNorm_f = yCurrNorm.to_double();
    yrefNorm_f = yrefNorm.to_double();
    #endif

    // #ifdef CONVERSIONS_MODE
    // output_type yCurr;
    // output_type yref;
    // yCurr = DAConvertY(yCurrDig);
    // yref  = DAConvertY(yrefDig);
    // #else 
    // yCurr = yCurrDig;
    // yref = yrefDig;
    // #endif

    // /* Normalize output samples if needed */
    // #ifdef NRMLZ
    // yCurrNorm = normalizeY(yCurr);
    // yrefNorm = normalizeY(yref);
    // #else
    // yCurrNorm = yCurr;
    // yrefNorm = yref;
    // #endif
    
    /* ------------------------------------------------------------------ */
    /*  Step 1b - Warm-start uOptNorm by shifting the previous solution   */
    /* ------------------------------------------------------------------ */
    /*
     * Previously this held uHist[0] constant across the whole horizon
     * ("freeze current input"), which is a poor guess whenever yref is
     * moving: MADS then spends its limited MADS_ITER budget fighting its
     * way away from a flat start instead of refining an already-good
     * trajectory, which is one of the reasons DELTAYNORM (the output-rate
     * constraint) gets violated on transients - see updateConstraintViolation.cpp.
     *
     * Instead, shift uOptPrev = u*(k-1), ..., u*(k-1+NhorU-1) - the full
     * sequence MADS returned on the previous call - by one step: entry i
     * becomes the previous call's entry i+1 (it was one step further into
     * the future, now it is one step closer), and the last entry is
     * repeated (zero-order hold past the end of the previous horizon,
     * the standard choice when no better prediction is available there).
     *
     * Edge case NhorU == 1 (nOpt == 1): the shift loop below has trip
     * count NhorU-1 == 0, so it never executes (no out-of-bounds access
     * to uOptPrev[i+1]), and the line after it degenerates to
     * uOptNorm[0] = uOptPrev[0], i.e. "reuse last step's only value" -
     * correct and in-bounds.
     */
    norm_input_type uOptNorm[NhorU];
    for (int i = 0; i < NhorU - 1; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        uOptNorm[i] = uOptPrev[i + 1];
    }
    uOptNorm[NhorU - 1] = uOptPrev[NhorU - 1]; // repeat last entry

    /* ------------------------------------------------------------------ */
    /*  Step 2 - [PL mode] Zonotope update                                */
    /* ------------------------------------------------------------------ */
    #if CTRL_MODE == CTRL_MODE_PL || CTRL_MODE == CTRL_MODE_AL
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
    #ifdef PRAGMAS
    /*
     * Declared here (their point of storage) rather than inside
     * boundStripZonotopeIntersectionNew: that function's own body
     * assigns to newCenter[i] and newGens[i][j] from fully-unrolled
     * loops (both the "changed" and "!changed" branches), so whichever
     * concrete array is bound to those parameters needs to support
     * concurrent, per-element writes. Complete partitioning is cheap
     * here (nTheta*nGens <= 36 elements across every ACTIVE_SYSTEM).
     */
    #pragma HLS ARRAY_PARTITION variable=newCenter complete dim=1
    #pragma HLS ARRAY_PARTITION variable=newGens   complete dim=0
    #endif

    /* In NRMLZ case, the inputs should be:
        * yCurr -> yCurr - yNormOffset - ([yHist, uHist] - offsets) * yNormGain * ((i < na) ? 1 : normGainsRatio) * Dg * c0
        * yHist, uHist -> ([yHist, uHist] - ioOffsets) * yNormGain * (i < na) ? 1 : normGainsRatio * Dg
        * the rest is the same
        In brief, phi should become (phi_norm-q)*my*Dm^-1*Dg
        */
    phi_type phi[nTheta];
    #ifdef PRAGMAS
    /*
     * boundStripZonotopeIntersectionNew reads phi via fully-unrolled
     * loops (the gproj computation), so - same reasoning as
     * newCenter/newGens above - phi is partitioned here, at its point of
     * declaration, rather than only on the callee's parameter.
     */
    #pragma HLS ARRAY_PARTITION variable=phi complete dim=1
    #endif
    strip_center_type stripCenter = yCurrNorm;
    #ifdef NRMLZ

    /* Compute \tilde{phi} - offsets */
    for(int i = 0; i < na; i++)
    {
        #ifdef PRAGMAS
        /* na<=3 independent elements: cheap to unroll outright, and
         * unrolling (unlike PIPELINE) makes no promise about how many
         * cycles the result takes, so it carries none of the II-Violation
         * risk documented in volumeZonotope.cpp / boundStripZonotope-
         * IntersectionNew - the scheduler is simply left free to place
         * this subtract in as many cycles as the target clock needs. */
        #pragma HLS UNROLL
        #endif
        phi[i] = (yHist[i] - yNormOffset);
    }
    for(int i = 0; i < nb; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        phi[i+na] = (uHist[i+nk-1] - uNormOffset);
    }

    /* Use (\tilde{phi} - offsets) to multiply by my*Dm^-1*c0 */
    stripCenter -= yNormOffset;
    for(int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        /*
         * This IS a genuine reduction (stripCenter accumulates across i),
         * unlike the elementwise loops above - but nTheta<=6 and the body
         * is a plain multiply+subtract (no division, unlike the matDet/
         * zonotopeVolume/boundStripZonotopeIntersectionNew loops fixed
         * elsewhere in this pass), so unrolling it into a short static
         * dependency chain is safe: there is no PIPELINE/II promise being
         * made here for HLS to fail to keep.
         */
        #pragma HLS UNROLL
        #endif
        stripCenter -= phi[i] * stripCoeffs.myInvDmc0[i];
    }

    /* Conclude phi computation */
    for(int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        phi[i] *= stripCoeffs.myInvDmDg[i];
    }

    #else /* Unnormalized case */
    /* The strip uses original, unnormalized I/O samples */
    for(int i = 0; i < na; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        phi[i] = yHist[i];
    }
    for(int i = 0; i < nb; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        phi[i+na] = uHist[i+nk-1];
    }

    #endif
    boundStripZonotopeIntersectionNew(stripCenter, phi, sigma,
                                    thetaCenter, thetaGens,
                                    newCenter, newGens);

    /* Copy result back into the shared global zonotope state. */
    for (int i = 0; i < nTheta; i++) {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        thetaCenter[i] = newCenter[i];
        for (int j = 0; j < nGens; j++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            thetaGens[i][j] = newGens[i][j];
        }
    }

    #ifdef DEBUG_PRINT
    for(int i = 0; i < nTheta; i++) {
        center_f[i] = thetaCenter[i].to_double();
        for(int j = 0; j < nGens; j++) gens_f[i][j] = thetaGens[i][j].to_double();
    }
    printf("\n--- PL: updated zonotope ---\n");
    printf("thetaCenter: ");
    for (int i = 0; i < nTheta; i++) printf("%f ", (double)thetaCenter[i]);
    printf("\nthetaGens:\n");
    for (int i = 0; i < nTheta; i++) {
        for (int j = 0; j < nGens; j++)
            printf("%8.6f ", (double)thetaGens[i][j]);
        printf("\n");
    }
    #endif
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
    #ifdef PRAGMAS
    /*
     * Declared here (its point of storage), not inside generateScenarios
     * or computeArxOutput. costFunctionArx's Nhor loop is pipelined below
     * with its inner Nscen loop unrolled (see costFunctionArx.cpp), so at
     * every pipeline stage all Nscen rows of thetaScenarios are read in
     * the same cycle: complete partitioning in both dimensions is what
     * makes that possible without BRAM port contention.
     */
    #pragma HLS ARRAY_PARTITION variable=thetaScenarios complete dim=0
    #endif
    #if defined(USE_SCENS_COST) || defined(USE_SCENS_CONSTR)
    generateScenarios(thetaScenarios, thetaCenter, thetaGens);
    #endif

    /* ------------------------------------------------------------------ */
    /*  Step 5 - Update output history: push y(k) into yHist             */
    /* ------------------------------------------------------------------ */
    for (int i = na - 1; i > 0; i--)
        yHist[i] = yHist[i - 1];
    yHist[0] = yCurrNorm;   /* yHist = [y(k), y(k−1), ..., y(k−na+1)] */

    /* ------------------------------------------------------------------ */
    /*  Step 6 - Snapshot input history for MADS                          */
    /* ------------------------------------------------------------------ */
    /*
     * uPast = [u(k−1), ..., u(k−nb−nk+1)] - needed by costFunctionArx to
     * roll out predictions for y(k+1), ..., y(k+N) alongside uOpt.
     * Must be taken BEFORE uHist is updated with uOpt[0] in step 9.
     */
    norm_input_type uPast[nb + nk - 2];
    for (int i = 0; i < nb + nk - 2; i++)
        uPast[i] = uHist[i];

    /* ------------------------------------------------------------------ */
    /*  Step 7 - Run MADS optimisation                                    */
    /* ------------------------------------------------------------------ */
    MADSARX(uOptNorm, uPast, yHist, yrefNorm, thetaScenarios);

    /* ------------------------------------------------------------------ */
    /*  Step 7b - Save this solution as next call's warm-start seed       */
    /* ------------------------------------------------------------------ */
    for (int i = 0; i < NhorU; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        uOptPrev[i] = uOptNorm[i];
    }

    /* ------------------------------------------------------------------ */
    /*  Step 8 - Update input history: push uOpt[0] into uHist           */
    /* ------------------------------------------------------------------ */
    for (int i = nb + nk - 2; i > 0; i--)
        uHist[i] = uHist[i - 1];
    uHist[0] = uOptNorm[0];   /* receding horizon: only u(k) is applied */
 
    uOptDig = ctrlU2dig(uOptNorm[0]);
    
    #ifdef DEBUG_PRINT
    double yHist_f[na], uHist_f[nb+nk-1];
    double uOptDig_f = uOptDig.to_double();
    double uOptNorm_f[NhorU];
    for(int i = 0; i < NhorU; i++) uOptNorm_f[i] = uOptNorm[i].to_double();
    for(int i = 0; i < na; i++) yHist_f[i] = yHist[i].to_double();
    for(int i = 0; i < nb+nk-1; i++) uHist_f[i] = uHist[i].to_double();
    #endif

    /* Denormalize output if NRMLZ is defined */
    // input_type uOpt;
    // #ifdef NRMLZ
    // uOpt = denormalizeU(uOptNorm[0]);
    // #else
    // uOpt = uOptNorm[0];
    // #endif
    
    /* ------------------------------------------------------------------ */
    /*  Step 9 - Convert uOpt back to digital                             */
    /* ------------------------------------------------------------------ */
    // #ifdef CONVERSIONS_MODE
    // uOptDig = ADConvertU(uOpt);
    // #else
    // uOptDig = uOpt;
    // #endif

    return uOptDig;
}
