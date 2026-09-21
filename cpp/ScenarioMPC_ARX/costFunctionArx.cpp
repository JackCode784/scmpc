/**
 * @file  costFunctionArx.cpp
 * @brief MPC cost function evaluation for a SISO ARX system.
 *
 * COST STRUCTURE
 * --------------
 * The MPC objective over a prediction horizon of N steps is:
 *
 *   J = sum_{k=0}^{N-2} [ u(k)*R*u(k)  +  outputWeight*(ypred_nom(k+1) - y_ref)^2
 *                                      +  outputWeight * scenariosContrib(k+1) ]
 *     +                  terminalOutputWeight*(ypred_nom(N) - y_ref)^2
 *     +                  outputWeight * scenariosContrib(N)
 *
 * where:
 *   ypred_nom(k+1)       - one-step prediction using the nominal parameter
 *                       vector thetaCenter (the current zonotope centre)
 *   scenariosContrib  - mean squared error over all Nscen uncertainty
 *                       scenarios, included only in PL/AL modes to penalise
 *                       poor performance across the uncertainty set
 *
 * The input cost term u*R*u is included for steps k = 0 ... N-2 (i.e. the
 * last input u(k = N-1) is not penalised separately because the terminal
 * cost terminalOutputWeight already handles the final output).
 *
 * CONTROL vs PREDICTION HORIZON
 * ------------------------------
 * currU has nOpt = NhorU entries: u(0), ..., u(NhorU-1).
 * For steps k >= NhorU the last applied input u(NhorU-1) is held constant
 * ("input blocking" / zero-order hold beyond the control horizon).
 * uSamples[0] is not re-assigned in the tail loop; it retains the value
 * set at k = NhorU-1, which is currU[NhorU-1].  This is correct.
 *
 * CONSTRAINT VIOLATION
 * ---------------------
 * cost[1] accumulates a penalty whenever a predicted output falls outside
 * [YMIN, YMAX] (output constraint) or an input falls outside [UMIN, UMAX]
 * (input constraint).  These are checked for every scenario and every step.
 *
 * HLS NOTES
 * ----------
 * • Whether the outer prediction loop (k = 0 ... Nhor-1) and/or the inner
 *   scenario loop (l = 0 ... Nscen-1) are UNROLLED - fully, partially, or
 *   not at all - and whether an ALLOCATION limit constrains how many
 *   multiplier instances the unrolled arithmetic may use, is controlled
 *   by PRAGMA_PROFILE in setup.h (PRAGMA_UNROLL_COSTFUNC_NHOR / _NSCEN,
 *   PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR, PRAGMA_LIMIT_COSTFUNC_MUL /
 *   PRAGMA_LATENCY_MUL_LIMIT below) - see that macro's comment for the
 *   measured area/latency numbers behind each profile. In short:
 *   fully unrolling both loops meets a 20kHz/50us controller() deadline
 *   (4151 cycles measured) but pushes LUT utilisation to ~100.8%
 *   (53614/53200) with no margin left for CTRL_MODE_PL or system
 *   integration; leaving both fully rolled uses ~37% LUT but is ~4.5x too
 *   slow for that deadline (~18700 cycles measured). Fully unrolling
 *   Nscen alone lands at 6603 cycles / 50% LUT - short of the deadline
 *   (~15.1kHz) but with a far better LUT-per-cycle-saved ratio than also
 *   fully unrolling Nhor, since Nhor is a genuine recurrence (unrolling
 *   it removes loop-FSM overhead more than it creates real parallel
 *   work) while Nscen's 4 scenarios are truly independent. Nhor's PARTIAL
 *   unroll factor exists to search the middle ground between fully
 *   rolled and fully unrolled at a finer grain than the three PRAGMA_PROFILE
 *   options alone provide. Neither loop is ever PIPELINEd: a PIPELINE
 *   II=1 + UNROLL combination was tried here once
 *   and reverted after synthesis reported a ~5x timing violation
 *   (Estimated 50.233 ns vs a 10 ns target) - a MANDATORY II=1 gave the
 *   scheduler almost no freedom to insert pipeline registers inside a
 *   loop that also carries a real recurrence (yPastCurr/uSamples feed
 *   from k into k+1). UNROLL does not carry that risk: it makes no
 *   scheduling promise for HLS to fail to keep, it just removes the
 *   loop's own FSM-transition overhead (and, for the l-loop, the forced
 *   one-scenario-at-a-time serialisation) and lets the ordinary list
 *   scheduler pack the resulting straight-line code into however many
 *   cycles the target clock period actually needs. The Nscen=4 scenarios
 *   are mutually independent within one k (each reads only its own
 *   yPastCurr[l] row), so unrolling the l-loop lets the scheduler
 *   evaluate them concurrently; only the k-recurrence and the l-loop's
 *   own reduction into scenariosContrib remain genuine dependency chains
 *   regardless of which loops are unrolled. Re-verify timing after
 *   synthesis whenever PRAGMA_PROFILE changes - unrolling removes the II
 *   promise but not the need to check the achieved combinational depth.
 * • yPastCurr and uSamples are unconditionally completely partitioned
 *   (see their declaration below), independent of PRAGMA_PROFILE: with
 *   either loop unrolled, every element of both arrays may need to be
 *   read/written in the same logical step, which a BRAM-backed array
 *   cannot support, so complete partitioning is required whenever
 *   PRAGMA_UNROLL_COSTFUNC_NHOR or _NSCEN is defined - and it costs
 *   essentially nothing ((Nscen+1)*na = 10 and nb+nk-1 = 2 elements for
 *   this design) when neither is, so there is no reason to also gate it.
 */

#include "setup.h"

void costFunctionArx(cost_type              cost[2],
                     const norm_output_type yPast          [na],
                     const norm_input_type  currU          [nOpt],
                     const norm_input_type  uPast          [nb + nk - 2],
                     const norm_output_type yref,
                     const theta_type       thetaScenarios [Nscen][nTheta])
{
    #ifdef PRAGMAS
    /*
     * NOT inlined: progressiveBarrierPollingArx calls this function
     * 2*nOpt times per MADS iteration (2*nOpt*MADS_ITER = 42 times per
     * controller() call). Keeping it a separate module means HLS builds
     * ONE instance of its datapath and reuses it sequentially across all
     * 42 calls; inlining would duplicate that datapath at every call
     * site instead (42x the LUTs/DSPs for identical arithmetic).
     */
    // #pragma HLS INLINE

    #ifdef PRAGMA_LIMIT_COSTFUNC_MUL
    /*
     * PRAGMA_PROFILE_LATENCY_SHARED only, and only when
     * PRAGMA_ENABLE_LATENCY_MUL_LIMIT (setup.h) is explicitly defined,
     * which it is NOT by default: capping costFunctionArx's multiplier
     * instances this way measured COUNTERPRODUCTIVE for the LUT problem
     * it was meant to fix - DSP dropped as intended (151 -> 91) but LUT
     * went UP (53614 -> 54821, 100.8% -> 103%), not down, because a
     * DSP48E1 slice is LUT-free dedicated silicon and forcing several
     * multiplies to time-share fewer of them costs LUT-built steering
     * multiplexers that here outweighed the saved DSP instances. See
     * setup.h's PRAGMA_PROFILE_LATENCY_SHARED and
     * PRAGMA_ENABLE_LATENCY_MUL_LIMIT comments for the full numbers and
     * reasoning, and for the corrected (opposite-direction) approach:
     * push fabric-mapped multiplies onto spare DSP48 slots via BIND_OP,
     * rather than restricting DSP-mapped ones via ALLOCATION.
     * Same ALLOCATION idiom already used in cpp/MADS_ISCAS23/admm.cpp,
     * for reference. HLS_PRAGMA (setup.h) is required, not a
     * plain #pragma, for the same macro-expansion reason documented on
     * the UNROLL factor= pragma below.
     */
    HLS_PRAGMA(HLS ALLOCATION operation instances=mul limit=PRAGMA_LATENCY_MUL_LIMIT)
    #endif
    #endif

    #ifdef DEBUG_PRINT
    double cost_f[2];
    double yPastCurr_f[Nscen+1][na], uSamples_f[nb+nk-1];
    double currU_f[nOpt], yNext_f;
    double input_cost_term_f, output_cost_term_f, scenariosContrib_f;
    double err_nom_f, err_s_f;
    double outputWeight_f = outputWeight.to_double();
    double terminalOutputWeight_f = terminalOutputWeight.to_double();
    double yref_f = yref.to_double();
    double center_f[nTheta];
    for(int i = 0; i < nTheta; i++) center_f[i] = thetaCenter[i].to_double();
    for(int i = 0; i < nOpt; i++) currU_f[i] = currU[i].to_double();
    #endif

    /* ------------------------------------------------------------------ */
    /*  Initialise cost accumulators                                       */
    /* ------------------------------------------------------------------ */
    cost[0] = 0;
    cost[1] = 0;

    /* ------------------------------------------------------------------ */
    /*  Local rolling-window state buffers                                 */
    /* ------------------------------------------------------------------ */
    /*
     * yPastCurr and uSamples are local copies that are shifted forward at
     * each prediction step, simulating the recurrence of the ARX model.
     *
     * uSamples layout (length nb + nk - 1):
     *   uSamples[0]   = u(k)       <= set from currU[k] or held constant
     *   uSamples[1]   = u(k-1)
     *   ...
     *   uSamples[nb+nk-2] = u(k-nb-nk+1)  <= from uPast[nb+nk-3]
     *
     * Initial fill: uSamples[1..nb+nk-2] <= uPast[0..nb+nk-3].
     * uSamples[0] is set at the start of each prediction step.
     */
    norm_output_type yPastCurr[Nscen+1][na];
    norm_input_type  uSamples [nb + nk - 1];

    #ifdef PRAGMAS
    /*
     * Both loops that index these arrays (k below, and l inside it) are
     * fully unrolled, so every row of yPastCurr and every element of
     * uSamples needs to be readable/writable in the same logical step.
     * Complete partitioning turns them into individual registers (max
     * (Nscen+1)*na = 10 and nb+nk-1 = 2 elements for this design) instead
     * of a small dual-port BRAM, which could not support that access
     * pattern.
     */
    #pragma HLS ARRAY_PARTITION variable=yPastCurr complete dim=0
    #pragma HLS ARRAY_PARTITION variable=uSamples  complete dim=1
    #endif

    for (int i = 0; i < Nscen+1; i++)
        for(int j = 0; j < na; j++)
            yPastCurr[i][j] = yPast[j];

    for (int i = 0; i < nb + nk - 2; i++)
        uSamples[i + 1] = uPast[i];

    /* ------------------------------------------------------------------ */
    /*  Input constraint check                                             */
    /* ------------------------------------------------------------------ */
    /*
     * Check all nOpt input values upfront.  If any violates [UMIN, UMAX]
     * we set cost[1] = 500 and stop immediately (the point is infeasible
     * regardless of what the output constraint check finds later).
     */
    for (int i = 0; i < nOpt; i++)
    {
        #ifdef PRAGMAS
        /*
         * UNROLL: safe now that the break below is gone. `cost[1] = 512`
         * is idempotent - checking every i and re-assigning the same
         * value whenever any of them violates the bound gives exactly
         * the same final cost[1] as stopping at the first violation did,
         * without a data-dependent trip count blocking full unrolling.
         */
        #pragma HLS UNROLL
        #endif
        if (currU[i] < UNORMMIN || currU[i] > UNORMMAX)
        {
            cost[1] = 512; // power of two
            /* No break: see comment above - stopping early bought nothing
             * but a variable-latency loop, since cost[0] is still needed
             * regardless of cost[1] (see progressiveBarrierPollingArx). */
        }
    }

    /* ------------------------------------------------------------------ */
    /*  Prediction loop  k = 0, 1, ..., Nhor - 1                           */
    /* ------------------------------------------------------------------ */
    /*
     * All Nhor steps share the same structure; the only differences are:
     *   * k < NhorU   : uSamples[0] = currU[k]  (within control horizon)
     *   * k >= NhorU  : uSamples[0] unchanged    (zero-order hold)
     *   * k < Nhor-1  : terminal weight = outputWeight      (stage cost)
     *   * k = Nhor-1  : terminal weight = terminalOutputWeight      (terminal cost, no input term)
     *
     * Encoding these as conditional expressions inside a single loop avoids
     * the three near-identical code blocks in the original and makes the
     * loop bounds statically known, which HLS requires for unrolling.
     */
    for (int k = 0; k < Nhor; k++)
    {
        /*
         * Outer #ifdef PRAGMAS is load-bearing here, unlike the plain
         * #ifdef PRAGMA_UNROLL_COSTFUNC_NHOR used elsewhere in this file:
         * PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR (setup.h) is defined
         * unconditionally (even for floating-point/software-simulation
         * builds where FIXED is undefined), so without this wrapper the
         * #elif below would still evaluate true in that mode and emit an
         * HLS pragma into a PC build - harmless per the C++ standard
         * (unrecognized pragmas are ignored), but inconsistent with
         * every other pragma in this codebase never appearing outside a
         * FIXED+PRAGMAS build.
         */
        #ifdef PRAGMAS
        #ifdef PRAGMA_UNROLL_COSTFUNC_NHOR
        /*
         * FULL UNROLL, not PIPELINE (see this file's HLS NOTES docstring
         * for why PIPELINE II=1 previously failed here and why UNROLL
         * does not carry the same risk). This IS a genuine recurrence
         * (yPastCurr/uSamples feed from k into k+1), so unrolling does
         * not create Nhor independent parallel copies of the work - it
         * turns the loop into Nhor sequential blocks of straight-line
         * code that the scheduler is free to pack as tightly as the
         * clock period allows, and to overlap with neighbouring blocks
         * wherever the true data dependencies permit. Only defined for
         * PRAGMA_PROFILE_LATENCY (setup.h) - the profile that measured
         * ~100.8% LUT utilisation for this trade.
         */
        #pragma HLS UNROLL
        #elif PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR > 1
        /*
         * PARTIAL unroll (PRAGMA_PROFILE_BALANCED only, factor > 1 - see
         * setup.h). MEASURED WORSE than leaving this loop fully rolled
         * for every factor tried (2, 3): more cycles AND more LUT than
         * factor=1, and the factor is left defined at 1 (i.e. this
         * branch is dead in practice) for exactly that reason - see
         * setup.h's PRAGMA_PROFILE and PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR
         * comments for the numbers and the likely cause (fusing N copies
         * of a body that already contains Nscen fully unrolled creates a
         * larger resource-allocation problem that Vitis's list-scheduler
         * handles worse, not better). Kept, rather than deleted, in case
         * costFunctionArx's structure changes enough later to revisit it.
         *
         * HLS_PRAGMA (setup.h) is required here, not a plain #pragma:
         * Vitis's pragma-argument parser does not macro-expand the
         * factor= field the way the #elif condition above (checked by
         * the ordinary C preprocessor) does - see setup.h's "HLS pragma
         * helper" comment for the full explanation and the exact error
         * this produced when this was a plain #pragma line.
         */
        HLS_PRAGMA(HLS UNROLL factor=PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR)
        #endif
        #endif
        /* If neither branch applies (PRAGMA_PROFILE_AREA, or BALANCED
         * with the factor left at 1, which is its current and only
         * sensible value - see above), the loop is left fully rolled. */
        /* --- Set current input ---------------------------------------- */
        if (k < NhorU)
            uSamples[0] = currU[k];
        /* else: uSamples[0] retains currU[NhorU-1] from the previous step */
        
        #ifdef DEBUG_PRINT
        for(int i = 0; i < Nscen; i++) for(int j = 0; j < na; j++) yPastCurr_f[i][j] = yPastCurr[i][j].to_double();
        for(int i = 0; i < nb+nk-2; i++) uSamples_f[i] = uSamples[i].to_double();
        input_cost_term_f = (uSamples_f[0] - uSamples_f[1]) * (uSamples_f[0] - uSamples_f[1]) * R.to_double();
        #endif

        /* --- Input cost term (all steps except the terminal one) ------- */
        /* --- Input term is difference with respect to previous sample -- */
        if (k < Nhor - 1){
            #ifndef FIXED
            cost[0] += (uSamples[0] - uSamples[1]) * R * (uSamples[0] - uSamples[1]);
            #else
            cost[0] += ((uSamples[0] - uSamples[1]) * (uSamples[0] - uSamples[1])) << log2R;
            #endif
        }
        #ifdef DEBUG_PRINT
        cost_f[0] = cost[0].to_double();
        #endif
        /* --- Scenario loop: constraint check + PL/AL cost contribution - */
        cost_type scenariosContrib = 0;

        #if defined(USE_SCENS_COST) || defined(USE_SCENS_CONSTR)
        for (int l = 0; l < Nscen; l++)
        {
            #ifdef PRAGMA_UNROLL_COSTFUNC_NSCEN
            /*
             * UNROLL: scenarios are mutually independent within one k
             * (each reads only its own yPastCurr[l] row and writes only
             * to the shared scenariosContrib accumulator, a genuine but
             * shallow - log2(Nscen)=2 levels - reduction), so the
             * scheduler can evaluate all Nscen copies of
             * computeArxOutput + updateConstraintViolation concurrently
             * subject only to that reduction and to the target clock
             * period. Defined for both PRAGMA_PROFILE_LATENCY and
             * PRAGMA_PROFILE_BALANCED (setup.h): this loop's Nscen-way
             * parallelism is expected to capture most of the latency win
             * on its own, since (unlike the k-loop above) there is no
             * true recurrence forcing serialisation across l.
             */
            #pragma HLS UNROLL
            #endif
            norm_output_type yNext = computeArxOutput(yPastCurr[l], uSamples,
                                                  thetaScenarios[l]);

            #ifdef DEBUG_PRINT
            yNext_f = yNext.to_double();
            #endif

            #ifdef USE_SCENS_COST
            err_type err_s = yNext - yref;
            scenariosContrib += (err_s * err_s);
            #endif

            #ifdef DEBUG_PRINT
            err_s_f = err_s.to_double();
            scenariosContrib_f = scenariosContrib.to_double();
            #endif
            
            #ifdef USE_SCENS_CONSTR
            updateConstraintViolation(cost, yNext, yPastCurr[l][0]);
            #endif

            #ifdef DEBUG_PRINT
            cost_f[1] = cost[1].to_double();
            #endif
        }
        #endif

        #ifdef USE_SCENS_COST
        /* Divide by Nscen to get the mean squared error across scenarios.
         * In software: floating-point division.
         * In hardware: right-shift by LOG2NSCEN (exact only if Nscen is a
         * power of 2, which is enforced by the static_assert in setup.h). */
        #ifndef FIXED
        scenariosContrib /= Nscen;
        #else
        scenariosContrib >>= LOG2NSCEN;
        #endif
        #endif

        #ifdef DEBUG_PRINT
        scenariosContrib_f = scenariosContrib.to_double();
        #endif

        /* --- Nominal prediction and output cost ----------------------- */
        norm_output_type yNext_nom = computeArxOutput(yPastCurr[Nscen], uSamples,
                                                   thetaCenter);
        updateConstraintViolation(cost, yNext_nom, yPastCurr[Nscen][0]);
        err_type err_nom = yNext_nom - yref;

        #ifdef DEBUG_PRINT
        yNext_f = yNext_nom.to_double();
        err_nom_f = err_nom.to_double();
        output_cost_term_f = (err_nom_f * err_nom_f + scenariosContrib_f);
        output_cost_term_f *= (k < Nhor - 1) ? outputWeight.to_double() : terminalOutputWeight.to_double();
        #endif

        /* Select stage weight outputWeight or terminal weight terminalOutputWeight. */
        #ifndef FIXED
        cost[0] += (((k < Nhor - 1) ? outputWeight : terminalOutputWeight) * (err_nom * err_nom + scenariosContrib));
        #else
        cost[0] += (err_nom * err_nom + scenariosContrib) << ((k < Nhor - 1) ? log2Q : log2P);
        #endif

        #ifdef DEBUG_PRINT
        cost_f[0] = cost[0].to_double();        
        #endif

        /* --- Shift rolling-window buffers for next prediction step ---- */
        for (int i = na - 1; i > 0; i--)
            yPastCurr[Nscen][i] = yPastCurr[Nscen][i - 1];
        yPastCurr[Nscen][0] = yNext_nom;

        for (int i = nb + nk - 2; i > 0; i--)
            uSamples[i] = uSamples[i - 1];
        /* uSamples[0] is set at the top of the next iteration            */
    }
}
