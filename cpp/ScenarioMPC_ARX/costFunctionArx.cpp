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
 * • The outer loop (k = 0 ... Nhor-1) has a static bound and is unrolled
 *   (#pragma HLS UNROLL) so that all N prediction steps are computed in
 *   parallel.  This trades area for latency.
 * • The inner scenario loop (l = 0 ... Nscen-1) can similarly be unrolled.
 * • yPastCurr and uSamples are local rolling-window buffers; they should
 *   NOT be partitioned (they are written sequentially, not randomly).
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
     * Both are read/written by the k-loop below, which is pipelined with
     * its inner scenario loop (l) fully unrolled: every pipeline stage
     * therefore needs concurrent access to all Nscen+1 rows of yPastCurr
     * and to every element of uSamples. Complete partitioning turns them
     * into individual registers (max (Nscen+1)*na = 10 and nb+nk-1 = 2
     * elements for this design), removing the BRAM-port limit that would
     * otherwise force the schedule back to one row/element per cycle.
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
        if (currU[i] < UNORMMIN || currU[i] > UNORMMAX)
        {
            cost[1] = 512; // power of two
            break;   /* break instead of return because in barrier polling cost[0]
                        is still used for determining optimal variable even if
                        cost[1] > 0 */
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
        #ifdef PRAGMAS
        /*
         * PIPELINE, not UNROLL: yPastCurr/uSamples make this a genuine
         * recurrence (step k+1 rolls forward the state written at step
         * k), so there is no independent work to unroll into parallel
         * copies - unrolling would just duplicate computeArxOutput's
         * datapath Nhor times for a chain that must still execute in
         * order. Pipelining reuses ONE copy of that datapath and starts
         * a new k-iteration every II cycles while the previous one is
         * still draining, which gives the same steady-state throughput
         * as unrolling without the extra area.
         *
         * Applying PIPELINE to this loop also makes Vitis HLS unroll the
         * inner Nscen loop (l) automatically, since pipelining requires
         * a single flattened loop body; it is unrolled explicitly below
         * anyway so the intent does not depend on that default.
         */
        #pragma HLS PIPELINE II=1
        #endif
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
            #ifdef PRAGMAS
            /* Scenarios are mutually independent within one prediction
             * step k (each reads only its own yPastCurr[l] row): safe,
             * and required by the outer PIPELINE, to unroll into Nscen
             * parallel computeArxOutput + updateConstraintViolation
             * instances (both are marked INLINE, so this is what
             * actually gets replicated Nscen-fold, not a function call). */
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
