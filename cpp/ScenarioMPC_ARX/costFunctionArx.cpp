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
 *                       vector thetaCenter (the current zonotope centre),
 *                       via its effective coefficients arxCoeff[Nscen] /
 *                       arxOffset[Nscen] (rows 0..Nscen-1 are the
 *                       scenarios' - see computeArxCoeffs)
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
 * HLS NOTES (pragma profiles: setup.h, PRAGMA_PROFILE)
 * ----------
 * Structure. Per call: Nhor prediction steps k (a genuine recurrence:
 * step k+1 needs the outputs of step k), and in each step one "model loop"
 * over the Nscen scenarios plus the nominal model (row Nscen of arxCoeff,
 * arxOffset and yPastCurr). The models are independent of each other
 * within a step: each reads and writes only its own yPastCurr row. The
 * nominal model used to have its own copy of the prediction code after
 * the scenario loop; it is now the loop's last iteration (l = Nscen), so
 * there is a single prediction datapath, with the same additions in the
 * same order as before.
 *
 * Pragmas in this file:
 * - INLINE off on the function, every profile: it stays one RTL module,
 *   so the number of instances is set by MADSARX's ALLOCATION pragma (and
 *   not by automatic inlining, which would copy it into each call site).
 * - ARRAY_PARTITION complete on yPastCurr and uSamples, every profile:
 *   (Nscen+1)*na + nb+nk-1 registers, read and written by the unrolled
 *   shift loops and, in the model loop, one row per iteration.
 * - UNROLL on the small na/nb/nOpt/Nscen+1 copy and shift loops, every
 *   profile: they are wiring and registers, not arithmetic.
 * - k loop: PIPELINE off, every profile. Its recurrence (k+1 needs k)
 *   gives nothing to overlap, and an earlier attempt to pipeline it
 *   (which also unrolls the model loop inside) missed a 10 ns clock by 5x.
 * - model loop: LOOP_FLATTEN off, every profile (keeps it a separate loop
 *   inside the k loop; flattening the two would break the per-step
 *   structure the recurrence needs), and
 *     AREA:              PIPELINE off. One model after the other, each
 *                        finishing before the next starts: the least
 *                        hardware (no pipeline registers).
 *     BALANCED, LATENCY: PIPELINE II=1. A new model enters the same
 *                        datapath every clock cycle, so a step takes about
 *                        Nscen+1 cycles plus the pipeline depth instead of
 *                        (Nscen+1) times the full prediction latency. The
 *                        datapath is not duplicated. DEPENDENCE ... false
 *                        on yPastCurr tells the tool what is true here:
 *                        iterations of this loop never touch the same row.
 *                        The only loop-carried values are the two
 *                        accumulators, one addition each per iteration
 *                        (see addConstraintViolation); if a 10 ns clock
 *                        cannot fit that addition, the tool reports it and
 *                        uses II=2.
 * The poll evaluations that call this function, and therefore how many
 * copies of it run in parallel, are set in progressiveBarrierPollingArx.cpp
 * and MADSARX.cpp.
 */

#include "setup.h"

void costFunctionArx(cost_type              cost[2],
                     const norm_output_type yPast          [na],
                     const norm_input_type  currU          [nOpt],
                     const norm_input_type  uPast          [nb + nk - 2],
                     const norm_output_type yref,
                     const arx_coeff_type   arxCoeff       [Nscen+1][nTheta],
                     const arx_coeff_type   arxOffset      [Nscen+1])
{
    #ifdef PRAGMAS
    /* One RTL module in every profile (see HLS NOTES) */
    #pragma HLS INLINE off
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
    for(int i = 0; i < nOpt; i++) currU_f[i] = currU[i].to_double();
    #endif

    /* Accumulators for cost[0] (tracking + input cost) and cost[1]
     * (constraint violation), written to cost[] at the end: the model loop
     * then accumulates into registers rather than through the cost[]
     * argument. */
    cost_type costTrack = 0;
    cost_type costViol  = 0;

    /*
     * Per-model output history: yPastCurr[l] = [y_l(k-1), ..., y_l(k-na)]
     * for scenario l (l < Nscen) and the nominal model (l = Nscen).
     * uSamples = [u(k), u(k-1), ..., u(k-nb-nk+1)], shared by all models:
     *   uSamples[0]   = u(k)       <= set from currU[k] or held constant
     *   uSamples[1]   = u(k-1)
     *   ...
     */
    norm_output_type yPastCurr[Nscen+1][na];
    norm_input_type  uSamples [nb + nk - 1];
    #ifdef PRAGMAS
    #pragma HLS ARRAY_PARTITION variable=yPastCurr complete dim=0
    #pragma HLS ARRAY_PARTITION variable=uSamples  complete dim=1
    #endif

    for (int i = 0; i < Nscen+1; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        for(int j = 0; j < na; j++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            yPastCurr[i][j] = yPast[j];
        }
    }

    for (int i = 0; i < nb + nk - 2; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        uSamples[i + 1] = uPast[i];
    }

    /* --- Input bound check (cheap, done once) ------------------------ */
    for (int i = 0; i < nOpt; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        if (currU[i] < UNORMMIN || currU[i] > UNORMMAX)
        {
            costViol = 512; // power of two
        }
    }

    /* Rows the model loop runs over: the Nscen scenarios (only when they
     * are used) and the nominal model, row Nscen (always). */
    #if defined(USE_SCENS_COST) || defined(USE_SCENS_CONSTR)
    constexpr int firstRow = 0;
    #else
    constexpr int firstRow = Nscen;
    #endif

    /* ------------------------------------------------------------------ */
    /*  Prediction loop  k = 0, 1, ..., Nhor - 1                           */
    /* ------------------------------------------------------------------ */
    /*
     *   * k < NhorU   : uSamples[0] = currU[k]  (within control horizon)
     *   * k >= NhorU  : uSamples[0] unchanged    (zero-order hold)
     *   * k < Nhor-1  : stage weight outputWeight, plus the input cost
     *   * k = Nhor-1  : terminal weight terminalOutputWeight, no input cost
     */
    for (int k = 0; k < Nhor; k++)
    {
        #ifdef PRAGMAS
        /* Every profile: genuine recurrence, nothing to overlap (HLS NOTES) */
        #pragma HLS PIPELINE off
        #endif

        /* --- Set current input ---------------------------------------- */
        if (k < NhorU)
            uSamples[0] = currU[k];

        #ifdef DEBUG_PRINT
        for(int i = 0; i < Nscen+1; i++) for(int j = 0; j < na; j++) yPastCurr_f[i][j] = yPastCurr[i][j].to_double();
        for(int i = 0; i < nb+nk-1; i++) uSamples_f[i] = uSamples[i].to_double();
        input_cost_term_f = (uSamples_f[0] - uSamples_f[1]) * (uSamples_f[0] - uSamples_f[1]) * R.to_double();
        #endif

        /* --- Input cost term (all steps except the terminal one) ------- */
        /* --- Input term is difference with respect to previous sample -- */
        if (k < Nhor - 1){
            #ifndef FIXED
            costTrack += (uSamples[0] - uSamples[1]) * R * (uSamples[0] - uSamples[1]);
            #else
            /* Multiplied by R, not shifted by a log2R: R is a power of two
             * only for some systems (0.125 for the buck converters, where a
             * constant power-of-two multiply is a shift in hardware), but
             * 5.5556 for the inverted pendulum, where a shift would have
             * silently used a different weight than floating point. Also
             * exact where the former "<< -3" was not: an ap_fixed shift
             * keeps the operand's format, so shifting right dropped its 3
             * lowest bits (~1e-10, below cost_type's resolution, but enough
             * to flip rare roundings - BUCK_LOSS FIXED output is therefore
             * not bit-identical to before, with equivalent behavior). */
            costTrack += ((uSamples[0] - uSamples[1]) * (uSamples[0] - uSamples[1])) * R;
            #endif
        }

        /* --- Model loop: the scenarios, then the nominal model ---------- */
        cost_type        scenariosContrib = 0;
        norm_output_type yNext_nom        = 0;
        for (int l = firstRow; l < Nscen + 1; l++)
        {
            #ifdef PRAGMAS
            #pragma HLS LOOP_FLATTEN off
            #ifdef PRAGMA_PIPELINE_MODEL_LOOPS
            /* BALANCED, LATENCY: one model per clock cycle (HLS NOTES) */
            #pragma HLS PIPELINE II=1
            #pragma HLS DEPENDENCE variable=yPastCurr type=inter false
            #else
            /* AREA: one model at a time (HLS NOTES) */
            #pragma HLS PIPELINE off
            #endif
            #endif
            norm_output_type yNext = computeArxOutput(yPastCurr[l], uSamples,
                                                      arxCoeff[l], arxOffset[l]);

            #ifdef DEBUG_PRINT
            yNext_f = yNext.to_double();
            #endif

            if (l < Nscen)
            {
                #ifdef USE_SCENS_COST
                err_type err_s = yNext - yref;
                scenariosContrib += (err_s * err_s);
                #ifdef DEBUG_PRINT
                err_s_f = err_s.to_double();
                scenariosContrib_f = scenariosContrib.to_double();
                #endif
                #endif

                #ifdef USE_SCENS_CONSTR
                addConstraintViolation(costViol, yNext, yPastCurr[l][0]);
                #endif
            }
            else
            {
                /* Nominal model: always constrained */
                addConstraintViolation(costViol, yNext, yPastCurr[l][0]);
                yNext_nom = yNext;
            }

            /* Update this model's output samples */
            for(int i = na - 1; i > 0; i--)
            {
                #ifdef PRAGMAS
                /* register copies only - see init loops above */
                #pragma HLS UNROLL
                #endif
                yPastCurr[l][i] = yPastCurr[l][i-1];
            }
            yPastCurr[l][0] = yNext;
        }

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

        /* --- Nominal output cost ------------------------------------- */
        err_type err_nom = yNext_nom - yref;

        #ifdef DEBUG_PRINT
        scenariosContrib_f = scenariosContrib.to_double();
        err_nom_f = err_nom.to_double();
        output_cost_term_f = (err_nom_f * err_nom_f + scenariosContrib_f);
        output_cost_term_f *= (k < Nhor - 1) ? outputWeight.to_double() : terminalOutputWeight.to_double();
        #endif

        /* Select stage weight outputWeight or terminal weight terminalOutputWeight. */
        #ifndef FIXED
        costTrack += (((k < Nhor - 1) ? outputWeight : terminalOutputWeight) * (err_nom * err_nom + scenariosContrib));
        #else
        costTrack += (err_nom * err_nom + scenariosContrib) << ((k < Nhor - 1) ? log2Q : log2P);
        #endif

        #ifdef DEBUG_PRINT
        cost_f[0] = costTrack.to_double();
        cost_f[1] = costViol.to_double();
        #endif

        /* --- Shift the input samples for the next prediction step ------ */
        for (int i = nb + nk - 2; i > 0; i--)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            uSamples[i] = uSamples[i - 1];
        }
    }

    cost[0] = costTrack;
    cost[1] = costViol;
}
