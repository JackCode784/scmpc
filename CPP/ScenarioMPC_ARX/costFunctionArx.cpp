/**
 * @file  costFunctionArx.cpp
 * @brief MPC cost function evaluation for a SISO ARX system.
 *
 * COST STRUCTURE
 * --------------
 * The MPC objective over a prediction horizon of N steps is:
 *
 *   J = Σ_{k=0}^{N-2} [ u(k)·R·u(k)  +  Q·(ŷ_nom(k+1) − y_ref)²
 *                                      +  Q · scenariosContrib(k+1) ]
 *     +                  P·(ŷ_nom(N) − y_ref)²
 *     +                  Q · scenariosContrib(N)
 *
 * where:
 *   ŷ_nom(k+1)       — one-step prediction using the nominal parameter
 *                       vector thetaCenter (the current zonotope centre)
 *   scenariosContrib  — mean squared error over all Nscen uncertainty
 *                       scenarios, included only in PL/AL modes to penalise
 *                       poor performance across the uncertainty set
 *
 * The input cost term u·R·u is included for steps k = 0 … N−2 (i.e. the
 * last input u(k = N−1) is not penalised separately because the terminal
 * cost P already handles the final output).
 *
 * CONTROL vs PREDICTION HORIZON
 * ------------------------------
 * currU has nOpt = NhorU entries: u(0), …, u(NhorU−1).
 * For steps k ≥ NhorU the last applied input u(NhorU−1) is held constant
 * ("input blocking" / zero-order hold beyond the control horizon).
 * uSamples[0] is not re-assigned in the tail loop; it retains the value
 * set at k = NhorU−1, which is currU[NhorU−1].  This is correct.
 *
 * CONSTRAINT VIOLATION
 * ---------------------
 * cost[1] accumulates a penalty whenever a predicted output falls outside
 * [YMIN, YMAX] (output constraint) or an input falls outside [UMIN, UMAX]
 * (input constraint).  These are checked for every scenario and every step.
 *
 * HLS NOTES
 * ----------
 * • The outer loop (k = 0 … Nhor−1) has a static bound and is unrolled
 *   (#pragma HLS UNROLL) so that all N prediction steps are computed in
 *   parallel.  This trades area for latency.
 * • The inner scenario loop (l = 0 … Nscen−1) can similarly be unrolled.
 * • yPastCurr and uSamples are local rolling-window buffers; they should
 *   NOT be partitioned (they are written sequentially, not randomly).
 */

#include "setup.h"
#ifdef FIXED
  #include "hls_math.h"
#endif

void costFunctionArx(cost_type         cost           [2],
                     const output_type yPast          [na],
                     const input_type  currU          [nOpt],
                     const input_type  uPast          [nb + nk - 2],
                     const output_type yref,
                     const theta_type  thetaScenarios [Nscen][nTheta])
{
#ifdef PRAGMAS
    /* Unrolling the outer prediction loop allows HLS to compute all N steps
     * in parallel.  Remove if the area cost is too high.                    */
    // #pragma HLS INLINE
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
     * uSamples layout (length nb + nk − 1):
     *   uSamples[0]   = u(k)       ← set from currU[k] or held constant
     *   uSamples[1]   = u(k−1)
     *   ...
     *   uSamples[nb+nk−2] = u(k−nb−nk+1)  ← from uPast[nb+nk−3]
     *
     * Initial fill: uSamples[1..nb+nk−2] ← uPast[0..nb+nk−3].
     * uSamples[0] is set at the start of each prediction step.
     */
    output_type yPastCurr[na];
    input_type  uSamples [nb + nk - 1];

    for (int i = 0; i < na; i++)
        yPastCurr[i] = yPast[i];

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
        if (currU[i] < UMIN || currU[i] > UMAX)
        {
            cost[1] = 500;
            return;   /* early exit: no point computing a cost for an
                         infeasible input sequence                        */
        }
    }

    /* ------------------------------------------------------------------ */
    /*  Prediction loop  k = 0, 1, …, Nhor − 1                           */
    /* ------------------------------------------------------------------ */
    /*
     * All Nhor steps share the same structure; the only differences are:
     *   • k < NhorU   : uSamples[0] = currU[k]  (within control horizon)
     *   • k >= NhorU  : uSamples[0] unchanged    (zero-order hold)
     *   • k < Nhor−1  : terminal weight = Q      (stage cost)
     *   • k = Nhor−1  : terminal weight = P      (terminal cost, no input term)
     *
     * Encoding these as conditional expressions inside a single loop avoids
     * the three near-identical code blocks in the original and makes the
     * loop bounds statically known, which HLS requires for unrolling.
     */
    for (int k = 0; k < Nhor; k++)
    {
#ifdef PRAGMAS
        #pragma HLS UNROLL
#endif
        /* --- Set current input ---------------------------------------- */
        if (k < NhorU)
            uSamples[0] = currU[k];
        /* else: uSamples[0] retains currU[NhorU−1] from the previous step */

        /* --- Input cost term (all steps except the terminal one) ------- */
        if (k < Nhor - 1)
            cost[0] += uSamples[0] * R * uSamples[0];

        /* --- Scenario loop: constraint check + PL/AL cost contribution - */
        cost_type scenariosContrib = 0;

        for (int l = 0; l < Nscen; l++)
        {
#ifdef PRAGMAS
            // #pragma HLS UNROLL
#endif
            output_type yNext = computeArxOutput(yPastCurr, uSamples,
                                                  thetaScenarios[l]);

#if CTRL_MODE == CTRL_MODE_PL || CTRL_MODE == CTRL_MODE_AL
            err_type err_s = yNext - yref;
            scenariosContrib += err_s * err_s;
#endif
            updateConstraintViolation(cost, yNext);
        }

#if CTRL_MODE == CTRL_MODE_PL || CTRL_MODE == CTRL_MODE_AL
        /* Divide by Nscen to get the mean squared error across scenarios.
         * In software: floating-point division.
         * In hardware: right-shift by LOG2NSCEN (exact only if Nscen is a
         * power of 2, which is enforced by the static_assert in setup.h). */
  #ifndef FIXED
        scenariosContrib /= Nscen;
  #else
        scenariosContrib = scenariosContrib >> LOG2NSCEN;
  #endif
#endif

        /* --- Nominal prediction and output cost ----------------------- */
        output_type yNext_nom = computeArxOutput(yPastCurr, uSamples,
                                                   thetaCenter);
        err_type err_nom = yNext_nom - yref;

        /* Select stage weight Q or terminal weight P. */
        cost[0] += ((k < Nhor - 1) ? Q : P) * (err_nom * err_nom + scenariosContrib);

        /* --- Shift rolling-window buffers for next prediction step ---- */
        for (int i = na - 1; i > 0; i--)
            yPastCurr[i] = yPastCurr[i - 1];
        yPastCurr[0] = yNext_nom;

        for (int i = nb + nk - 2; i > 0; i--)
            uSamples[i] = uSamples[i - 1];
        /* uSamples[0] is set at the top of the next iteration            */
    }
}
