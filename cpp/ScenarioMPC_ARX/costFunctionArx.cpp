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
 * - uSamples: ARRAY_PARTITION complete, every profile (nb+nk-1
 *   registers, shifted by an unrolled loop).
 * - yPastCurr: ARRAY_PARTITION complete on dim 2 only, plus BIND_STORAGE
 *   LUTRAM, every profile: na small RAMs (one per past sample) of Nscen+1
 *   words (one per model), so the model loop reads row l through a RAM
 *   address, not a multiplexer over registers. This is what lets the
 *   model loop pipeline: on registers, the tool cannot tell that
 *   iteration l+1 does not read the register iteration l just wrote, so
 *   it treated the whole prediction (row mux, ARX products, adder tree,
 *   error, accumulation) as a one-cycle recurrence - 21 ns on a 10 ns
 *   clock with Vitis HLS 2026.1 - and DEPENDENCE cannot override that for
 *   scalars. On a RAM it can (below). Each RAM sees one read and one write
 *   per iteration (ram_2p). The rows are not initialized: at k = 0 every
 *   model reads the measured history yPast instead (same values as the
 *   former copy loop, without Nscen+1 RAM writes).
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
 *                        The only loop-carried values are then the two
 *                        accumulators, written so that each is ONE plain
 *                        addition per iteration (FIXED; see
 *                        ScenAcc and ViolAcc below), with results
 *                        bit-identical to the saturating, rounding
 *                        cost_type additions they replace.
 * The poll evaluations that call this function, and therefore how many
 * copies of it run in parallel, are set in progressiveBarrierPollingArx.cpp
 * and MADSARX.cpp.
 */

#include "setup.h"

#ifdef FIXED
/*
 * Accumulators of the model loop (FIXED). A cost_type addition rounds
 * (AP_RND_CONV) and saturates (AP_SAT): an adder, a rounding incrementer
 * and a clamp in series. In the pipelined model loop that whole chain is
 * the loop-carried path, so it must fit one clock cycle for II=1. These
 * two accumulators give the same numbers with one plain adder in the loop.
 */

/* Constraint violation: costViol = (512 if an input is out of bounds) +
 * one constraintViolation() per model and step. Every term is >= 0 and
 * exact in cost_type, so the saturating running sum equals the exact sum
 * clamped once at the end, min(sum, cost_type max), whatever the order.
 * ViolAcc holds the exact sum: no clamp in the loop. Its integer bits
 * cover the largest possible sum - a term is below 2 * 2^iwidth of
 * norm_output_type (two differences of norm_output_type values). */
cost_type constraintViolation(const norm_output_type yCurr, const norm_output_type yPast);   /* updateConstraintViolation.cpp */
constexpr int ceilLog2(long long v) { return (v <= 1) ? 0 : 1 + ceilLog2((v + 1) / 2); }
constexpr long long VIOL_SUM_MAX = 512 + (long long)(Nscen + 1) * Nhor * (2LL << norm_output_type::iwidth);
constexpr int VIOL_ACC_I = ceilLog2(VIOL_SUM_MAX + 1);
typedef ap_ufixed<VIOL_ACC_I + cost_type::width - cost_type::iwidth, VIOL_ACC_I, AP_TRN, AP_WRAP> ViolAcc;

/* Scenario tracking error: scenariosContrib += err_s*err_s, rounded
 * (round half to even) to cost_type's grid at every addition. With the
 * running sum s on that grid, write the square as P + f: P its value
 * truncated to the grid and f in [0, 1 LSB) the dropped part. Then
 *   round(s + P + f) = s + P + inc,
 *   inc = 1 if f > LSB/2, or if f == LSB/2 and s + P is odd, else 0,
 * and the parity of s + P is lsb(s) XOR lsb(P). So P, P + 1 and the two
 * comparisons are computed from the square alone, outside the
 * recurrence; the loop keeps lsb(s) -> select -> one addition. No clamp:
 * the sum is below Nscen * (2^iwidth of err_type)^2 (static_assert). */
constexpr int SQ_FRAC   = 2 * (err_type::width - err_type::iwidth);
constexpr int COST_FRAC = cost_type::width - cost_type::iwidth;
constexpr int SQ_DROP   = SQ_FRAC - COST_FRAC;   /* bits rounded off per addition */
static_assert(SQ_DROP > 0, "ScenAcc assumes err_type^2 has more fractional bits than cost_type.");
static_assert((long long)Nscen << (2 * err_type::iwidth) < (1LL << cost_type::iwidth),
    "ScenAcc assumes the scenario error sum cannot saturate cost_type.");
struct ScenAcc
{
    ap_uint<cost_type::width> s = 0;   /* cost_type's bits */

    void add(const err_type err)
    {
        #ifdef PRAGMAS
        #pragma HLS INLINE
        #endif
        const ap_fixed<2 * err_type::width, 2 * err_type::iwidth> sq = err * err;   /* exact, >= 0 */
        const ap_uint<2 * err_type::width> raw = sq.range();
        const ap_uint<SQ_DROP> f = raw.range(SQ_DROP - 1, 0);
        const ap_uint<SQ_DROP> half = ap_uint<SQ_DROP>(1) << (SQ_DROP - 1);
        const ap_uint<cost_type::width> P  = raw >> SQ_DROP;
        const ap_uint<cost_type::width> P1 = P + 1;
        const bool gt = f > half, eq = f == half;
        /* loop-carried part */
        const bool inc = gt || (eq && (s[0] != P[0]));
        s += inc ? P1 : P;
    }

    cost_type value() const
    {
        cost_type v;
        v.range() = s;
        return v;
    }
};
#endif

void costFunctionArx(cost_type              cost[2],
                     const norm_output_type yPast          [na],
                     const norm_input_type  currU          [nOpt],
                     const norm_input_type  uPast          [nUKnown],
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
    #ifdef FIXED
    ViolAcc   costViol  = 0;   /* exact sum, clamped to cost_type at the end */
    #else
    cost_type costViol  = 0;
    #endif

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
    /* yPastCurr: na RAMs of Nscen+1 words, rows not initialized - the
     * first step reads yPast instead (HLS NOTES) */
    #pragma HLS ARRAY_PARTITION variable=yPastCurr complete dim=2
    #pragma HLS BIND_STORAGE    variable=yPastCurr type=ram_2p impl=lutram
    #pragma HLS ARRAY_PARTITION variable=uSamples  complete dim=1
    #endif

    /* Known inputs: uPast = [u(k-1), ...] without delay compensation, so
     * uSamples[0] = u(k) comes from currU at step 0; with it (DC_STEPS = 1,
     * setup.h) uPast = [u(k), u(k-1), ...] and u(k) is known: the decision
     * variables start at step DC_STEPS. */
    for (int i = 0; i < nUKnown; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        uSamples[i + 1 - DC_STEPS] = uPast[i];
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
     * With j = k - DC_STEPS (DC_STEPS = 1 with DELAY_COMPENSATION, whose
     * first step uses the known input u(k) already in uSamples[0]):
     *   * 0 <= j < NhorU : uSamples[0] = currU[j] (within control horizon)
     *   * j >= NhorU     : uSamples[0] unchanged  (zero-order hold)
     *   * k < Nhor-1  : stage weight outputWeight, plus the input cost
     *                   (not for a known input: k < DC_STEPS)
     *   * k = Nhor-1  : terminal weight terminalOutputWeight, no input cost
     */
    for (int k = 0; k < Nhor; k++)
    {
        #ifdef PRAGMAS
        /* Every profile: genuine recurrence, nothing to overlap (HLS NOTES) */
        #pragma HLS PIPELINE off
        #endif

        /* --- Set current input ---------------------------------------- */
        if (k >= DC_STEPS && k - DC_STEPS < NhorU)
            uSamples[0] = currU[k - DC_STEPS];

        #ifdef DEBUG_PRINT
        for(int i = 0; i < Nscen+1; i++) for(int j = 0; j < na; j++) yPastCurr_f[i][j] = (k == 0) ? yPast[j].to_double() : yPastCurr[i][j].to_double();
        for(int i = 0; i < nb+nk-1; i++) uSamples_f[i] = uSamples[i].to_double();
        input_cost_term_f = (uSamples_f[0] - uSamples_f[1]) * (uSamples_f[0] - uSamples_f[1]) * R.to_double();
        #endif

        /* --- Input cost term (all steps except the terminal one) ------- */
        /* --- Input term is difference with respect to previous sample -- */
        if (k >= DC_STEPS && k < Nhor - 1){
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
        #ifdef FIXED
        ScenAcc          scenAcc;
        #endif
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
            /* This model's past outputs: the measured ones at the first
             * step (every model starts from them), its own row after. */
            norm_output_type yRow[na];
            #ifdef PRAGMAS
            #pragma HLS ARRAY_PARTITION variable=yRow complete dim=1
            #endif
            for (int i = 0; i < na; i++)
            {
                #ifdef PRAGMAS
                #pragma HLS UNROLL
                #endif
                yRow[i] = (k == 0) ? yPast[i] : yPastCurr[l][i];
            }

            norm_output_type yNext = computeArxOutput(yRow, uSamples,
                                                      arxCoeff[l], arxOffset[l]);

            #ifdef DEBUG_PRINT
            yNext_f = yNext.to_double();
            #endif

            if (l < Nscen)
            {
                #ifdef USE_SCENS_COST
                err_type err_s = yNext - yref;
                #ifdef FIXED
                scenAcc.add(err_s);
                #else
                scenariosContrib += (err_s * err_s);
                #endif
                #ifdef DEBUG_PRINT
                err_s_f = err_s.to_double();
                scenariosContrib_f = scenAcc.value().to_double();
                #endif
                #endif

                #ifdef USE_SCENS_CONSTR
                #ifdef FIXED
                costViol += constraintViolation(yNext, yRow[0]);
                #else
                addConstraintViolation(costViol, yNext, yRow[0]);
                #endif
                #endif
            }
            else
            {
                /* Nominal model: always constrained */
                #ifdef FIXED
                costViol += constraintViolation(yNext, yRow[0]);
                #else
                addConstraintViolation(costViol, yNext, yRow[0]);
                #endif
                yNext_nom = yNext;
            }

            /* Update this model's output samples */
            for(int i = na - 1; i > 0; i--)
            {
                #ifdef PRAGMAS
                /* one write per column RAM (HLS NOTES) */
                #pragma HLS UNROLL
                #endif
                yPastCurr[l][i] = yRow[i-1];
            }
            yPastCurr[l][0] = yNext;
        }

        #if defined(FIXED) && defined(USE_SCENS_COST)
        scenariosContrib = scenAcc.value();
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
    cost[1] = costViol;   /* FIXED: clamps the exact sum to cost_type (AP_SAT) */
}
