/**
 * @file  setup.h
 * @brief Single-knob experiment configuration for SCMPC on ARX systems.
 *
 * HOW TO CONFIGURE AN EXPERIMENT
 * --------------------------------
 * 1. Set ACTIVE_SYSTEM to the desired plant         (one #define below).
 * 2. Set CTRL_MODE to the desired algorithm mode  (one #define below).
 * 3. Comment / uncomment FIXED for HW vs SW target  (one #define below).
 * Everything else (dimensions, bounds, ADC/DAC gains, prototypes) is
 * derived automatically.
 *
 * FILE RELATIONSHIPS
 * ------------------
 *   types.h         - data-type aliases (included first)
 *   system_configs.h- system definitions
 *   setup.h         - this file; algorithm-level constants and prototypes
 *
 * VITIS HLS 2021.1 COMPATIBILITY NOTES
 * --------------------------------------
 * * "constexpr int" is fully supported (C++14 mode).  Used for all
 *   integer algorithm parameters (Nhor, NhorU, etc.).
 * * "static const <type>" is used for fixed-point constants because
 *   ap_fixed<> does not have a constexpr constructor.
 * * na, nb, nk, nTheta, nOpt remain #define macros because array sizes
 *   in function prototypes  (e.g. theta[nTheta])  must be integral
 *   constant expressions visible at the point of declaration, and #define
 *   guarantees this across every C++ standard and every HLS front-end.
 * * static_assert is used for invariant checks; it produces a clean
 *   compile error and has zero RTL impact.
 * * All mutable global state (zonotope centre/generators, output/input
 *   history) is DEFINED in controller.cpp (external linkage - no "static"
 *   keyword) and DECLARED here as "extern".  Any .cpp file that includes
 *   setup.h therefore refers to the same, single, physical storage as
 *   controller.cpp.  This is the standard C/C++ idiom for shared mutable
 *   globals: one definition, one or more extern declarations.
 *   Using "static" in controller.cpp would give each translation unit its
 *   own private copy - the opposite of what is required here.
 */

#pragma once

/* ======================================================================
   HLS pragma helper - macro-argument pragmas
   ======================================================================
   #if/#elif/#define are handled by the ORDINARY C preprocessor, which
   unconditionally macro-expands every identifier in them - this is why
   e.g. `#if PRAGMA_PROFILE == PRAGMA_PROFILE_LATENCY` below works exactly
   as expected. `#pragma HLS ...` lines are different: Vitis HLS 2021.1
   reads their argument text with its OWN, separate pragma-argument
   parser, which does NOT perform the same macro substitution for at
   least numeric fields like ARRAY_PARTITION's `factor=` or UNROLL's
   `factor=` - it expects a literal integer or a real C++ identifier
   there, not a preprocessor macro, and fails with something like
   "use of undeclared identifier <MACRO_NAME>" if given one directly.
   (Confirmed the hard way: `#pragma HLS UNROLL factor=SOME_MACRO` failed
   this way in costFunctionArx.cpp, one line after an #elif that checked
   the very same macro and worked fine - two different parsers.)

   HLS_PRAGMA(...) below is the standard, portable fix, not a Vitis-
   specific workaround: it uses the C99/C++11 _Pragma() OPERATOR (as
   opposed to the #pragma DIRECTIVE) together with the classic two-level
   stringize trick (STR/XSTR). Because the argument to HLS_PRAGMA_XSTR
   is used through an intermediate macro rather than stringized directly,
   the ordinary preprocessor fully macro-expands it BEFORE stringizing,
   so Vitis's pragma-argument parser only ever sees the already-resolved
   literal text (e.g. "HLS UNROLL factor=2"), never the macro name.
   Usage:  HLS_PRAGMA(HLS UNROLL factor=SOME_INT_MACRO)
   in place of:  #pragma HLS UNROLL factor=SOME_INT_MACRO   (broken)
   Only needed when a pragma argument is itself a #define'd macro; a
   pragma written entirely with literal tokens (as almost all of them are
   in this codebase) needs no help and should stay a plain #pragma line.
   ====================================================================== */
#define HLS_PRAGMA_XSTR(x) #x
#define HLS_PRAGMA_STR(x)  HLS_PRAGMA_XSTR(x)
#define HLS_PRAGMA(x)      _Pragma(HLS_PRAGMA_STR(x))

/* ======================================================================
   TARGET SELECTION - edit only these three lines between experiments
   ====================================================================== */

/** Hardware synthesis target.  Comment out for PC simulation. */
//   #define FIXED            /* fixed point representation */
// #define CONVERSIONS_MODE /* ADC/DAC conversions */
// #define NRMLZ               /* normalization */
#define USE_SCENS_COST      /* Enable scenarios cost contribution */
#define USE_SCENS_CONSTR    /* Enable scenarios constraints contribution */
// #define DELAY_COMPENSATION  /* computational delay compensation (see below) */
// #define PRNG_STDLIB         /* use rand() as prng */

/** Active plant - choose one of:
 * SYSTEM_SIMPLE,
 * SYSTEM_BENCHMARK,
 * SYSTEM_MILANO,
 * SYSTEM_BUCK,
 * SYSTEM_BUCK_LOSS,
 * SYSTEM_BUCK_ALBERTO,
 * SYSTEM_GAIN_DEMO  - "is scenario MPC worth it" demo; also comment out
 *                     NRMLZ and CONVERSIONS_MODE for this one specifically
 *                     (see system_configs.h's SYSTEM_GAIN_DEMO comment) */
#define ACTIVE_SYSTEM   SYSTEM_INVERTED_PENDULUM

/**
 * Controller algorithm mode - choose one of:
 *   CTRL_MODE_SCMPC  : scenario-based robust MPC only (no online learning)
 *   CTRL_MODE_PL     : passive learning  - update uncertainty set from data
 *   CTRL_MODE_AL     : active learning   - actively minimizes uncertainty set size
 */
#define CTRL_MODE_SCMPC  0
#define CTRL_MODE_PL     1
#define CTRL_MODE_AL     2
#define CTRL_MODE        CTRL_MODE_SCMPC

/**
 * HLS pragma profile (Vitis HLS 2026.1, xc7z020-clg484-1, 10 ns clock).
 *
 * Where the time goes. controller() runs MADS: 1 + MADS_ITER*2*nOpt cost
 * evaluations (61 for the pendulum: Nhor 15, NhorU 3, Nscen 16,
 * MADS_ITER 10). Each evaluation (costFunctionArx) runs Nhor prediction
 * steps, a recurrence, and in each step one prediction per model: the
 * Nscen scenarios plus the nominal model, independent of each other. Every
 * other part of controller() runs once per call and is small. So the
 * profiles differ in exactly two places: how the Nscen+1 models of a step
 * share a prediction datapath, and how the 2*nOpt poll evaluations of a
 * MADS iteration share costFunctionArx. All three compute the same
 * numbers (C simulation is identical); only the hardware schedule
 * differs.
 *
 *   PRAGMA_PROFILE_AREA     minimum area. One costFunctionArx instance,
 *                           models one after the other (model loop
 *                           PIPELINE off), the nTheta products of a
 *                           prediction on ONE multiplier (term loop rolled).
 *                           Estimate: Nhor*(Nscen+1)*L evaluation cycles,
 *                           L ~ 10-15 cycles per sequential prediction:
 *                           ~2e5 cycles per call, ~2 ms of the 10 ms period.
 *   PRAGMA_PROFILE_BALANCED one costFunctionArx instance, model loop
 *                           PIPELINE II=1: a new model enters the same
 *                           datapath every cycle, so a step takes about
 *                           Nscen+1 cycles plus the pipeline depth D (~10).
 *                           Estimate: 61*Nhor*(Nscen+1+D) ~ 2.5e4 cycles
 *                           (~0.25 ms), with little more area than AREA:
 *                           pipeline registers and nTheta multipliers.
 *   PRAGMA_PROFILE_LATENCY  minimum latency. As BALANCED, plus the 2*nOpt
 *                           poll evaluations of each MADS iteration run in
 *                           parallel on 2*nOpt costFunctionArx instances:
 *                           (1 + MADS_ITER) evaluation times per call.
 *                           Estimate: ~5e3 cycles (~50 us), with about
 *                           2*nOpt times BALANCED's cost-function area.
 * The estimates are mine, from the loop structure; replace them with the
 * synthesis report's numbers.
 *
 * Why not the former profiles. They were measured on the buck converter
 * (Nhor 5, Nscen 4) with Vitis HLS 2021.1. LATENCY fully unrolled both
 * cost-function loops: Nhor*(Nscen+1) = 255 predictions for the pendulum,
 * ~4 DSPs each, against 220 DSPs on this device, so it cannot fit.
 * BALANCED unrolled the Nscen loop, but its Nscen+1 results are summed
 * into saturating accumulators one after the other (a saturating add is
 * not associative, so the tool cannot regroup it into a tree): with 17
 * models that chain was slower than streaming them through one
 * pipelined datapath, which is what BALANCED does now.
 *
 * Pragmas, everywhere (PRAGMAS is defined with FIXED):
 *   - profile-dependent, as above: costFunctionArx.cpp (model loop),
 *     progressiveBarrierPollingArx.cpp (poll evaluations), MADSARX.cpp
 *     (ALLOCATION: number of costFunctionArx instances),
 *     computeArxOutput.cpp (ARX term loop; coefficient row loop pipelined
 *     like the model loop).
 *   - the same in every profile, each explained where it is: INTERFACE
 *     (controller.cpp: ap_none scalar ports, ap_ctrl_hs start/done);
 *     ARRAY_PARTITION complete on every small array that unrolled code
 *     reads in parallel; UNROLL on loops over na, nb, nk, nTheta, nGens,
 *     nUnc or nOpt (wiring and a few operators); PIPELINE II=1 on the
 *     random-number loops (generateScenarios, generatePollMatrixArx,
 *     pseudoRandArx); INLINE on small leaf functions and INLINE off on
 *     costFunctionArx (so ALLOCATION, not automatic inlining, sets its
 *     instance count); PIPELINE off on the loops that are recurrences (the
 *     MADS iteration loop, the Nhor prediction loop) and LOOP_FLATTEN off
 *     on the model loop. The explicit "off" forms matter with recent tool
 *     versions, which pipeline loops and inline small functions on their
 *     own when nothing says otherwise.
 */
#define PRAGMA_PROFILE_LATENCY         0
#define PRAGMA_PROFILE_BALANCED        1
#define PRAGMA_PROFILE_AREA            2
#define PRAGMA_PROFILE           PRAGMA_PROFILE_BALANCED

/* ======================================================================
   Derived feature flags (do NOT edit)
   ====================================================================== */
#ifdef FIXED
    #define PRAGMAS         /* enable all HLS synthesis pragmas in .cpp files  */
    #include <ap_fixed.h>   /* include fixed point data types */
    #undef PRNG_STDLIB      /* can't use rand() in fixed point */
    // #define DEBUG_PRINT      /* debug printfs */

    /*
     * Translate PRAGMA_PROFILE into the flags and pragma macros the
     * algorithm files use (see the PRAGMA_PROFILE comment above):
     *   PRAGMA_PIPELINE_MODEL_LOOPS  model loop and coefficient row loop pipelined
     *   PRAGMA_PARALLEL_POLLS        poll evaluations unrolled (in parallel)
     *   PRAGMA_COSTFUNC_INSTANCES    costFunctionArx instances (ALLOCATION limit)
     *   PRAGMA_MODEL_LOOP            pragma for the coefficient row loop
     *   PRAGMA_ARX_TERMS             pragma for the ARX nTheta-term loop
     * PRAGMA_COSTFUNC_INSTANCES is a literal because a pragma option must
     * be one: static_assert'ed against 2*nOpt further below.
     */
    #if PRAGMA_PROFILE == PRAGMA_PROFILE_LATENCY
        #define PRAGMA_PIPELINE_MODEL_LOOPS
        #define PRAGMA_PARALLEL_POLLS
        #define PRAGMA_COSTFUNC_INSTANCES  6
        #define PRAGMA_MODEL_LOOP          HLS_PRAGMA(HLS PIPELINE II=1)
        #define PRAGMA_ARX_TERMS           HLS_PRAGMA(HLS UNROLL)
    #elif PRAGMA_PROFILE == PRAGMA_PROFILE_BALANCED
        #define PRAGMA_PIPELINE_MODEL_LOOPS
        #define PRAGMA_COSTFUNC_INSTANCES  1
        #define PRAGMA_MODEL_LOOP          HLS_PRAGMA(HLS PIPELINE II=1)
        #define PRAGMA_ARX_TERMS           HLS_PRAGMA(HLS UNROLL)
    #elif PRAGMA_PROFILE == PRAGMA_PROFILE_AREA
        #define PRAGMA_COSTFUNC_INSTANCES  1
        #define PRAGMA_MODEL_LOOP          HLS_PRAGMA(HLS PIPELINE off)
        #define PRAGMA_ARX_TERMS           HLS_PRAGMA(HLS PIPELINE off)
    #else
        #error "Unrecognized PRAGMA_PROFILE!"
    #endif
#else
    #undef DEBUG_PRINT /* use only in fixed point mode */
#endif

/* ======================================================================
    Macros for printing active modes in main.
    These are automatically derived from the target selection section.
   ====================================================================== */
#ifdef FIXED
    #define FIXED_PRINT "true"
#else 
    #define FIXED_PRINT "false"
#endif
#ifdef CONVERSIONS_MODE
    #define CONVERSIONS_MODE_PRINT "true"
#else 
    #define CONVERSIONS_MODE_PRINT "false"
#endif
#ifdef NRMLZ
    #define NRMLZ_PRINT "true"
#else 
    #define NRMLZ_PRINT "false"
#endif
#ifdef USE_SCENS_COST
    #define USE_SCENS_COST_PRINT "true"
#else
    #define USE_SCENS_COST_PRINT "false"
#endif
#ifdef USE_SCENS_CONSTR
    #define USE_SCENS_CONSTR_PRINT "true"
#else
    #define USE_SCENS_CONSTR_PRINT "false"
#endif
#ifdef PRNG_STDLIB
    #define PRNG_STDLIB_PRINT "true"
#else 
    #define PRNG_STDLIB_PRINT "false"
#endif


/* ======================================================================
   Core headers
   ====================================================================== */
/* types.h is included by system_configs.h itself, AFTER the SYSTEM_* ids
 * it needs - including it here first would bring the always-true
 * "#if ACTIVE_SYSTEM == ..." problem back (see system_configs.h). */
#include "system_configs.h"

/* ======================================================================
   Compile-time consistency checks
   ====================================================================== */
#if (ACTIVE_SYSTEM != SYSTEM_SIMPLE) && \
    (ACTIVE_SYSTEM != SYSTEM_BENCHMARK) && \
    (ACTIVE_SYSTEM != SYSTEM_MILANO) && \
    (ACTIVE_SYSTEM != SYSTEM_BUCK) && \
    (ACTIVE_SYSTEM != SYSTEM_BUCK_LOSS) && \
    (ACTIVE_SYSTEM != SYSTEM_BUCK_ALBERTO) && \
    (ACTIVE_SYSTEM != SYSTEM_GAIN_DEMO) && \
    (ACTIVE_SYSTEM != SYSTEM_INVERTED_PENDULUM)
  #error "Unrecognized ACTIVE_SYSTEM!"
#endif

#if (CTRL_MODE != CTRL_MODE_SCMPC) && \
    (CTRL_MODE != CTRL_MODE_PL) && \
    (CTRL_MODE != CTRL_MODE_AL)
  #error "Unrecognized CTRL_MODE!"
#endif

/* ======================================================================
   UNCERTAIN PARAMETERS - the zonotope's own dimension nUnc <= nTheta
   ======================================================================
   A parameter whose row of GENERATORS_UNNORM is entirely zero is known
   exactly (theta_i = THETA_NOMINAL_UNNORM[i] for every point of the
   zonotope). The zonotope, the scenarios and the PL/AL strip update live
   only in the space of the nUnc UNCERTAIN parameters; certain ones are
   re-inserted as known constants wherever the full nTheta-vector is
   needed (computeArxCoeffs for predictions; the PL strip's known part).

   Why not simply keep a zero row in an nTheta-dimensional zonotope: its
   volume is then 0 for every candidate in boundStripZonotopeIntersection-
   New's minimum-volume selection, so "tmpVol < bestVol" is never true
   and PL learning silently stops (measured: PL output identical to
   SCMPC's). In the nUnc-dimensional space the volume is positive and
   meaningful, and generateScenarios/strip update/volume work on nUnc
   rows instead of nTheta.

   Detected at compile time from GENERATORS_UNNORM (C++14 constexpr):
     nUnc                   number of uncertain parameters
     UNC_MAP.full[k]        full index i of the k-th uncertain parameter
     UNC_MAP.red[i]         reduced index k of parameter i, or -1 if certain
   Both tables are compile-time constants: indexed with loop variables of
   fully unrolled loops they fold away completely in hardware.
   ====================================================================== */
constexpr double GENERATORS_UNNORM_TABLE[nTheta][nGens]  = { GENERATORS_UNNORM };
constexpr double THETA_NOMINAL_UNNORM_TABLE[nTheta]      = { THETA_NOMINAL_UNNORM };

struct UncertainMap
{
    int n;              /* number of uncertain parameters                */
    int full[nTheta];   /* [k] -> full index, valid for k < n            */
    int red [nTheta];   /* [i] -> reduced index, or -1 if i is certain   */
};

constexpr UncertainMap makeUncertainMap()
{
    UncertainMap m{};
    for (int i = 0; i < nTheta; i++)
    {
        bool uncertain = false;
        for (int j = 0; j < nGens; j++)
            if (GENERATORS_UNNORM_TABLE[i][j] != 0.0) uncertain = true;
        m.red[i] = uncertain ? m.n : -1;
        if (uncertain) m.full[m.n++] = i;
    }
    return m;
}

constexpr UncertainMap UNC_MAP = makeUncertainMap();
constexpr int nUnc = UNC_MAP.n;

static_assert(nUnc >= 1,
    "Every row of GENERATORS_UNNORM is zero: the model is fully known, so there "
    "is no uncertainty for the zonotope/scenario machinery to represent.");
static_assert(nGens >= nUnc,
    "Fewer generators than uncertain parameters: the zonotope is flat (zero "
    "volume) in the uncertain space, which disables PL/AL's volume-based update.");

/* ======================================================================
   CONTROLLER STATE
   ======================================================================
   These variables are DEFINED (storage allocated) in controller.cpp and
   DECLARED here as extern so that every translation unit that includes
   setup.h reads and writes the same physical location.

   Why extern and not static:
     "static" at file scope gives the variable INTERNAL linkage - each
     .cpp file that includes the header gets its own private copy.  Those
     copies are independent and go out of sync the moment any one of them
     is updated.  "extern" gives the variable EXTERNAL linkage: there is
     exactly ONE copy (defined in controller.cpp) and all files share it.

   thetaCenter / thetaGens - the current parameter zonotope
     Z(k) = { thetaCenter + thetaGens*ξ : || ξ ||inf <= 1 },
     over the nUnc UNCERTAIN parameters only (see UNC_MAP above).
     Updated each step by boundStripZonotopeIntersection (PL/AL modes).

   yHist / uHist - ARX regressor history buffers
     yHist[0] = y(k-1), yHist[1] = y(k-2), ..., yHist[na-1]  = y(k-na)
     uHist[0] = u(k-1), uHist[1] = u(k-2), ..., uHist[nb+nk-2] = u(k-nb-nk+1)
     Updated each step inside controller().

   uOptPrev - MADS warm-start seed, NOT part of the ARX regressor
     uOptPrev[0..NhorU-1] = u*(k-1), ..., u*(k-1+NhorU-1), i.e. the full
     control sequence MADS returned on the PREVIOUS call. controller()
     shifts this by one step (dropping the already-applied first entry
     and repeating the last one) to seed uOptNorm for the current call,
     instead of holding uHist[0] constant across the whole horizon. This
     gives MADS a much better starting point once the plant is tracking
     a moving reference, which reduces (without eliminating) transient
     DELTAYNORM violations right after a reference step.

     Its initial value (U_PREV_UNNORM, system_configs.h) MUST encode the
     same cold-start operating point as Y_HIST_UNNORM/U_HIST_UNNORM - unlike
     the old flat warm start, this one is never re-anchored to the real
     applied input, so an inconsistent cold start can seed a persistent
     closed-loop problem rather than a one-call transient.
   ====================================================================== */
extern theta_type  thetaCenter[nUnc];
extern theta_type  thetaGens  [nUnc][nGens];
extern norm_output_type yHist[na];
extern norm_input_type  uHist[nb + nk - 1];
/* uOptPrev[NhorU] is declared extern further below, once NhorU itself
 * has been defined (see "MPC PARAMETERS" section). */


/* ======================================================================
   MPC PARAMETERS
   ====================================================================== */

/** Prediction horizon N: controller optimises over the next N steps. */
constexpr int Nhor  = 15;

/**
 * Control horizon Nu <= N: the input sequence u(k), ..., u(k+Nu-1) is
 * optimised; u is held constant from step Nu to N ("input blocking").
 */
constexpr int NhorU = 3;

/**
 * Number of uncertainty scenarios Nscen, EXCLUDING the nominal system.
 * The cost is evaluated on Nscen + 1 models simultaneously.
 * Must be a power of 2 (enables bit-shift index arithmetic in hardware).
 */
constexpr int Nscen      = 16;   /* should always be a power of 2 */
constexpr int LOG2NSCEN  = 4;   /* must satisfy (1 << LOG2NSCEN) == Nscen */

static_assert(NhorU <= Nhor,
    "Control horizon NhorU must not exceed prediction horizon Nhor.");
static_assert((1 << LOG2NSCEN) == Nscen,
    "LOG2NSCEN must equal log2(Nscen).  Update both consistently.");
static_assert(Nhor >= nk, 
    "Prediction horizon must be at least system delay.");
static_assert(Nhor - nk >= NhorU - 1,
    "Control horizon is longer than maximum.");

/*
 * COMPUTATIONAL DELAY COMPENSATION (DELAY_COMPENSATION, selected above)
 *
 * Without it, the call at instant k returns u(k), computed from y(k), and
 * the model assumes u(k) is applied AT instant k. In real time it only
 * exists at k*Ts + dt (dt = the controller's latency): until then the DAC
 * still holds u(k-1), which the model does not know.
 *
 * With it, the call at instant k returns u(k+1), to be applied at instant
 * k+1 (the DAC is written on the next sampling tick, not when the result
 * is ready: works for any latency dt < Ts). The input applied during the
 * current interval, u(k), was returned by the previous call and is known:
 * the prediction uses it, and the decision variables become u(k+1),
 * u(k+2), ... One extra sample of loop delay, but exact and modelled,
 * and the input changes exactly at the sampling instants. The test bench
 * applies each output one sample later (plant.h); without compensation,
 * TB_COMPUTE_DELAY_FRACTION emulates the latency dt instead.
 *
 * DC_STEPS is the number of known future inputs at each call (0 or 1),
 * nUKnown the length of the known-input array passed to the optimizer:
 *   without: [u(k-1), ..., u(k-nb-nk+2)]          (nb+nk-2 values)
 *   with:    [u(k), u(k-1), ..., u(k-nb-nk+2)]    (nb+nk-1 values)
 */
#ifdef DELAY_COMPENSATION
constexpr int DC_STEPS = 1;
#else
constexpr int DC_STEPS = 0;
#endif
constexpr int nUKnown = nb + nk - 2 + DC_STEPS;
static_assert(Nhor - nk - DC_STEPS >= NhorU - 1,
    "With DELAY_COMPENSATION the last decision u(k+NhorU) must still reach the horizon: Nhor >= NhorU + nk.");

/** Number of scalar optimisation variables (= Nu for a SISO system). */
#define nOpt NhorU

#if defined(PRAGMAS) && defined(PRAGMA_PARALLEL_POLLS)
static_assert(PRAGMA_COSTFUNC_INSTANCES == 2 * nOpt,
    "PRAGMA_PROFILE_LATENCY: set PRAGMA_COSTFUNC_INSTANCES (setup.h) to 2*NhorU, one costFunctionArx per poll point.");
#endif

/** MADS warm-start seed - see "CONTROLLER STATE" comment above.
 *  Declared here, rather than alongside thetaCenter/yHist/uHist, because
 *  its size depends on NhorU, which is only just defined above. */
extern norm_input_type uOptPrev[NhorU];

/* ======================================================================
   MPC COST-FUNCTION WEIGHTS
   ======================================================================
   Stage cost per step:  l(y, u) = outputWeight*(y - y_ref)^2 + R*u^2
   Terminal cost:        V_f(y)   = terminalOutputWeight*(y(k+N) - y_ref)^2
   ====================================================================== */
static const output_weight_type terminalOutputWeight =  16.0;   /* terminal output weight */
static const output_weight_type outputWeight =  16.0;   /* stage   output weight  */
static const input_weight_type RBaseLine = 0.36; /* so that R = 0.125 = 2^(-3) in NRMLZ */
/* 
    log2Q/log2P are used for shift operations instead of multiplications in
    the FIXED cost function computation, so outputWeight/terminalOutputWeight
    must equal 2^log2Q / 2^log2P (testMain.cpp warns otherwise). R is
    multiplied, not shifted, so it can be any value.
*/
constexpr int log2Q = 4;
constexpr int log2P = 4;

/* ======================================================================
   MADS SOLVER PARAMETERS
   ======================================================================
   The Mesh Adaptive Direct Search (MADS) algorithm manages a frame size
   updated multiple times in a single time step(success: +c, failure: -c).
   Sizes are stored in log-scale (frameIdx, meshIdx) as integers for
   efficient hardware arithmetic.
   ====================================================================== */
constexpr int MADS_ITER = 10;   /* MADS iterations per controller call     */
constexpr int TAU       = 1;   /* frame-size update base                  */
constexpr int MADS_C    = 1;   /* frame-size exponent step  (integer > 0) */
constexpr int FRAME_EXP_MIN = -20; /* frameExp minimum value */

/*
 * expC = 2^{-MADS_C}: pre-computed scaling factor for the mesh update.
 * In FIXED mode this is exactly 0.5 = 2^{-1}, needing only 1 integer bit,
 * so a 1-bit ap_ufixed is used to save multiplier resources.
 * If MADS_C changes, update expC and its type consistently.
 */
#ifdef FIXED
  static const ap_ufixed<1, 0, AP_TRN, AP_WRAP> expC = 0.5;
//   static const norm_input_type expC = 0.5;
#else
  static const householder_type expC = 0.5;
#endif

/*
 * D0[j]: initial log2 frame-size exponent for optimisation variable j.
 * Initial frame size = tau^D0[j].  More negative -> finer initial mesh.
 * HLS hint: #pragma HLS ARRAY_PARTITION variable=D0 complete dim=1
 */
static const mesh_exp_type D0_VAL = -8; // used in MADSARX, frameIdx init
// static const mesh_exp_type D0[nOpt] = { -8, -8, -8 }; // not used

/* ======================================================================
   ADC / DAC CONVERTER PARAMETERS
   ======================================================================
   Linear conversion between the 12-bit integer domain ({0,...,4095}) and
   the physical domain used by the algorithm.

   Output channel:
     y_dig = YADCGain * (y_an - YBias)         [ADC: analogue -> digital]
     y_an  = YDACGain *  y_dig + YBias          [DAC: digital -> analogue]

   Input channel:
     u_dig = UADCGain * (u_an - UBias_an)       (UBias maps midscale -> 0)
     u_an  = UDACGain *  u_dig + UBias_an

   In software mode the gains are derived from UMINPHYS/UMAXPHYS/YMINPHYS/YMAXPHYS.
   In fixed-point mode they are explicit literals (ap_fixed<> prevents
   compile-time arithmetic on non-constexpr types).
   ====================================================================== */
#ifdef CONVERSIONS_MODE
constexpr int ADC_MAX   = 4095;
constexpr int ADC_MIN   = 0;
constexpr int ADC_RANGE = ADC_MAX - ADC_MIN;

static const output_adc_coeff_type YADCGain = double(ADC_RANGE) / double(YMAXPHYS - YMINPHYS);
static const output_dac_coeff_type YDACGain = double(YMAXPHYS - YMINPHYS) / double(ADC_RANGE);
static const digital_output_type  YBias    = (double(ADC_MIN)*double(YMAXPHYS) - double(ADC_MAX)*double(YMINPHYS)) / double(YMAXPHYS - YMINPHYS);
static const input_adc_coeff_type UADCGain = double(ADC_RANGE) / double(UMAXPHYS - UMINPHYS);
static const input_dac_coeff_type UDACGain = double(UMAXPHYS - UMINPHYS) / double(ADC_RANGE);
static const digital_input_type   UBias    = (double(ADC_MIN)*double(UMAXPHYS) - double(ADC_MAX)*double(UMINPHYS)) / double(UMAXPHYS - UMINPHYS);
#endif

#ifdef NRMLZ
/* Set these from user? */
static const norm_output_type YNORMMAXPHYS = 1;
static const norm_output_type YNORMMINPHYS = -1;
static const norm_input_type UNORMMAXPHYS = 1;
static const norm_input_type UNORMMINPHYS = -1;

static const y_norm_coeff_type yNormGain = double(YNORMMAXPHYS - YNORMMINPHYS) / double(YMAXPHYS - YMINPHYS);
static const u_norm_coeff_type uNormGain = double(UNORMMAXPHYS - UNORMMINPHYS) / double(UMAXPHYS - UMINPHYS);
static const u_norm_inv_coeff_type uNormGainInverse = double(UMAXPHYS - UMINPHYS) / double(UNORMMAXPHYS - UNORMMINPHYS);
static const norm_output_type yNormOffset = double(YNORMMINPHYS*YMAXPHYS - YNORMMAXPHYS*YMINPHYS) / double(YMAXPHYS - YMINPHYS);
static const norm_input_type uNormOffset = double(UNORMMINPHYS*UMAXPHYS - UNORMMAXPHYS*UMINPHYS) / double(UMAXPHYS - UMINPHYS);

static const norm_output_type YNORMMAX = double(yNormGain)*double(YMAX) + double(yNormOffset);
static const norm_output_type YNORMMIN = double(yNormGain)*double(YMIN) + double(yNormOffset);
static const norm_input_type UNORMMAX = double(uNormGain)*double(UMAX) + double(uNormOffset);
static const norm_input_type UNORMMIN = double(uNormGain)*double(UMIN) + double(uNormOffset);

static const input_weight_type R = double(RBaseLine) * double(yNormGain) * double(yNormGain) / (double(uNormGain) * double(uNormGain)); // 2^(-3) for the buck converters

/* Normalization, use "normalized" noise in output strip */
const norm_noise_type sigma = double(yNormGain)*double(SIGMA_UNNORM);
static const norm_output_type DELTAYNORM = double(yNormGain) * double(DELTAY);

/*
 * Strip-intersection normalization constants (Bravo et al. zonotope
 * update - only actually READ by controller.cpp's CTRL_MODE_PL/AL
 * branch, but declared unconditionally here like everything else in
 * this file's NRMLZ block).
 *
 * DERIVATION (see matlab/testNormalizationEquivalence.m for the
 * reference MATLAB version this reproduces - verified against
 * SYSTEM_BUCK_LOSS's previously hand-pasted MATLAB output to 12+
 * significant digits):
 *
 *   Z0 = (THETA_NOMINAL_UNNORM, GENERATORS_UNNORM), the INITIAL zonotope
 *        in UNNORMALIZED (physical parameter) space - deliberately NOT
 *        thetaCenter/thetaGens (see the note below for why).
 *   Dg[i] = sum_j |GENERATORS_UNNORM[i][j]| - Z0's interval-hull
 *           half-width in parameter i (row-sum of |generators|).
 *   Dm[i] = yNormGain for i<na, uNormGain for i>=na - per-parameter
 *           I/O normalization gain (block-diagonal in the MATLAB).
 *   q[i]  = yNormOffset for i<na, uNormOffset for i>=na.
 *
 *   myInvDmDg[i]  = (yNormGain/Dm[i]) * Dg[i]
 *   myInvDmc0[i]  = (yNormGain/Dm[i]) * THETA_NOMINAL_UNNORM[i]
 *   qmyInvDmDg[i] = -q[i] * myInvDmDg[i]
 *   qmyInvDmc0    = -sum_i q[i] * myInvDmc0[i]
 *
 * For i<na, yNormGain/Dm[i] = yNormGain/yNormGain = 1, so those entries
 * collapse to exactly Dg[i] / THETA_NOMINAL_UNNORM[i] with no gain
 * factor at all - which is why SYSTEM_BUCK_LOSS's first two (na=2)
 * hardcoded myInvDmDg/myInvDmc0 entries above used to just BE the
 * row-sum and the center component, unscaled.
 *
 * WHY NOT thetaCenter/thetaGens: Z0 above is the INITIAL zonotope, not
 * whatever PL/AL mode has since updated thetaCenter/thetaGens to -
 * reading the mutable extern globals here would silently give the
 * wrong answer after the first update.
 * It also sidesteps a real hazard: theta_type's ap_fixed constructor is
 * not constexpr (see this file's own header comment), so
 * thetaCenter/thetaGens's initialization in controller.cpp is DYNAMIC
 * initialization, and C++ gives NO ordering guarantee between dynamic
 * initializers in different translation units - reading them from HERE
 * (a different .cpp file, via the extern declaration) could observe
 * them still zero, depending on link order. The function below instead
 * builds its own FUNCTION-LOCAL c0/G straight from the macros, so the
 * only things it depends on (yNormGain, uNormGain, yNormOffset,
 * uNormOffset, na) are ordinary same-translation-unit globals declared
 * earlier in THIS file - C++ guarantees in-order initialization within
 * one translation unit, so that dependency is always safe.
 *
 * CONSTRUCT: an ordinary (non-constexpr - ap_fixed<> still has no
 * constexpr constructor) function, called exactly once, its result
 * assigned into "static const" variables - the same "compute via a
 * double expression at static-initialization time" idiom yNormGain/
 * uNormGain above already use, just generalised from one scalar to a
 * small aggregate of arrays by bundling them in a plain struct (structs
 * with array members ARE returnable/copyable by value in C++, unlike
 * bare arrays, so no std::array or other container is needed - keeping
 * this to the same plain-C-array style as everywhere else in the
 * codebase). All arithmetic is done in double via explicit casts,
 * exactly like yNormGain/uNormGain, so no ap_fixed rounding mode acts
 * on any INTERMEDIATE step, only on the final assignment into each
 * struct member. The '<0.0 ? - : ' absolute value (rather than
 * std::abs, which would need a new #include) matches the same by-hand
 * pattern testMain.cpp/volumeZonotope.cpp already use for an
 * ap_fixed-typed sign flip (matDet's result).
 *
 * computeArxCoeffs (computeArxOutput.cpp - folds them into per-scenario
 * effective ARX coefficients once per controller() call) and
 * controller.cpp's PL/AL strip update read these as
 * stripCoeffs.myInvDmDg[i] etc. directly, rather than through a "static const auto&" reference alias to
 * each member: a reference-to-array-of-ap_fixed is no different, in
 * principle, from a reference-to-array-of-double (reference binding
 * doesn't depend on the element type - it resolves to the underlying
 * storage before any element-type arithmetic comes into play), but
 * that claim was only checked numerically here in double/host-
 * simulation mode (no Vitis HLS/ap_fixed.h available in this
 * environment) - accessing the struct directly removes the need to
 * rely on that reasoning at all, leaving only the same "static const
 * computed via a non-constexpr function" mechanism this file's
 * existing scalar constants (yNormGain, uNormGain, R, DELTAYNORM, ...)
 * already depend on successfully.
 */
struct StripCoeffs
{
    strip_coeff_type      myInvDmDg[nTheta];
    strip_q_coeff_type    qmyInvDmDg[nTheta];
    strip_coeff_c0_type   myInvDmc0[nTheta];
    strip_q_coeff_c0_type qmyInvDmc0;
};

static StripCoeffs computeStripCoeffs()
{
    theta_type c0[nTheta]        = { THETA_NOMINAL_UNNORM };  // Z0.c
    theta_type G [nTheta][nGens] = { GENERATORS_UNNORM };      // Z0.G

    StripCoeffs out{};
    double qmyInvDmc0Acc = 0.0;

    for (int i = 0; i < nTheta; i++)
    {
        double Dg_i = 0.0;
        for (int j = 0; j < nGens; j++)
        {
            double g = double(G[i][j]);
            Dg_i += (g < 0.0) ? -g : g;
        }

        const bool   isY        = (i < na);
        const double gainRatio  = isY ? 1.0 : double(yNormGain) / double(uNormGain);
        const double qi         = isY ? double(yNormOffset)    : double(uNormOffset);

        const double myInvDmDg_i = gainRatio * Dg_i;
        const double myInvDmc0_i = gainRatio * double(c0[i]);

        out.myInvDmDg[i]  = myInvDmDg_i;
        out.qmyInvDmDg[i] = -qi * myInvDmDg_i;
        out.myInvDmc0[i]  = myInvDmc0_i;
        qmyInvDmc0Acc    += qi * myInvDmc0_i;
    }

    out.qmyInvDmc0 = -qmyInvDmc0Acc;
    return out;
}

static const StripCoeffs stripCoeffs = computeStripCoeffs();
#else
const norm_noise_type sigma = SIGMA_UNNORM; // no normalization, use normal noise in output strip
static const input_weight_type R = RBaseLine;   /* stage   input  weight  */
static const norm_output_type YNORMMAXPHYS = YMAXPHYS;
static const norm_output_type YNORMMINPHYS = YMINPHYS;
static const norm_input_type UNORMMAXPHYS = UMAXPHYS;
static const norm_input_type UNORMMINPHYS = UMINPHYS;
static const norm_output_type DELTAYNORM = DELTAY;
static const norm_output_type YNORMMAX = YMAX;
static const norm_output_type YNORMMIN = YMIN;
static const norm_input_type UNORMMAX = UMAX;
static const norm_input_type UNORMMIN = UMIN;
#endif

/* Choose appropriate coefficients based on the operation modes.
    These serve as coefficients and offsets for computing
    conversions functions. */
#ifdef CONVERSIONS_MODE
    #ifdef NRMLZ
    static const dig2ctrl_type yConvCoeff = double(yNormGain)*double(YDACGain);
    static const ctrl2dig_type uConvCoeff = double(UADCGain)*double(uNormGainInverse);
    static const norm_output_type yConvOffset = double(yNormOffset) - double(YDACGain)*double(YBias)*double(yNormGain);
    static const digital_input_type uConvOffset = double(UBias) - double(uNormOffset)*double(uNormGainInverse)*double(UADCGain);
    #else
    static const dig2ctrl_type yConvCoeff = YDACGain;
    static const ctrl2dig_type uConvCoeff = UADCGain;
    static const norm_output_type yConvOffset = double(-YBias)*double(YDACGain);
    static const digital_input_type uConvOffset = UBias;
    #endif
    #else
    #ifdef NRMLZ
    static const dig2ctrl_type yConvCoeff = yNormGain;
    static const ctrl2dig_type uConvCoeff = uNormGainInverse;
    static const norm_output_type yConvOffset = yNormOffset;
    static const digital_input_type uConvOffset = double(-uNormOffset)*double(uNormGainInverse);
    #else
    static const dig2ctrl_type yConvCoeff = 1;
    static const ctrl2dig_type uConvCoeff = 1;
    static const norm_output_type yConvOffset = 0;
    static const digital_input_type uConvOffset = 0;
    #endif
#endif

/* ======================================================================
   FUNCTION PROTOTYPES
   ====================================================================== */

/* --- Controller lifecycle -------------------------------------------- */

/** Top-level controller step (HLS top function).
 *  Accepts the current output and reference as 12-bit digital samples;
 *  returns the optimal input sequence as 12-bit digital samples.
 *
 *  HLS interface pragmas (add in controller.cpp, not here):
 *    #pragma HLS INTERFACE ap_none port=yCurrDig
 *    #pragma HLS INTERFACE ap_none port=yrefDig
 *    #pragma HLS INTERFACE ap_ctrl_hs port=return
 */
digital_input_type controller(const digital_output_type yCurrDig,
                              const digital_output_type yrefDig);

/* --- ARX model -------------------------------------------------------- */

/** One-step-ahead prediction:
 *  ypred(k) = offset + coeff^T * [y(k-1),...,y(k-na), u(k-nk),...,u(k-nk-nb+1)]^T
 *  with coeff/offset from computeArxCoeffs (coeff = theta, offset = 0
 *  without NRMLZ). */
norm_output_type computeArxOutput(const norm_output_type yPast[na],
                                  const norm_input_type  uSamples[nb+nk-1],
                                  const arx_coeff_type   coeff[nTheta],
                                  const arx_coeff_type   offset);

/** Effective ARX coefficients/offset (normalization folded in) for the
 *  Nscen scenarios (rows 0..Nscen-1) and the zonotope centre (row Nscen);
 *  computed once per controller() call. */
void computeArxCoeffs(const theta_type thetaScenarios[Nscen][nUnc],
                      const theta_type thetaNominal  [nUnc],
                      arx_coeff_type   coeff         [Nscen+1][nTheta],
                      arx_coeff_type   offset        [Nscen+1]);

/* --- MPC cost function ------------------------------------------------ */

/** Evaluate scenario-based MPC cost over the control horizon.
 *  cost[0] = objective value;  cost[1] = constraint violation. */
void costFunctionArx(cost_type              cost[2],
                     const norm_output_type yPast          [na],
                     const norm_input_type  currU          [nOpt],
                     const norm_input_type  uPast          [nUKnown],
                     const norm_output_type yref,
                     const arx_coeff_type   arxCoeff       [Nscen+1][nTheta],
                     const arx_coeff_type   arxOffset      [Nscen+1]);

/* --- MADS poll-step --------------------------------------------------- */

/** Build 2*nOpt poll directions from a random unit vector and the
 *  current mesh/frame exponent vectors. */
void generatePollDirectionsArx(const rand_type     randomVector[nOpt],
                                const mesh_exp_type frameIdx    [nOpt],
                                const mesh_exp_type meshIdx     [nOpt],
                                direction_type      directions  [nOpt][2*nOpt]);

/** Scale the poll directions into a full poll matrix. */
void generatePollMatrixArx(const norm_input_type currU[nOpt], 
                            const mesh_exp_type frameIdx[nOpt], 
                            const mesh_exp_type meshIdx[nOpt],
                            norm_input_type pollMatrix[nOpt][2 * nOpt]);

/* --- Scenario generation --------------------------------------------- */

/**
 * Fill thetaScenarios[0..Nscen-1] with Nscen parameter vectors drawn
 * uniformly at random from  Z = { center + gens*ξ  :  ||ξ||inf <= 1 }.
 *
 * @param thetaScenarios [out] Nscen * nTheta array of scenario vectors.
 * @param center         [in]  Zonotope centre, length nTheta.
 * @param gens           [in]  Generator matrix, nTheta * nGens
 *                              (row-major); only the first nGen columns
 *                              are read.
 * @param nGen           [in]  Number of active generator columns
 *                              (= nTheta after interval-hull reduction;
 *                              may be larger for the initial zonotope).
 */
void generateScenarios(theta_type       thetaScenarios[Nscen][nUnc],
                       const theta_type center        [nUnc],
                       const theta_type gens          [nUnc][nGens]);

/* --- MADS main loop --------------------------------------------------- */

/** Run MADS_ITER iterations and return the optimal input sequence. */
void MADSARX(norm_input_type uOpt[nOpt], 
            const norm_input_type uInit[nUKnown], 
            const norm_output_type yInit[na], 
            const norm_output_type yref, 
            const arx_coeff_type arxCoeff[Nscen+1][nTheta],
            const arx_coeff_type arxOffset[Nscen+1]);

/** Evaluate all 2*nOpt poll candidates; update the best feasible point
 *  via the progressive barrier strategy. */
void progressiveBarrierPollingArx(cost_type bestCost[2], 
                                    norm_input_type bestPoint[nOpt], 
                                    const norm_output_type yPast[na], 
                                    const norm_input_type uPast[nUKnown], 
                                    const norm_output_type yref, 
                                    const norm_input_type pollMatrix[nOpt][2 * nOpt], 
                                    mesh_exp_type frameExp[nOpt], 
                                    const arx_coeff_type arxCoeff[Nscen+1][nTheta],
                                    const arx_coeff_type arxOffset[Nscen+1]);

/* --- Pseudo-random generation (three overloads) ----------------------- */
/** Random number in [-1, 1] */
rand_type pseudoRandArx();

/** nGens random coefficients in [-1, 1] for zonotope scenario sampling. */
void pseudoRandArx(rand_type coeffs[nGens]);

/* --- Test-bench support (never called by the synthesized design) ------ */
/** Controller state (zonotope, I/O histories, warm start) and its random
 *  generator back to their power-on values - one call each per new
 *  simulation run, so runs are independent (see testMain.cpp). */
void resetControllerState();
void pseudoRandReset();

/* --- Constraint violation --------------------------------------------- */

/** Add to acc the constraint violation of one predicted output yCurr (box and rate
 *  constraints): costFunctionArx accumulates it into cost[1], the
 *  progressive-barrier violation term. */
void addConstraintViolation(cost_type &acc, const norm_output_type yCurr, const norm_output_type yPast);

/* --- ADC / DAC conversions -------------------------------------------- */
#ifdef CONVERSIONS_MODE
digital_output_type ADConvertY(const output_type       yAn);
input_type          DAConvertU(const digital_input_type  uDig);
#endif

#ifdef NRMLZ
norm_output_type normalizeY(output_type yAn);
input_type denormalizeU(norm_input_type uNorm);
#endif

/* Conversion functions to/from type used in controller */
norm_output_type dig2ctrlY(const digital_output_type yDig);
digital_input_type ctrlU2dig(const norm_input_type uCtrl);

/* --- Passive learning (PL mode only) ---------------------------------- */
#if CTRL_MODE == CTRL_MODE_PL || CTRL_MODE == CTRL_MODE_AL

/**
 * Intersect the current parameter zonotope with the measurement strip
 *   S(k) = { theta : |y(k) - phi(k)^T theta| <= epsilon }
 * where phi(k) = [y(k-1),...,y(k-na), u(k-nk),...]^T is the ARX regressor.
 *
 * The algorithm used here preserves the number of generator columns, so
 * the output zonotope has the same shape as the input (no reduction 
 * step is required).
 *
 * Inputs:
 *   yCurr         - current measurement y(k)
 *   yPast         - output history  [y(k-1), ..., y(k-na)]
 *   uSamplesIn    - input  history  [u(k-1), ..., u(k-nb-nk+1)]
 *   phi           - regressor restricted to the nUnc uncertain
 *                   parameters; stripCenter already has the certain
 *                   parameters' known contribution removed (controller.cpp)
 *   oldCenter     - current zonotope centre  (length nUnc)
 *   oldGens       - current generator matrix (nUnc * nGens);
 *                   only the first nGen columns are read
 *   nGen          - number of active generator columns
 *
 * Outputs:
 *   newCenter     - updated zonotope centre  (length nUnc)
 *   newGens       - updated generator matrix (nUnc * nGens);
 *                   exactly nGen columns are written (same as input)
 */
void boundStripZonotopeIntersectionNew(const strip_center_type stripCenter, 
                                    const phi_type phi[nUnc],
                                    const norm_noise_type stripRadius,
                                    const theta_type oldCenter[nUnc],
                                    const theta_type oldGens[nUnc][nGens], 
                                    theta_type newCenter[nUnc], 
                                    theta_type newGens[nUnc][nGens]);

#endif  /* CTRL_MODE == CTRL_MODE_PL */
// Zonotope volume computation
vol_type zonotopeVolume(const theta_type G[nUnc][nGens]);

// (Generators) matrix determinant computation
det_type matDet(const theta_type M[nUnc][nUnc]);

