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
  #define FIXED            /* fixed point representation */
#define CONVERSIONS_MODE /* ADC/DAC conversions */
#define NRMLZ               /* normalization */
#define USE_SCENS_COST      /* Enable scenarios cost contribution */
#define USE_SCENS_CONSTR    /* Enable scenarios constraints contribution */
// #define PRNG_STDLIB         /* use rand() as prng */

/** Active plant - choose one of: 
 * SYSTEM_SIMPLE, 
 * SYSTEM_BENCHMARK, 
 * SYSTEM_MILANO, 
 * SYSTEM_BUCK,
 * SYSTEM_BUCK_LOSS */
#define ACTIVE_SYSTEM   SYSTEM_BUCK_LOSS

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
 * HLS pragma profile - selects between measured area/latency trade-offs
 * at the few spots in this codebase where synthesis has shown them to be
 * in genuine tension (currently just costFunctionArx's Nhor/Nscen
 * loops). Everything else this design's pragmas do (ARRAY_PARTITION,
 * INLINE on small leaf functions, UNROLL on other na/nb/nTheta/nOpt-
 * sized loops, PIPELINE on the PRNG draws) is unconditionally
 * beneficial - "free" wins with no real trade-off - and is NOT gated by
 * this switch. Numbers below are for CTRL_MODE_SCMPC on xc7z020-clg484-1
 * at a 10 ns target clock; re-measure for other configurations.
 *
 * Choose one of:
 *   PRAGMA_PROFILE_LATENCY  : unroll BOTH costFunctionArx loops.
 *                             Measured: 4151 cycles (41.51 us, >=20kHz
 *                             margin) but LUT utilisation at ~100.8%
 *                             (53614/53200) - no room left for
 *                             CTRL_MODE_PL's extra logic or for the
 *                             AXI/clock/reset infrastructure needed to
 *                             integrate this IP into a real system.
 *   PRAGMA_PROFILE_BALANCED : unroll the Nscen (scenario) loop fully,
 *                             and PARTIALLY unroll Nhor (the genuine
 *                             k-recurrence) by
 *                             PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR below
 *                             (see costFunctionArx.cpp). Measured with
 *                             factor left at 1 (Nhor fully rolled): 6603
 *                             cycles (66.03 us, ~15.1 kHz - short of a
 *                             20kHz/5000-cycle target) at 39% DSP / 13%
 *                             FF / 50% LUT.
 *                             Confirmed by comparing all three profiles:
 *                             unrolling Nscen (AREA->BALANCED) bought a
 *                             2.83x speedup for +13 LUT points; ADDITIONALLY
 *                             fully unrolling Nhor (BALANCED->LATENCY)
 *                             only bought a further 1.59x speedup for
 *                             +50 LUT points - ~18x worse LUT-per-cycle-
 *                             saved, because Nhor's cross-iteration
 *                             dependency means unrolling it mostly
 *                             removes loop-FSM overhead rather than
 *                             creating genuine parallel work.
 *
 *                             A partial UNROLL factor on Nhor
 *                             (PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR below)
 *                             was tried as a finer-grained dial between
 *                             fully-rolled and fully-unrolled, and
 *                             MEASURED WORSE ON BOTH AXES: factor=2 gave
 *                             18943-19459 cycles at 65% LUT (worse than
 *                             fully-ROLLED Nhor's 6603 cycles / 50% LUT),
 *                             and factor=3 gave 25780-26124 cycles at 80%
 *                             LUT (worse than even PRAGMA_PROFILE_AREA's
 *                             ~18700-cycle fully-rolled baseline). Do NOT
 *                             raise the factor above 1 for this loop.
 *                             Likely cause: Nhor's body already contains
 *                             Nscen fully unrolled (4 parallel
 *                             computeArxOutput chains); partially
 *                             unrolling Nhor by N fuses N copies of that
 *                             already-4-way-parallel body into one larger
 *                             combinational block (N*4 concurrent
 *                             computeArxOutput instances instead of 4),
 *                             and Vitis's list-scheduler evidently finds
 *                             a WORSE schedule for that bigger, more
 *                             tangled resource-allocation problem, not a
 *                             better one - HLS unrolling is a heuristic,
 *                             not an exact optimizer, and is not always
 *                             monotonically beneficial. The factor is
 *                             left in place (at 1, i.e. disabled) for
 *                             documentation/future reference rather than
 *                             removed outright.
 *   PRAGMA_PROFILE_AREA     : leave both loops rolled (time-multiplexed).
 *                             Measured: ~18700 cycles (187 us, ~5.3 kHz)
 *                             but LUT utilisation back down to ~37%. Use
 *                             this whenever the >=20kHz requirement does
 *                             not apply.
 *   PRAGMA_PROFILE_LATENCY_SHARED : same loop structure as
 *                             PRAGMA_PROFILE_LATENCY (both loops fully
 *                             unrolled), originally intended to add
 *                             #pragma HLS ALLOCATION operation
 *                             instances=mul limit=PRAGMA_LATENCY_MUL_LIMIT
 *                             (below) inside costFunctionArx to cap
 *                             multiplier instances and recover LUT
 *                             margin. MEASURED COUNTERPRODUCTIVE for that
 *                             purpose and left DISABLED by default
 *                             (PRAGMA_LATENCY_MUL_LIMIT's value is
 *                             ignored unless PRAGMA_ENABLE_LATENCY_MUL_LIMIT
 *                             is also defined below, which it is not) -
 *                             at limit=48: 4022 cycles (an improvement -
 *                             latency was never the problem with this
 *                             lever), DSP down to 41% (91, from
 *                             PRAGMA_PROFILE_LATENCY's 151) exactly as
 *                             intended, but LUT went UP to 103%
 *                             (54821/53200), not down.
 *                             Reason (a real architectural point, not
 *                             just an unlucky measurement): a DSP48E1
 *                             slice on this chip is a dedicated hard
 *                             macro that consumes ZERO general-fabric
 *                             LUTs when instantiated - so letting Vitis
 *                             freely build 151 of them (PRAGMA_PROFILE_
 *                             LATENCY) is essentially LUT-free for that
 *                             arithmetic. Forcing those down to a shared
 *                             pool of 48 via ALLOCATION does not make the
 *                             multiply work disappear: it forces excess
 *                             multiplies to time-share far fewer physical
 *                             instances, which needs LUT-built steering
 *                             multiplexers (selecting which operand pair
 *                             feeds each shared multiplier this cycle,
 *                             and where the result goes) - and here that
 *                             routing overhead cost MORE LUTs than the
 *                             saved DSP48 instances were ever costing.
 *                             ALLOCATION-based sharing trades a
 *                             LUT-free resource for a LUT-cost one: the
 *                             wrong direction when LUT specifically is
 *                             the scarce resource and DSP has headroom.
 *                             The corrected direction, not yet
 *                             implemented pending a detailed per-multiply
 *                             report to target it precisely rather than
 *                             guessing: #pragma HLS BIND_OP ... impl=dsp
 *                             on specific multiplies Vitis is currently
 *                             implementing in fabric, to move THAT LUT
 *                             cost onto the spare DSP48 slots instead -
 *                             pushing load onto the free, LUT-free
 *                             resource rather than restricting it.
 */
#define PRAGMA_PROFILE_LATENCY         0
#define PRAGMA_PROFILE_BALANCED        1
#define PRAGMA_PROFILE_AREA            2
#define PRAGMA_PROFILE_LATENCY_SHARED  3
#define PRAGMA_PROFILE           PRAGMA_PROFILE_BALANCED

/**
 * Kill switch for the ALLOCATION-based multiplier limit under
 * PRAGMA_PROFILE_LATENCY_SHARED (costFunctionArx.cpp), left UNDEFINED
 * (disabled) because that limit measured counterproductive - see
 * PRAGMA_PROFILE_LATENCY_SHARED's comment above for the numbers and the
 * architectural reason (DSP48 is LUT-free hard silicon; ALLOCATION-forced
 * sharing trades it for LUT-cost steering logic). Left as an explicit
 * opt-in rather than deleting the mechanism, in case a future, different
 * bottleneck makes DSP-sharing the right call again.
 */
// #define PRAGMA_ENABLE_LATENCY_MUL_LIMIT

/**
 * Multiplier instance limit for costFunctionArx under
 * PRAGMA_PROFILE_LATENCY_SHARED, only applied if
 * PRAGMA_ENABLE_LATENCY_MUL_LIMIT above is also defined (currently is
 * not - see that macro's comment). At 48: 91 DSP (41%, down from
 * PRAGMA_PROFILE_LATENCY's 151/68%) but 54821 LUT (103%, UP from
 * PRAGMA_PROFILE_LATENCY's 53614/100.8%) and 4022 cycles (down from
 * 4151). Kept for reference/comparison, not because raising or lowering
 * it further is expected to help the LUT problem.
 */
#define PRAGMA_LATENCY_MUL_LIMIT  48

/**
 * Partial-unroll factor for costFunctionArx's Nhor loop, used only under
 * PRAGMA_PROFILE_BALANCED (see above and costFunctionArx.cpp). 1 = fully
 * rolled (no UNROLL pragma emitted at all). MEASURED WORSE than 1 for
 * every value tried (2, 3) - see the PRAGMA_PROFILE comment above for
 * the numbers and the likely reason. Left at 1; do not raise it for this
 * particular loop unless costFunctionArx's structure changes enough to
 * revisit the reasoning (e.g. if Nscen's unroll is ever reduced or
 * removed, which would shrink each round back down and might change
 * this conclusion).
 */
#define PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR  1

/* ======================================================================
   Derived feature flags (do NOT edit)
   ====================================================================== */
#ifdef FIXED
    #define PRAGMAS         /* enable all HLS synthesis pragmas in .cpp files  */
    #include <ap_fixed.h>   /* include fixed point data types */
    #undef PRNG_STDLIB      /* can't use rand() in fixed point */
    // #define DEBUG_PRINT      /* debug printfs */

    /*
     * Translate the PRAGMA_PROFILE selector above into small,
     * descriptively-named flags that the algorithm files check
     * individually. Keeping the translation here, in one place, means
     * costFunctionArx.cpp never needs to know the numeric profile
     * values or list every profile that wants a given loop unrolled -
     * it just checks whether ITS flag is defined.
     */
    #if PRAGMA_PROFILE == PRAGMA_PROFILE_LATENCY
        #define PRAGMA_UNROLL_COSTFUNC_NHOR
        #define PRAGMA_UNROLL_COSTFUNC_NSCEN
    #elif PRAGMA_PROFILE == PRAGMA_PROFILE_BALANCED
        #define PRAGMA_UNROLL_COSTFUNC_NSCEN
        /* NHOR: PARTIAL unroll by PRAGMA_COSTFUNC_NHOR_UNROLL_FACTOR,
         * not left bare - see costFunctionArx.cpp for how factor=1
         * (the default until this is swept) reduces to no pragma. */
    #elif PRAGMA_PROFILE == PRAGMA_PROFILE_AREA
        /* Both intentionally left rolled - see PRAGMA_PROFILE comment above. */
    #elif PRAGMA_PROFILE == PRAGMA_PROFILE_LATENCY_SHARED
        #define PRAGMA_UNROLL_COSTFUNC_NHOR
        #define PRAGMA_UNROLL_COSTFUNC_NSCEN
        #ifdef PRAGMA_ENABLE_LATENCY_MUL_LIMIT
            #define PRAGMA_LIMIT_COSTFUNC_MUL
        #endif
        /* PRAGMA_ENABLE_LATENCY_MUL_LIMIT is NOT defined above by default -
         * the ALLOCATION limit it would gate measured counterproductive
         * (see PRAGMA_PROFILE_LATENCY_SHARED's comment). Without it, this
         * profile is currently identical to PRAGMA_PROFILE_LATENCY. */
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
#include "types.h"
#include "system_configs.h"

/* ======================================================================
   Compile-time consistency checks
   ====================================================================== */
#if (ACTIVE_SYSTEM != SYSTEM_SIMPLE) && \
    (ACTIVE_SYSTEM != SYSTEM_BENCHMARK) && \
    (ACTIVE_SYSTEM != SYSTEM_MILANO) && \
    (ACTIVE_SYSTEM != SYSTEM_BUCK) && \
    (ACTIVE_SYSTEM != SYSTEM_BUCK_LOSS)
  #error "Unrecognized ACTIVE_SYSTEM!"
#endif

#if (CTRL_MODE != CTRL_MODE_SCMPC) && \
    (CTRL_MODE != CTRL_MODE_PL) && \
    (CTRL_MODE != CTRL_MODE_AL)
  #error "Unrecognized CTRL_MODE!"
#endif

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
     Z(k) = { thetaCenter + thetaGens*ξ : || ξ ||inf <= 1 }.
     Updated each step by boundStripZonotopeIntersection (PL/AL modes).

   yHist / uHist - ARX regressor history buffers
     yHist[0] = y(k-1), yHist[1] = y(k-2), ..., yHist[na-1]  = y(k-na)
     uHist[0] = u(k-1), uHist[1] = u(k-2), ..., uHist[nb+nk-2] = u(k-nb-nk+1)
     Updated each step inside controller().
   ====================================================================== */
extern theta_type  thetaCenter[nTheta];
extern theta_type  thetaGens  [nTheta][nGens];
extern norm_output_type yHist[na];
extern norm_input_type  uHist[nb + nk - 1];


/* ======================================================================
   MPC PARAMETERS
   ====================================================================== */

/** Prediction horizon N: controller optimises over the next N steps. */
constexpr int Nhor  = 5;

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
constexpr int Nscen      = 4;   /* should always be a power of 2 */
constexpr int LOG2NSCEN  = 2;   /* must satisfy (1 << LOG2NSCEN) == Nscen */

static_assert(NhorU <= Nhor,
    "Control horizon NhorU must not exceed prediction horizon Nhor.");
static_assert((1 << LOG2NSCEN) == Nscen,
    "LOG2NSCEN must equal log2(Nscen).  Update both consistently.");
static_assert(Nhor >= nk, 
    "Prediction horizon must be at least system delay.");
static_assert(Nhor - nk >= NhorU - 1,
    "Control horizon is longer than maximum.");

/** Number of scalar optimisation variables (= Nu for a SISO system). */
#define nOpt NhorU

/* ======================================================================
   MPC COST-FUNCTION WEIGHTS
   ======================================================================
   Stage cost per step:  l(y, u) = outputWeight*(y - y_ref)^2 + R*u^2
   Terminal cost:        V_f(y)   = terminalOutputWeight*(y(k+N) - y_ref)^2
   ====================================================================== */
static const output_weight_type terminalOutputWeight =  4.0;   /* terminal output weight */
static const output_weight_type outputWeight =  4.0;   /* stage   output weight  */
static const input_weight_type RBaseLine = 12.5; /* so that R = 0.125 = 2^(-3) in NRMLZ */
/* 
    log2X are used for shift operations instead of multiplications in
    cost function computation. These should be consistent with the outputWeight, terminalOutputWeight, R
    values and with R value in particular since it depends on FIXED and NRMLZ
    operation modes.
*/
constexpr int log2Q = 2;
constexpr int log2P = 2;

/* ======================================================================
   MADS SOLVER PARAMETERS
   ======================================================================
   The Mesh Adaptive Direct Search (MADS) algorithm manages a frame size
   updated multiple times in a single time step(success: +c, failure: -c).
   Sizes are stored in log-scale (frameIdx, meshIdx) as integers for
   efficient hardware arithmetic.
   ====================================================================== */
constexpr int MADS_ITER = 7;   /* MADS iterations per controller call     */
constexpr int TAU       = 1;   /* frame-size update base                  */
constexpr int MADS_C    = 1;   /* frame-size exponent step  (integer > 0) */
constexpr int FRAME_EXP_MIN = -12; /* frameExp minimum value */

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

   In software mode the gains are derived from UMIN/UMAX/YMIN/YMAX.
   In fixed-point mode they are explicit literals (ap_fixed<> prevents
   compile-time arithmetic on non-constexpr types).
   ====================================================================== */
#ifdef CONVERSIONS_MODE
constexpr int ADC_MAX   = 4095;
constexpr int ADC_MIN   = 0;
constexpr int ADC_RANGE = ADC_MAX - ADC_MIN;

static const output_adc_coeff_type YADCGain = double(ADC_RANGE) / double(YMAX - YMIN);
static const output_dac_coeff_type YDACGain = double(YMAX - YMIN) / double(ADC_RANGE);
static const digital_output_type  YBias    = (double(ADC_MIN)*double(YMAX) - double(ADC_MAX)*double(YMIN)) / double(YMAX - YMIN);
static const input_adc_coeff_type UADCGain = double(ADC_RANGE) / double(UMAX - UMIN);
static const input_dac_coeff_type UDACGain = double(UMAX - UMIN) / double(ADC_RANGE);
static const digital_input_type   UBias    = (double(ADC_MIN)*double(UMAX) - double(ADC_MAX)*double(UMIN)) / double(UMAX - UMIN);
#endif

#ifdef NRMLZ
/* Set these from user? */
static const norm_output_type YNORMMAX = 1;
static const norm_output_type YNORMMIN = -1;
static const norm_input_type UNORMMAX = 1;
static const norm_input_type UNORMMIN = -1;

static const y_norm_coeff_type yNormGain = double(YNORMMAX - YNORMMIN) / double(YMAX - YMIN);
static const u_norm_coeff_type uNormGain = double(UNORMMAX - UNORMMIN) / double(UMAX - UMIN);
static const u_norm_inv_coeff_type uNormGainInverse = double(UMAX - UMIN) / double(UNORMMAX - UNORMMIN);
static const norm_output_type yNormOffset = double(YNORMMIN*YMAX - YNORMMAX*YMIN) / double(YMAX - YMIN);
static const norm_input_type uNormOffset = double(UNORMMIN*UMAX - UNORMMAX*UMIN) / double(UMAX - UMIN);

static const input_weight_type R = double(RBaseLine) * double(yNormGain) * double(yNormGain) / (double(uNormGain) * double(uNormGain)); // 2^(-3)
constexpr int log2R = -3;

/* Normalization, use "normalized" noise in output strip */
const norm_noise_type sigma = double(yNormGain)*double(SIGMA_UNNORM);
static const norm_output_type DELTAYNORM = double(yNormGain) * double(DELTAY);

/* Pre-computed in MATLAB 
 * A better way to compute these offline is needed.
*/
#if ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS || ACTIVE_SYSTEM == SYSTEM_BUCK
static const strip_coeff_type myInvDmDg[nTheta] = {0.221790861648681, 0.179200105412089, 0.033957216705635};
static const strip_q_coeff_type qmyInvDmDg[nTheta] = {0.221790861648681,   0.179200105412089,   0.033957216705635}; // includes '-' sign for output sample prediction
static const strip_coeff_c0_type myInvDmc0[nTheta] = {1.817972144631566,  -0.871463786797535,   0.045754355723659};
static const strip_q_coeff_c0_type qmyInvDmc0 = 0.992262713557689; // includes '-' sign for output sample prediction
#else
#error "Conversion variables for unknwon system could not be defined!"
#endif
#else
const norm_noise_type sigma = SIGMA_UNNORM; // no normalization, use normal noise in output strip
static const input_weight_type R = RBaseLine;   /* stage   input  weight  */
constexpr int log2R = 4;
static const norm_output_type YNORMMAX = YMAX;
static const norm_output_type YNORMMIN = YMIN;
static const norm_input_type UNORMMAX = UMAX;
static const norm_input_type UNORMMIN = UMIN;
static const norm_output_type DELTAYNORM = DELTAY;
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
 *  ypred(k) = theta^T * [y(k-1),...,y(k-na), u(k-nk),...,u(k-nk-nb+1)]^T */
norm_output_type computeArxOutput(const norm_output_type yPast[na],
                                  const norm_input_type uSamples[nb+nk-1],
                                  const theta_type theta[nTheta]);

/* --- MPC cost function ------------------------------------------------ */

/** Evaluate scenario-based MPC cost over the control horizon.
 *  cost[0] = objective value;  cost[1] = constraint violation. */
void costFunctionArx(cost_type              cost[2],
                     const norm_output_type yPast          [na],
                     const norm_input_type  currU          [nOpt],
                     const norm_input_type  uPast          [nb + nk - 2],
                     const norm_output_type yref,
                     const theta_type       thetaScenarios [Nscen][nTheta]);

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
void generateScenarios(theta_type       thetaScenarios[Nscen][nTheta],
                       const theta_type center        [nTheta],
                       const theta_type gens          [nTheta][nGens]);

/* --- MADS main loop --------------------------------------------------- */

/** Run MADS_ITER iterations and return the optimal input sequence. */
void MADSARX(norm_input_type uOpt[nOpt], 
            const norm_input_type uInit[nb + nk - 2], 
            const norm_output_type yInit[na], 
            const norm_output_type yref, 
            const theta_type thetaScenarios[Nscen][nTheta]);

/** Evaluate all 2*nOpt poll candidates; update the best feasible point
 *  via the progressive barrier strategy. */
void progressiveBarrierPollingArx(cost_type bestCost[2], 
                                    norm_input_type bestPoint[nOpt], 
                                    const norm_output_type yPast[na], 
                                    const norm_input_type uPast[nb + nk - 2], 
                                    const norm_output_type yref, 
                                    const norm_input_type pollMatrix[nOpt][2 * nOpt], 
                                    mesh_exp_type frameExp[nOpt], 
                                    const theta_type thetaScenarios[Nscen][nTheta]);

/* --- Pseudo-random generation (three overloads) ----------------------- */
/** Random number in [-1, 1] */
rand_type pseudoRandArx();

/** nGens random coefficients in [-1, 1] for zonotope scenario sampling. */
void pseudoRandArx(rand_type coeffs[nGens]);

/* --- Constraint violation --------------------------------------------- */

/** Update cost[1] (the progressive-barrier violation term) given the
 *  current predicted output currY and the bounds YMIN/YMAX. */
void updateConstraintViolation(cost_type cost[2], const norm_output_type yCurr, const norm_output_type yPast);

/* --- ADC / DAC conversions -------------------------------------------- */
#ifdef CONVERSIONS_MODE
digital_output_type ADConvertY(const output_type       yAn);
output_type         DAConvertY(const digital_output_type yDig);
digital_input_type  ADConvertU(const input_type         uAn);
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
 *   oldCenter     - current zonotope centre  (length nTheta)
 *   oldGens       - current generator matrix (nTheta * nGens);
 *                   only the first nGen columns are read
 *   nGen          - number of active generator columns
 *
 * Outputs:
 *   newCenter     - updated zonotope centre  (length nTheta)
 *   newGens       - updated generator matrix (nTheta * nGens);
 *                   exactly nGen columns are written (same as input)
 */
void boundStripZonotopeIntersectionNew(const strip_center_type stripCenter, 
                                    const phi_type phi[nTheta],
                                    const norm_noise_type stripRadius,
                                    const theta_type oldCenter[nTheta],
                                    const theta_type oldGens[nTheta][nGens], 
                                    theta_type newCenter[nTheta], 
                                    theta_type newGens[nTheta][nGens]);

#endif  /* CTRL_MODE == CTRL_MODE_PL */
// Zonotope volume computation
vol_type zonotopeVolume(const theta_type G[nTheta][nGens]);

// (Generators) matrix determinant computation
det_type matDet(const theta_type M[nTheta][nTheta]);

