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
   TARGET SELECTION - edit only these three lines between experiments
   ====================================================================== */

/** Hardware synthesis target.  Comment out for PC simulation. */
// #define FIXED

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
#define CTRL_MODE        CTRL_MODE_PL

/* ======================================================================
   Derived feature flags (do NOT edit)
   ====================================================================== */
#ifdef FIXED
  #define PRAGMAS           /* enable HLS synthesis pragmas in .cpp files  */
  #define CONVERSIONS_MODE  /* ADC/DAC functions always needed in hardware  */
  #include <ap_fixed.h>
#else
  // #define DEBUG_PRINT    /* uncomment to see debug printfs */
  // #define PRNG_STDLIB    /* use rand() as prng */
  // #define CONVERSIONS_MODE  /* sue ADC/DAC functions */
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
   ARX MODEL DIMENSIONS  (compile-time macros - must remain #define)
   ======================================================================
   These are #define macros rather than constexpr ints because they appear
   as array sizes inside function prototypes, where the C++ standard
   requires an integral constant expression at the point of declaration.
   constexpr works in a single-TU build under C++14 but #define is the
   safest choice across all HLS front-ends.
   ====================================================================== */
#if   ACTIVE_SYSTEM == SYSTEM_SIMPLE
  #define na   2
  #define nb   1
  #define nk   2
  #define nGens 3
  #define sigma 0.0
#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
  #define na   2
  #define nb   2
  #define nk   1
  #define nGens 6
  #define sigma 0.20
#elif ACTIVE_SYSTEM == SYSTEM_MILANO
  #define na   3
  #define nb   3
  #define nk   1
  #define nGens 6
  #define sigma 4.4655
#elif ACTIVE_SYSTEM == SYSTEM_BUCK
  #define na   2
  #define nb   1
  #define nk   2
  #define nGens 3
  #define sigma 0.2
#elif ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
  #define na  2
  #define nb   1
  #define nk   2
  #define nGens 3
  #define sigma 0.02
#endif

#define nTheta  (na + nb)       /* total ARX parameter count */

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
extern output_type yHist[na];
extern input_type  uHist[nb + nk - 1];


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
constexpr int Nscen      = 4;
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
   INPUT / OUTPUT HARD CONSTRAINTS
   ======================================================================
   These are the box constraints used by the progressive barrier in MADSARX.
   They must be consistent with the ones for the ACTIVE_SYSTEM.
   They are repeated as separate constants because ap_fixed<> prevents
   deriving them from the config struct at compile time via constexpr.
   ====================================================================== */
#if   ACTIVE_SYSTEM == SYSTEM_SIMPLE
  static const input_type  UMIN = -0.3;
  static const input_type  UMAX =  0.3;
  static const output_type YMIN =  0.0;
  static const output_type YMAX =  8.0;

#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
  static const input_type  UMIN = -1.9;
  static const input_type  UMAX =  1.9;
  static const output_type YMIN = -10.0;
  static const output_type YMAX =  8.0;

#elif ACTIVE_SYSTEM == SYSTEM_MILANO
static const input_type  UMIN = -200.0;
static const input_type  UMAX =  200.0;
static const output_type YMIN = -110.0;
static const output_type YMAX =  110.0;

#elif ACTIVE_SYSTEM == SYSTEM_BUCK
static const input_type  UMIN = 0;
static const input_type  UMAX = 1;
static const output_type YMIN = 0;
static const output_type YMAX = 10;

#elif ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
static const input_type  UMIN = 0;
static const input_type  UMAX = 1;
static const output_type YMIN = 0;
static const output_type YMAX = 10;

#endif

/* ======================================================================
   MPC COST-FUNCTION WEIGHTS
   ======================================================================
   Stage cost per step:  l(y, u) = Q*(y - y_ref)^2 + R*u^2
   Terminal cost:        V_f(y)   = P*(y(k+N) - y_ref)^2
   ====================================================================== */
static const weights_type P =  5.0;   /* terminal output weight */
static const weights_type Q =  5.0;   /* stage   output weight  */
static const weights_type R = 0.1;   /* stage   input  weight  */

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

/*
 * expC = 2^{-MADS_C}: pre-computed scaling factor for the mesh update.
 * In FIXED mode this is exactly 0.5 = 2^{-1}, needing only 1 integer bit,
 * so a 1-bit ap_ufixed is used to save multiplier resources.
 * If MADS_C changes, update expC and its type consistently.
 */
#ifdef FIXED
  static const ap_ufixed<1, 0, AP_TRN, AP_WRAP> expC = 0.5;
#else
  static const input_type expC = 0.5;
#endif

/*
 * D0[j]: initial log2 frame-size exponent for optimisation variable j.
 * Initial frame size = tau^D0[j].  More negative -> finer initial mesh.
 * HLS hint: #pragma HLS ARRAY_PARTITION variable=D0 complete dim=1
 */
#define D0_VAL -8 // used in MADSARX, frameIdx init
static const mesh_exp_type D0[nOpt] = { -8, -8, -8 };

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
constexpr int ADC_MAX   = 4095;
constexpr int ADC_MIN   = 0;
constexpr int ADC_RANGE = ADC_MAX - ADC_MIN;

static const conv_type            YADCGain = (conv_type)ADC_RANGE / (conv_type)(YMAX - YMIN);
static const conv_type            YDACGain = (conv_type)(YMAX - YMIN) / (conv_type)ADC_RANGE;
static const digital_output_type  YBias    = (conv_type)(ADC_MIN*YMAX - ADC_MAX*YMIN) / (conv_type)(YMAX - YMIN);
static const conv_type            UADCGain = (conv_type)ADC_RANGE / (conv_type)(UMAX - UMIN);
static const conv_type            UDACGain = (conv_type)(UMAX - UMIN) / (conv_type)ADC_RANGE;
static const digital_input_type   UBias    = (conv_type)(ADC_MIN*UMAX - ADC_MAX*UMIN) / (conv_type)(UMAX - UMIN);

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
output_type computeArxOutput(const output_type yPast   [na],
                              const input_type  uSamples[nb + nk - 1],
                              const theta_type  theta   [nTheta]);

/* --- MPC cost function ------------------------------------------------ */

/** Evaluate scenario-based MPC cost over the control horizon.
 *  cost[0] = objective value;  cost[1] = constraint violation. */
void costFunctionArx(cost_type         cost            [2],
                     const output_type yPast           [na],
                     const input_type  currU           [nOpt],
                     const input_type  uPast           [nb + nk - 2],
                     const output_type yref,
                     const theta_type  thetaScenarios  [Nscen][nTheta]);

/* --- MADS poll-step --------------------------------------------------- */

/** Build 2*nOpt poll directions from a random unit vector and the
 *  current mesh/frame exponent vectors. */
void generatePollDirectionsArx(const rand_type     randomVector[nOpt],
                                const mesh_exp_type frameIdx    [nOpt],
                                const mesh_exp_type meshIdx     [nOpt],
                                direction_type      directions  [nOpt][2*nOpt]);

/** Scale the poll directions into a full poll matrix. */
void generatePollMatrixArx(const input_type    currU     [nOpt],
                            const mesh_exp_type frameIdx  [nOpt],
                            const mesh_exp_type meshIdx   [nOpt],
                            input_type          pollMatrix[nOpt][2*nOpt]);

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
void MADSARX(input_type uOpt[nOpt], 
                const input_type uInit[nb + nk - 2], 
                const output_type yInit[na], 
                const output_type yref, 
                const theta_type thetaScenarios[Nscen][nTheta]);

/** Evaluate all 2*nOpt poll candidates; update the best feasible point
 *  via the progressive barrier strategy. */
void progressiveBarrierPollingArx(cost_type         bestCost   [2],
                                   input_type        bestPoint  [nOpt],
                                   const output_type yPast      [na],
                                   const input_type  uPast      [nb + nk - 2],
                                   output_type       yref,
                                   const input_type  pollMatrix [nOpt][2*nOpt],
                                   mesh_exp_type     frameExp   [nOpt],
                                   const theta_type  thetaScenarios[Nscen][nTheta]);

/* --- Pseudo-random generation (three overloads) ----------------------- */
/** Random number in [-1, 1] */
rand_type pseudoRandArx();

/** nGens random coefficients in [-1, 1] for zonotope scenario sampling. */
void pseudoRandArx(rand_type coeffs[nGens]);

/* --- Constraint violation --------------------------------------------- */

/** Update cost[1] (the progressive-barrier violation term) given the
 *  current predicted output currY and the bounds YMIN/YMAX. */
void updateConstraintViolation(cost_type cost[2], const output_type currY);

/* --- ADC / DAC conversions -------------------------------------------- */
#ifdef CONVERSIONS_MODE
digital_output_type ADConvertY(const output_type       yAn);
output_type         DAConvertY(const digital_output_type yDig);
digital_input_type  ADConvertU(const input_type         uAn);
input_type          DAConvertU(const digital_input_type  uDig);
#endif

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
void boundStripZonotopeIntersectionNew(const output_type yCurr, 
                                    const output_type yPast[na], 
                                    const input_type uSamples[nb + nk - 1], 
                                    const theta_type oldCenter[nTheta],
                                    const theta_type oldGens[nTheta][nGens], 
                                    theta_type newCenter[nTheta], 
                                    theta_type newGens[nTheta][nGens]);

// Zonotope volume computation
alg_type zonotopeVolume(const theta_type G[nTheta][nGens]);

// (Generators) matrix determinant computation
alg_type matDet(const theta_type M[nTheta][nTheta]);

#endif  /* CTRL_MODE == CTRL_MODE_PL */
