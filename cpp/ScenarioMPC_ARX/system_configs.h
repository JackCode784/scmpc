/**
 * @file  system_configs.h
 * @brief Compile-time system configurations for SCMPC on ARX systems.
 *
 * System registry
 * ---------------
 *  SYSTEM_SIMPLE     (id 0)  na=2, nb=1, nk=2 - original non-CMPLSYS ARX
 *  SYSTEM_BENCHMARK  (id 1)  na=2, nb=2, nk=1 - benchmark ARX (selectSys.m)
 *  SYSTEM_MILANO     (id 2)  na=3, nb=3, nk=1 - original CMPLSYS (BESS)
 *  SYSTEM_BUCK       (id 3)  na=2, nb=1, nk=2 - buck power converter
 *  SYSTEM_BUCK_LOSS  (id 4)  na=2, nb=1, nk=2 - buck power converter with loss resistance
 *  SYSTEM_GAIN_DEMO  (id 5)  na=1, nb=1, nk=2 - minimal demo: is scenario MPC worth it?
 *
 * Why fixed-size arrays in the structs?
 * --------------------------------------
 * C++ (and ANSI C) require every struct member to have a size known at
 * compile time - it is impossible to write "theta_type c[nTheta]" inside
 * a struct because nTheta is a runtime value for a generic struct.
 * Vitis HLS adds a stronger constraint: it forbids ALL dynamic memory
 * allocation (no new/delete, no std::vector) because hardware circuits
 * must be fully sized at synthesis time.
 *
 * Generator-matrix shape for PL mode
 * ------------------------------------
 * The passive-learning (PL) strip-intersection step expands the generator
 * matrix from nTheta columns to nTheta+1 (a temporary wider zonotope).
 * The subsequent interval-hull reduction brings it back to nTheta columns
 * (a square matrix).  Therefore the PERSISTENT controller state always
 * holds a square nTheta*nTheta generator matrix.
 */

#pragma once

/* ======================================================================
   System identifiers - plain integers so they work in #if expressions.
   ======================================================================
   Defined BEFORE #include "types.h": types.h selects its system-dependent
   fixed-point types with "#if ACTIVE_SYSTEM == SYSTEM_...". In an #if, an
   identifier that is not (yet) a macro evaluates to 0, so if these ids
   were defined afterwards every such comparison would read 0 == 0 and be
   true for EVERY system - which is how SYSTEM_BUCK_ALBERTO used to get
   SYSTEM_BUCK_LOSS's output_type, whose +-16 range saturates its
   YMAXPHYS = 100 and YMAX = 85 in FIXED builds.
   ====================================================================== */
#define SYSTEM_SIMPLE     0
#define SYSTEM_BENCHMARK  1
#define SYSTEM_MILANO     2
#define SYSTEM_BUCK       3
#define SYSTEM_BUCK_LOSS  4
#define SYSTEM_BUCK_ALBERTO  5
#define SYSTEM_GAIN_DEMO  6
#define SYSTEM_INVERTED_PENDULUM  7

#include "types.h"

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
#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
  #define na   2
  #define nb   2
  #define nk   1
  #define nGens 6
  #elif ACTIVE_SYSTEM == SYSTEM_MILANO
  #define na   3
  #define nb   3
  #define nk   1
  #define nGens 6
  #elif ACTIVE_SYSTEM == SYSTEM_BUCK
  #define na   2
  #define nb   1
  #define nk   2
  #define nGens 3
  #elif ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
  #define na  2
  #define nb   1
  #define nk   2
  #define nGens 3
  #elif ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
  #define na  2
  #define nb   1
  #define nk   2
  #define nGens 3
  #elif ACTIVE_SYSTEM == SYSTEM_GAIN_DEMO
  #define na   1
  #define nb   1
  #define nk   2
  #define nGens 2
  #elif ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
  #define   na  2
  #define   nb  1
  #define   nk  2
  #define   nGens  3
  #endif
  #define nTheta  (na + nb)       /* total ARX parameter count */
  
  
  /* ======================================================================
  INPUT / OUTPUT HARD CONSTRAINTS
  ======================================================================
  These are the box constraints used by the progressive barrier in MADSARX.
  They must be consistent with the ones for the ACTIVE_SYSTEM.
  Each NAME_V is the constant's value as a compile-time double (already
  rounded to its type by apq<>), and NAME builds the fixed-point value
  where it is used: see "Compile-time constants" at the end of types.h
  for why there are no static const ap_fixed globals.
  ====================================================================== */
#if   ACTIVE_SYSTEM == SYSTEM_SIMPLE
constexpr double UMINPHYS_V = apq<input_type>(-0.3);
#define UMINPHYS (input_type(UMINPHYS_V))
constexpr double UMAXPHYS_V = apq<input_type>(0.3);
#define UMAXPHYS (input_type(UMAXPHYS_V))
constexpr double YMINPHYS_V = apq<output_type>(0.0);
#define YMINPHYS (output_type(YMINPHYS_V))
constexpr double YMAXPHYS_V = apq<output_type>(8.0);
#define YMAXPHYS (output_type(YMAXPHYS_V))
constexpr double UMIN_V = apq<input_type>(UMINPHYS_V);
#define UMIN (input_type(UMIN_V))
constexpr double UMAX_V = apq<input_type>(UMAXPHYS_V);
#define UMAX (input_type(UMAX_V))
constexpr double YMIN_V = apq<output_type>(YMINPHYS_V);
#define YMIN (output_type(YMIN_V))
constexpr double YMAX_V = apq<output_type>(YMAXPHYS_V);
#define YMAX (output_type(YMAX_V))
static const noise_type SIGMA_UNNORM` = 0.0;

#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
constexpr double UMINPHYS_V = apq<input_type>(-1.9);
#define UMINPHYS (input_type(UMINPHYS_V))
constexpr double UMAXPHYS_V = apq<input_type>(1.9);
#define UMAXPHYS (input_type(UMAXPHYS_V))
constexpr double YMINPHYS_V = apq<output_type>(-10.0);
#define YMINPHYS (output_type(YMINPHYS_V))
constexpr double YMAXPHYS_V = apq<output_type>(8.0);
#define YMAXPHYS (output_type(YMAXPHYS_V))
constexpr double UMIN_V = apq<input_type>(UMINPHYS_V);
#define UMIN (input_type(UMIN_V))
constexpr double UMAX_V = apq<input_type>(UMAXPHYS_V);
#define UMAX (input_type(UMAX_V))
constexpr double YMIN_V = apq<output_type>(YMINPHYS_V);
#define YMIN (output_type(YMIN_V))
constexpr double YMAX_V = apq<output_type>(YMAXPHYS_V);
#define YMAX (output_type(YMAX_V))
constexpr double SIGMA_UNNORM_V = apq<noise_type>(0.20);
#define SIGMA_UNNORM (noise_type(SIGMA_UNNORM_V))

#elif ACTIVE_SYSTEM == SYSTEM_MILANO
constexpr double UMINPHYS_V = apq<input_type>(-200.0);
#define UMINPHYS (input_type(UMINPHYS_V))
constexpr double UMAXPHYS_V = apq<input_type>(200.0);
#define UMAXPHYS (input_type(UMAXPHYS_V))
constexpr double YMINPHYS_V = apq<output_type>(-110.0);
#define YMINPHYS (output_type(YMINPHYS_V))
constexpr double YMAXPHYS_V = apq<output_type>(110.0);
#define YMAXPHYS (output_type(YMAXPHYS_V))
constexpr double UMIN_V = apq<input_type>(UMINPHYS_V);
#define UMIN (input_type(UMIN_V))
constexpr double UMAX_V = apq<input_type>(UMAXPHYS_V);
#define UMAX (input_type(UMAX_V))
constexpr double YMIN_V = apq<output_type>(YMINPHYS_V);
#define YMIN (output_type(YMIN_V))
constexpr double YMAX_V = apq<output_type>(YMAXPHYS_V);
#define YMAX (output_type(YMAX_V))
constexpr double SIGMA_UNNORM_V = apq<noise_type>(4.4655);
#define SIGMA_UNNORM (noise_type(SIGMA_UNNORM_V))

#elif ACTIVE_SYSTEM == SYSTEM_BUCK
constexpr double UMINPHYS_V = apq<input_type>(0);
#define UMINPHYS (input_type(UMINPHYS_V))
constexpr double UMAXPHYS_V = apq<input_type>(1);
#define UMAXPHYS (input_type(UMAXPHYS_V))
constexpr double YMINPHYS_V = apq<output_type>(0);
#define YMINPHYS (output_type(YMINPHYS_V))
constexpr double YMAXPHYS_V = apq<output_type>(10);
#define YMAXPHYS (output_type(YMAXPHYS_V))
constexpr double UMIN_V = apq<input_type>(UMINPHYS_V);
#define UMIN (input_type(UMIN_V))
constexpr double UMAX_V = apq<input_type>(UMAXPHYS_V);
#define UMAX (input_type(UMAX_V))
constexpr double YMIN_V = apq<output_type>(YMINPHYS_V);
#define YMIN (output_type(YMIN_V))
constexpr double YMAX_V = apq<output_type>(YMAXPHYS_V);
#define YMAX (output_type(YMAX_V))
constexpr double SIGMA_UNNORM_V = apq<noise_type>(0.2);
#define SIGMA_UNNORM (noise_type(SIGMA_UNNORM_V))

#elif ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
constexpr double UMINPHYS_V = apq<input_type>(0);
#define UMINPHYS (input_type(UMINPHYS_V))
constexpr double UMAXPHYS_V = apq<input_type>(1);
#define UMAXPHYS (input_type(UMAXPHYS_V))
constexpr double YMINPHYS_V = apq<output_type>(0);
#define YMINPHYS (output_type(YMINPHYS_V))
constexpr double YMAXPHYS_V = apq<output_type>(10);
#define YMAXPHYS (output_type(YMAXPHYS_V))
constexpr double UMIN_V = apq<input_type>(UMINPHYS_V);
#define UMIN (input_type(UMIN_V))
constexpr double UMAX_V = apq<input_type>(UMAXPHYS_V);
#define UMAX (input_type(UMAX_V))
constexpr double YMIN_V = apq<output_type>(YMINPHYS_V);
#define YMIN (output_type(YMIN_V))
constexpr double YMAX_V = apq<output_type>(YMAXPHYS_V);
#define YMAX (output_type(YMAX_V))
constexpr double DELTAY_V = apq<output_type>(0.1);
#define DELTAY (output_type(DELTAY_V))
constexpr double SIGMA_UNNORM_V = apq<noise_type>(0.02);
#define SIGMA_UNNORM (noise_type(SIGMA_UNNORM_V))

#elif ACTIVE_SYSTEM == SYSTEM_GAIN_DEMO
constexpr double UMINPHYS_V = apq<input_type>(0);
#define UMINPHYS (input_type(UMINPHYS_V))
constexpr double UMAXPHYS_V = apq<input_type>(1.5);
#define UMAXPHYS (input_type(UMAXPHYS_V))
constexpr double YMINPHYS_V = apq<output_type>(0);
#define YMINPHYS (output_type(YMINPHYS_V))
constexpr double YMAXPHYS_V = apq<output_type>(7);
#define YMAXPHYS (output_type(YMAXPHYS_V))
constexpr double UMIN_V = apq<input_type>(UMINPHYS_V);
#define UMIN (input_type(UMIN_V))
constexpr double UMAX_V = apq<input_type>(UMAXPHYS_V);
#define UMAX (input_type(UMAX_V))
constexpr double YMIN_V = apq<output_type>(YMINPHYS_V);
#define YMIN (output_type(YMIN_V))
constexpr double YMAX_V = apq<output_type>(5);
#define YMAX (output_type(YMAX_V))
constexpr double DELTAY_V = apq<output_type>(100); /* effectively unconstrained: this demo is about the static YMAX bound, not the rate limit */
#define DELTAY (output_type(DELTAY_V))
constexpr double SIGMA_UNNORM_V = apq<noise_type>(0.02);
#define SIGMA_UNNORM (noise_type(SIGMA_UNNORM_V))

#elif ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
constexpr double UMINPHYS_V = apq<input_type>(0);
#define UMINPHYS (input_type(UMINPHYS_V))
constexpr double UMAXPHYS_V = apq<input_type>(1);
#define UMAXPHYS (input_type(UMAXPHYS_V))
constexpr double YMINPHYS_V = apq<output_type>(0);
#define YMINPHYS (output_type(YMINPHYS_V))
constexpr double YMAXPHYS_V = apq<output_type>(100);
#define YMAXPHYS (output_type(YMAXPHYS_V))
constexpr double UMIN_V = apq<input_type>(UMINPHYS_V);
#define UMIN (input_type(UMIN_V))
constexpr double UMAX_V = apq<input_type>(UMAXPHYS_V);
#define UMAX (input_type(UMAX_V))
constexpr double YMIN_V = apq<output_type>(YMINPHYS_V);
#define YMIN (output_type(YMIN_V))
constexpr double YMAX_V = apq<output_type>(85);
#define YMAX (output_type(YMAX_V))
constexpr double DELTAY_V = apq<output_type>(5);
#define DELTAY (output_type(DELTAY_V))
constexpr double SIGMA_UNNORM_V = apq<noise_type>(0.2);
#define SIGMA_UNNORM (noise_type(SIGMA_UNNORM_V))

#elif ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
constexpr double UMINPHYS_V = apq<input_type>(-3);
#define UMINPHYS (input_type(UMINPHYS_V))
constexpr double UMAXPHYS_V = apq<input_type>(3);
#define UMAXPHYS (input_type(UMAXPHYS_V))
constexpr double YMINPHYS_V = apq<output_type>(-0.9);
#define YMINPHYS (output_type(YMINPHYS_V))
constexpr double YMAXPHYS_V = apq<output_type>(0.9);
#define YMAXPHYS (output_type(YMAXPHYS_V))
constexpr double UMIN_V = apq<input_type>(UMINPHYS_V);
#define UMIN (input_type(UMIN_V))
constexpr double UMAX_V = apq<input_type>(UMAXPHYS_V);
#define UMAX (input_type(UMAX_V))
constexpr double YMIN_V = apq<output_type>(YMINPHYS_V);
#define YMIN (output_type(YMIN_V))
constexpr double YMAX_V = apq<output_type>(YMAXPHYS_V);
#define YMAX (output_type(YMAX_V))
constexpr double DELTAY_V = apq<output_type>(10e-3);
#define DELTAY (output_type(DELTAY_V))
constexpr double SIGMA_UNNORM_V = apq<noise_type>(0.00);
#define SIGMA_UNNORM (noise_type(SIGMA_UNNORM_V))

#endif

/* ======================================================================
   INITIAL ZONOTOPE AND INITIAL CONDITIONS - physical (unnormalized) units
   ======================================================================
   Per system, only physical values are given here:
     THETA_NOMINAL_UNNORM  initial zonotope centre c0        (nTheta)
     GENERATORS_UNNORM     initial zonotope generators G     (nTheta x nGens)
                           A parameter known EXACTLY gets an all-zero row:
                           it is then left out of the zonotope, which only
                           spans the nUnc uncertain parameters (setup.h,
                           UNC_MAP), and enters predictions as its
                           THETA_NOMINAL_UNNORM value.
     THETA_TRUE_INIT       true plant parameters (testMain.cpp only)
     Y_HIST_UNNORM         initial output samples y(-1), ..., y(-na)
     U_HIST_UNNORM         initial input samples, nb+nk-1 values
     U_PREV_UNNORM         initial MADS warm start, NhorU values - must be
                           the same operating point as U_HIST_UNNORM (see
                           uOptPrev in controller.cpp)
   With NRMLZ, controller.cpp's computeControllerInit() derives the
   normalized zonotope and samples from these at compile time
   (see there); without NRMLZ it copies them unchanged.
   ====================================================================== */
#if ACTIVE_SYSTEM == SYSTEM_SIMPLE
    #define THETA_NOMINAL_UNNORM   2.0, -1.0, 1.0
    #define GENERATORS_UNNORM \
            { 0.0,  0.0,  0.0}, \
            { 0.0,  0.0,  0.0}, \
            { 0.0,  0.0,  0.5}
    #define THETA_TRUE_INIT     2.0,  -1.0,   1.0 // == THETA_NOMINAL_UNNORM
    /* cold start at physical 0, as for the other systems */
    #define Y_HIST_UNNORM 0, 0
    #define U_HIST_UNNORM 0, 0
    #define U_PREV_UNNORM 0, 0, 0
#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
    #define THETA_NOMINAL_UNNORM  1.50,  -0.70,   1.00,   0.50
    #define GENERATORS_UNNORM      \
            {  0.080,  0.020,  0.005,  0.000,  0.010,  0.000 }, \
            { -0.010,  0.060,  0.000,  0.008,  0.000,  0.005 }, \
            {  0.000,  0.010,  0.070, -0.015,  0.000,  0.010 }, \
            {  0.005,  0.000,  0.012,  0.055, -0.010,  0.000 }
    #define THETA_TRUE_INIT     1.5540,  -0.7433,   1.0570,   0.4761
    #define Y_HIST_UNNORM 0, 0
    #define U_HIST_UNNORM 0, 0
    #define U_PREV_UNNORM 0, 0, 0
#elif ACTIVE_SYSTEM == SYSTEM_MILANO
    #define THETA_NOMINAL_UNNORM     0.7921,  0.1524, -0.1668,  0.0842,  0.0442,  0.0860 
    #define GENERATORS_UNNORM  \
            { -0.8959, -0.4594, -0.0026, -0.0027, -0.0187,  0.0080 },   \
            {  1.3452, -0.1401,  0.0030,  0.0111, -0.0283,  0.0079 },   \
            { -0.5326,  0.4275, -0.0051,  0.0289, -0.0362,  0.0077 },   \
            { -0.0082,  0.0028,  0.0524,  0.0788,  0.0367,  0.0085 },   \
            {  0.0859,  0.0502, -0.1015, -0.0126,  0.0271,  0.0086 },   \
            {  0.0021,  0.1180,  0.0538, -0.0985,  0.0126,  0.0086 }    
    #define THETA_TRUE_INIT     0.7921,  0.1524, -0.1668,  0.0842,  0.0442,  0.0860
    #define Y_HIST_UNNORM 0, 0, 0
    #define U_HIST_UNNORM 0, 0, 0
    #define U_PREV_UNNORM 0, 0, 0
#elif ACTIVE_SYSTEM == SYSTEM_BUCK
    #define THETA_NOMINAL_UNNORM  1.8889, -0.9333, 0.4444
    #define GENERATORS_UNNORM \
            { 0.0625,        0,         0}, \
            {      0,   0.0606,         0}, \
            {      0,        0,    0.2500}  
    #define THETA_TRUE_INIT        1.8612,  -0.9276,    0.6732
    #define Y_HIST_UNNORM 0, 0
    #define U_HIST_UNNORM 0, 0
    #define U_PREV_UNNORM 0, 0, 0
#elif ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
#define THETA_NOMINAL_UNNORM 1.817972144631566,  -0.871463786797535,   0.457543557236585
#define GENERATORS_UNNORM \
        {-0.093616869443703,   0.124811247863353,   0.003362744341625}, \
        {0.057591158572667,  -0.117838849718767,   0.003770097120654}, \
        {0.271071396611763,   0.068140402887871,   0.000360367556716}
#define THETA_TRUE_INIT     1.857831352244614,  -0.933661885378246,   0.683201544134083
#define Y_HIST_UNNORM 0, 0
#define U_HIST_UNNORM 0, 0      // nb+nk-1=2
#define U_PREV_UNNORM 0, 0, 0   // NhorU
#elif ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
#define THETA_NOMINAL_UNNORM 1.733749265658302,  -0.881912845894497,  14.298236163643255
#define GENERATORS_UNNORM \
    {-0.068444143534821,   0.137725841350229,   0.004638817609457}, \
    {-0.008278303729254,  -0.132496642357720,   0.004822306118286}, \
    {7.403892173947610,   0.001125040669095,   0.000048274664814}
#define THETA_TRUE_INIT 1.753571428571429,  -0.894444444444444,  13.888888888888886
#define Y_HIST_UNNORM 0, 0
#define U_HIST_UNNORM 0, 0
#define U_PREV_UNNORM 0, 0, 0

#elif ACTIVE_SYSTEM == SYSTEM_GAIN_DEMO
/**
 * Minimal demonstration system: IS SCENARIO MPC WORTH IT?
 * ------------------------------------------------------------------
 * y(k) = a*y(k-1) + b*u(k-2)   (na=1, nb=1, nk=2 - a first-order ARX
 * with a two-step actuation delay, structurally identical to
 * SYSTEM_BUCK/SYSTEM_BUCK_LOSS's own (nb,nk)=(1,2), just na=1 instead
 * of na=2 for the simplest possible hand-checkable dynamics).
 *
 * theta = [a, b]. The zonotope is diagonal (independent per-parameter
 * uncertainty), exactly like SYSTEM_SIMPLE/SYSTEM_BUCK:
 *   thetaCenter = [a0, b0] = [0.5, 2.0]
 *   GENERATORS  = diag(deltaA, deltaB) = diag(0.05, 1.0)
 *   => a in [0.45, 0.55], b in [1.0, 3.0]
 *
 * thetaTrue = [0.5, 3.0]: 'a' sits EXACTLY at its nominal/center value
 * (ξ_a=0 - the pole is assumed known, only carried as a generator for
 * structural realism/non-singularity of the generator matrix, and to
 * keep matDet(thetaGens) - called unconditionally in testMain.cpp -
 * well-defined; it is deliberately NOT the source of the mismatch this
 * system demonstrates), while 'b' (the INPUT GAIN) sits EXACTLY at the
 * zonotope's vertex ξ_b=+1, i.e. the worst case the controller's own
 * uncertainty description claims is possible.
 *
 * THE IDEALIZED (STATIC) ARGUMENT
 * ---------------------------------
 * DC gain is b/(1-a): G_nominal = 2/0.5 = 4, G_true = 3/0.5 = 6 - the
 * true plant is 50% "stronger" than the model the controller nominally
 * trusts. With YMAX=5 and a reference that steps from 1.0 to 4.9 (see
 * testMain.cpp's generateReference, SYSTEM_GAIN_DEMO branch), a
 * controller that only ever evaluates thetaCenter would compute
 * u* = yref/G_nominal = 4.9/4 = 1.225 (within UMAX=1.5), predicting
 * y=4.9 - safely under YMAX. Applied to the TRUE plant, the actual
 * steady-state output would be G_true*u* = 6*1.225 = 7.35 - a 47%
 * breach the nominal-only controller never sees coming.
 *
 * WHAT ACTUALLY HAPPENS IN CLOSED LOOP (measured, floating point,
 * NRMLZ/FIXED/CONVERSIONS_MODE all undefined - see setup.h's TARGET
 * SELECTION block - Nscen=4, MADS_ITER=7, the project's real values;
 * mean/max taken over simulation steps 200-300, i.e. 100+ steps after
 * the reference step, once the closed loop has settled into its
 * long-run behaviour rather than a transient):
 *
 *   USE_SCENS_CONSTR undefined (constraint checked only against
 *   thetaCenter - certainty-equivalent MPC): output settles at a
 *   PERSISTENT y ~= 6.3-6.6 (26-32% over YMAX=5) - not a transient
 *   overshoot that decays, a genuine steady violation, confirmed
 *   stable out to 1200 steps.
 *
 *   USE_SCENS_CONSTR defined (the codebase's actual default: also
 *   check cost[1] against Nscen=4 parameter draws sampled at random
 *   from the SAME zonotope every prediction step - see
 *   costFunctionArx.cpp/generateScenarios.cpp): output settles at
 *   y ~= 5.6-5.75 (12-15% over YMAX) - roughly HALF the exceedance of
 *   the certainty-equivalent case, a real and substantial improvement,
 *   but NOT a full elimination.
 *
 * WHY NOT A FULL ELIMINATION AT Nscen=4
 * -----------------------------------------
 * This is sample-based, not exact worst-case, robustness: scenarios
 * are redrawn at random every call rather than fixed at the zonotope's
 * vertices, so whether a given call's Nscen*Nhor=20 draws happen to
 * sample close to the b=3 vertex varies call to call, and
 * progressiveBarrierPollingArx's mesh resets to a small fixed size
 * (D0_VAL, setup.h) at the start of every controller() call, so MADS
 * can only make a small, local correction per call rather than jump
 * straight to the fully-robust input. The residual gap is a SAMPLE
 * BUDGET effect, not a broken mechanism: repeating this exact
 * experiment with Nscen bumped to 16 (in an isolated, non-committed
 * build - Nscen is a shared setup.h constant, changing it here would
 * also change SYSTEM_BUCK_LOSS's hardware area) reduced the exceedance
 * to ~2%; Nscen=64 reduced it to ~3 violations out of 100 samples
 * checked, essentially eliminating it. This confirms the mechanism
 * converges toward the idealized worst-case-robust result as the
 * scenario budget grows - it is simply underpowered, not incorrect, at
 * the Nscen=4 budget this project's hardware profiles are sized for.
 *
 * SCOPE: validated in floating point with NRMLZ, FIXED and
 * CONVERSIONS_MODE all undefined - CTRL_MODE_SCMPC only.
 *   - NRMLZ is unsupported: setup.h's NRMLZ branch requires
 *     MATLAB-precomputed myInvDmDg/myInvDmc0/... constants (see
 *     setup.h, "Pre-computed in MATLAB") that only exist for
 *     SYSTEM_BUCK/SYSTEM_BUCK_LOSS and are irrelevant to
 *     CTRL_MODE_SCMPC anyway (they only feed the CTRL_MODE_PL/AL
 *     strip-intersection code path in controller.cpp).
 *   - CONVERSIONS_MODE should stay OFF for this system specifically:
 *     with it on, the 12-bit ADC's full-scale range is [YMIN,YMAX], so
 *     once the true output exceeds YMAX the SENSOR reading fed back to
 *     the controller saturates at the rail - a realistic effect in
 *     general, but here it quietly changes what the controller
 *     believes y(k-1) is once a violation is already underway,
 *     confounding the comparison above with an extra, unintended
 *     nonlinearity. (Fixed in passing: conversions.cpp's ctrlU2dig had
 *     a latent bug referencing ADC_MIN/ADC_MAX - only defined under
 *     CONVERSIONS_MODE - whenever FIXED was undefined regardless of
 *     CONVERSIONS_MODE, making "FIXED off + CONVERSIONS_MODE off" fail
 *     to compile at all before this system needed it.)
 *   - FIXED-point/HLS synthesis support (bit widths in types.h) has
 *     not been added for this system - it exists to answer "is
 *     scenario MPC worth it", not to be synthesised.
 */
#define THETA_NOMINAL_UNNORM  0.5, 2.0
#define GENERATORS_UNNORM \
    {0.05,   0}, \
    {0,      1.0}
#define THETA_TRUE_INIT     0.5, 3.0
#define Y_HIST_UNNORM 0     // na=1
#define U_HIST_UNNORM 0, 0  // nb+nk-1=2
#define U_PREV_UNNORM 0, 0, 0  // NhorU
#elif ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
#define THETA_NOMINAL_UNNORM 2.000000000000000, -0.998821001683502, 0.000751434382693
#define GENERATORS_UNNORM \
    {-0.000001,                       0,                        0}, \
    {0, 1e-3*-0.281338543972914, 1e-4*-0.961862828825779}, \
    {0, 1e-3*-0.386439223972734, 1e-4*0.700262993444492}
#define THETA_TRUE_INIT THETA_NOMINAL_UNNORM
//2.0000000000,   -0.998637500000000,   0.001205632716049
//2.0, -0.999091666666667, 0.0003572245084590763
//2.000000000000000,   -0.998637500000000,   0.001205632716049
#define Y_HIST_UNNORM -0.78, -0.78
#define U_HIST_UNNORM 0, 0
#define U_PREV_UNNORM 0, 0, 0
#endif
