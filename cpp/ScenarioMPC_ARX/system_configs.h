/**
 * @file  system_configs.h
 * @brief System registry: system ids, the active system's file, its types.
 *
 * System registry (one file per system in systems/)
 * ---------------------------------------------------
 *  SYSTEM_SIMPLE            (id 0)  na=2, nb=1, nd=2  original non-CMPLSYS ARX       system_simple.h
 *  SYSTEM_BENCHMARK         (id 1)  na=2, nb=2, nd=1  benchmark ARX (selectSys.m)     system_benchmark.h
 *  SYSTEM_MILANO            (id 2)  na=3, nb=3, nd=1  original CMPLSYS (BESS)         system_milano.h
 *  SYSTEM_BUCK              (id 3)  na=2, nb=1, nd=2  buck power converter            system_buck.h
 *  SYSTEM_BUCK_LOSS         (id 4)  na=2, nb=1, nd=2  buck with loss resistance       system_buck.h
 *  SYSTEM_BUCK_ALBERTO      (id 5)  na=2, nb=1, nd=2  buck, 0-100 V output            system_alberto.h
 *  SYSTEM_GAIN_DEMO         (id 6)  na=1, nb=1, nd=2  "is scenario MPC worth it" demo system_gain_demo.h
 *  SYSTEM_INVERTED_PENDULUM (id 7)  na=2, nb=1, nd=2  inverted pendulum               system_pendulum.h
 *
 * WHAT EVERY SYSTEM FILE DEFINES - the controller's MODEL of the system,
 * compiled into the hardware (the true plant is test-bench code, plant.h):
 *
 *   Dimensions (constexpr int)
 *     na, nb, nd, nGens, nTheta = na + nb
 *
 *   Physical constants (constexpr double), checked below
 *     UMINPHYS, UMAXPHYS, YMINPHYS, YMAXPHYS
 *                    range of the converters and of the normalization
 *     UMIN, UMAX, YMIN, YMAX
 *                    box constraints, inside the ranges above
 *     DELTAY         rate constraint |y(k) - y(k-1)| <= DELTAY. A system
 *                    without a rate limit sets the whole output range
 *                    YMAXPHYS - YMINPHYS: a larger step would leave
 *                    [YMINPHYS, YMAXPHYS] anyway.
 *     SIGMA_UNNORM   bound of the output noise (strip half-width in PL)
 *     Ts             sampling time [s]: the one the ARX model was
 *                    discretized at; logged in the output files
 *
 *   Initial zonotope and initial conditions (constexpr double), physical units
 *     thetaNominalInit  initial zonotope centre c0           (nTheta)
 *     generatorsInit    initial zonotope generators G        (nTheta x nGens)
 *                         A parameter known EXACTLY gets an all-zero row:
 *                         it is then left out of the zonotope, which only
 *                         spans the nUnc uncertain parameters (setup.h,
 *                         UNC_MAP), and enters predictions as its
 *                         thetaNominalInit value.
 *     thetaTrueInit     true plant parameters (test bench only)
 *     yHistInit         initial output samples y(-1), ..., y(-na)
 *     uHistInit         initial input samples, nb+nd-1 values
 *   With NRMLZ, controller.cpp's initControllerState() derives the
 *   normalized zonotope and samples from these at static-initialization
 *   time; without NRMLZ it copies them unchanged.
 *
 *   Fixed-point types (#ifdef FIXED): the ones whose range depends on the
 *     system (see types.h). A system without them is floating point only,
 *     and its file stops a FIXED build with #error.
 *
 * A system file includes nothing: it only uses ACTIVE_SYSTEM, the mode
 * flags (FIXED, NRMLZ, CONVERSIONS_MODE: setup.h), <ap_fixed.h> and
 * WORD_LENGTH (below).
 */

#pragma once

/* ======================================================================
   System identifiers - plain integers so they work in #if expressions.
   ======================================================================
   In an #if, an identifier that is not (yet) a macro evaluates to 0, so
   if these ids were defined after a file that tests
   "#if ACTIVE_SYSTEM == SYSTEM_...", every such comparison would read
   0 == 0 and be true for EVERY system - which is how SYSTEM_BUCK_ALBERTO
   once got SYSTEM_BUCK_LOSS's output_type, whose +-16 range saturates its
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

#ifdef FIXED
#include <ap_fixed.h>
/** Word length of most fixed-point types (system files and types.h). */
constexpr int WORD_LENGTH = 18;
#endif

/* ======================================================================
   The active system. Its file comes BEFORE types.h: types.h's aliases
   (norm_output_type without NRMLZ, ...) refer to the system's types.
   ====================================================================== */
#if ACTIVE_SYSTEM == SYSTEM_SIMPLE
#define SYSTEM_FILE "systems/system_simple.h"
#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
#define SYSTEM_FILE "systems/system_benchmark.h"
#elif ACTIVE_SYSTEM == SYSTEM_MILANO
#define SYSTEM_FILE "systems/system_milano.h"
#elif ACTIVE_SYSTEM == SYSTEM_BUCK || ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
#define SYSTEM_FILE "systems/system_buck.h"
#elif ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
#define SYSTEM_FILE "systems/system_alberto.h"
#elif ACTIVE_SYSTEM == SYSTEM_GAIN_DEMO
#define SYSTEM_FILE "systems/system_gain_demo.h"
#elif ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
#define SYSTEM_FILE "systems/system_pendulum.h"
#else
#error "Unrecognized ACTIVE_SYSTEM."
#endif

#include SYSTEM_FILE
#include "types.h"

/* ======================================================================
   Consistency of the system's constants (all constexpr double)
   ====================================================================== */
static_assert(nTheta == na + nb, "nTheta must be na + nb.");
static_assert(UMINPHYS < UMAXPHYS && YMINPHYS < YMAXPHYS,
    "Empty physical range: need UMINPHYS < UMAXPHYS and YMINPHYS < YMAXPHYS.");
static_assert(UMINPHYS <= UMIN && UMIN < UMAX && UMAX <= UMAXPHYS,
    "Input constraints must lie inside the physical range: UMINPHYS <= UMIN < UMAX <= UMAXPHYS.");
static_assert(YMINPHYS <= YMIN && YMIN < YMAX && YMAX <= YMAXPHYS,
    "Output constraints must lie inside the physical range: YMINPHYS <= YMIN < YMAX <= YMAXPHYS.");
static_assert(DELTAY > 0.0, "The rate constraint DELTAY must be positive.");
static_assert(SIGMA_UNNORM >= 0.0, "The noise bound SIGMA_UNNORM must be non-negative.");
static_assert(Ts > 0.0, "The sampling time Ts must be positive.");
