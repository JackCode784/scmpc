/**
 * @file  system_configs.h
 * @brief Compile-time system configurations for SCMPC on ARX systems.
 *
 * System registry
 * ---------------
 *  SYSTEM_SIMPLE     (id 0)  na=2, nb=1, nd=2 - original non-CMPLSYS ARX
 *  SYSTEM_BENCHMARK  (id 1)  na=2, nb=2, nd=1 - benchmark ARX (selectSys.m)
 *  SYSTEM_MILANO     (id 2)  na=3, nb=3, nd=1 - original CMPLSYS (BESS)
 *  SYSTEM_BUCK       (id 3)  na=2, nb=1, nd=2 - buck power converter
 *  SYSTEM_BUCK_LOSS  (id 4)  na=2, nb=1, nd=2 - buck power converter with loss resistance
 *  SYSTEM_GAIN_DEMO  (id 5)  na=1, nb=1, nd=2 - minimal demo: is scenario MPC worth it?
 *  others... (see below).
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
/**
 * Each system is defined by an header file with the same name.
 * Each file contains system dimensions (na, nb, nd, nGens, nTheta = na + nb),
 * input/output constraints (physical ones, *Phys, and desired ones without 
 * any suffix) and initial conditions for input/output samples and uncertainty
 * zonotope 
 */

/* ======================================================================
  INPUT / OUTPUT HARD CONSTRAINTS
  ======================================================================
  These are the box constraints used by the progressive barrier in MADSARX.
  They must be consistent with the ones for the ACTIVE_SYSTEM.
  They are repeated as separate constants because ap_fixed<> prevents
  deriving them from the config struct at compile time via constexpr.
  ====================================================================== */

 /* ======================================================================
   INITIAL ZONOTOPE AND INITIAL CONDITIONS - physical (unnormalized) units
   ======================================================================
   Per system, only physical values are given here:
     thetaNominalInit  initial zonotope centre c0        (nTheta)
     generatorsInit     initial zonotope generators G     (nTheta x nGens)
                           A parameter known EXACTLY gets an all-zero row:
                           it is then left out of the zonotope, which only
                           spans the nUnc uncertain parameters (setup.h,
                           UNC_MAP), and enters predictions as its
                           thetaNominalInit value.
     thetaTrueInit       true plant parameters (testMain.cpp only)
     yHistInit         initial output samples y(-1), ..., y(-na)
     uHistInit         initial input samples, nb+nd-1 values
   With NRMLZ, controller.cpp's initControllerState() derives the
   normalized zonotope and samples from these at static-initialization
   time (see there); without NRMLZ it copies them unchanged.
   ====================================================================== */
#if   ACTIVE_SYSTEM == SYSTEM_SIMPLE
  #include "systems/system_simple.h"
#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
  #include "systems/system_benchmark.h"
  #elif ACTIVE_SYSTEM == SYSTEM_MILANO
  #include "systems/system_milano.h"
  #elif ACTIVE_SYSTEM == SYSTEM_BUCK
  #include "systems/system_buck.h"
  #elif ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
  #include "systems/system_buck.h"
  #elif ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
  #include "systems/system_alberto.h"
  #elif ACTIVE_SYSTEM == SYSTEM_GAIN_DEMO
  #include "systems/system_gain_demo.h"
  #elif ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
  #include "systems/system_pendulum.h"
  #endif
