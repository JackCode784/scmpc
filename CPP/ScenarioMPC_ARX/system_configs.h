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
#include "types.h"

/* ======================================================================
   System identifiers - plain integers so they work in #if expressions.
   ====================================================================== */
#define SYSTEM_SIMPLE     0
#define SYSTEM_BENCHMARK  1
#define SYSTEM_MILANO     2
#define SYSTEM_BUCK       3
#define SYSTEM_BUCK_LOSS  4

#if ACTIVE_SYSTEM == SYSTEM_SIMPLE
    #define THETA_NOMINAL_INIT   2.0, -1.0, 1.0
    #define GENERATORS_INIT \
            { 0.0,  0.0,  0.0}, \
            { 0.0,  0.0,  0.0}, \
            { 0.0,  0.0,  0.5}
    #define THETA_TRUE_INIT     2.0,  -1.0,   1.0 // == THETA_NOMINAL_INIT
#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
    #define THETA_NOMINAL_INIT  1.50,  -0.70,   1.00,   0.50
    #define GENERATORS_INIT      \
            {  0.080,  0.020,  0.005,  0.000,  0.010,  0.000 }, \
            { -0.010,  0.060,  0.000,  0.008,  0.000,  0.005 }, \
            {  0.000,  0.010,  0.070, -0.015,  0.000,  0.010 }, \
            {  0.005,  0.000,  0.012,  0.055, -0.010,  0.000 }
    #define THETA_TRUE_INIT     1.5540,  -0.7433,   1.0570,   0.4761
#elif ACTIVE_SYSTEM == SYSTEM_MILANO
    #define THETA_NOMINAL_INIT     0.7921,  0.1524, -0.1668,  0.0842,  0.0442,  0.0860 
    #define GENERATORS_INIT  \
            { -0.8959, -0.4594, -0.0026, -0.0027, -0.0187,  0.0080 },   \
            {  1.3452, -0.1401,  0.0030,  0.0111, -0.0283,  0.0079 },   \
            { -0.5326,  0.4275, -0.0051,  0.0289, -0.0362,  0.0077 },   \
            { -0.0082,  0.0028,  0.0524,  0.0788,  0.0367,  0.0085 },   \
            {  0.0859,  0.0502, -0.1015, -0.0126,  0.0271,  0.0086 },   \
            {  0.0021,  0.1180,  0.0538, -0.0985,  0.0126,  0.0086 }    
    #define THETA_TRUE_INIT     0.7921,  0.1524, -0.1668,  0.0842,  0.0442,  0.0860
#elif ACTIVE_SYSTEM == SYSTEM_BUCK
    #define THETA_NOMINAL_INIT  1.8889, -0.9333, 0.4444
    #define GENERATORS_INIT \
            { 0.0625,        0,         0}, \
            {      0,   0.0606,         0}, \
            {      0,        0,    0.2500}  
    #define THETA_TRUE_INIT        1.8612,  -0.9276,    0.6732
#elif ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
    #define THETA_NOMINAL_INIT      1.817972144631566,  -0.871463786797535,   0.457543557236585
    #define GENERATORS_INIT \
          {-0.093616869443703,   0.124811247863353,   0.003362744341625}, \
            {0.057591158572667,  -0.117838849718767,   0.003770097120654}, \
            {0.271071396611763,   0.068140402887871,   0.000360367556716}
    #define THETA_TRUE_INIT     1.857831352244614,  -0.933661885378246,   0.683201544134083
#endif
