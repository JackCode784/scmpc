/* Header file for SYSTEM_BENCHMARK */
#pragma once
#if ACTIVE_SYSTEM == SYSTEM_BENCHMARK
constexpr int na      = 2;
constexpr int nb      = 2;
constexpr int nd      = 1;
constexpr int nGens   = 6;
constexpr int nTheta = na + nb;

constexpr double UMINPHYS = -1.9;
constexpr double UMAXPHYS =  1.9;
constexpr double YMINPHYS = -10.0;
constexpr double YMAXPHYS =  8.0;
constexpr double UMIN = UMINPHYS;
constexpr double UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = YMAXPHYS;
constexpr double SIGMA_UNNORM = 0.20;
/* DELTAY missing */

/* Sampling time, logged in output files for printing time in seconds.
    This is just a conventional value, no sampling time was specified. */
constexpr double Ts = 10e-3;

constexpr double thetaNominalInit[nTheta] = { 1.50,  -0.70,   1.00,   0.50};
constexpr double generatorsInit[nTheta][nGens] = {
        {  0.080,  0.020,  0.005,  0.000,  0.010,  0.000 }, 
        { -0.010,  0.060,  0.000,  0.008,  0.000,  0.005 }, 
        {  0.000,  0.010,  0.070, -0.015,  0.000,  0.010 }, 
        {  0.005,  0.000,  0.012,  0.055, -0.010,  0.000 }};
constexpr double thetaTrueInit[nTheta] = {1.5540,  -0.7433,   1.0570,   0.4761};
constexpr double yHistInit[na] = {0, 0};
constexpr double uHistInit[nb+nd-1] = {0, 0};
#endif