/* Header file for SYSTEM_SIMPLE */
#pragma once
#include "../setup.h"
#if ACTIVE_SYSTEM == SYSTEM_SIMPLE
constexpr int na = 2;
constexpr int nb = 1;
constexpr int nd = 2;
constexpr int nGens = 3;
constexpr int   nTheta = na + nb;

constexpr double UMINPHYS = -0.3;
constexpr double UMAXPHYS =  0.3;
constexpr double YMINPHYS =  0.0;
constexpr double YMAXPHYS =  8.0;
constexpr double UMIN = UMINPHYS;
constexpr double UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = YMAXPHYS;
constexpr double SIGMA_UNNORM = 0.0;
/* DELTAY, SIGMA_UNNORM - to be defined */

/* Sampling time, logged in output files for printing time in seconds.
    This is just a conventional value, no sampling time was specified. */
constexpr double Ts = 10e-3;

constexpr double thetaNominalInit[nTheta] = {2.0, -1.0, 1.0};
constexpr double generatorsInit[nTheta][nGens] = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.5}};
constexpr double thetaTrueInit[nTheta] = {2.0, -1.0, 1.0}; // == thetaNominalInit
/* cold start at physical 0, as for the other systems */
constexpr double yHistInit[na] = {0, 0};
constexpr double uHistInit[nb + nd - 1] = {0, 0};
#endif