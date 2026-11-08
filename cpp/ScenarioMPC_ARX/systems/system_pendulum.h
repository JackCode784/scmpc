/* Header file for pendulum system */
#pragma once
#include "../setup.h"
#if ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
constexpr int   na    = 2;
constexpr int   nb    = 1;
constexpr int   nd    = 2;
constexpr int   nGens = 3;
constexpr int   nTheta = na + nb;

/** None of these quantities are in the controller's 
 * datapath, so they can all be double - these are used 
 * to compute constants in setup.h */
constexpr double UMINPHYS = -3;
constexpr double UMAXPHYS = 3;
constexpr double YMINPHYS = -0.9;
constexpr double YMAXPHYS = 0.9;
constexpr double UMIN = UMINPHYS;
constexpr double UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = YMAXPHYS;
constexpr double DELTAY = 10e-3;
constexpr double SIGMA_UNNORM = 0.00;

/* The plant derives l, m from Ts, changing it silently changes
    since they are derived from theta params and Ts (l = Ts^2 * g / (theta[1] + 1)) */
constexpr double Ts = 10e-3;    // Sampling time, logged in output files for printing time in seconds

constexpr double thetaNominalInit[nTheta] = {2.000000000000000, -0.998821001683502, 0.000751434382693};
constexpr double generatorsInit[nTheta][nGens] = {
    {-0.000001, 0, 0},
    {0, 1e-3 * -0.281338543972914, 1e-4 * -0.961862828825779},
    {0, 1e-3 * -0.386439223972734, 1e-4 * 0.700262993444492}};
static constexpr const double (&thetaTrueInit)[nTheta] = thetaNominalInit;
//2.0000000000,   -0.998637500000000,   0.001205632716049
//2.0, -0.999091666666667, 0.0003572245084590763
//2.000000000000000,   -0.998637500000000,   0.001205632716049
constexpr double yHistInit[na] = {-0.78, -0.78};
constexpr double uHistInit[nb+nd-1] = {0, 0};
#endif