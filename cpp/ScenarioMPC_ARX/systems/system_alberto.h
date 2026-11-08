/* Header file for SYSTEM_BUCK_ALBERTO */
#pragma once
#include "../setup.h"
#if ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO

constexpr int na      = 2;
constexpr int nb      = 1;
constexpr int nd      = 2;
constexpr int nGens   = 3;
constexpr int nTheta = na + nb;

constexpr double  UMINPHYS = 0;
constexpr double  UMAXPHYS = 1;
constexpr double YMINPHYS = 0;
constexpr double YMAXPHYS = 100;
constexpr double  UMIN = UMINPHYS;
constexpr double  UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = 85;
constexpr double DELTAY = 5;
constexpr double SIGMA_UNNORM = 0.2;
constexpr double Ts = 10e-3;    // Sampling time, logged in output files for printing time in seconds. Conventional value


constexpr double thetaNominalInit[nTheta] = {1.733749265658302,  -0.881912845894497,  14.298236163643255};
constexpr double generatorsInit[nTheta][nGens] = {
    {-0.068444143534821,   0.137725841350229,   0.004638817609457},
    {-0.008278303729254,  -0.132496642357720,   0.004822306118286},
    {7.403892173947610,   0.001125040669095,   0.000048274664814}};
constexpr double thetaTrueInit[nTheta] = {1.753571428571429,  -0.894444444444444,  13.888888888888886};
constexpr double yHistInit[na] ={ 0, 0};
constexpr double uHistInit[nb+nd-1] ={ 0, 0};
#endif