/* Header file for SYSTEM_BUCK and SYSTEM_BUCK_LOSS */
#pragma once
#include "../setup.h"
#if ACTIVE_SYSTEM == SYSTEM_BUCK || ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
constexpr int na      = 2;
constexpr int nb      = 1;
constexpr int nd      = 2;
constexpr int nGens   = 3;
constexpr int nTheta = na + nb;

constexpr double UMINPHYS = 0;
constexpr double UMAXPHYS = 1;
constexpr double YMINPHYS = 0;
constexpr double YMAXPHYS = 10;
constexpr double UMIN = UMINPHYS;
constexpr double UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = YMAXPHYS;
constexpr double DELTAY = 0.1;
constexpr double SIGMA_UNNORM = 0.2; // for buck_loss is 0.02
constexpr double Ts = 1e-5;    // Sampling time, logged in output files for printing time in seconds

#if ACTIVE_SYSTEM == SYSTEM_BUCK
constexpr double thetaNominalInit[nTheta] = { 1.8889, -0.9333, 0.4444};
constexpr double generatorsInit[nTheta][nGens] = {
        { 0.0625,        0,         0},
        {      0,   0.0606,         0},
        {      0,        0,    0.2500}};
constexpr double thetaTrueInit[nTheta] = {1.8612, -0.9276, 0.6732};
constexpr double yHistInit[na] = {0, 0};
constexpr double uHistInit[nb+nd-1] = {0, 0};
#endif

#if ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
constexpr double thetaNominalInit[nTheta] = {1.817972144631566,  -0.871463786797535,   0.457543557236585};
constexpr double generatorsInit[nTheta][nGens] = {
        {-0.093616869443703,   0.124811247863353,   0.003362744341625},
        {0.057591158572667,  -0.117838849718767,   0.003770097120654}, 
        {0.271071396611763,   0.068140402887871,   0.000360367556716}};
constexpr double thetaTrueInit[nTheta] = {1.857831352244614, -0.933661885378246, 0.683201544134083};
constexpr double yHistInit[na] = {0, 0};
constexpr double uHistInit[nb+nd-1] = {0, 0};
#endif

#endif