/* Header file for SYSTEM_MILANO */
#pragma once
#if ACTIVE_SYSTEM == SYSTEM_MILANO
constexpr int na      = 3;
constexpr int nb      = 3;
constexpr int nd      = 1;
constexpr int nGens   = 6;
constexpr int   nTheta = na + nb;

constexpr double UMINPHYS = -200.0;
constexpr double UMAXPHYS =  200.0;
constexpr double YMINPHYS = -110.0;
constexpr double YMAXPHYS =  110.0;
constexpr double UMIN = UMINPHYS;
constexpr double UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = YMAXPHYS;
constexpr double SIGMA_UNNORM = 4.4655;
/* No rate limit (this system predates the rate constraint): the whole
 * output range, which no step inside [YMINPHYS, YMAXPHYS] can exceed. */
constexpr double DELTAY = YMAXPHYS - YMINPHYS;

/* Sampling time, logged in output files for printing time in seconds.
    This is just a conventional value, no sampling time was specified. */
constexpr double Ts = 10e-3;

constexpr double thetaNominalInit[nTheta] = {    0.7921,  0.1524, -0.1668,  0.0842,  0.0442,  0.0860 };
constexpr double generatorsInit[nTheta][nGens] =  {
        { -0.8959, -0.4594, -0.0026, -0.0027, -0.0187,  0.0080 },
        {  1.3452, -0.1401,  0.0030,  0.0111, -0.0283,  0.0079 },
        { -0.5326,  0.4275, -0.0051,  0.0289, -0.0362,  0.0077 },
        { -0.0082,  0.0028,  0.0524,  0.0788,  0.0367,  0.0085 },
        {  0.0859,  0.0502, -0.1015, -0.0126,  0.0271,  0.0086 },
        {  0.0021,  0.1180,  0.0538, -0.0985,  0.0126,  0.0086 }};
constexpr double thetaTrueInit[nTheta] = {0.7921, 0.1524, -0.1668, 0.0842, 0.0442, 0.0860};
constexpr double yHistInit[na] = {0, 0, 0};
constexpr double uHistInit[nb+nd-1] = {0, 0, 0};

#ifdef FIXED
#error "SYSTEM_MILANO has no fixed-point types (system_configs.h): floating point only."
#endif

#endif
