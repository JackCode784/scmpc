/* Header file for pendulum system */
#pragma once
#include "../setup.h"
#include "../plant.h"
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
constexpr double thetaTrueInit[nTheta] = {2.000000000000000, -0.998821001683502, 0.000751434382693};
//2.0000000000,   -0.998637500000000,   0.001205632716049
//2.0, -0.999091666666667, 0.0003572245084590763
//2.000000000000000,   -0.998637500000000,   0.001205632716049
constexpr double yHistInit[na] = {-0.78, -0.78};
constexpr double uHistInit[nb+nd-1] = {0, 0};

#if USE_ARX == 0
/* ----------------------------------------------------------------------
   Inverted pendulum: thetaddot = g/l*sin(theta) + u/(m*l^2), y = theta.
   ----------------------------------------------------------------------
   The controller's ARX model is the forward-Euler discretization, with
   step Ts, of the LINEARIZED equation (sin(theta) ~ theta):
     y(k) = 2*y(k-1) + (-1 + Ts^2*g/l)*y(k-2) + Ts^2/(m*l^2)*u(k-2)
   i.e. theta = [2, -1 + Ts^2*g/l, Ts^2/(m*l^2)] (na=2, nb=1, nd=2). The
   nominal l, m are recovered from thetaNominalInit through this map
   (l ~ 0.83, m ~ 0.19 for the current values in system_configs.h), so there is a
   single source of truth.

   A random true plant draws l and m uniformly in nominal*(1 +- REL_UNC);
   with USE_ARX 0 its output comes from the ARX model above
   with those l, m, with 1 from the nonlinear equation - the same physical
   population either way, so the two are directly comparable.

   The nonlinear equation is integrated with PENDULUM_EULER_SUBSTEPS
   forward-Euler sub-steps per sample, input held constant over the
   sample (zero-order hold). Besides sin(theta), this differs from the ARX
   model in a second, smaller way: fine integration lets u(k-1) already
   affect y(k), which the one-step Euler model (nd = 2) cannot represent -
   a genuine model mismatch the robust controller has to absorb.
   ---------------------------------------------------------------------- */
/** 1: true output from the nonlinear equation; 0: from the ARX model. */
constexpr double PEND_G     = 9.81;   /* [m/s^2]                               */
constexpr double PEND_REL_UNC = 0.2;  /* l, m drawn in nominal*(1 +- 0.2)      */
constexpr int    PENDULUM_EULER_SUBSTEPS = 100;

static_assert(na == 2 && nb == 1 && nd == 2,
    "plant.h's pendulum map assumes the ARX structure na=2, nb=1, nd=2.");

struct Plant
{
    double l, m;              /* physical parameters                       */
    double theta[nTheta];     /* equivalent ARX parameters                 */
    double yMeasHist[na];             /* y(k-1), ..., y(k-na)   (measured)          */
    double uHist[nb + nd - 1];    /* u(k-1), ..., u(k-nb-nd+1)                  */
    double x[2];              /* nonlinear state: angle, angular velocity   */
    double uEarly;            /* input held during the first fEarly of the  */
    double fEarly;            /* last interval, before u[0] took effect     */
};

inline void pendulumThetaFromPhys(double l, double m, double theta[nTheta])
{
    theta[0] = 2.0;
    theta[1] = -1.0 + Ts * Ts * PEND_G / l;
    theta[2] = Ts * Ts / (m * l * l);
}

inline void pendulumPhysFromTheta(const double theta[nTheta], double &l, double &m)
{
    l = Ts * Ts * PEND_G / (theta[1] + 1.0);
    m = Ts * Ts / (theta[2] * l * l);
}

inline void plantSampleTrue(Plant &p, TbRng &rng, bool random)
{
    if (random)
    {
        double lNom, mNom;
        pendulumPhysFromTheta(thetaNominalInit, lNom, mNom);
        p.l = lNom * (1.0 + PEND_REL_UNC * rng.uniform());
        p.m = mNom * (1.0 + PEND_REL_UNC * rng.uniform());
        pendulumThetaFromPhys(p.l, p.m, p.theta);
    }
    else
    {
        /* thetaTrueInit exactly for the ARX model; l, m derived from it
         * for the nonlinear one */
        for (int i = 0; i < nTheta; i++) p.theta[i] = thetaTrueInit[i];
        pendulumPhysFromTheta(thetaTrueInit, p.l, p.m);
    }
}

inline void plantReset(Plant &p)
{
    for (int i = 0; i < na; i++)          p.yMeasHist[i] = yHistInit[i];
    for (int i = 0; i < nb + nd - 1; i++) p.uHist[i] = uHistInit[i];
    /* State consistent with the output history: angle y(-1), velocity
     * from the last two samples. */
    p.x[0] = yHistInit[0];
    p.x[1] = (yHistInit[0] - yHistInit[1]) / Ts;
    p.uEarly = uHistInit[0];
    p.fEarly = 0.0;
}

inline double plantNextOutput(Plant &p)
{
    #if USE_ARX
    const double h = Ts / PENDULUM_EULER_SUBSTEPS;
    /* Zero-order hold of u(k-1) = u[0], after uEarly for the first fEarly
     * of the interval (controller latency, testbench_settings::computeDelayFraction) */
    const int nEarly = int(std::lround(p.fEarly * PENDULUM_EULER_SUBSTEPS));
    for (int s = 0; s < PENDULUM_EULER_SUBSTEPS; s++)
    {
        const double uHold = (s < nEarly) ? p.uEarly : p.uHist[0];
        const double angle = p.x[0], vel = p.x[1];
        p.x[0] = angle + h * vel;
        p.x[1] = vel + h * (PEND_G / p.l * std::sin(angle) + uHold / (p.m * p.l * p.l));
    }
    return p.x[0];
    #else
    double y = 0.0;
    for (int i = 0; i < nTheta; i++)
        y += ((i < na) ? p.yMeasHist[i] : p.uHist[i - na + nd - 1]) * p.theta[i];
    return y;
    #endif
}

inline void plantPrintParams(const Plant &p, FILE *f, bool header)
{
    if (header) { for (int i = 0; i < nTheta; i++) fprintf(f, " theta%d", i + 1); fprintf(f, " l m"); }
    else        { for (int i = 0; i < nTheta; i++) fprintf(f, " %.12g", p.theta[i]); fprintf(f, " %.6g %.6g", p.l, p.m); }
}

inline void plantPush(Plant &p, double yMeas, double uEarly, double u, double fEarly)
{
    for (int i = na - 1; i > 0; i--)          p.yMeasHist[i] = p.yMeasHist[i - 1];
    p.yMeasHist[0] = yMeas;
    for (int i = nb + nd - 2; i > 0; i--)     p.uHist[i] = p.uHist[i - 1];
    #if USE_ARX == 0
    p.uHist[0] = u;               /* the Euler sub-steps apply the switch */
    p.uEarly = uEarly;
    p.fEarly = fEarly;
    #else
    /* one input per interval: its average */
    p.uHist[0] = (fEarly == 0.0) ? u : fEarly * uEarly + (1.0 - fEarly) * u;
    #endif
}

#endif

#endif