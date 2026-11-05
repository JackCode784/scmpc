/**
 * @file  plant.h
 * @brief TEST-BENCH ONLY: the true system ("plant") the controller is tested
 *        against, and the settings of multi-run experiments.
 *
 * Included only by testMain.cpp, never by a file Vitis synthesizes, so it
 * can freely use double, <cmath> and <random>. Header-only on purpose: the
 * Vitis project needs no additional source file.
 *
 * WHY THIS IS NOT IN system_configs.h
 *   system_configs.h describes the controller's MODEL of the system
 *   (nominal parameters, uncertainty zonotope, bounds) and is compiled into
 *   the hardware. The plant is the REAL system, which the controller does
 *   not know exactly: the same ARX structure with other parameters, or a
 *   different model altogether (the pendulum's nonlinear ODE). Keeping it
 *   here keeps test-bench code out of synthesized headers.
 *
 * INTERFACE - one Plant object per simulation run:
 *   plantSampleTrue(p, rng, random)  this run's true parameters
 *   plantReset(p)                    physical initial conditions (*Init)
 *   plantNextOutput(p)               y(k), noise-free, physical units
 *   plantPush(p, yMeas, u)           record measured y(k) and applied u(k)
 *   plantPrintParams(p, f, header)   this run's parameters, for the summary
 *
 * The plant keeps its OWN history of physical outputs and inputs. (The old
 * test bench read the controller's yHistCtrl/uHistCtrl instead, which ties the true
 * system to controller internals: with NRMLZ those are normalized, with
 * CONVERSIONS_MODE they carry ADC/DAC quantization, and the pendulum's
 * nonlinear model used to be silently skipped whenever NRMLZ was on.)
 *
 * Default for every system: an ARX plant with the controller's structure
 * (na, nb, nd). A system with a different true model provides its own
 * version of the functions below, selected by #if ACTIVE_SYSTEM (see the
 * inverted pendulum).
 */
#pragma once

#include "setup.h"
#include <cmath>
#include <cstdio>
#include <random>

/* ======================================================================
   Test-bench random generator
   ======================================================================
   std::mt19937's raw output is fixed by the C++ standard, but
   std::uniform_real_distribution's is not (libstdc++, MSVC, ... may
   differ), so uniform() is computed by hand from the raw output: the same
   seed then gives the same numbers on every compiler and OS. */
struct TbRng
{
    std::mt19937 gen;
    explicit TbRng(unsigned int seed) : gen(seed) {}
    /** Uniform in [-1, 1). */
    double uniform() { return 2.0 * (double(gen()) / 4294967296.0) - 1.0; }
};

/* ======================================================================
   PLANT
   ====================================================================== */
#if ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
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
   with PENDULUM_NONLINEAR 0 its output comes from the ARX model above
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
#define PENDULUM_NONLINEAR  1

constexpr double PEND_G     = 9.81;   /* [m/s^2]                               */
constexpr double PEND_REL_UNC = 0.2;  /* l, m drawn in nominal*(1 +- 0.2)      */
constexpr int    PENDULUM_EULER_SUBSTEPS = 100;

static_assert(na == 2 && nb == 1 && nd == 2,
    "plant.h's pendulum map assumes the ARX structure na=2, nb=1, nd=2.");

#if PENDULUM_NONLINEAR
#define PLANT_TAG "nl"
#else
#define PLANT_TAG "arx"
#endif

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
    #if PENDULUM_NONLINEAR
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

#else
/* ----------------------------------------------------------------------
   Default: ARX plant with the controller's structure.
   ----------------------------------------------------------------------
   A random true plant draws theta = c0 + G*xi, xi uniform in [-1, 1]^nGens
   (c0 = thetaNominalInit, G = generatorsInit): a point of the
   controller's initial uncertainty zonotope, which is exactly the set the
   robust controller is designed to handle. Certain parameters (all-zero
   generator rows) automatically keep their nominal value.
   ---------------------------------------------------------------------- */
#define PLANT_TAG "arx"

struct Plant
{
    double theta[nTheta];
    double yMeasHist[na];         /* y(k-1), ..., y(k-na)   (measured)          */
    double uHist[nb + nd - 1];    /* u(k-1), ..., u(k-nb-nd+1)                  */
};

inline void plantSampleTrue(Plant &p, TbRng &rng, bool random)
{
    if (random)
    {
        double xi[nGens];
        for (int j = 0; j < nGens; j++) xi[j] = rng.uniform();
        for (int i = 0; i < nTheta; i++)
        {
            p.theta[i] = thetaNominalInit[i];
            for (int j = 0; j < nGens; j++)
                p.theta[i] += generatorsInit[i][j] * xi[j];
        }
    }
    else
    {
        for (int i = 0; i < nTheta; i++) p.theta[i] = thetaTrueInit[i];
    }
}

inline void plantReset(Plant &p)
{
    for (int i = 0; i < na; i++)          p.yMeasHist[i] = yHistInit[i];
    for (int i = 0; i < nb + nd - 1; i++) p.uHist[i] = uHistInit[i];
}

inline double plantNextOutput(Plant &p)
{
    double y = 0.0;
    for (int i = 0; i < nTheta; i++)
        y += ((i < na) ? p.yMeasHist[i] : p.uHist[i - na + nd - 1]) * p.theta[i];
    return y;
}

inline void plantPrintParams(const Plant &p, FILE *f, bool header)
{
    for (int i = 0; i < nTheta; i++)
        header ? fprintf(f, " theta%d", i + 1) : fprintf(f, " %.12g", p.theta[i]);
}
#endif

/** Common to every plant: shift in the measured output y(k) and the input
 *  of the interval [k*Ts, (k+1)*Ts) once the controller has run: uEarly for
 *  its first fraction fEarly, then u (fEarly = 0: u for the whole interval,
 *  the original test bench). The ARX regression uses the MEASURED (noisy)
 *  past outputs - an equation-error model, as before. */
inline void plantPush(Plant &p, double yMeas, double uEarly, double u, double fEarly)
{
    for (int i = na - 1; i > 0; i--)          p.yMeasHist[i] = p.yMeasHist[i - 1];
    p.yMeasHist[0] = yMeas;
    for (int i = nb + nd - 2; i > 0; i--)     p.uHist[i] = p.uHist[i - 1];
    #if ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM && PENDULUM_NONLINEAR
    p.uHist[0] = u;               /* the Euler sub-steps apply the switch */
    p.uEarly = uEarly;
    p.fEarly = fEarly;
    #else
    /* one input per interval: its average */
    p.uHist[0] = (fEarly == 0.0) ? u : fEarly * uEarly + (1.0 - fEarly) * u;
    #endif
}
