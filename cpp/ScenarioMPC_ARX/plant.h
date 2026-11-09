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
 * INTERFACE - one ArxPlant object per simulation run:
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
 * version of the functions below, selected by SYSTEM_FILE and PLANT_MODEL (see the
 * inverted pendulum).
 */
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

/* ----------------------------------------------------------------------
   Default: ARX plant with the controller's structure.
   ----------------------------------------------------------------------
   A random true plant draws theta = c0 + G*xi, xi uniform in [-1, 1]^nGens
   (c0 = thetaNominalInit, G = generatorsInit): a point of the
   controller's initial uncertainty zonotope, which is exactly the set the
   robust controller is designed to handle. Certain parameters (all-zero
   generator rows) automatically keep their nominal value.
   ---------------------------------------------------------------------- */
struct ArxPlant
{
    double theta[nTheta];
    double yMeasHist[na];         /* y(k-1), ..., y(k-na)   (measured)          */
    double uHist[nb + nd - 1];    /* u(k-1), ..., u(k-nb-nd+1)                  */
    double uEarly, uLate, fEarly; /* the last interval exactly: uEarly for fEarly*Ts, then uLate */
};

inline void plantSampleTrue(ArxPlant &p, TbRng &rng, bool random)
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

inline void plantReset(ArxPlant &p)
{
    for (int i = 0; i < na; i++)          p.yMeasHist[i] = yHistInit[i];
    for (int i = 0; i < nb + nd - 1; i++) p.uHist[i] = uHistInit[i];
    p.fEarly = 0.0;
    p.uEarly = uHistInit[0];
    p.uLate = uHistInit[0];
}

inline double plantNextOutput(ArxPlant &p)
{
    double y = 0.0;
    for (int i = 0; i < nTheta; i++)
        y += ((i < na) ? p.yMeasHist[i] : p.uHist[i - na + nd - 1]) * p.theta[i];
    return y;
}

inline void plantPrintParams(const ArxPlant &p, FILE *f, bool header)
{
    for (int i = 0; i < nTheta; i++)
        header ? fprintf(f, " theta%d", i + 1) : fprintf(f, " %.12g", p.theta[i]);
}


/** Common to every plant: shift in the measured output y(k) and the input
 *  of the interval [k*Ts, (k+1)*Ts) once the controller has run: uEarly for
 *  its first fraction fEarly, then u (fEarly = 0: u for the whole interval,
 *  the original test bench). The ARX regression uses the MEASURED (noisy)
 *  past outputs - an equation-error model, as before. */
inline void plantPush(ArxPlant &p, double yMeas, double uEarly, double u, double fEarly)
{
    for (int i = na - 1; i > 0; i--)          p.yMeasHist[i] = p.yMeasHist[i - 1];
    p.yMeasHist[0] = yMeas;
    for (int i = nb + nd - 2; i > 0; i--)     p.uHist[i] = p.uHist[i - 1];
    /* one input per interval: its average */
    p.uHist[0] = (fEarly == 0.0) ? u : fEarly * uEarly + (1.0 - fEarly) * u;
    p.uEarly = uEarly; p.uLate = u; p.fEarly = fEarly;
}

#define SCMPC_TESTBENCH
#include SYSTEM_FILE      /* second pass: the system's test-bench section */
#ifndef PLANT_TAG         /* no model of its own for this PLANT_MODEL: the ARX plant */
  using Plant = ArxPlant;
  #define PLANT_TAG "arx"
#endif