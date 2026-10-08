/**
 * @file  testMain.cpp
 * @brief PC-side closed-loop simulation for SCMPC on an ARX system.
 *
 * This file is a SOFTWARE-ONLY simulation harness.  It is never synthesised
 * by Vitis HLS; it exists to verify controller behaviour before flashing
 * hardware. It serves as the testbench for Vitis HLS.
 *
 * SIMULATION STRUCTURE
 * ---------------------
 * N_RUNS independent runs (plant.h). Each run:
 *  - draws its true plant (plantSampleTrue) and measurement noise from the
 *    test-bench generator, and resets the plant, the controller state and
 *    the controller's random generator - as if the hardware had just been
 *    reset, so every run is independent of the previous ones;
 *  - at each time step k:
 *     1. asks the plant for y(k) (plantNextOutput) and adds noise;
 *     2. converts y(k) and yref to digital samples (CONVERSIONS_MODE);
 *     3. calls controller(), which returns the optimal digital input;
 *     4. converts it back to physical units, applies it to the plant
 *        (plantPush) and logs everything.
 * The test bench only lets things "play": what the true system is, and
 * how its output is generated, lives in plant.h.
 *
 * OUTPUT FILES
 * ------------
 * N_RUNS == 1: "output.txt", one row per step (format unchanged; the
 *              columns depend on the compilation mode) - plotOutputCpp.m.
 * N_RUNS  > 1: "runs_<ctrl>_<plant>_<run>.txt", same format, one per run,
 *              plus "runs_<ctrl>_<plant>_summary.txt", one row per run with
 *              the true plant parameters and constraint-violation metrics -
 *              plotMultipleOutputsCpp.m. <ctrl> is scen / scencost /
 *              scenconstr / noscen (from USE_SCENS_COST/USE_SCENS_CONSTR)
 *              and <plant> is PLANT_TAG (plant.h), so builds with and
 *              without scenarios write side by side, never over each other.
 *
 * NOTE ON VARIABLE-LENGTH ARRAYS
 * --------------------------------
 * nSim is a constexpr so that ySim[nSim] and uSim[nSim] are arrays with
 * compile-time-known sizes, NOT variable-length arrays (VLAs). VLAs are
 * arrays the size of which is a runtime variable. Such data structures
 * are forbidden.
 */

#include "setup.h"
#include "plant.h"
#include <stdio.h>

#if defined(USE_SCENS_COST) && defined(USE_SCENS_CONSTR)
  #define TB_CTRL_TAG "scen"
#elif defined(USE_SCENS_COST)
  #define TB_CTRL_TAG "scencost"
#elif defined(USE_SCENS_CONSTR)
  #define TB_CTRL_TAG "scenconstr"
#else
  #define TB_CTRL_TAG "noscen"
#endif

inline void generateReference(output_type yref[], int nSim);

int main(void)
{
    printf("\n=== SCMPC ARX simulation  |  system=%d  mode=%d ===\n\n",
           ACTIVE_SYSTEM, CTRL_MODE);
    printf("Operation modes:\n");
    printf("\tFixed point: %s\n", FIXED_PRINT);
    printf("\tADC/DAC: %s\n", CONVERSIONS_MODE_PRINT);
    printf("\tNormalization: %s\n", NRMLZ_PRINT);
    printf("\tUse scenarios in cost: %s\n", USE_SCENS_COST_PRINT);
    printf("\tUse scenarios constraints: %s\n", USE_SCENS_CONSTR_PRINT);
    printf("\tUse rand(): %s\n", PRNG_STDLIB_PRINT);
    /* FIXED applies the output weights as shifts by log2Q/log2P
     * (costFunctionArx.cpp): any mismatch means FIXED and floating point
     * optimize different costs. */
    if (double(outputWeight) != std::ldexp(1.0, log2Q) ||
        double(terminalOutputWeight) != std::ldexp(1.0, log2P))
        printf("WARNING: outputWeight/terminalOutputWeight = %g/%g but 2^log2Q/2^log2P = %g/%g:\n"
               "         FIXED and floating point will use different weights (setup.h).\n",
               double(outputWeight), double(terminalOutputWeight),
               std::ldexp(1.0, log2Q), std::ldexp(1.0, log2P));
    printf("Experiment (plant.h):\n");
    printf("\tRuns: %d, true plant: %s, plant model: %s, seed: %u\n",
           N_RUNS, RANDOM_TRUE_PLANT ? "random" : "THETA_TRUE_INIT",
           PLANT_TAG, (unsigned)TB_SEED);
    printf("\n\n");

    /* ------------------------------------------------------------------ */
    /*  Simulation parameters                                              */
    /* ------------------------------------------------------------------ */
    constexpr int nSim = 300;

    /* Arrays to log one run's trajectory (reused by every run). */
    output_type ySim[nSim];
    input_type  uSim[nSim];
    output_type yref[nSim]; // output follows this
    output_type yCurr;
    vol_type volumes[nSim];
    noise_type noise[nSim];
    #if defined(CONVERSIONS_MODE)
    digital_output_type ySimDig[nSim];
    digital_input_type  uSimDig[nSim];
    #endif

    /* Reference: the same for every run */
    generateReference(yref, nSim);

    TbRng rng(TB_SEED);

    FILE* summary = nullptr;
    char fileName[128];
    if (N_RUNS > 1)
    {
        snprintf(fileName, sizeof fileName, "runs_%s_%s_summary.txt", TB_CTRL_TAG, PLANT_TAG);
        summary = fopen(fileName, "w");
        if (!summary) { printf("ERROR: could not open %s for writing.\n", fileName); return 1; }
    }
    int runsViolY = 0, runsViolDy = 0;

    bool coSimFailed = false;   /* set by the co-simulation check below */
    for (int run = 0; run < N_RUNS; run++)
    {
    /* ------------------------------------------------------------------ */
    /*  This run's true system, noise and a freshly reset controller      */
    /* ------------------------------------------------------------------ */
    Plant plant;
    plantSampleTrue(plant, rng, RANDOM_TRUE_PLANT != 0);
    plantReset(plant);
    resetControllerState();
    pseudoRandReset();

    /* Measurement noise, uniform in [-SIGMA_UNNORM, SIGMA_UNNORM] */
    for (int i = 0; i < nSim; i++) noise[i] = rng.uniform() * double(SIGMA_UNNORM);

    #ifdef DEBUG_PRINT
    double ySim_f[nSim], yref_f[nSim], volumes_f[nSim], noise_f[nSim], yCurr_f, uOpt_f;
    for(int i = 0; i < nSim; i++)           noise_f[i] = noise[i].to_double();
    for(int i = 0; i < nSim; i++)           yref_f[i] = yref[i].to_double();
    #endif

    /* ------------------------------------------------------------------ */
    /*  Closed-loop simulation                                            */
    /* ------------------------------------------------------------------ */
    for (int k = 0; k < nSim; k++)
    {
        // Current zonotope volume computation
        /* sum of |det| over nUnc-column subsets: = |det(thetaGens)| when
         * the generator matrix is square, and still defined when it is not */
        volumes[k] = zonotopeVolume(thetaGens);

        #ifdef DEBUG_PRINT
        volumes_f[k] = volumes[k].to_double();
        #endif

        /* --- True plant output y(k), plus measurement noise ----------- */
        yCurr = plantNextOutput(plant);
        #ifdef DEBUG_PRINT
        yCurr_f = yCurr.to_double();
        #endif
        yCurr += noise[k];
        ySim[k] = yCurr;
        #ifdef DEBUG_PRINT
        ySim_f[k] = ySim[k].to_double();
        #endif

        /* --- Convert y(k) to digital ----------------------------------- */
        #ifdef CONVERSIONS_MODE
        digital_output_type yrefDig = ADConvertY(yref[k]);
        digital_output_type yCurrDig = ADConvertY(yCurr);
        ySimDig[k] = yCurrDig;
        #else
        digital_output_type yrefDig = yref[k];
        digital_output_type yCurrDig = yCurr;
        #endif

        /* --- Call controller ------------------------------------------- */
        /*
         * controller() receives y(k) and yref as digital samples.
         * It returns the updated uOptDig sequence, whose first element
         * uOptDig[0] is u(k) in the receding-horizon sense.
         * The controller internally updates its own history; we do NOT
         * duplicate that update here.
         */
        digital_input_type uOptDig = controller(yCurrDig, yrefDig);

        /* Receding-horizon: only u(k) = uOpt[0] is applied. */
        #ifdef CONVERSIONS_MODE
        uSimDig[k] = uOptDig;
        uSim[k] = DAConvertU(uOptDig);
        #else
        uSim[k] = uOptDig;
        #endif

        #ifdef DEBUG_PRINT
        uOpt_f = uSim[k].to_double();
        #endif

        /* --- The plant records the measured y(k) and the applied u(k) --- */
        plantPush(plant, double(ySim[k]), double(uSim[k]));
    }

    /* ------------------------------------------------------------------ */
    /*  Write this run's trajectory to file                               */
    /* ------------------------------------------------------------------ */
    if (N_RUNS == 1)
        snprintf(fileName, sizeof fileName, "output.txt");
    else
        snprintf(fileName, sizeof fileName, "runs_%s_%s_%03d.txt", TB_CTRL_TAG, PLANT_TAG, run);
    FILE* fp = fopen(fileName, "w");
    if (!fp) {
        printf("ERROR: could not open %s for writing.\n", fileName);
        return 1;
    }

    /* Digital columns only exist with ADC/DAC conversions (FIXED without
     * CONVERSIONS_MODE used to print uSimDig/ySimDig, which are not
     * declared in that mode, and failed to compile) */
    #if defined(CONVERSIONS_MODE)
        fprintf(fp, "uSim ySim yref uMin uMax yMin yMax deltaY uSimDig ySimDig vol\n");
    #else
        fprintf(fp, "uSim ySim yref uMin uMax yMin yMax deltaY vol\n");
    #endif

    for (int k = 0; k < nSim; k++)
    {
#if defined(FIXED) && defined(CONVERSIONS_MODE)
        fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %e\n",
                uSim[k].to_float(), ySim[k].to_float(),
                yref[k].to_float(),
                UMIN.to_float(), UMAX.to_float(),
                YMIN.to_float(), YMAX.to_float(),
                DELTAY.to_float(),
                uSimDig[k].to_float(), ySimDig[k].to_float(),
                volumes[k].to_float()/volumes[0].to_float());
#elif defined(FIXED)
        fprintf(fp, "%f %f %f %f %f %f %f %f %e\n",
                uSim[k].to_float(), ySim[k].to_float(),
                yref[k].to_float(),
                UMIN.to_float(), UMAX.to_float(),
                YMIN.to_float(), YMAX.to_float(),
                DELTAY.to_float(),
                volumes[k].to_float()/volumes[0].to_float());
#elif defined(CONVERSIONS_MODE)
        fprintf(fp, "%f %f %f %f %f %f %f %f %d %d %e\n",
                (double)uSim[k], (double)ySim[k],
                (double)yref[k],
                (double)UMIN, (double)UMAX,
                (double)YMIN, (double)YMAX,
                (double)DELTAY,
                (int)uSimDig[k], (int)ySimDig[k],
                (double)volumes[k]/(double)volumes[0]);
#else
        fprintf(fp, "%f %f %f %f %f %f %f %f %e\n",
                (double)uSim[k], (double)ySim[k],
                (double)yref[k],
                (double)UMIN, (double)UMAX,
                (double)YMIN, (double)YMAX,
                (double)DELTAY,
                (double)volumes[k]/(double)volumes[0]);
#endif
    }

    fclose(fp);

    /* ------------------------------------------------------------------ */
    /*  Constraint-violation metrics (physical units)                     */
    /* ------------------------------------------------------------------ */
    /*
     * maxViolY : largest distance of y(k) outside [YMIN, YMAX]  (0 = none)
     * nViolY   : number of samples outside [YMIN, YMAX]
     * maxViolDy, nViolDy: the same for |y(k) - y(k-1)| > DELTAY
     * rmse     : tracking RMSE, sqrt(mean((yref - y)^2))
     */
    double maxViolY = 0.0, maxViolDy = 0.0, sqErr = 0.0;
    int nViolY = 0, nViolDy = 0;
    for (int k = 0; k < nSim; k++)
    {
        const double y = double(ySim[k]);
        const double v = (y > double(YMAX)) ? y - double(YMAX) :
                         (y < double(YMIN)) ? double(YMIN) - y : 0.0;
        if (v > 0.0) { nViolY++; if (v > maxViolY) maxViolY = v; }
        if (k > 0)
        {
            const double dy = y - double(ySim[k - 1]);
            const double vd = ((dy < 0.0) ? -dy : dy) - double(DELTAY);
            if (vd > 0.0) { nViolDy++; if (vd > maxViolDy) maxViolDy = vd; }
        }
        const double e = double(yref[k]) - y;
        sqErr += e * e;
    }
    const double rmse = std::sqrt(sqErr / nSim);
    runsViolY  += (nViolY  > 0);
    runsViolDy += (nViolDy > 0);

    printf("%s run %3d: y outside [yMin,yMax] %3d samples (max %.4g), |dy| > deltaY %3d samples (max %.4g), RMSE %.4g\n",
           fileName, run, nViolY, maxViolY, nViolDy, maxViolDy, rmse);
    fflush(stdout); /* progress is visible during long (e.g. FIXED) multi-run simulations */

    if (summary)
    {
        if (run == 0)
        {
            fprintf(summary, "run");
            plantPrintParams(plant, summary, true);
            fprintf(summary, " maxViolY nViolY maxViolDy nViolDy rmse\n");
        }
        fprintf(summary, "%d", run);
        plantPrintParams(plant, summary, false);
        fprintf(summary, " %.6e %d %.6e %d %.6e\n", maxViolY, nViolY, maxViolDy, nViolDy, rmse);
    }

    /* ------------------------------------------------------------------ */
    /*  C/RTL co-simulation check (run 0)                                 */
    /* ------------------------------------------------------------------ */
    /*
     * Vitis reports co-simulation as passed whenever main() returns 0: it
     * does not compare the RTL with the C model by itself. So C simulation
     * saves run 0's applied inputs as the golden reference, and
     * co-simulation (which Vitis builds with __RTL_SIMULATION__ and runs
     * in <solution>/sim/wrapc*, two levels below <solution>/csim/build)
     * compares the RTL's against it: any difference makes main() return 1,
     * i.e. a FAILED co-simulation. Run C simulation first.
     */
    if (run == 0)
    {
        #ifndef __RTL_SIMULATION__
        FILE* g = fopen("golden_u.txt", "w");
        if (g)
        {
            for (int k = 0; k < nSim; k++) fprintf(g, "%.17g\n", double(uSim[k]));
            fclose(g);
        }
        #else
        FILE* g = fopen("../../csim/build/golden_u.txt", "r");
        if (!g) g = fopen("golden_u.txt", "r");
        if (!g)
        {
            printf("CO-SIM CHECK FAILED: golden_u.txt not found - run C simulation first.\n");
            coSimFailed = true;
        }
        else
        {
            int nMismatch = 0, firstMismatch = -1;
            for (int k = 0; k < nSim; k++)
            {
                double ug;
                if (fscanf(g, "%lf", &ug) != 1 || ug != double(uSim[k]))
                {
                    if (firstMismatch < 0) firstMismatch = k;
                    nMismatch++;
                }
            }
            fclose(g);
            if (nMismatch)
            {
                printf("CO-SIM CHECK FAILED: the RTL's input u(k) differs from C simulation in %d of %d samples (first at k = %d).\n",
                       nMismatch, nSim, firstMismatch);
                coSimFailed = true;
            }
            else
                printf("CO-SIM CHECK PASSED: the RTL's input u(k) matches C simulation in all %d samples.\n", nSim);
        }
        #endif
    }
    } /* runs */

    if (summary)
    {
        fclose(summary);
        printf("\n%d runs: %d with y outside [yMin,yMax], %d with |dy| > deltaY.\n",
               N_RUNS, runsViolY, runsViolDy);
    }
    printf("Simulation complete.\n");
    return coSimFailed ? 1 : 0;
}

/* ======================================================================
   generateReference()
   Fills yref[nSim] with the reference signal for the active
   system and reference type.

   @param yref  [out] Reference array, length nSim.
   ====================================================================== */
inline void generateReference(output_type yref[], int nSim)
{
#if   ACTIVE_SYSTEM == SYSTEM_SIMPLE
    for(int i = 0; i < nSim; i++) yref[i] = (output_type)5;
#elif ACTIVE_SYSTEM == SYSTEM_BENCHMARK
    for(int i = 0; i < nSim; i++) yref[i] = (i <  nSim / 2) ? (output_type)-1.2 : (output_type)1.2;
#elif ACTIVE_SYSTEM == SYSTEM_MILANO
    for (int k = 0; k < nSim; k++) yref[k] = (output_type)100.0 * (output_type)pseudoRandArx();
#elif ACTIVE_SYSTEM == SYSTEM_BUCK
    for(int i = 0; i < nSim; i++) yref[i] = (i < 75) ? 3 :
                                            (i < 150) ? 4 :
                                            (i < 225) ? 5 : 7;
#elif ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
    for(int i = 0; i < nSim; i++) yref[i] = (i < 75) ? 3 :
                                            (i < 150) ? 4 :
                                            (i < 225) ? 5 : 7;
    // for(int i = 0; i < nSim; i++) yref[i] = (sin(2 * M_PI / 3 * i * 1e-2) > 0 ? 10 * sin(2 * M_PI / 3 * i * 1e-2) : -10 * sin(2 * M_PI / 3 * i * 1e-2));
#elif ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
    for(int i = 0; i < nSim; i++) yref[i] = (i < 75) ? 80 :
                                            (i < 150) ? 60 :
                                            (i < 225) ? 40 : 80;
#elif ACTIVE_SYSTEM == SYSTEM_GAIN_DEMO
    /*
     * Safe/low reference for the first third (both certainty-equivalent
     * and scenario MPC behave identically here - no risk near YMAX),
     * then a step up to 4.9 (just under YMAX=5) for the rest of the run
     * - the regime where the nominal model's underestimated gain (4 vs.
     * the true 6) matters. See system_configs.h's SYSTEM_GAIN_DEMO
     * comment for the full derivation and the measured closed-loop
     * numbers this reference profile produces.
     */
    for(int i = 0; i < nSim; i++) yref[i] = (i < 100) ? 1.0 : 4.9;
#elif ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
    for(int i = 0; i < nSim; i++) yref[i] = 0.78;
#endif  /* ACTIVE_SYSTEM */
}
