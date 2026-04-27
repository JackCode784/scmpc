/**
 * @file  testMain.cpp
 * @brief PC-side closed-loop simulation for SCMPC on an ARX system.
 *
 * This file is a SOFTWARE-ONLY simulation harness.  It is never synthesised
 * by Vitis HLS; it exists to verify controller behaviour before flashing
 * hardware.
 *
 * SIMULATION STRUCTURE
 * ---------------------
 * At each time step k the harness:
 *  1. Simulates the plant (true ARX with thetaTrue) to obtain y(k).
 *  2. Converts y(k) and yref to digital samples (12-bit integers).
 *  3. Calls controller(), which returns the optimal digital input.
 *  4. Converts the digital input back to physical units for the plant.
 *  5. Updates the plant's own state buffers (separate from the controller's).
 *
 * CONTROLLER STATE
 * --------------------------------
 * The controller has its own history arrays (yHist, uHist) used only
 * inside its file to simulate the true process. It updates autonomously 
 * and start at zero.
 *
 * OUTPUT FILE FORMAT
 * ------------------
 * "output.txt" has one row per simulation step; the columns depend on
 * the compilation mode
 * 
 * NOTE ON VARIABLE-LENGTH ARRAYS
 * --------------------------------
 * nSim is a constexpr so that ySim[nSim] and uSim[nSim] are arrays with
 * compile-time-known sizes, NOT variable-length arrays (VLAs).  VLAs are
 * a GCC extension not present in standard C++14 and unsupported by some
 * compilers.
 */

#include "setup.h"
#include <stdio.h>

#ifdef PRNG_STDLIB
  #include <cstdlib>
  #include <time.h>
#endif

int main(void)
{
    printf("\n=== SCMPC ARX simulation  |  system=%d  mode=%d ===\n\n",
           ACTIVE_SYSTEM, CTRL_MODE);

#ifdef PRNG_STDLIB
    srand(time(NULL));   /* different seed each run */
#endif

    /* ------------------------------------------------------------------ */
    /*  Simulation parameters                                              */
    /* ------------------------------------------------------------------ */
    constexpr int nSim = 300;

    /* Arrays to log the full simulation trajectory. */
    output_type ySim[nSim];
    input_type  uSim[nSim];
    alg_type volumes[nSim];

    /* Measurement noise amplitude and values */
    rand_type noise[nSim];
    for(int i=0; i < nSim; i++) noise[i] = pseudoRandArx(); // noise in [-1, 1]

#if defined(CONVERSIONS_MODE) || defined(FIXED)
    digital_output_type ySimDig[nSim];
    digital_input_type  uSimDig[nSim];
#endif

    /* ------------------------------------------------------------------ */
    /*  True system (plant)                                               */
    /* ------------------------------------------------------------------ */
    /*
     * thetaTrue is the parameter vector of the REAL plant used to generate
     * the output data.  It is taken from ACTIVE_CONFIG.thetaTrue — the
     * scenario chosen at compile time in system_configs.h.
     * To test robustness, change the values in CONFIG_* and recompile;
     * the controller's initial zonotope will or will not contain thetaTrue
     * depending on how far the mismatch is.
     */
    theta_type thetaTrue[nTheta] = { THETA_TRUE_INIT };

    /* ------------------------------------------------------------------ */
    /*  Reference signal and initial controller input                     */
    /* ------------------------------------------------------------------ */
    output_type yref[nSim] = {0};   /* constant reference — replace with a richer
                               signal (PRBS, sinusoid) as needed           */

    /* Start the controller with u=0 for the entire horizon. */
    input_type          uOpt   [NhorU];
    digital_input_type  uOptDig[NhorU];
    for (int i = 0; i < NhorU; i++) {
        uOpt[i]    = 0;
        uOptDig[i] = ADConvertU(uOpt[i]);
    }

    
    /* ------------------------------------------------------------------ */
    /*  Closed-loop simulation                                             */
    /* ------------------------------------------------------------------ */
    for (int k = 0; k < nSim; k++)
    {
        /* --- Simulate plant output y(k) -------------------------------- */
        /*
        * computeArxOutput evaluates
        *   y(k) = θᵀ · [y(k−1),…,y(k−na), u(k−nk),…,u(k−nk−nb+1)]ᵀ
        */

        /* Reference output only for buck(loss) systems */
        yref[k] = (k < 75) ? 3 :
                (k < 150) ? 4 :
                (k < 225) ? 5 : 7;

        // Current zonotope volume computation
        volumes[k] = zonotopeVolume(thetaGens);
        volumes[k] = (volumes[k] < 0) ? -volumes[k] : volumes[k];

        #if defined(CONVERSIONS_MODE) || defined(FIXED)
            digital_output_type yrefDig = ADConvertY(yref[k]);
        #endif
        output_type yCurr = computeArxOutput(yHist, uHist, thetaTrue) + (output_type)noise[k] * (output_type)sigma;
        ySim[k] = yCurr;

        /* --- Convert y(k) to digital ----------------------------------- */
#if defined(CONVERSIONS_MODE) || defined(FIXED)
        digital_output_type yCurrDig = ADConvertY(yCurr);
        ySimDig[k] = yCurrDig;
#endif

        /* --- Call controller ------------------------------------------- */
        /*
         * controller() receives y(k) and yref as digital samples.
         * It returns the updated uOptDig sequence, whose first element
         * uOptDig[0] is u(k) in the receding-horizon sense.
         * The controller internally updates its own history; we do NOT
         * duplicate that update here.
         */
#if defined(CONVERSIONS_MODE) || defined(FIXED)
        controller(uOptDig, yCurrDig, yrefDig);
#else
        /*
         * Without CONVERSIONS_MODE the digital conversion path is not
         * compiled.  A direct float/double interface could be added here
         * for a fully analogue simulation, but the current harness always
         * compiles CONVERSIONS_MODE in software mode (see setup.h).
         */
        controller(uOptDig, ADConvertY(yCurr), ADConvertY(yref));
#endif
        /* Receding-horizon: only u(k) = uOpt[0] is applied. */
        uSimDig[k] = uOptDig[0];
        for (int i = 0; i < NhorU; i++)
            uOpt[i] = DAConvertU(uOptDig[i]);
        uSim[k] = uOpt[0];
    }

    /* ------------------------------------------------------------------ */
    /*  Write simulation output to file                                    */
    /* ------------------------------------------------------------------ */
    FILE* fp = fopen("output.txt", "w");
    if (!fp) {
        printf("ERROR: could not open output.txt for writing.\n");
        return 1;
    }

    #if defined(FIXED) || defined(CONVERSIONS_MODE)
        fprintf(fp, "uSim ySim yref uMin uMax yMin yMax uSimDig ySimDig vol\n");
    #else
        fprintf(fp, "uSim ySim yref uMin uMax yMin yMax vol\n");
    #endif

    for (int k = 0; k < nSim; k++)
    {
#ifdef FIXED
        fprintf(fp, "%f %f %f %f %f %f %f %f %f\n",
                uSim[k].to_float(),    ySim[k].to_float(),
                yref[k].to_float(),
                UMIN.to_float(), UMAX.to_float(),
                YMIN.to_float(), YMAX.to_float(),
                uSimDig[k].to_float(), ySimDig[k].to_float());
#elif defined(CONVERSIONS_MODE)
        fprintf(fp, "%f %f %f %f %f %f %f %d %d %e\n",
                (double)uSim[k], (double)ySim[k],
                (double)yref[k],
                (double)UMIN, (double)UMAX,
                (double)YMIN, (double)YMAX,
                (int)uSimDig[k], (int)ySimDig[k],
                (double)volumes[k]/(double)volumes[0]);
#else
        fprintf(fp, "%f %f %f %f %f %f %f %e\n", 
                (double)uSim[k], (double)ySim[k],
                (double)yref[k],
                (double)UMIN, (double)UMAX,
                (double)YMIN, (double)YMAX,
                (double)volumes[k]/(double)volumes[0]);
#endif
    }

    fclose(fp);
    printf("Simulation complete.\nResults written to output.txt.\n");
    return 0;
}
