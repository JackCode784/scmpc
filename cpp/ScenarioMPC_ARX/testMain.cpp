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
 * At each time step k the harness:
 *  1. Simulates the plant (true ARX with thetaTrue) to obtain y(k).
 *  2. Converts y(k) and yref to digital samples (12-bit integers) - this mimics an AD conversion.
 *  3. Calls controller(), which returns the optimal digital input.
 *  4. Converts the digital input back to physical units for the plant.
 *
 * CONTROLLER STATE
 * --------------------------------
 * The controller has the global history arrays (yHist, uHist), 
 * updated autonomously at each time step k and starting at zero.
 *
 * OUTPUT FILE FORMAT
 * ------------------
 * "output.txt" has one row per simulation step; the columns depend on
 * the compilation mode.
 * 
 * NOTE ON VARIABLE-LENGTH ARRAYS
 * --------------------------------
 * nSim is a constexpr so that ySim[nSim] and uSim[nSim] are arrays with
 * compile-time-known sizes, NOT variable-length arrays (VLAs). VLAs are
 * arrays the size of which is a runtime variable. Such data structures 
 * are forbidden.
 */

#include "setup.h"
#include <stdio.h>

#ifdef PRNG_STDLIB
  #include <cstdlib>
  #include <time.h>
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
    printf("\tUse rand(): %s\n", PRNG_STDLIB_PRINT);
    printf("\n\n");

    #ifdef PRNG_STDLIB
    srand(time(NULL));  // set random seed
    #endif

    /* ------------------------------------------------------------------ */
    /*  Simulation parameters                                              */
    /* ------------------------------------------------------------------ */
    constexpr int nSim = 300;

    /* Arrays to log the full simulation trajectory. */
    output_type ySim[nSim];
    input_type  uSim[nSim];
    output_type yref[nSim]; // output follows this
    output_type yCurr;
    vol_type volumes[nSim];
    noise_type noise[nSim];
    /* ------------------------------------------------------------------ */
    /*  True system (plant)                                               */
    /* ------------------------------------------------------------------ */
    /*
     * thetaTrue is the parameter vector of the REAL plant used to generate
     * the output data. It is taken from system_configs.h, depending on 
     * the chosen ACTIVE_SYSTEM.
     * The controller's initial zonotope has to contain thetaTrue and,
     * hopefully, the updated zonotopes over time will still contain it.
     */
    theta_type thetaTrue[nTheta] = { THETA_TRUE_INIT };
    #if defined(CONVERSIONS_MODE)
    digital_output_type ySimDig[nSim];
    digital_input_type  uSimDig[nSim];
    #ifdef DEBUG_PRINT
    double yrefDig_f[nSim], ySimDig_f[nSim];
    #endif
    #endif

    /* Measurement noise amplitude and values */
    for(int i=0; i < nSim; i++) noise[i] = pseudoRandArx() * SIGMA_UNNORM; // noise in [-1, 1]
    
    // generate reference trajectory based on ACTIVE_SYSTEM
    generateReference(yref, nSim);
    
    #ifdef DEBUG_PRINT
    double ySim_f[nSim], yref_f[nSim], volumes_f[nSim], noise_f[nSim], thetaTrue_f[nTheta], yCurr_f, uOpt_f;
    double yHist_f[na], uHist_f[nb];
    for(int i = 0; i < nSim; i++)           noise_f[i] = noise[i].to_double();
    for(int i = 0; i < nSim; i++)           yref_f[i] = yref[i].to_double();
    for(int i = 0; i < nTheta; i++)         thetaTrue_f[i] = thetaTrue[i].to_double();

    #ifdef NRMLZ
    double yNormGain_f, yNormOffset_f, uNormGain_f, uNormOffset_f;
    yNormGain_f = yNormGain.to_double();
    yNormOffset_f = yNormOffset.to_double();
    uNormGain_f = uNormGain.to_double();
    uNormOffset_f = uNormOffset.to_double();
    #endif
    #endif
    
    /* ------------------------------------------------------------------ */
    /*  Closed-loop simulation                                            */
    /* ------------------------------------------------------------------ */
    for (int k = 0; k < nSim; k++)
    {
        /* --- Simulate plant output y(k) -------------------------------- */
        /*
        * computeArxOutput evaluates
        *   y(k) = [y(k−1),...,y(k−na), u(k−nk),...,u(k−nk−nb+1)]^T * thetaTrue
        */

        // Current zonotope volume computation
        volumes[k] = matDet(thetaGens);
        volumes[k] = (volumes[k] < 0) ? (vol_type)(-volumes[k]) : volumes[k];

        #ifdef DEBUG_PRINT
        volumes_f[k] = volumes[k].to_double();
        for(int i = 0; i < na; i++)             yHist_f[i] = yHist[i].to_double();
        for(int i = 0; i < nb + nk - 1; i++)    uHist_f[i] = uHist[i].to_double();
        #endif

        yCurr = 0;
        #ifdef NRMLZ
        for(int i = 0; i < nTheta; i++) 
            yCurr += ((i < na) ? double(yHist[i] - yNormOffset)/double(yNormGain) : double(uHist[i-na+nk-1] - uNormOffset)/double(uNormGain)) * double(thetaTrue[i]);
        #else
        for(int i = 0; i < nTheta; i++)
            yCurr += ((i < na) ? yHist[i] : uHist[i-na+nk-1]) * thetaTrue[i];
        #endif
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
        #ifdef DEBUG_PRINT
        yrefDig_f[k] = yrefDig.to_double();
        ySimDig_f[k] = yCurrDig.to_double();
        #endif
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

        #ifdef DEBUG_PRINT
        uOpt_f = uOptDig.to_double(); // digital value if CONVERSIONS_MODE
        for(int i = 0; i < na; i++) yHist_f[i] = yHist[i].to_double();
        for(int i = 0; i < nb+nk-1; i++) uHist_f[i] = uHist[i].to_double();
        #endif
        
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
        fprintf(fp, "%f %f %f %f %f %f %f %f %f %e\n",
                uSim[k].to_float(), ySim[k].to_float(),
                yref[k].to_float(),
                UMIN.to_float(), UMAX.to_float(),
                YMIN.to_float(), YMAX.to_float(),
                uSimDig[k].to_float(), ySimDig[k].to_float(),
                volumes[k].to_float()/volumes[0].to_float());
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
#endif  /* ACTIVE_SYSTEM */
}
