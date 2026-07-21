/**
 * @file  boundStripZonotopeIntersection.cpp
 * @brief Zonotope-strip intersection functions for bounded-error PL/AL.
 *
 * TWO FUNCTIONS
 * -------------
 * boundStripZonotopeIntersection()
 *   Original algorithm (lambda-optimal, Combastel 2003 / Bravo 2006 II).
 *   Produces a zonotope with nTheta+1 generator columns from one with nTheta.
 *   Kept for reference and regression testing.
 *
 * boundStripZonotopeIntersectionNew()
 *   Volume-minimising algorithm (Bravo et al. 2006, Algorithm 1).
 *   Preserves the generator count nGens; selects the generator to align with
 *   the strip normal by exhaustive search over nGens candidates, keeping the
 *   candidate that minimises the new zonotope volume.
 *
 * STRIP REPRESENTATION
 * ---------------------
 * The strip  S = { theta : |h'theta − y| <= sigma }  is represented by two offsets:
 *   offset[0] = y + sigma   (upper bound of  h'theta)
 *   offset[1] = sigma − y   (upper bound of −h'theta, i.e. negative of lower bound)
 *
 * Given a zonotope Z = { c + Gξ : ‖ξ‖∞ ≤ 1 }, the support of Z along ±h is:
 *   supOffset[0] =  h'c + sum_j |h'g_j|   (max of  h'theta over Z)
 *   supOffset[1] = −h'c + sum_j |h'g_j|   (max of −h'theta over Z)
 *
 * The tight strip is the intersection of S with the projection of Z:
 *   tightOffset[i] = min(offset[i], supOffset[i])
 *
 * From the tight offsets, the tight strip centre and radius are:
 *   tsc = (tightOffset[0] − tightOffset[1]) / 2
 *   tsr = (tightOffset[0] + tightOffset[1]) / 2
 *
 * BRAVO ALGORITHM (new function)
 * --------------------------------
 * For each candidate generator index s = 0…nGens−1 with h'g_s != 0:
 *   1. New selected generator:  g_s^new = (tsr / h'g_s) * g_s
 *   2. Other generators:        g_j^new = g_j − (h'g_j / h'g_s) * g_s,  j != s
 *   3. New centre:              c^new   = c + (tsc − h'c) / h'g_s * g_s
 *   4. Compute volume of the candidate zonotope.
 * Keep the candidate with the smallest volume.
 * If no candidate improves on the original, return the original unchanged.
 *
 * STRIP HALF-WIDTH: EPSILON vs sigma
 * ------------------------------------
 * EPSILON is the worst-case noise bound (deterministic upper bound on |e(k)|).
 * sigma is the noise standard deviation (a statistical parameter).
 * The strip MUST use EPSILON — using sigma would make the strip too narrow,
 * potentially excluding the true parameter theta* on noise realisations above sigma,
 * which defeats the bounded-error guarantee.
 */

#include "setup.h"

#if CTRL_MODE == CTRL_MODE_PL || CTRL_MODE == CTRL_MODE_AL
/* ======================================================================
 * boundStripZonotopeIntersection  (original algorithm)
 *
 * Bounds the intersection of a strip and a zonotope with another zonotope,
 * the size of which is optimized with an analytic formula for a lambda parameter
 *
 *  THERE IS ONE DIVISION HERE!!!
   ====================================================================== */
void boundStripZonotopeIntersection(const output_type stripCenter, 
                                    const output_type yPast[na], 
                                    const input_type uSamples[nb + nk - 1], 
                                    const theta_type oldCenter[nTheta],
                                    const theta_type oldGens[nTheta][nTheta], 
                                    theta_type newCenter[nTheta], 
                                    theta_type newGens[nTheta][nTheta + 1])
{
    // Lambda computation
    alg_type lambda[nTheta];
    alg_type tmp[nTheta][nTheta];
    alg_type div = (alg_type)sigma * (alg_type)sigma;

    for (int i = 0; i < nTheta; i++)
    {
        // newCenter[i] = oldCenter[i]; // useful afterwards
        lambda[i] = 0;

        for (int j = 0; j < nTheta; j++)
        {
            tmp[i][j] = 0;
            for (int k = 0; k < nTheta; k++)
                tmp[i][j] += (alg_type)(oldGens[i][k] * oldGens[j][k]);
        }

        for (int j = 0; j < nTheta; j++)
        {
            // From most recent to oldest sample
            lambda[i] += tmp[i][j] * 
                        ((j < na) ? (alg_type)yPast[j] 
                                : (alg_type)uSamples[j - na + nk - 1]);
        }
    }

    for (int i = 0; i < nTheta; i++)
    {
        div += ((i < na) ? (alg_type)yPast[i] : (alg_type)uSamples[i - na + nk - 1]) * lambda[i];
    }

    for (int i = 0; i < nTheta; i++) 
        lambda[i] /= div;

    /* New center */
    alg_type term = stripCenter;
    for (int i = 0; i < nTheta; i++)
    {
        term -= ((i < na) ? (alg_type)yPast[i] : (alg_type)uSamples[i - na + nk - 1]) * (alg_type)oldCenter[i];
    }

    for (int i = 0; i < nTheta; i++)
        newCenter[i] = (theta_type)(lambda[i] * term) + oldCenter[i];
        
    /* New generators */
    for (int i = 0; i < nTheta; i++)
    {
        newGens[i][nTheta] = (theta_type)(lambda[i] * (alg_type)sigma);

        for (int j = 0; j < nTheta; j++)
        {
            newGens[i][j] = 0; // preparation for upcoming for loop
            tmp[i][j] = (i == j) ? (alg_type)1 : (alg_type)0;
            tmp[i][j] -= lambda[i] * 
                        ((j < na) ? (alg_type)yPast[j] : (alg_type)uSamples[j - na + nk - 1]);
        }

        for (int j = 0; j < nTheta; j++)
        {
            for (int k = 0; k < nTheta; k++)
            {
                newGens[i][j] += (theta_type)(tmp[i][k] * (alg_type)oldGens[k][j]);
            }
        }
    }
}

/* 
    New Bravo et al. 2006 algorithm for zonotope update.
    Check for DIVISIONS, since they are costly on
    resource-constrained hardware like FPGAs/microcontrollers.
*/
void boundStripZonotopeIntersectionNew(const strip_center_type stripCenter, 
                                    const phi_type phi[nTheta],
                                    const norm_noise_type stripRadius,
                                    const theta_type oldCenter[nTheta],
                                    const theta_type oldGens[nTheta][nGens], 
                                    theta_type newCenter[nTheta], 
                                    theta_type newGens[nTheta][nGens])
{
    /* ------------------------------------------------------------------
    * Step 1 — Assemble the regressor vector phi = [yPast | uSamples]
    * ------------------------------------------------------------------ */
   
   /* Normal strip generation */
    output_strip_offset_type stripOffset[2] = {stripCenter + stripRadius, stripRadius - stripCenter};
    
    /* Support strip for current zonotope */
    proj_type cproj = 0;
    for(int i = 0; i < nTheta; i++) cproj += phi[i] * oldCenter[i];

    proj_type gproj[nGens];
    support_strip_offset_type supStripOffset[2] = {0, 0};
    
    for(int i = 0; i < nGens; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        gproj[i] = 0;
        for(int j = 0; j < nTheta; j++) gproj[i] += phi[j] * oldGens[j][i];

        supStripOffset[0] += (gproj[i] < 0) ? (-gproj[i]) : gproj[i];
    }
    supStripOffset[1] = supStripOffset[0] - cproj;
    supStripOffset[0] += cproj;
    
    /* Tight strip */
    tight_strip_offset_type tightStripOffset[2] = {
        (stripOffset[0] < supStripOffset[0]) ? stripOffset[0] : supStripOffset[0],
        (stripOffset[1] < supStripOffset[1]) ? stripOffset[1] : supStripOffset[1]
    };
    tight_strip_center_type tsc = tightStripOffset[0] - tightStripOffset[1];
    tight_strip_radius_type tsr = tightStripOffset[0] + tightStripOffset[1];
    #ifdef FIXED
    tsc = tsc >> 1;
    tsr = tsr >> 1;
    #else
    tsc /= 2;
    tsr /= 2;
    #endif

    /* Search over generator candidates */
    alg_type bestVol = zonotopeVolume(oldGens);
    int changed = 0;
    theta_type tmpGens[nTheta][nGens];
    
    for(int genId = 0; genId < nGens; genId++)
    {
        #ifdef PRAGMAS
        #pragma HLS PIPELINE
        #endif

        if(gproj[genId] != (alg_type)0)
        {
            alg_type gprojinv = (alg_type)1 / gproj[genId];
            for(int i = 0; i < nTheta; i++)
            {
                #ifdef PRAGMAS
                #pragma HLS UNROLL
                #endif
                for(int j = 0; j < nGens; j++) 
                {                    
                    #ifdef PRAGMAS
                    #pragma HLS UNROLL
                    #endif
                    tmpGens[i][j] = (j == genId) ? 
                                    (theta_type)((theta_type)(tsr * gprojinv) * oldGens[i][j]) :
                                    (theta_type)(oldGens[i][j] - ((theta_type)(gproj[j] * gprojinv) * oldGens[i][genId]));
                }
            }
            alg_type tmpVol = zonotopeVolume(tmpGens);

            // Change best zonotope if volume is smaller
            if(tmpVol < bestVol)
            {
                changed = 1;
                bestVol = tmpVol;
                for(int i = 0; i < nTheta; i++)
                {
                    #ifdef PRAGMAS
                    #pragma HLS UNROLL
                    #endif
                    newCenter[i] = oldCenter[i] + (theta_type)(tsc - cproj) * (theta_type)gprojinv * oldGens[i][genId];

                    for(int j = 0; j < nGens; j++) 
                    {
                        #ifdef PRAGMAS
                        #pragma HLS UNROLL
                        #endif
                        newGens[i][j] = tmpGens[i][j];
                    }
                }
            }
        }        
    }
    
    if(!changed)
    {
        for(int i = 0; i < nTheta; i++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            newCenter[i] = oldCenter[i];
            for(int j = 0; j < nGens; j++) 
            {
                #ifdef PRAGMAS
                #pragma HLS UNROLL
                #endif
                newGens[i][j] = oldGens[i][j];
            }
        }
    }
}

#endif