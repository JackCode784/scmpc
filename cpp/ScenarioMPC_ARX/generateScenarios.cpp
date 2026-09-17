/**
 * @file  generateScenarios.cpp
 * @brief Scenario generation for the scenario-based robust MPC.
 *
 * THEORY
 * ------
 * The parameter uncertainty is represented as the zonotope
 *
 *   Z = { center + G·ξ  :  ‖ξ‖∞ ≤ 1 }  ⊂  ℝ^nTheta
 *
 * where  G ∈ ℝ^{nTheta × nGens}  is the generator matrix and
 *        ξ ∈ ℝ^nGens             is a coefficient vector with each entry in [−1, 1].
 *
 * nTheta is the number of parameters (= na + nb) — the number of ROWS of G.
 * nGens   is the number of generators        — the number of COLUMNS of G.
 *
 * These two dimensions are independent.  In particular:
 *   - CONFIG_BENCHMARK initially has nTheta=4 rows and nGens=6 columns.
 *   - After the PL interval-hull reduction, nGens is brought back to nTheta,
 *     making G square.  From that point on nGens == nTheta.
 *   - CONFIG_MILANO has nTheta=nGens=6 from the start (square matrix).
 *   - CONFIG_SIMPLE  has nTheta=nGens=3 from the start (diagonal matrix).
 *
 * Drawing a random ξ ∈ [−1,1]^nGens and computing θ = center + G·ξ gives
 * one scenario — a parameter vector guaranteed to lie inside Z.
 *
 * WHY nGens IS AN EXPLICIT ARGUMENT
 * ----------------------------------
 * The coefficients ξ must have one entry per generator COLUMN, so their
 * count is nGens, not nTheta.  Hardcoding nTheta as the coefficient count
 * would silently give wrong results whenever nGens ≠ nTheta (e.g. when
 * called with the initial CONFIG_BENCHMARK zonotope, which has 6 generators
 * for 4 parameters).
 *
 * The caller passes nGens (the extern global defined in controller.cpp)
 * which is set once by controllerInit() from ACTIVE_CONFIG.Z0.nGens and
 * remains constant because boundStripZonotopeIntersection preserves the
 * generator count.  Passing it explicitly rather than reading the global
 * directly keeps the data flow visible to the HLS scheduler.
 *
 * The array size for gens uses nGens (the compile-time upper bound)
 * because C++ function parameters cannot have a runtime-variable second
 * dimension.  Only the first nGens columns are accessed.
 */

#include "setup.h"

/**
 * Fill thetaScenarios[0..Nscen−1] with Nscen parameter vectors drawn
 * uniformly at random from  Z = { center + gens·ξ  :  ‖ξ‖∞ ≤ 1 }.
 *
 * @param thetaScenarios [out]  Nscen × nTheta array of scenario vectors.
 * @param center         [in]   Zonotope centre, length nTheta.
 * @param gens           [in]   Generator matrix, nTheta rows × nGens columns,
 *                               stored row-major: gens[i][j] is the i-th
 *                               component of the j-th generator.
 *                               The array is declared with nGens columns;
 *                               only columns 0 … nGens−1 are read.
 * @param nGens           [in]   Number of active generator columns.
 *                               Must satisfy  1 ≤ nGens ≤ nGens.
 *
 * HLS synthesis hints (place in this function body before the loops):
 *   #pragma HLS ARRAY_PARTITION variable=gens   complete dim=2
 *   #pragma HLS ARRAY_PARTITION variable=center complete dim=1
 * These allow all nGens columns of gens and all entries of center to be
 * read in a single clock cycle, enabling full pipelining of the inner loop.
 */
void generateScenarios(theta_type       thetaScenarios[Nscen][nTheta],
                       const theta_type center        [nTheta],
                       const theta_type gens          [nTheta][nGens])
{
    #ifdef PRAGMAS
    /*
     * gens and center are complete-partitioned so every entry can be read
     * in the same cycle by the fully-unrolled j/k loops below (nTheta and
     * nGens are both <= 6, so this costs a handful of extra read ports,
     * not BRAMs). thetaScenarios itself is partitioned by the caller
     * (controller.cpp), at its point of declaration.
     */
    #pragma HLS ARRAY_PARTITION variable=gens   complete dim=0
    #pragma HLS ARRAY_PARTITION variable=center complete dim=1
    #endif

    /*
     * coeffs[k] ∈ [−1, 1] — one random coefficient per generator column.
     * The array is sized to the compile-time maximum; only the first nGens
     * entries are written by pseudoRandArx and read in the inner loop.
     *
     * pseudoRandArx() uses the XOR-shift PRNG (hardware-safe) or rand()
     * if PRNG_STDLIB is defined (software simulation only).
     */
    rand_type coeffs[nGens];
    #ifdef PRAGMAS
    #pragma HLS ARRAY_PARTITION variable=coeffs complete dim=1
    #endif

    for (int i = 0; i < Nscen; i++)
    {
        #ifdef PRAGMAS
        /*
         * NOT unrolled: pseudoRandArx(coeffs) advances the single shared
         * xorshift32 state (see pseudoRand.cpp), so scenario i+1's random
         * draw genuinely depends on scenario i's - the Nscen iterations
         * cannot be computed in parallel. PIPELINE still overlaps the
         * dot-product arithmetic of one scenario with the next draw.
         */
        #pragma HLS PIPELINE
        #endif

        /* Draw nGens independent random coefficients ξ₀, …, ξ_{nGens−1}. */
        pseudoRandArx(coeffs);

        /*
         * Compute  θ_i = center + gens · coeffs:
         *
         *   thetaScenarios[i][j] = center[j]
         *                        + Σ_{k=0}^{nGens−1}  gens[j][k] · coeffs[k]
         *
         * theta_j accumulates the dot product for row j before writing to
         * thetaScenarios.  Accumulating into a local variable rather than
         * directly into thetaScenarios[i][j] makes it explicit to HLS that
         * there is no loop-carried dependency on the output array.
         */
        for (int j = 0; j < nTheta; j++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            theta_type theta_j = center[j];

            for (int k = 0; k < nGens; k++)
            {
                #ifdef PRAGMAS
                #pragma HLS UNROLL
                #endif
                theta_j += gens[j][k] * coeffs[k];
            }

            thetaScenarios[i][j] = theta_j;
        }
    }
}
