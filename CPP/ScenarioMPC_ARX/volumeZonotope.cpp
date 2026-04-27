/**
 * @file  matDet.cpp
 * @brief Determinant of a square matrix and zonotope volume computation.
 *
 * CONTENTS
 * --------
 *  matDet()         — determinant of an nTheta×nTheta matrix, nTheta ≤ nTheta
 *  zonotopeVolume() — 2^nTheta · Σ_{S} |det(G_S)|, summed over all nTheta-column
 *                     subsets S of the generator matrix G ∈ ℝ^{nTheta×m}
 *
 * ALGORITHM: GAUSSIAN ELIMINATION WITH PARTIAL PIVOTING
 * -------------------------------------------------------
 * Starting from the working copy A (initialised from M with padding rows
 * set to identity), at each step k:
 *
 *   1. Find the row p ≥ k with the largest |A[p][k]| among active rows
 *      (partial pivoting — improves numerical stability and avoids
 *      division by zero for non-singular matrices).
 *   2. Swap rows p and k; flip the sign of the determinant.
 *   3. For each row i > k: subtract (A[i][k]/A[k][k]) · row k from row i,
 *      zeroing out A[i][k].
 *
 * After nTheta steps A is upper-triangular.  The determinant is the product of
 * the diagonal entries multiplied by the accumulated sign.
 *
 * Complexity: O(n³) arithmetic operations.  For nTheta = nTheta = 6
 * this is at most 216 multiply-accumulates — trivial for any FPGA.
 *
 * HLS DESIGN CHOICES
 * -------------------
 * • All loops run to the compile-time constant nTheta so that
 *   #pragma HLS UNROLL can fully unroll them.  Variable-bound loops
 *   cannot be fully unrolled by the HLS scheduler.
 *
 * • The padding region (rows and columns with index ≥ nTheta) is initialised
 *   to the identity matrix.  Padding rows have zero entries in all active
 *   columns, so the elimination factor for them is always zero — they are
 *   never modified.  Their diagonal entries contribute factor 1 to the
 *   final product, leaving the result correct.
 *
 * • Partial pivoting uses conditional assignment (the ternary operator)
 *   rather than an if/break, keeping the loop body regular and pipeline-
 *   friendly.
 *
 * • The permutation sign is stored as alg_type (+1 or −1) to avoid a
 *   type conversion at the final multiply.
 *
 * • No dynamic memory allocation, no recursion, no variable-length arrays.
 *
 * ZONOTOPE VOLUME
 * ----------------
 * The volume of the zonotope Z = { c + G·ξ : ‖ξ‖∞ ≤ 1 } ⊂ ℝ^nTheta with
 * G ∈ ℝ^{nTheta×m}  (nTheta = nTheta, m = nGens) is:
 *
 *   Vol(Z) = 2^nTheta · Σ_{S ⊆ [m], |S|=nTheta}  |det(G_S)|
 *
 * where G_S is the nTheta×nTheta submatrix formed by selecting the columns in S.
 * The sum has C(m, nTheta) terms; for m = nTheta = 6 this is just |det(G)| itself.
 *
 * Column subsets are enumerated by iterating over all 2^nGens
 * bitmasks (at most 2^6 = 64 iterations) and selecting those with exactly
 * nTheta set bits.  The loop bound is a compile-time constant; HLS can unroll
 * it completely.
 *
 * The factor 2^nTheta is NOT applied here — the caller can multiply if needed.
 * This avoids overflow for large nTheta and keeps the function general.
 */

#include "setup.h"

/* ======================================================================
   matDet
   ======================================================================
   @param M    [in]  Square matrix, row-major.  Only the nTheta×nTheta leading
                     sub-block is used; entries at indices ≥ nTheta are ignored.
                     Column dimension declared as nTheta because
                     C++ requires a compile-time constant second dimension.
   @param nTheta    [in]  Active matrix size, 1 ≤ nTheta ≤ nTheta.
   @return           det(M[0..nTheta-1][0..nTheta-1]).  Returns 0 for singular M.
   ====================================================================== */
alg_type matDet(const theta_type M[nTheta][nTheta])
{
    /* Working copy: initialise active block from M, padding to identity. */
    alg_type A[nTheta][nTheta];

    for (int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        for (int j = 0; j < nTheta; j++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            A[i][j] = (alg_type)M[i][j];

        }
    }

    alg_type sign = 1;

    /* ------------------------------------------------------------------ */
    /*  Elimination steps k = 0 … nTheta − 1                     */
    /* ------------------------------------------------------------------ */
    for (int k = 0; k < nTheta; k++)
    {
        #ifdef PRAGMAS
        #pragma HLS PIPELINE
        #endif

        /* --- Partial pivoting: find row with largest |A[i][k]|, i ≥ k -- */
        int      pivotRow = k;
        alg_type pivotAbs = (A[k][k] < 0) ? -A[k][k] : A[k][k];

        for (int i = k + 1; i < nTheta; i++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            alg_type absVal = (A[i][k] < 0) ? -A[i][k] : A[i][k];
            /*
             * Only swap when within the active block (i < nTheta).
             * Padding rows have A[i][k] = 0 for k < nTheta (see init above),
             * so absVal = 0 and this condition is never true for them.
             */
            if (absVal > pivotAbs)
            {
                pivotAbs = absVal;
                pivotRow = i;
            }
        }

        /* --- Row swap -------------------------------------------------- */
        if (pivotRow != k)
        {
            for (int j = 0; j < nTheta; j++)
            {
                #ifdef PRAGMAS
                #pragma HLS UNROLL
                #endif
                alg_type tmp  = A[k][j];
                A[k][j]       = A[pivotRow][j];
                A[pivotRow][j] = tmp;
            }
            sign = -sign;
        }

        /* --- Check for singularity ------------------------------------- */
        /*
         * If the pivot is zero the matrix is singular (or we are in the
         * padding region where A[k][k] = 1 by construction, k ≥ nTheta).
         * Return 0 only for active columns; padding pivots are always 1.
         */
        if (A[k][k] == (alg_type)0)
            return (alg_type)0;

        /* --- Eliminate rows below pivot -------------------------------- */
        for (int i = k + 1; i < nTheta; i++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            alg_type factor = A[i][k] / A[k][k];

            for (int j = k; j < nTheta; j++)
            {
                #ifdef PRAGMAS
                #pragma HLS UNROLL
                #endif
                A[i][j] -= factor * A[k][j];
            }
        }
    }

    /* ------------------------------------------------------------------ */
    /*  Determinant = sign × product of diagonal entries                  */
    /* ------------------------------------------------------------------ */
    alg_type det = sign;

    for (int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        /*
         * Only multiply active diagonal entries.  Padding entries are 1
         * (identity init, never modified), so skipping them is equivalent
         * to multiplying by 1 — but the explicit guard is cleaner.
         */
        det *= A[i][i];
    }
    return det;
}


/* ======================================================================
   zonotopeVolume
   ======================================================================
   Computes  Σ_{S ⊆ [nGens], |S|=nTheta}  |det(G_S)|
   i.e. the sum of absolute determinants of all nTheta×nTheta submatrices
   of G formed by choosing nTheta columns from the nGens available.
   Multiply the result by 2^nTheta to get the true zonotope volume.

   @param G      [in]  Generator matrix, nTheta rows × nGens columns.
                        Declared with nTheta rows and nGens
                        columns; only the [0..nTheta-1][0..nGens-1] block
                        is read.
   @param nTheta  [in]  Number of active rows    (= nTheta = na + nb).
   @param nGens  [in]  Number of active columns (= nGens).
   @return             Σ |det(G_S)|.  Returns 0 if nGens < nTheta.

   HLS note: the outer loop runs to 2^nGens = 64 (compile-time
   constant).  With PRAGMAS defined, #pragma HLS UNROLL will fully
   unroll it into 64 parallel submatrix-extraction + matDet chains.
   This is the maximum area / minimum latency configuration; if area
   is constrained, remove the unroll pragma and let HLS pipeline instead.
   ====================================================================== */
alg_type zonotopeVolume(const theta_type G[nTheta][nGens])
{
    alg_type vol = 0;

    /*
     * Iterate over all 2^nGens = 64 bitmasks.
     * A bitmask represents a subset of column indices: bit j is set iff
     * column j is included.  We select only masks with exactly nTheta set
     * bits, all within [0, nGens).
     *
     * popcount and the "valid" check are computed without break/continue
     * so the loop body is regular and HLS can pipeline it.
     */
    for (int mask = 0; mask < (1 << nGens); mask++)
    {
        #ifdef PRAGMAS
        // #pragma HLS UNROLL   /* enable for minimum latency; high area cost */
        #pragma HLS PIPELINE
        #endif

        /* Count set bits */
        int nbits = 0;

        for (int j = 0; j < nGens; j++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            if (mask & (1 << j)) nbits++;
        }

        /*
         * Only process valid masks (exactly nTheta bits, all in range).
         * Using a conditional accumulation rather than continue keeps
         * the loop body regular for HLS pipelining.
         */
        if (nbits == nTheta)
        {
            /* Extract the nTheta × nTheta submatrix corresponding to mask. */
            alg_type sub[nTheta][nTheta];

            /* Fill active columns in the order they appear in the mask. */
            int col = 0;
            for (int j = 0; j < nGens; j++)
            {
                #ifdef PRAGMAS
                #pragma HLS UNROLL
                #endif
                if (mask & (1 << j))
                {
                    for (int i = 0; i < nTheta; i++)
                        sub[i][col] = (alg_type)G[i][j];
                    col++;
                }
            }

            alg_type d = matDet(sub);
            vol += (d < (alg_type)0) ? -d : d;
        }
    }

    /* The multiplication by 2^nTheta is not included here
        and can be performed by the function caller. */
    return vol;
}