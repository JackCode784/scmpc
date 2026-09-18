/**
 * @file  volumeZonotope.cpp
 * @brief Determinant of a square matrix and zonotope volume computation.
 *
 * CONTENTS
 * --------
 *  matDet()         - determinant of an nTheta*nTheta matrix, nTheta <= nTheta
 *  zonotopeVolume() - 2^nTheta * sum_{S} |det(G_S)|, summed over all nTheta-column
 *                     subsets S of the generator matrix G \in R^{nTheta*m}
 *
 * ALGORITHM: GAUSSIAN ELIMINATION WITH PARTIAL PIVOTING
 * -------------------------------------------------------
 * Starting from the working copy A (initialised from M), 
 * at each step k:
 *
 *   1. Find the row p >= k with the largest |A[p][k]| among active rows
 *      (partial pivoting - improves numerical stability and avoids
 *      division by zero for non-singular matrices).
 *   2. Swap rows p and k; flip the sign of the determinant.
 *   3. For each row i > k: subtract (A[i][k]/A[k][k]) * row k from row i,
 *      zeroing out A[i][k].
 *
 * After nTheta steps A is upper-triangular.  The determinant is the product of
 * the diagonal entries multiplied by the accumulated sign.
 *
 * Complexity: O(n^3) arithmetic operations.  For nTheta = nTheta = 6
 * this is at most 216 multiply-accumulates - trivial for any FPGA.
 *
 * HLS DESIGN CHOICES
 * -------------------
 * * All loops run to the compile-time constant nTheta so that
 *   #pragma HLS UNROLL can fully unroll them.  Variable-bound loops
 *   cannot be fully unrolled by the HLS scheduler.
 *
 * * Partial pivoting uses conditional assignment (the ternary operator)
 *   rather than an if/break, keeping the loop body regular and pipeline-
 *   friendly.
 *
 * * The permutation sign is stored as det_type (+1 or -1) to avoid a
 *   type conversion at the final multiply.
 *
 * * No dynamic memory allocation, no recursion, no variable-length arrays.
 *
 * ZONOTOPE VOLUME
 * ----------------
 * The volume of the zonotope Z = { c + G*ξ : ||ξ||inf <= 1 } \in R^nTheta with
 * G \in R^{nTheta*m}  (nTheta = nTheta, m = nGens) is:
 *
 *   Vol(Z) = 2^nTheta * sum_{S \in [m], |S|=nTheta}  |det(G_S)|
 *
 * where G_S is the nTheta*nTheta submatrix formed by selecting the columns in S.
 * The sum has C(m, nTheta) terms; for m = nTheta = 6 this is just |det(G)| itself.
 *
 * Column subsets are enumerated by iterating over all 2^nGens
 * bitmasks (at most 2^6 = 64 iterations) and selecting those with exactly
 * nTheta set bits.  The loop bound is a compile-time constant; HLS can unroll
 * it completely.
 *
 * The factor 2^nTheta is NOT applied here - the caller can multiply if needed.
 * This avoids overflow for large nTheta and keeps the function general.
 */

#include "setup.h"

/* ======================================================================
   matDet
   ======================================================================
   @param M    [in]  Square matrix, row-major.  Only the nTheta*nTheta leading
                     sub-block is used; entries at indices >= nTheta are ignored.
                     Column dimension declared as nTheta because
                     C++ requires a compile-time constant second dimension.
   @return           det(M[0..nTheta-1][0..nTheta-1]).  Returns 0 for singular M.
   ====================================================================== */
det_type matDet(const theta_type M[nTheta][nTheta])
{
    /* Working copy */
    elim_type A[nTheta][nTheta];

    #ifdef DEBUG_PRINT
    double A_f[nTheta][nTheta], pivotAbs_f, absVal_f;
    #endif

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
            A[i][j] = M[i][j];

            #ifdef DEBUG_PRINT
            A_f[i][j] = A[i][j].to_double();
            #endif
        }
    }

    det_type sign = 1;

    /*
     * Sticky "singular" flag, replacing an early `return 0` that used to
     * sit inside the k-loop below. A `return` (or break/continue) inside
     * a loop makes the function's completion time depend on runtime data
     * - the loop can finish in fewer iterations than its static trip
     * count - which is exactly the kind of construct that made Vitis HLS
     * report matDet's (and, once compounded through zonotopeVolume and
     * boundStripZonotopeIntersectionNew, controller()'s) latency as
     * unbounded ("?") once this loop was no longer forced into a fixed
     * schedule by #pragma HLS PIPELINE (see that pragma's own comment
     * below). Tracking "was any pivot exactly zero" in a flag instead,
     * and gating the final result on it, keeps every loop's trip count a
     * compile-time constant - same principle already used deliberately
     * elsewhere in this file (see zonotopeVolume's docstring on avoiding
     * break/continue) and in MADSARX.cpp's mesh-update loop.
     */
    bool singular = false;

    /* ------------------------------------------------------------------ */
    /*  Elimination steps k = 0 ... nTheta - 1                     */
    /* ------------------------------------------------------------------ */
    for (int k = 0; k < nTheta; k++)
    {
        #ifdef PRAGMAS
        /*
         * Deliberately NOT pipelined (reverted from a bare `PIPELINE`,
         * which defaults to a target II=1). This is a genuine Gaussian-
         * elimination recurrence - row k+1's pivot search and elimination
         * read A as left by step k - AND its elimination step below
         * (`factor = A[i][k] / A[k][k]`) is an ap_fixed DIVISION, unrolled
         * across up to nTheta-1 rows. Division is not a single-cycle
         * operation the way multiply/add are; Vitis HLS 2021.1 reported
         * an II Violation here (target 1, unreachable) once this function
         * was actually exercised from CTRL_MODE_PL, because neither the
         * recurrence nor the divider's own latency can be squeezed into
         * 1 cycle. Leaving the loop unpipelined lets HLS schedule the
         * division and elimination across as many cycles as they
         * genuinely need; there is no throughput requirement to justify
         * forcing otherwise, since matDet is called at most once per
         * zonotopeVolume() call for any nTheta==nGens system (see below)
         * and zonotopeVolume itself runs only a handful of times per
         * controller() call in PL/AL mode, not in a tight repeated loop.
         */
        #endif

        /* --- Partial pivoting: find row with largest |A[i][k]|, i >= k -- */
        int      pivotRow = k;
        elim_type pivotAbs = (A[k][k] < 0) ? elim_type(-A[k][k]) : A[k][k];
        #ifdef DEBUG_PRINT
        pivotAbs_f = pivotAbs.to_double();
        #endif

        for (int i = k + 1; i < nTheta; i++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            elim_type absVal = (A[i][k] < 0) ? elim_type(-A[i][k]) : A[i][k];
            #ifdef DEBUG_PRINT
            absVal_f = absVal.to_double();
            #endif
            if (absVal > pivotAbs)
            {
                pivotAbs = absVal;
                pivotRow = i;
                #ifdef DEBUG_PRINT
                pivotAbs_f = pivotAbs.to_double();
                #endif
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
                elim_type tmp  = A[k][j];
                A[k][j]       = A[pivotRow][j];
                A[pivotRow][j] = tmp;

                #ifdef DEBUG_PRINT
                A_f[k][j] = A[k][j].to_double();
                A_f[pivotRow][j] = A[pivotRow][j].to_double();
                #endif
            }
            sign = -sign;
        }

        /* --- Check for singularity ------------------------------------- */
        /*
         * If the pivot is zero the matrix is singular: the determinant
         * is 0 regardless of what any later elimination step computes.
         * `singular` is never cleared once set (a plain assignment, not
         * an OR-into-itself, but since it only ever gets set to true this
         * has the same sticky effect), matching the original code's
         * early return firing on the FIRST zero pivot encountered.
         */
        if (A[k][k] == 0)
            singular = true;

        /* --- Eliminate rows below pivot -------------------------------- */
        // Alternative
        // elim_type partialFactor = 1 / A[k][k];
        for (int i = k + 1; i < nTheta; i++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            /*
             * Note this ternary does not stop the divider hardware from
             * being fed a zero divisor when singular - Vitis HLS
             * synthesizes A[i][k]/A[k][k] combinationally regardless (a
             * mux selects between its result and 0 afterwards, it does
             * not gate the division itself), and a fixed-point divider
             * given a zero divisor produces some implementation-defined
             * saturated value, not a runtime fault - hardware always
             * produces *some* output for *any* input. The guard exists
             * to stop that meaningless value from propagating into A: it
             * is not load-bearing for the RETURNED determinant, which is
             * already forced to 0 by `det`'s initial value below once
             * singular is set (0 times anything stays 0), but it keeps
             * A's contents well-defined for the DEBUG_PRINT dump.
             */
            theta_type factor = singular ? theta_type(0) : theta_type(A[i][k] / A[k][k]);

            #ifdef DEBUG_PRINT
            double factor_f = factor.to_double();
            #endif

            // Alternative
            // theta_type factor = partialFactor * A[i][k];
            // A[i][k] = 0;
            // and upcoming for starts from j=k+1.

            for (int j = k; j < nTheta; j++)
            {
                #ifdef PRAGMAS
                #pragma HLS UNROLL
                #endif
                A[i][j] -= factor * A[k][j];
                
                #ifdef DEBUG_PRINT
                A_f[i][j] = A[i][j].to_double();
                #endif
            }
        }
    }

    /* ------------------------------------------------------------------ */
    /*  Determinant = sign * product of diagonal entries                  */
    /*  (0 instead, unconditionally, if any pivot was exactly zero)       */
    /* ------------------------------------------------------------------ */
    det_type det = singular ? det_type(0) : det_type(sign);

    for (int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        det *= A[i][i];

        #ifdef DEBUG_PRINT
        double det_f = det.to_double();
        #endif
    }
    return det;
}


/* ======================================================================
   zonotopeVolume
   ======================================================================
   Computes  sum_{S \in [nGens], |S|=nTheta}  |det(G_S)|
   i.e. the sum of absolute determinants of all nTheta*nTheta submatrices
   of G formed by choosing nTheta columns from the nGens available.
   Multiply the result by 2^nTheta to get the true zonotope volume.

   @param G      [in]  Generator matrix, nTheta rows * nGens columns.
                        Declared with nTheta rows and nGens
                        columns; only the [0..nTheta-1][0..nGens-1] block
                        is read.
   @param nTheta  [in]  Number of active rows    (= nTheta = na + nb).
   @param nGens  [in]  Number of active columns (= nGens).
   @return             sum |det(G_S)|.  Returns 0 if nGens < nTheta.

   HLS note: the outer loop runs to 2^nGens = 64 (compile-time
   constant).  With PRAGMAS defined, #pragma HLS UNROLL will fully
   unroll it into 64 parallel submatrix-extraction + matDet chains.
   This is the maximum area / minimum latency configuration; if area
   is constrained, remove the unroll pragma and let HLS pipeline instead.
   ====================================================================== */
vol_type zonotopeVolume(const theta_type G[nTheta][nGens])
{
    vol_type vol = 0;

    #ifdef DEBUG_PRINT
    double vol_f, sub_f[nTheta][nTheta], d_f;
    #endif

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
        /*
         * Deliberately neither UNROLL nor PIPELINE.
         *   UNROLL would instantiate 2^nGens (up to 64) parallel copies
         *   of matDet - each one now an honestly-multi-cycle Gaussian
         *   elimination with real dividers (see matDet's own comment
         *   above) - which is a large amount of hardware for a function
         *   that, in PL/AL mode, runs a handful of times per controller()
         *   call, not in a throughput-critical loop.
         *   PIPELINE (bare, target II=1) is what actually caused the II
         *   Violation reported from CTRL_MODE_PL synthesis: most masks
         *   here are cheap bit-counting, but any mask with exactly nTheta
         *   set bits calls matDet, whose own latency (now multi-cycle by
         *   design, not II=1) cannot be hidden inside a 1-cycle
         *   initiation interval for this loop - the achieved II was
         *   necessarily bounded below by matDet's latency regardless of
         *   what was requested.
         * Left unpipelined, HLS reuses one matDet instance sequentially
         * across whichever masks actually qualify, which is the correct
         * area/latency tradeoff here given how rarely this function runs.
         */
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
            /* Extract the nTheta * nTheta submatrix corresponding to mask. */
            theta_type sub[nTheta][nTheta];

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
                        sub[i][col] = G[i][j];
                    col++;
                }
            }
            #ifdef DEBUG_PRINT
            for(int i = 0; i < nTheta; i++)
            {
                for(int j = 0; j < nTheta; j++) sub_f[i][j] = sub[i][j].to_double();
            }
            #endif

            det_type d = matDet(sub);
            vol += (d < 0) ? vol_type(-d) : vol_type(d);

            #ifdef DEBUG_PRINT
            d_f = d.to_double();
            vol_f = vol.to_double();
            #endif
        }
    }

    /* The multiplication by 2^nTheta is not included here
        and can be performed by the function caller. */
    return vol;
}