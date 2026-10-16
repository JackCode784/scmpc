#include "setup.h"

/**
 * Add to acc the output constraint violation of one predicted sample: how
 * far yCurr is outside [YNORMMIN, YNORMMAX], plus how far the step
 * yCurr - yPast exceeds +-DELTAYNORM (nothing when both constraints hold).
 *
 * FIXED: the two terms are summed first and added to acc ONCE, so that
 * costFunctionArx's pipelined model loop has a single addition in its
 * loop-carried accumulator instead of two chained ones (a shorter
 * critical path for II=1). The result is the same number as two separate
 * additions: both terms are differences of norm_output_type values, so
 * they and their sum are exact in cost_type (which has more fractional
 * bits), and adding non-negative terms into a saturating accumulator
 * gives min(sum, max) whatever the grouping. Floating point keeps the two
 * additions (there, regrouping could round differently).
 */
#ifdef FIXED
/**
 * The violation term addConstraintViolation adds: >= 0, and exact in
 * cost_type (both parts are differences of norm_output_type values, which
 * has fewer fractional bits). costFunctionArx's model loop adds it to its
 * own exact accumulator (ViolAcc there) instead of a cost_type one.
 */
cost_type constraintViolation(const norm_output_type yCurr, const norm_output_type yPast)
{
    #ifdef PRAGMAS
    #pragma HLS INLINE
    #endif
    cost_type viol = 0;
    if (yCurr > YNORMMAX)
        viol += (yCurr - YNORMMAX);
    else if (yCurr < YNORMMIN)
        viol += (YNORMMIN - yCurr);

    if(yCurr - yPast > DELTAYNORM)
        viol += yCurr - yPast - DELTAYNORM;
    else if(yCurr - yPast < -DELTAYNORM)
        viol += yPast - yCurr - DELTAYNORM;
    return viol;
}
#endif

void addConstraintViolation(cost_type &acc, const norm_output_type yCurr, const norm_output_type yPast)
{
    #ifdef PRAGMAS
    /* Small compare-and-subtract leaf called from inside costFunctionArx's
     * model loop: inlining merges it into that loop's datapath (and its
     * pipeline, in the profiles that pipeline that loop) instead of adding
     * a call boundary there. Same in every profile. */
    #pragma HLS INLINE
    #endif

    #ifdef FIXED
    acc += constraintViolation(yCurr, yPast);
    #else
    if (yCurr > YNORMMAX)
        acc += (yCurr - YNORMMAX);
    else if (yCurr < YNORMMIN)
        acc += (YNORMMIN - yCurr);

    if(yCurr - yPast > DELTAYNORM)
        acc += yCurr - yPast - DELTAYNORM;
    else if(yCurr - yPast < -DELTAYNORM)
        acc += yPast - yCurr - DELTAYNORM;
    #endif
}
