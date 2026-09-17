#include "setup.h"

/** Update output constraint violation. */
void updateConstraintViolation(cost_type cost[2], const norm_output_type yCurr, const norm_output_type yPast)
{
    #ifdef PRAGMAS
    /* Tiny compare-and-accumulate leaf, called up to (Nscen+1)*Nhor times
     * per costFunctionArx invocation from inside a pipelined loop:
     * inlining lets it merge into that loop's datapath instead of adding
     * a call/return boundary at every one of those call sites. */
    #pragma HLS INLINE
    #endif
    
    if (yCurr > YNORMMAX)
        cost[1] += (yCurr - YNORMMAX);
    else if (yCurr < YNORMMIN)
        cost[1] += (YNORMMIN - yCurr);

    if(yCurr - yPast > DELTAYNORM)
        cost[1] += yCurr - yPast;
    else if(yCurr - yPast < -DELTAYNORM)
        cost[1] += yPast - yCurr;
}
