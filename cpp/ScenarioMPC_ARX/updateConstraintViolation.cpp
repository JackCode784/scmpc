#include "setup.h"

/** Update output constraint violation. */
void updateConstraintViolation(cost_type cost[2], const norm_output_type yCurr, const norm_output_type yPast)
{
    #ifdef PRAGMAS
    //  #pragma HLS INLINE
    #endif
    
    if (yCurr > YNORMMAX)
        cost[1] += (yCurr - YNORMMAX);

    if (yCurr < YNORMMIN)
        cost[1] += (YNORMMIN - yCurr);

    if(yCurr - yPast > DELTAYNORM)
        cost[1] += yCurr - yPast;

    if(yCurr - yPast < -DELTAYNORM)
        cost[1] += yPast - yCurr;
}
