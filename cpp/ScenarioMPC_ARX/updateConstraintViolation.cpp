#include "setup.h"

/** Update output constraint violation. */
void updateConstraintViolation(cost_type cost[2], const norm_output_type yCurr)
{
    #ifdef PRAGMAS
    //  #pragma HLS INLINE
    #endif
    
    if (yCurr > YNORMMAX)
        cost[1] += (cost_type)(yCurr - YNORMMAX);

    if (yCurr < YNORMMIN)
        cost[1] += (cost_type)(YNORMMIN - yCurr);
}
