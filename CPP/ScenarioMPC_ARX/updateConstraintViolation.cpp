#include "setup.h"

void updateConstraintViolation(cost_type cost[2], output_type yCurr)
{
    #ifdef PRAGMAS
    //  #pragma HLS INLINE
    #endif
    
    if (yCurr > YMAX)
        cost[1] += yCurr - YMAX;

    if (yCurr < YMIN)
        cost[1] += YMIN - yCurr;
}
