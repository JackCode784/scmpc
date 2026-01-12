#include "setup.h"

void updateConstraintViolation(cost_type cost[2], output_type currY)
{
    #ifdef PRAGMAS
    //  #pragma HLS INLINE
    #endif
    
    if (currY > YMAX)
        cost[1] += currY - YMAX;

    if (currY < YMIN)
        cost[1] += YMIN - currY;
}
