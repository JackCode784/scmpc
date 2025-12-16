#include "setup.h"

void updateConstraintViolation(cost_type cost[2], const output_type currY[])
{
    if (currY[0] > YMAX[0])
        cost[1] += currY[0] - YMAX[0];

    if (currY[0] < YMIN[0])
        cost[1] += YMIN[0] - currY[0];
}
