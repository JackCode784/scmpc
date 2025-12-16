#include "setup.h"

void updateConstraintViolation(cost_type cost[2], const output_type currY[])
{
    if (currY[0] > ymax[0])
        cost[1] += currY[0] - ymax[0];

    if (currY[0] < ymin[0])
        cost[1] += ymin[0] - currY[0];
}
