#include "setup.h"

void updateConstraintViolation(cost_type cost[2], output_type currY)
{
    if (currY > YMAX)
        cost[1] += currY - YMAX;

    if (currY < YMIN)
        cost[1] += YMIN - currY;
}
