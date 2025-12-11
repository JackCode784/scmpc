#include "controller.h"
#include "hls_math.h"

void costFunction(data_cost cost[2], fxd point[nDim_CTRL], fxd x[nX], fxd ref[nRef]) {

	// cost function computed in the current point
	data_cost costFcn[2];

	integrateSystem(costFcn, A1, B1, G1, point, x, ref);

	cost[0] = costFcn[0];
	cost[1] = costFcn[1];





}
