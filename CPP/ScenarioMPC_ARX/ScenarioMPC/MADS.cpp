#include "controller.h"

void MADS(fxd optimum[nDim_CTRL], fxd x[nX], fxd ref[nRef]) {

	// initial predefined frame size
	data_meshExp frameIdx[nDim_CTRL];

	// mesh size
	data_meshExp meshIdx[nDim_CTRL];

	// columns are poll points
	fxd pollMatrix[nDim_CTRL][2*nDim_CTRL];

	// cost function and constraints violation
	data_cost initialCost[2];

	for (int i = 0; i < nDim_CTRL; i++) {
		frameIdx[i] = D[i];
	}

	// compute the cost function and the constraints violation in the initial point
	costFunction(initialCost, optimum, x, ref);

	// MADS algo iterates K times
	for (int i = 0; i < K; i++) {

		// update mesh size
		for (int i = 0; i < nDim_CTRL; i++) {
			if (frameIdx[i] < -12) {
				break;
			} else if (frameIdx[i] < 0) {
				meshIdx[i] = frameIdx[i] + frameIdx[i];
			} else {
				meshIdx[i] = frameIdx[i];
			}
		}

		generatePollMatrix(optimum, frameIdx, meshIdx, pollMatrix);

		progressiveBarrierPolling(initialCost, optimum, pollMatrix, frameIdx, x, ref);
	}

}
