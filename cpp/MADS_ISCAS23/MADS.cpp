#include "myLib.h"

void MADS(data_u optimum[N], data_x x[NX], data_x par[NP], data_x ref[NREF]) {

	// initial predefined frame size
	data_meshExp frameIdx[N];

	// mesh size
	data_meshExp meshIdx[N];

	// columns are poll points
	data_u pollMatrix[N][2*N];

	// cost function and constraints violation
	data_cost initialCost[2];

	for (int i = 0; i < N; i++) {
		frameIdx[i] = D[i];
	}

	// compute the cost function and the constraints violation in the initial point
	costFunction(initialCost, optimum, x, par, ref);

	// MADS algo iterates K times
	for (int i = 0; i < K; i++) {

		// update mesh size
		for (int i = 0; i < N; i++) {
			if (frameIdx[i] < -12) {
				break;
			} else if (frameIdx[i] < 0) {
				meshIdx[i] = frameIdx[i] + frameIdx[i];
			} else {
				meshIdx[i] = frameIdx[i];
			}
		}

		generatePollMatrix(optimum, frameIdx, meshIdx, pollMatrix);

		progressiveBarrierPolling(initialCost, optimum, pollMatrix, frameIdx, x, par, ref);
	}

}
