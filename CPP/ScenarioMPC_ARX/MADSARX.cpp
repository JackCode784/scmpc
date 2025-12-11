#include "setup.h"

// Uses MADS alg to compute optimal input sequence for ARX system
void MADSARX(input_type uOpt[nOpt], input_type uInit[nb+nd-1], output_type yInit[na], output_type yref[ny], theta_type thetaScenarios[Nscen][nTheta], hzn_type predictionHzn, hzn_type controlHzn)
{
	int frameIdx[nOpt];						// initial predefined frame size
	int meshIdx[nOpt];						// mesh size
	input_type pollMatrix[nOpt][2*nOpt];	// columns are poll points
	cost_type initialCost[2];	 			// cost function and constraints violation

	for (int i = 0; i < nOpt; i++)
		frameIdx[i] = D[i];	// initialize frame exp

	// Compute the cost function and the constraints violation in the initial point
	costFunctionArx(initialCost, yInit, uOpt, uInit, yref, predictionHzn, controlHzn, thetaScenarios);

	// MADS alg iterates K times
	for (int iter = 0; iter < MADS_ITER; iter++) {

		// update mesh size
		for (int i = 0; i < nOpt; i++) {
			if (frameIdx[i] < -12) {
				break;
			} else if (frameIdx[i] < 0) {
				meshIdx[i] = frameIdx[i] + frameIdx[i];
			} else {
				meshIdx[i] = frameIdx[i];
			}
		}

		generatePollMatrixArx(uOpt, frameIdx, meshIdx, pollMatrix);

		progressiveBarrierPollingArx(initialCost, uOpt, yInit, uInit, yref, predictionHzn, controlHzn, pollMatrix, frameIdx, thetaScenarios);
	}

}
