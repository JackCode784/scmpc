#include "setup.h"

// Uses MADS alg to compute optimal input sequence for ARX system
void MADSARX(input_type uOpt[nOpt], const input_type uInit[nb+nd-1], const output_type yInit[na], const output_type yref, const theta_type thetaScenarios[Nscen][nTheta])
{
	mesh_exp_type frameExp[nOpt];						// initial predefined frame size
	mesh_exp_type meshExp[nOpt];						// mesh size
	input_type pollMatrix[nOpt][2*nOpt];	// columns are poll points
	cost_type initialCost[2];	 			// cost function and constraints violation

	for (int i = 0; i < nOpt; i++)
		frameExp[i] = D[i];	// initialize frame exp

	// Compute the cost function and the constraints violation in the initial point
	costFunctionArx(initialCost, yInit, uOpt, uInit, yref, thetaScenarios);

	// MADS alg iterates K times
	for (int iter = 0; iter < MADS_ITER; iter++) {

#ifndef __SYNTHESIS__
		// float cost = initialCost[0].to_float();
#endif

		// update mesh size
		for (int i = 0; i < nOpt; i++) {
			if (frameExp[i] < -12) {
				break;
			} else if (frameExp[i] < 0) {
				meshExp[i] = frameExp[i] + frameExp[i];
			} else {
				meshExp[i] = frameExp[i];
			}
		}

		generatePollMatrixArx(uOpt, frameExp, meshExp, pollMatrix);

		progressiveBarrierPollingArx(initialCost, uOpt, yInit, uInit, yref, pollMatrix, frameExp, thetaScenarios);
	}

}
