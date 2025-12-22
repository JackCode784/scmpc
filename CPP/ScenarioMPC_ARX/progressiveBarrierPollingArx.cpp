#include "setup.h"

// This computes the next optimal point from the possible poll points in pollMatrix.
// currentCost is the cost for currentPoint
// currentPoint is updated when a new best point is found
void progressiveBarrierPollingArx(cost_type bestCost[2], input_type bestPoint[nOpt], const output_type yPast[na], const input_type uPast[nb + nd - 1], const output_type yref, const input_type pollMatrix[nOpt][2 * nOpt], mesh_exp_type frameExp[nOpt], const theta_type thetaScenarios[Nscen + 1][nTheta])
{
	// #pragma HLS ALLOCATION instances=costFunction limit=6 function

	// flag indicating if one of the polling points is better than the current one
	bool success = false;

	// point under test
	input_type testPoint[nOpt];
	// #pragma HLS ARRAY_PARTITION variable = testPoint dim = 1 complete

	// cost function and constraints violation of the test point
	// #pragma HLS ARRAY_PARTITION variable = costTest dim = 1 complete
	cost_type costTestPoint[2];
	cost_type originalCost = bestCost[0];

	for (int i = 0; i < 2 * nOpt; i++)
	{
		// #pragma HLS PIPELINE
		//		#pragma HLS UNROLL

		// extract one point from the polling matrix
		for (int j = 0; j < nOpt; j++)
		{
			// #pragma HLS UNROLL
			testPoint[j] = pollMatrix[j][i];
		}

		// compute the cost function and the constraints violation in a test point
		costFunctionArx(costTestPoint, yPast, testPoint, uPast, yref, thetaScenarios);

		// if the constraints are violated "more" in the test point than
		// the current point, skip to the next test point
		if (bestCost[1] == 0 && costTestPoint[1] == 0 && costTestPoint[0] < bestCost[0])
		{
			success = true;
			for (int i = 0; i < nOpt; i++)
				bestPoint[i] = testPoint[i];
			bestCost[0] = costTestPoint[0];
			bestCost[1] = costTestPoint[1];	// possibly useless, bestCost[1] == costTestPoint[1] == 0 already
		}
		else if (bestCost[1] > 0 && costTestPoint[1] < bestCost[1] || (costTestPoint[1] == bestCost[1] && costTestPoint[0] < bestCost[0]))
		{
			success = true;
			for (int i = 0; i < nOpt; i++)
				bestPoint[i] = testPoint[i];
			bestCost[0] = costTestPoint[0];
			bestCost[1] = costTestPoint[1];
		}
	}

	// update the frame size
	if (success == true)
	{
		if (bestCost[0] < originalCost)
		{
			for (int i = 0; i < nOpt; i++)
			{
				// #pragma HLS UNROLL
				frameExp[i] = frameExp[i] + TAU;
			}
		}
	}
	else
	{
		for (int i = 0; i < nOpt; i++)
		{
			// #pragma HLS UNROLL
			frameExp[i] = frameExp[i] - TAU;
		}
	}
}
