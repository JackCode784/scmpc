#include "myLib.h"

void progressiveBarrierPolling(data_cost cost[2], data_u currentPoint[N], data_u pollMatrix[N][2*N], data_meshExp frameSize[N], data_x x[NX], data_x par[NP], data_x ref[NREF]) {
//#pragma HLS ALLOCATION instances=costFunction limit=6 function

	// flag indicating if one of the polling points is better than the current one
	static bool success = false;

	// point under test
	data_u testPoint[N];
	#pragma HLS ARRAY_PARTITION variable=testPoint dim=1 complete

	// cost function and constraints violation of the test point
	data_cost costTest[2];
	#pragma HLS ARRAY_PARTITION variable=costTest dim=1 complete

	for (int i = 0; i < 2*N; i++) {
		#pragma HLS PIPELINE
//		#pragma HLS UNROLL

		// extract one point from the polling matrix
		for (int j = 0; j < N; j++) {
			#pragma HLS UNROLL
			testPoint[j] = pollMatrix[j][i];
		}

		// compute the cost function and the constraints violation in a test point
		costFunction(costTest, testPoint, x, par, ref);

		// if the constraints are violated "more" in the test point than
		// the current point, skip to the next test point
		if (costTest[1] > cost[1]) {
			continue;
		}
		// if the cost function is smaller in the test point than the
		// current point, or if the constraints are violated "less" in
		// the test point than the current point, then the iteration
		// is successful
		else if (costTest[0] < cost[0]) {

			// this test point becomes the new optimum
			for (int j = 0; j < N; j++) {
				#pragma HLS UNROLL
				currentPoint[j] = testPoint[j];
			}
			cost[0] = costTest[0];
			cost[1] = costTest[1];
			success = true;
			//break;
		}
	}

	// update the frame size
	if (success == true) {
		for (int i = 0; i < N; i++) {
			#pragma HLS UNROLL
			frameSize[i] = frameSize[i] + TAU;
		}
		success = false;
	} else {
		for (int i = 0; i < N; i++) {
			#pragma HLS UNROLL
			frameSize[i] = frameSize[i] - TAU;
		}
	}
}
