#include "setup.h"

// This computes the next optimal point from the possible poll points in pollMatrix.
// currentCost is the cost for currentPoint
// currentPoint is updated when a new best point is found
void progressiveBarrierPollingArx(cost_type bestCost[2], 
								norm_input_type bestPoint[nOpt], 
								const norm_output_type yPast[na], 
								const norm_input_type uPast[nb + nk - 2], 
								const norm_output_type yref, 
								const norm_input_type pollMatrix[nOpt][2 * nOpt], 
								mesh_exp_type frameExp[nOpt], 
								const theta_type thetaScenarios[Nscen][nTheta])
{
	#ifdef PRAGMAS
	/*
	 * Left OFF deliberately: costFunctionArx is already called from a
	 * plain (non-unrolled) loop below, so HLS already builds exactly one
	 * instance of it and reuses it sequentially across all 2*nOpt calls -
	 * ALLOCATION limit=... only matters when unrolling/pipelining would
	 * otherwise duplicate the callee, which is not the case here.
	 */
	// #pragma HLS ALLOCATION instances=costFunctionArx limit=6 function
	#endif

	#ifdef DEBUG_PRINT
	double bestPoint_f[nOpt];
	double testPoint_f[nOpt], costTestPoint_f[2], bestCost_f[2], originalCost_f;
	bool success_b;
	double pollMatrix_f[nOpt][2*nOpt];
	double frameExp_f[nOpt];
	double yref_f = yref.to_double();
	double yPast_f[na], uPast_f[nb+nk-2];
	double thetaScenarios_f[Nscen][nTheta];

	for(int i = 0; i < nOpt; i++) 
	{
		frameExp_f[i] = frameExp[i].to_double(); 
		for(int j = 0; j < 2*nOpt; j++) pollMatrix_f[i][j] = pollMatrix[i][j].to_double();
	}

	for(int i = 0; i < Nscen; i++) for(int j = 0; j < nTheta; j++) thetaScenarios_f[i][j] = thetaScenarios[i][j].to_double();
	for(int i = 0; i < na; i++) yPast_f[i] = yPast[i].to_double();
	for(int i = 0; i < nb+nk-2; i++) uPast_f[i] = uPast[i].to_double();
	#endif

	// is there a better poll point than the current?
	#ifdef FIXED
	ap_uint<1> success = 0;
	#else
	bool success = 0;
	#endif

	// point under test
	norm_input_type testPoint[nOpt];
	#ifdef PRAGMAS
	#pragma HLS ARRAY_PARTITION variable=testPoint dim=1 complete
	#endif

	// cost function and constraints violation of the test point
	cost_type costTestPoint[2];
	const cost_type originalCost = bestCost[0];

	#ifdef PRAGMAS
	#pragma HLS ARRAY_PARTITION variable=costTestPoint dim=1 complete
	#endif

	#ifdef DEBUG_PRINT
	originalCost_f = originalCost.to_double();
	bestCost_f[0] = bestCost[0].to_double();
	bestCost_f[1] = bestCost[1].to_double();
	#endif

	for (int i = 0; i < 2 * nOpt; i++)
	{
		#ifdef PRAGMAS
		/*
		 * Deliberately NOT unrolled or pipelined. Unrolling would
		 * instantiate 2*nOpt physical copies of costFunctionArx's whole
		 * datapath (itself already Nscen-way parallel inside, see
		 * costFunctionArx.cpp) - far too much for the XC7Z020's 220
		 * DSP48E1 slices. This loop also carries a genuine recurrence
		 * through bestCost/bestPoint (each candidate is compared against
		 * the best found so far), so poll point i+1 cannot be evaluated
		 * against a stale "best". Once the design is synthesizing
		 * cleanly, PIPELINE can be tried here as a later optimisation -
		 * it only needs costFunctionArx to accept a new call before the
		 * previous one fully drains - but check the achieved II and DSP
		 * utilisation in the synthesis report before keeping it, since
		 * the callee's own pipeline depth sets a floor on how much
		 * overlap is actually achievable.
		 */
		#endif

		// extract one point from the polling matrix
		for (int j = 0; j < nOpt; j++)
		{
			#ifdef PRAGMAS
			#pragma HLS UNROLL
			#endif
			testPoint[j] = pollMatrix[j][i];

			#ifdef DEBUG_PRINT
			testPoint_f[j] = testPoint[j].to_double();
			#endif
		}

		// compute the cost function and the constraints violation in a test point
		costFunctionArx(costTestPoint, yPast, testPoint, uPast, yref, thetaScenarios);
		#ifdef DEBUG_PRINT
		costTestPoint_f[0] = costTestPoint[0].to_double();
		costTestPoint_f[1] = costTestPoint[1].to_double();
		#endif

		// if the constraints are violated "more" in the test point than
		// the current point, skip to the next test point
		if (bestCost[1] == 0 && costTestPoint[1] == 0 && costTestPoint[0] < bestCost[0])
		{
			success = 1;
			for (int i = 0; i < nOpt; i++)
			{
				#ifdef PRAGMAS
				#pragma HLS UNROLL
				#endif
				bestPoint[i] = testPoint[i];
				
				#ifdef DEBUG_PRINT
				bestPoint_f[i] = bestPoint[i].to_double();
				#endif
			}
			bestCost[0] = costTestPoint[0];
			// bestCost[1] = costTestPoint[1];	// possibly useless, bestCost[1] == costTestPoint[1] == 0 already

			#ifdef DEBUG_PRINT
			bestCost_f[0] = bestCost[0].to_double();
			bestCost_f[1] = bestCost[1].to_double(); // possibly useless, see above
			#endif
		}
		else if ((bestCost[1] > 0 && costTestPoint[1] < bestCost[1]) || (costTestPoint[1] == bestCost[1] && costTestPoint[0] < bestCost[0]))
		{
			success = 1;
			for (int i = 0; i < nOpt; i++)
			{
				#ifdef PRAGMAS
				#pragma HLS UNROLL
				#endif
				bestPoint[i] = testPoint[i];

				#ifdef DEBUG_PRINT
				bestPoint_f[i] = bestPoint[i].to_double();
				#endif
			}
			bestCost[0] = costTestPoint[0];
			bestCost[1] = costTestPoint[1];

			#ifdef DEBUG_PRINT
			bestCost_f[0] = bestCost[0].to_double();
			bestCost_f[1] = bestCost[1].to_double();
			#endif
		}
	}

	// update the frame size
	if (success == 1)
	{
		if (bestCost[0] < originalCost)
		{
			for (int i = 0; i < nOpt; i++)
			{
				#ifdef PRAGMAS
				#pragma HLS UNROLL
				#endif
				frameExp[i] = frameExp[i] + TAU;

				#ifdef DEBUG_PRINT
				frameExp_f[i] = frameExp[i].to_double();
				#endif
			}
		}
	}
	else
	{
		for (int i = 0; i < nOpt; i++)
		{
			#ifdef PRAGMAS
			#pragma HLS UNROLL
			#endif
			frameExp[i] = frameExp[i] - TAU;
			
			#ifdef DEBUG_PRINT
			frameExp_f[i] = frameExp[i].to_double();
			#endif
		}
	}

	#ifdef DEBUG_PRINT
	success_b = success.to_bool();
	#endif
	return;
}
