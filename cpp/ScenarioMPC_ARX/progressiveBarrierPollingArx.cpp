#include "setup.h"

// This computes the next optimal point from the possible poll points in pollMatrix.
// currentCost is the cost for currentPoint
// currentPoint is updated when a new best point is found
void progressiveBarrierPollingArx(cost_type bestCost[2], 
								norm_input_type bestPoint[nOpt], 
								const norm_output_type yPast[na], 
								const norm_input_type uPast[nUKnown], 
								const norm_output_type yref, 
								const norm_input_type pollMatrix[nOpt][2 * nOpt], 
								mesh_exp_type frameExp[nOpt], 
								const arx_coeff_type arxCoeff[Nscen+1][nTheta],
								const arx_coeff_type arxOffset[Nscen+1],
								const bool startOnly)
{
	#ifdef PRAGMAS
	/*
	 * Inlined into MADSARX in every profile, so that the costFunctionArx
	 * call below - the design's only one: MADSARX's starting point goes
	 * through it too (startOnly) - is in MADSARX, where its ALLOCATION
	 * pragma sets how many costFunctionArx instances the design gets (see
	 * MADSARX.cpp). There is one call to this function per MADS iteration,
	 * so inlining it does not duplicate anything.
	 */
	#pragma HLS INLINE
	#endif

	#ifdef DEBUG_PRINT
	double bestPoint_f[nOpt];
	double testPoint_f[nOpt], costTestPoint_f[2], bestCost_f[2], originalCost_f;
	bool success_b;
	double pollMatrix_f[nOpt][2*nOpt];
	double frameExp_f[nOpt];
	double yref_f = yref.to_double();
	double yPast_f[na], uPast_f[nUKnown];
	double arxCoeff_f[Nscen+1][nTheta], arxOffset_f[Nscen+1];

	for(int i = 0; i < nOpt; i++) 
	{
		frameExp_f[i] = frameExp[i].to_double(); 
		for(int j = 0; j < 2*nOpt; j++) pollMatrix_f[i][j] = pollMatrix[i][j].to_double();
	}

	for(int i = 0; i < Nscen+1; i++) { arxOffset_f[i] = arxOffset[i].to_double(); for(int j = 0; j < nTheta; j++) arxCoeff_f[i][j] = arxCoeff[i][j].to_double(); }
	for(int i = 0; i < na; i++) yPast_f[i] = yPast[i].to_double();
	for(int i = 0; i < nUKnown; i++) uPast_f[i] = uPast[i].to_double();
	#endif

	// is there a better poll point than the current?
	#ifdef FIXED
	ap_uint<1> success = 0;
	#else
	bool success = 0;
	#endif

	/*
	 * Phase 1: cost and constraint violation of every poll point. The
	 * evaluations are independent of each other (each reads only its own
	 * pollMatrix column and the shared, read-only state), so their order
	 * does not matter; phase 2 then compares them in the original order, so
	 * the selected point is the same as with the former evaluate-and-compare
	 * loop. startOnly (MADSARX's iteration -1): column 0 only, which holds
	 * the starting point, and its result becomes bestCost - nothing else.
	 */
	cost_type pollCost[2 * nOpt];
	cost_type pollViol[2 * nOpt];
	#ifdef PRAGMAS
	#pragma HLS ARRAY_PARTITION variable=pollCost complete dim=1
	#pragma HLS ARRAY_PARTITION variable=pollViol complete dim=1
	#endif
	for (int i = 0; i < 2 * nOpt; i++)
	{
		#ifdef PRAGMAS
		#ifdef PRAGMA_PARALLEL_POLLS
		/*
		 * LATENCY: unrolled, the 2*nOpt evaluations run at the same time on
		 * 2*nOpt costFunctionArx instances (MADSARX's ALLOCATION allows that
		 * many), so a MADS iteration costs one evaluation's time.
		 */
		#pragma HLS UNROLL
		#else
		/*
		 * AREA, BALANCED: one evaluation after the other on a single
		 * costFunctionArx instance. PIPELINE off: its call is long and
		 * internally pipelined already, overlapping calls would only add
		 * control logic.
		 */
		#pragma HLS PIPELINE off
		#endif
		#endif

		if (startOnly && i > 0)
			continue;

		// point under test (a fresh local per evaluation, so unrolled
		// evaluations do not share it)
		norm_input_type testPoint[nOpt];
		cost_type costTestPoint[2];
		#ifdef PRAGMAS
		#pragma HLS ARRAY_PARTITION variable=testPoint dim=1 complete
		#pragma HLS ARRAY_PARTITION variable=costTestPoint dim=1 complete
		#endif
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
		costFunctionArx(costTestPoint, yPast, testPoint, uPast, yref, arxCoeff, arxOffset);
		pollCost[i] = costTestPoint[0];
		pollViol[i] = costTestPoint[1];
		#ifdef DEBUG_PRINT
		costTestPoint_f[0] = costTestPoint[0].to_double();
		costTestPoint_f[1] = costTestPoint[1].to_double();
		#endif
	}

	if (startOnly)
	{
		bestCost[0] = pollCost[0];
		bestCost[1] = pollViol[0];
		return;
	}

	const cost_type originalCost = bestCost[0];

	#ifdef DEBUG_PRINT
	originalCost_f = originalCost.to_double();
	bestCost_f[0] = bestCost[0].to_double();
	bestCost_f[1] = bestCost[1].to_double();
	#endif

	/* Phase 2: progressive-barrier selection, in poll order (unrolled in
	 * every profile: 2*nOpt comparisons, no arithmetic worth sharing) */
	for (int i = 0; i < 2 * nOpt; i++)
	{
		#ifdef PRAGMAS
		#pragma HLS UNROLL
		#endif

		// if the constraints are violated "more" in the test point than
		// the current point, skip to the next test point
		if (bestCost[1] == 0 && pollViol[i] == 0 && pollCost[i] < bestCost[0])
		{
			success = 1;
			for (int j = 0; j < nOpt; j++)
			{
				#ifdef PRAGMAS
				#pragma HLS UNROLL
				#endif
				bestPoint[j] = pollMatrix[j][i];

				#ifdef DEBUG_PRINT
				bestPoint_f[j] = bestPoint[j].to_double();
				#endif
			}
			bestCost[0] = pollCost[i];
			// bestCost[1] = pollViol[i];	// possibly useless, bestCost[1] == pollViol[i] == 0 already

			#ifdef DEBUG_PRINT
			bestCost_f[0] = bestCost[0].to_double();
			bestCost_f[1] = bestCost[1].to_double(); // possibly useless, see above
			#endif
		}
		else if ((bestCost[1] > 0 && pollViol[i] < bestCost[1]) || (pollViol[i] == bestCost[1] && pollCost[i] < bestCost[0]))
		{
			success = 1;
			for (int j = 0; j < nOpt; j++)
			{
				#ifdef PRAGMAS
				#pragma HLS UNROLL
				#endif
				bestPoint[j] = pollMatrix[j][i];

				#ifdef DEBUG_PRINT
				bestPoint_f[j] = bestPoint[j].to_double();
				#endif
			}
			bestCost[0] = pollCost[i];
			bestCost[1] = pollViol[i];

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
