#include "setup.h"

// Uses MADS alg to compute optimal input sequence for ARX system
void MADSARX(norm_input_type uOpt[nOpt], const norm_input_type uInit[nb + nk - 2], const norm_output_type yInit[na], const norm_output_type yref, const theta_type thetaScenarios[Nscen][nTheta])
{
	#ifdef DEBUG_PRINT
	double frameExp_f[nOpt], meshExp_f[nOpt], cost_f[2];
	double pollMatrix_f[nOpt][2*nOpt];
	double uOpt_f[nOpt];
	#endif

	mesh_exp_type frameExp[nOpt];		   // initial predefined frame size
	mesh_exp_type meshExp[nOpt];		   // mesh size
	norm_input_type pollMatrix[nOpt][2 * nOpt]; // columns are poll points
	cost_type cost[2];					   // cost function and constraints violation

	#ifdef PRAGMAS
	/* Read/written elementwise by unrolled logic in generatePollMatrixArx,
	 * generatePollDirectionsArx and progressiveBarrierPollingArx (all
	 * called with these exact arrays, no local copy in between): declared
	 * here, at their point of storage. */
	#pragma HLS ARRAY_PARTITION variable=frameExp   complete dim=1
	#pragma HLS ARRAY_PARTITION variable=meshExp    complete dim=1
	#pragma HLS ARRAY_PARTITION variable=pollMatrix complete dim=0
	#pragma HLS ARRAY_PARTITION variable=cost       complete dim=1
	#endif

	for (int i = 0; i < nOpt; i++)
		frameExp[i] = D0_VAL; // initialize frame exp

	#ifdef DEBUG_PRINT
	for(int i = 0; i < nOpt; i++) frameExp_f[i] = frameExp[i].to_double();
	for(int i = 0; i < nOpt; i++) uOpt_f[i] = uOpt[i].to_double();
	#endif

	// Compute the cost function and the constraints violation in the initial point
	costFunctionArx(cost, yInit, uOpt, uInit, yref, thetaScenarios);

	#ifdef DEBUG_PRINT
	cost_f[0] = cost[0].to_double();
	cost_f[1] = cost[1].to_double();
	#endif

	// MADS alg iterates K times
	#ifdef PRAGMAS
	/*
	 * Deliberately no UNROLL/PIPELINE here: this is the direct-search
	 * algorithm's own outer loop. Iteration iter+1's poll matrix depends
	 * on frameExp as updated by progressiveBarrierPollingArx at the END
	 * of iteration iter, so the iterations are a true sequential
	 * recurrence, not a candidate for either directive - MADS_ITER=7
	 * physical copies of the whole poll-and-evaluate datapath would be
	 * both wrong (each needs the previous one's result) and far too
	 * large to fit. PIPELINE off documents that this is a deliberate
	 * choice, not an oversight, matching the convention already used in
	 * cpp/MADS_ISCAS23/admm.cpp for the same kind of loop.
	 */
	#pragma HLS PIPELINE off
	#endif
	for (int iter = 0; iter < MADS_ITER; iter++)
	{
		// update mesh size
		for (int i = 0; i < nOpt; i++)
		{
			/*
			 * NOTE: this "break" makes the trip count of this inner loop
			 * data-dependent (it stops updating meshExp for all i once
			 * ANY dimension's frameExp[i] drops below FRAME_EXP_MIN).
			 * Every other loop in this codebase over an nOpt/nTheta-sized
			 * range deliberately avoids break/continue for exactly this
			 * reason (see the comments in volumeZonotope.cpp) so that it
			 * stays eligible for #pragma HLS UNROLL. This one cannot be
			 * unrolled or pipelined as written - Vitis HLS needs a
			 * statically-bounded trip count for both. If the intent is
			 * really "stop updating every dimension once the first one
			 * bottoms out", that is worth double-checking against the
			 * MADS reference algorithm: mesh/frame sizes are normally
			 * per-dimension quantities updated independently, so a more
			 * HLS-friendly (and UNROLL-able) rewrite would replace the
			 * break with a per-dimension guard, e.g.
			 *   if (frameExp[i] >= FRAME_EXP_MIN) meshExp[i] = ...;
			 * which also happens to be closer to the reference behaviour.
			 */
			if (frameExp[i] < FRAME_EXP_MIN)
			{
				break;
			}
			// Mesh update
			meshExp[i] = (frameExp[i] < 0) ? (frameExp[i] << 1) : frameExp[i];
			// else if (frameExp[i] < 0)
			// {
			// 	meshExp[i] = frameExp[i] + frameExp[i];
			// }
			// else
			// {
			// 	meshExp[i] = frameExp[i];
			// }

			#ifdef DEBUG_PRINT
			frameExp_f[i] = frameExp[i].to_double();
			meshExp_f[i] = meshExp[i].to_double();
			#endif
		}

		generatePollMatrixArx(uOpt, frameExp, meshExp, pollMatrix);

		#ifdef DEBUG_PRINT
		for(int i = 0; i < nOpt; i++) {
			for(int j = 0; j < 2*nOpt; j++) pollMatrix_f[i][j] = pollMatrix[i][j].to_double();
		}
		#endif

		/* BUG: success == 0 always, input never changes wrt original value [-1...,-1] <-> [0,...,0] */
		progressiveBarrierPollingArx(cost, uOpt, yInit, uInit, yref, pollMatrix, frameExp, thetaScenarios);

		#ifdef DEBUG_PRINT
		for(int i = 0; i < nOpt; i++) uOpt_f[i] = uOpt[i].to_double();
		cost_f[0] = cost[0].to_double();
		cost_f[1] = cost[1].to_double();
		#endif
	}
}
