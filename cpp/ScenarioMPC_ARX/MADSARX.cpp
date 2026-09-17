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
	for (int iter = 0; iter < MADS_ITER; iter++)
	{
		// update mesh size
		for (int i = 0; i < nOpt; i++)
		{
			#ifdef PRAGMAS
			#pragma HLS UNROLL
			#endif
			/* Old method: HLS unfriendly because of break */
			// if (frameExp[i] < FRAME_EXP_MIN)
			// {
			// 	break;
			// }
			// else if (frameExp[i] < 0)
			// {
			// 	meshExp[i] = frameExp[i] + frameExp[i];
			// }
			// else
			// {
			// 	meshExp[i] = frameExp[i];
			// }

			/* Alternative to break statement */
			if(frameExp[i] >= FRAME_EXP_MIN) meshExp[i] = (frameExp[i] < 0) ? (frameExp[i] << 1) : frameExp[i];

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
