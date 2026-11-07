#include "setup.h"

// Generate matrix whose columns are poll points for the computation of cost function in MADS alg
void generatePollMatrixArx(const norm_input_type currU[nOpt], const mesh_exp_type frameExp[nOpt], const mesh_exp_type meshExp[nOpt], norm_input_type pollMatrix[nOpt][2 * nOpt])
{
	#ifdef PRAGMAS
	// #pragma HLS INLINE
	#endif

	#ifdef DEBUG_PRINT
	double frameExp_f[nOpt], meshExp_f[nOpt];
	double directions_f[nOpt][2*nOpt];
	double randVec_f[nOpt];
	double pollMatrix_f[nOpt][2*nOpt];
	double currU_f[nOpt];
	for(int i = 0; i < nOpt; i++) {
		frameExp_f[i] = frameExp[i].to_double(); 
		meshExp_f[i] = meshExp[i].to_double(); 
		currU_f[i]=currU[i].to_double(); }
	#endif

	// polling directions
	direction_type directions[nOpt][2 * nOpt];

	// vector for directions generation
	rand_type randomVector[nOpt];

	#ifdef PRAGMAS
	#pragma HLS ARRAY_PARTITION variable=directions   complete dim=0
	#pragma HLS ARRAY_PARTITION variable=randomVector complete dim=1
	#endif

	// generate random vector in [-1, 1]
	for(int i = 0; i < nOpt; i++)
	{
		#ifdef PRAGMAS
		/* pseudoRandArx() advances the single shared xorshift32 state
		 * (pseudoRand.cpp): draw i+1 depends on draw i, so this loop
		 * cannot be unrolled. PIPELINE II=1 still issues one draw/cycle. */
		#pragma HLS PIPELINE II=1
		#endif
		randomVector[i] = pseudoRandArx();
	}

	#ifdef DEBUG_PRINT
	for(int i = 0; i < nOpt; i++) randVec_f[i] = randomVector[i].to_double();
	#endif

	// generate poll directions starting with a random vector
	generatePollDirectionsArx(randomVector, frameExp, meshExp, directions);

	#ifdef DEBUG_PRINT
	for(int i = 0; i < nOpt; i++) for(int j = 0; j < 2*nOpt; j++) directions_f[i][j] = directions[i][j].to_double();
	#endif

	// fill the columns with polling points
	for (int i = 0; i < nOpt; i++)
	{
		#ifdef PRAGMAS
		/* Row i of pollMatrix only depends on row i of directions/currU:
		 * independent across i, so fully unrolling (nOpt rows) is cheap. */
		#pragma HLS UNROLL
		#endif

		#ifndef FIXED
		mesh_type mesh = 1;

		if (meshExp[i] < 0)
		{
			for (int j = 0; j < -meshExp[i]; j++)
				mesh /= 2;
		}
		else
		{
			for (int j = 0; j < meshExp[i]; j++)
				mesh *= 2;
		}
		
		for (int j = 0; j < 2 * nOpt; j++)
		{
			#ifdef PRAGMAS
			// #pragma HLS UNROLL
			#endif
			pollMatrix[i][j] = currU[i] + (mesh * directions[i][j]);
		}
		#else /* fixed point */
		/* Use shift operation */
		// for(int j = 0; j < 2 * nOpt; j++) pollMatrix[i][j] = currU[i] + (directions[i][j] << meshExp[i]);

		/* Use multiplication */
		/* BUG: expression directions[i][j] * (1 << meshExp[i]) returns wrong
		value because 1 is an integer. */
		for(int j = 0; j < 2*nOpt; j++)
		{
			#ifdef PRAGMAS
			#pragma HLS UNROLL
			#endif
			pollMatrix[i][j] = currU[i] + (directions[i][j] << meshExp[i]);

			#ifdef DEBUG_PRINT
			double a = (directions[i][j] << meshExp[i]).to_double();
			pollMatrix_f[i][j] = pollMatrix[i][j].to_double();
			#endif
		}
		#endif

	}
}
