#include "setup.h"

// Generate matrix whose columns are poll points for the computation of cost function in MADS alg
void generatePollMatrixArx(const norm_input_type currU[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], norm_input_type pollMatrix[nOpt][2 * nOpt])
{
	#ifdef PRAGMAS
	// #pragma HLS INLINE
	#endif

	// polling directions
	direction_type directions[nOpt][2 * nOpt];

	// vector for directions generation
	rand_type randomVector[nOpt];

	// generate random vector in [-1, 1]
	for(int i = 0; i < nOpt; i++)
		randomVector[i] = pseudoRandArx();

	// generate poll directions starting with a random vector
	generatePollDirectionsArx(randomVector, frameIdx, meshIdx, directions);

	// fill the columns with polling points
	for (int i = 0; i < nOpt; i++)
	{
		#ifdef PRAGMAS
		// #pragma HLS UNROLL
		#endif

		#ifndef FIXED
		mesh_type mesh = 1;

		if (meshIdx[i] < 0)
		{
			for (int j = 0; j < -meshIdx[i]; j++)
				mesh /= 2;
		}
		else
		{
			for (int j = 0; j < meshIdx[i]; j++)
				mesh *= 2;
		}
		
		for (int j = 0; j < 2 * nOpt; j++)
		{
			#ifdef PRAGMAS
			// #pragma HLS UNROLL
			#endif
			pollMatrix[i][j] = currU[i] + (mesh * directions[i][j]);
		}
		#else 
		/* Using fixed point */
		if(meshIdx[i] < 0)
		{
			for(int j = 0; j < 2 * nOpt; j++)
				pollMatrix[i][j] = currU[i] + (directions[i][j] >> -meshIdx[i]);
		}
		else
		{
			for(int j = 0; j < 2 * nOpt; j++)
				pollMatrix[i][j] = currU[i] + (directions[i][j] << meshIdx[i]);
		}
		#endif
	}
}
