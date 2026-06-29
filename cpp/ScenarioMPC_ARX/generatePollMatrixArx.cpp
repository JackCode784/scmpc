#include "setup.h"

// Generate matrix whose columns are poll points for the computation of cost function in MADS alg
void generatePollMatrixArx(const norm_input_type currU[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], norm_input_type pollMatrix[nOpt][2 * nOpt])
{
	#ifdef PRAGMAS
	// #pragma HLS INLINE
	#endif

	// mesh size (2^meshIdx)
	mesh_type mesh[nOpt];

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

		mesh[i] = (mesh_type)1;

		if (meshIdx[i] < (mesh_type)0)
		{
			#ifdef FIXED
			mesh[i] = mesh[i] >> (-meshIdx[i]);
			#else
			for (int j = 0; j < -meshIdx[i]; j++)
				mesh[i] /= 2;
			#endif
		}
		else
		{
			#ifdef FIXED
			mesh[i] = mesh[i] << meshIdx[i];
			#else
			for (int j = 0; j < meshIdx[i]; j++)
				mesh[i] *= 2;
			#endif
		}

		for (int j = 0; j < 2 * nOpt; j++)
		{
			#ifdef PRAGMAS
			// #pragma HLS UNROLL
			#endif
		
			pollMatrix[i][j] = currU[i] + (norm_input_type)(mesh[i] * directions[i][j]);
		}
	}
}
