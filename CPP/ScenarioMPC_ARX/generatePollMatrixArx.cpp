#include "setup.h"

// Generate matrix whose columns are poll points for the computation of cost function in MADS alg
void generatePollMatrixArx(const input_type currU[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], input_type pollMatrix[nOpt][2 * nOpt])
{
	// mesh size (2^meshIdx)
	mesh_type mesh[nOpt];

	// polling directions
	direction_type directions[nOpt][2 * nOpt];

	// vector for directions generation
	rand_type randomVector[nOpt];

	// vector randomly in [-1, 1]
	pseudoRandArx(randomVector);

	// generate poll directions starting with a random vector
	generatePollDirectionsArx(randomVector, frameIdx, meshIdx, directions);

	// fill the columns with polling points
	for (int i = 0; i < nOpt; i++)
	{
		mesh[i] = 1;

		if (meshIdx[i] < 0)
		{
			#ifndef DEBUG_MODE
			mesh[i] = mesh[i] >> (-meshIdx[i]);
			#else
			for (int j = 0; j < -meshIdx[i]; j++)
				mesh[i] /= 2;
			#endif
		}
		else
		{
			#ifndef DEBUG_MODE
			mesh[i] = mesh[i] << meshIdx[i];
			#else
			for (int j = 0; j < meshIdx[i]; j++)
				mesh[i] *= 2;
			#endif
			}

		for (int j = 0; j < 2 * nOpt; j++)
		{
			pollMatrix[i][j] = currU[i] + (mesh[i] * directions[i][j]);
		}
	}
}
