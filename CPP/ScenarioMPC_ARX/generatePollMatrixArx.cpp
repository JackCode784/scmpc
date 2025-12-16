#include "setup.h"
#include "math.h"

// Generate matrix whose columns are poll points for the computation of cost function in MADS alg
void generatePollMatrixArx(const input_type currU[nOpt], const int frameIdx[nOpt], const int meshIdx[nOpt], input_type pollMatrix[nOpt][2 * nOpt])
{
	// internal copy of the current point
	input_type currU_int[nOpt];

	// mesh size (2^meshIdx)
	input_type mesh[nOpt];

	// polling directions
	input_type directions[nOpt][2 * nOpt];

	// vector for directions generation
	input_type randomVector[nOpt];

	// internal copy of the current point
	for (int i = 0; i < nOpt; i++)
	{
		currU_int[i] = currU[i];

		mesh[i] = 1;

		if (meshIdx[i] < 0)
		{
			// mesh[i] = mesh[i] >> (-meshIdx[i]);
			for (int j = 0; j < -meshIdx[i]; j++)
			{
				mesh[i] /= 2;
			}
		}
		else
		{
			// mesh[i] = mesh[i] << meshIdx[i];
			for (int j = 0; j < meshIdx[i]; j++)
			{
				mesh[i] *= 2;
			}
		}
	}

	// vector randomly in [-1, 1]
	pseudoRandArx(randomVector);

	// generate poll directions starting with a random vector
	generatePollDirectionsArx(randomVector, frameIdx, meshIdx, directions);

	// fill the columns with polling points
	for (int i = 0; i < nOpt; i++)
	{
		for (int j = 0; j < 2 * nOpt; j++)
		{
			pollMatrix[i][j] = currU_int[i] + (mesh[i] * directions[i][j]);
		}
	}
}
