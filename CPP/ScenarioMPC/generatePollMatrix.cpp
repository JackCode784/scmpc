#include "controller.h"
#include "math.h"

void generatePollMatrix(fxd currentPoint[nDim_CTRL], data_meshExp frameIdx[nDim_CTRL], data_meshExp meshIdx[nDim_CTRL], fxd pollMatrix[nDim_CTRL][2 * nDim_CTRL])
{
	// internal copy of the current point
	fxd currentPoint_int[nDim_CTRL];

	// mesh size
	data_meshSize mesh[nDim_CTRL];

	// polling directions
	data_dir directions[nDim_CTRL][2 * nDim_CTRL];

	// vector for directions generation
	data_rand randomVector[nDim_CTRL];

	// internal copy of the current point
	for (int i = 0; i < nDim_CTRL; i++)
	{
		currentPoint_int[i] = currentPoint[i];

		mesh[i] = 1;

		if (meshIdx[i] < 0)
		{
			mesh[i] = mesh[i] >> (-meshIdx[i]);
			//			for (int j = 0; j < -meshIdx[i]; j++) {
			//				mesh[i] /= 2;
			//			}
		}
		else
		{
			mesh[i] = mesh[i] << meshIdx[i];
			//			for (int j = 0; j < meshIdx[i]; j++) {
			//				mesh[i] *= 2;
			//			}
		}
	}

	// vector randomly filled
	pseudoRand(randomVector);

	// generate poll directions starting with a random vector
	generatePollDirections(randomVector, frameIdx, meshIdx, directions);

	// fill the columns with polling points
	for (int i = 0; i < nDim_CTRL; i++)
	{
		for (int j = 0; j < 2 * nDim_CTRL; j++)
		{
			pollMatrix[i][j] = currentPoint_int[i] + (mesh[i] * directions[i][j]);
		}
	}
}
