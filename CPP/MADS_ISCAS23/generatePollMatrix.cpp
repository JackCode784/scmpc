#include "myLib.h"
#include "math.h"

void generatePollMatrix(data_u currentPoint[N], data_meshExp frameIdx[N], data_meshExp meshIdx[N], data_u pollMatrix[N][2 * N])
{
	// internal copy of the current point
	data_u currentPoint_int[N];

	// mesh size
	data_meshSize mesh[N];

	// polling directions
	data_dir directions[N][2 * N];

	// vector for directions generation
	data_rand randomVector[N];

	// internal copy of the current point
	for (int i = 0; i < N; i++) {
		currentPoint_int[i] = currentPoint[i];

		mesh[i] = 1;

		if (meshIdx[i] < 0) {
			mesh[i] = mesh[i] >> (-meshIdx[i]);
//			for (int j = 0; j < -meshIdx[i]; j++) {
//				mesh[i] /= 2;
//			}
		} else {
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
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < 2 * N; j++) {
			pollMatrix[i][j] = currentPoint_int[i] + (mesh[i] * directions[i][j]);
		}
	}

}
