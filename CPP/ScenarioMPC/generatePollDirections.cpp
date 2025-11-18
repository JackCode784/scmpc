#include "controller.h"
#include "hls_math.h"

void generatePollDirections(data_rand randomVector[nDim_CTRL], data_meshExp frameIdx[nDim_CTRL], data_meshExp meshIdx[nDim_CTRL], data_dir directions[nDim_CTRL][2 * nDim_CTRL])
{

	// Householder matrix
	data_rand H[nDim_CTRL][nDim_CTRL] = {0};

	// norms of every column of the Householder matrix
	data_rand hMax[nDim_CTRL] = {0};

	// columns are polling directions
	data_dir B[nDim_CTRL][nDim_CTRL];

	data_meshExp frameMeshDiff[nDim_CTRL];
	data_meshSize mesh[nDim_CTRL];

	for (int i = 0; i < nDim_CTRL; i++)
	{
		frameMeshDiff[i] = frameIdx[i] - meshIdx[i] + C;
		mesh[i] = 1;
		mesh[i] = mesh[i] << frameMeshDiff[i];
		//		for (int j = 0; j < frameMeshDiff[i]; j++) {
		//			mesh[i] *= 2;
		//		}
	}

	// generate Householder matrix starting with a random vector
	generateHouseholderMatrix(randomVector, H);

	//	// compute the infinity norm of every column of the Householder matrix
	//	generateHMax(hMax, H);

	// fill B matrix with polling directions
	for (int i = 0; i < nDim_CTRL; i++)
	{
		for (int j = 0; j < nDim_CTRL; j++)
		{
			if (H[i][j] > expC)
			{
				H[i][j] = expC;
			}
			else if (H[i][j] < -expC)
			{
				H[i][j] = -expC;
			}
			//			H[i][j] /= hMax[j];
			B[i][j] = hls::round(mesh[i] * H[i][j]);
		}
	}
	for (int i = 0; i < nDim_CTRL; i++)
	{
		for (int j = 0; j < nDim_CTRL; j++)
		{
			directions[i][j] = B[i][j];
			directions[i][j + nDim_CTRL] = -B[i][j];
		}
	}
}

// generate the Householder matrix starting with a random vector
void generateHouseholderMatrix(data_rand v[nDim_CTRL], data_rand H[nDim_CTRL][nDim_CTRL])
{
	//	data_rand norm = 0;
	//
	//	for (int i = 0; i < N; i++) {
	//		norm += v[i] * v[i];
	//	}

	for (int i = 0; i < nDim_CTRL; i++)
	{
		for (int j = 0; j < nDim_CTRL; j++)
		{
			H[i][j] = -2 * v[i] * v[j];
			if (i == j)
			{
				H[i][j]++;
				//				H[i][j] += norm;
			}
		}
	}
}

//// compute the infinity norm of each column of the matrix H
// void generateHMax(data_rand hMax[N], data_rand H[N][N]) {
//	for (int j = 0; j < N; j++) {
//		for (int i = 0; i < N; i++) {
//			if (hls::fabs(H[i][j]) > hMax[j]) {
//				hMax[j] = hls::fabs(H[i][j]);
//			}
//		}
//	}
// }
