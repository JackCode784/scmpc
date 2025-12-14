#include "setup.h"
#include <cmath>
// #include "hls_math.h"

void generateHouseholderMatrix(input_type v[nOpt], input_type H[nOpt][nOpt]);

// Genearte a matrix of directions to be summed to the current point in MADS iteration
void generatePollDirectionsArx(input_type randomVector[nOpt], int frameIdx[nOpt], int meshIdx[nOpt], input_type directions[nOpt][2 * nOpt])
{

	// Householder matrix
	input_type H[nOpt][nOpt] = {0};

	// norms of every column of the Householder matrix
	// input_type hMax[nOpt] = {0};

	// columns are polling directions
	input_type B[nOpt][nOpt];

	int frameMeshDiff[nOpt];
	input_type mesh[nOpt];

	// for (int i = 0; i < nOpt; i++)
	// {
	// 	frameMeshDiff[i] = frameIdx[i] - meshIdx[i] + C;
	// 	mesh[i] = 1;
	// 	// // mesh[i] = mesh[i] << frameMeshDiff[i];
	// 	for (int j = 0; j < frameMeshDiff[i]; j++)
	// 	{
	// 		mesh[i] *= 2;
	// 	}
	// }

	// generate Householder matrix starting with a random vector
	generateHouseholderMatrix(randomVector, H);

	// compute the infinity norm of every column of the Householder matrix
	//	generateHMax(hMax, H);

	// fill B matrix with polling directions
	for (int i = 0; i < nOpt; i++)
	{
		frameMeshDiff[i] = frameIdx[i] - meshIdx[i] + C;
		mesh[i] = 1;
		// // mesh[i] = mesh[i] << frameMeshDiff[i];
		for (int j = 0; j < frameMeshDiff[i]; j++)
		{
			mesh[i] *= 2;
		}

		for (int j = 0; j < nOpt; j++)
		{
			// Saturates householder matrix entries
			if (H[i][j] > expC)
			{
				H[i][j] = expC;
			}
			else if (H[i][j] < -expC)
			{
				H[i][j] = -expC;
			}
			//			H[i][j] /= hMax[j];
			// B[i][j] = hls::round(mesh[i] * H[i][j]);
			B[i][j] = round(mesh[i] * H[i][j]);
			directions[i][j] = B[i][j];
			directions[i][j + nOpt] = -B[i][j];
		}
	}
}

// generate the Householder matrix starting with a random vector
void generateHouseholderMatrix(input_type v[nOpt], input_type H[nOpt][nOpt])
{
	//	input_type norm = 0;
	//
	//	for (int i = 0; i < N; i++) {
	//		norm += v[i] * v[i];
	//	}

	for (int i = 0; i < nOpt; i++)
	{
		for (int j = 0; j < nOpt; j++)
		{
			H[i][j] = -2 * v[i] * v[j];
			if (i == j)
			{
				H[i][j]++;
				// H[i][j] += norm;
			}
		}
	}
}

// // compute the infinity norm of each column of the matrix H
// void generateHMax(input_type hMax[N], input_type H[N][N])
// {
// 	for (int j = 0; j < N; j++)
// 	{
// 		for (int i = 0; i < N; i++)
// 		{
// 			if (hls::fabs(H[i][j]) > hMax[j])
// 			{
// 				hMax[j] = hls::fabs(H[i][j]);
// 			}
// 		}
// 	}
// }
