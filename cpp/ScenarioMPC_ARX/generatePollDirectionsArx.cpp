#include "setup.h"
#ifndef FIXED
#include <cmath>	// useless?
#else
#include "hls_math.h"
#endif

void generateHouseholderMatrix(const rand_type v[nOpt], norm_input_type H[nOpt][nOpt]);

// Genearte a matrix of directions to be summed to the current point in MADS iteration
void generatePollDirectionsArx(const rand_type randomVector[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], direction_type directions[nOpt][2 * nOpt])
{

	// Householder matrix
	norm_input_type H[nOpt][nOpt];

	// columns are polling directions
	direction_type B[nOpt][nOpt];

	mesh_exp_type frameMeshDiff[nOpt];
	mesh_type mesh[nOpt];

	// generate Householder matrix starting with a random vector
	generateHouseholderMatrix(randomVector, H);

	// fill B matrix with polling directions
	for (int i = 0; i < nOpt; i++)
	{
		#ifdef PRAGMAS	
		// #pragma HLS UNROLL
		#endif

		frameMeshDiff[i] = frameIdx[i] - meshIdx[i];
		
		#ifndef FIXED
		mesh[i] = 1;
		for (int j = 0; j < frameMeshDiff[i]; j++)
			mesh[i] *= 2;

		for (int j = 0; j < nOpt; j++)
		{
			#ifdef PRAGMAS
			#pragma HLS UNROLL
			#endif

			B[i][j] = round(mesh[i] * H[i][j]);
			directions[i][j] = B[i][j];
			directions[i][j + nOpt] = -B[i][j];
		}
		#else
		// B[i][j] = (direction_type)(hls::round(mesh[i] * H[i][j]));
		B[i][j] = hls::round((frameMeshDiff[i] < 0) ? H[i][j] >> -frameMeshDiff[i] : H[i][j] << frameMeshDiff[i]);

		for(int j = 0; j < nOpt; j++)
		{
			directions[i][j] = B[i][j];
			directions[i][j + nOpt] = -B[i][j];
		}
		#endif
	}
}

// generate the Householder matrix starting with a random vector
void generateHouseholderMatrix(const rand_type v[nOpt], norm_input_type H[nOpt][nOpt])
{
	//	input_type norm = 0;
	//
	//	for (int i = 0; i < N; i++) {
	//		norm += v[i] * v[i];
	//	}

	for (int i = 0; i < nOpt; i++)
	{
		#ifdef PRAGMAS
		#pragma HLS UNROLL
		#endif

		for (int j = 0; j < nOpt; j++)
		{
		#ifdef PRAGMAS
		#pragma HLS UNROLL
		#endif
		
			H[i][j] = (norm_input_type)(-2 * v[i] * v[j]);
		}

		// Sum with identity matrix
		H[i][i]++;
		// H[i][i] += norm;

		for(int j = 0; j < nOpt; j++)
		{
			// Saturates householder matrix entries
			H[i][j] = (H[i][j] > 1) ? 1 :
						((H[i][j] < -1) ? -1 : H[i][j]);
			// H[i][j] /= hMax[j];
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
