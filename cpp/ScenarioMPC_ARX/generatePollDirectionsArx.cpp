#include "setup.h"
#ifndef FIXED
#include <cmath>	// useless?
#else
#include "hls_math.h"
#endif

void generateHouseholderMatrix(const rand_type v[nOpt], householder_type H[nOpt][nOpt]);

// Genearte a matrix of directions to be summed to the current point in MADS iteration
void generatePollDirectionsArx(const rand_type randomVector[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], direction_type directions[nOpt][2 * nOpt])
{

	// Householder matrix
	householder_type H[nOpt][nOpt];

	// columns are polling directions
	direction_type B[nOpt][nOpt];

	mesh_exp_type frameMeshDiff[nOpt];

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
		mesh_type mesh = 1;
		for (int j = 0; j < frameMeshDiff[i] + MADS_C; j++)
			mesh *= 2;

		for (int j = 0; j < nOpt; j++)
		{
			#ifdef PRAGMAS
			#pragma HLS UNROLL
			#endif

			B[i][j] = round(mesh * H[i][j]);
			directions[i][j] = B[i][j];
			directions[i][j + nOpt] = -B[i][j];
		}
		#else
		
		for(int j = 0; j < nOpt; j++)
		{
			// B[i][j] = (direction_type)(hls::round(mesh[i] * H[i][j]));
			B[i][j] = (frameMeshDiff[i] + MADS_C < 0) ? H[i][j] >> -(frameMeshDiff[i]+MADS_C) : H[i][j] << frameMeshDiff[i]+MADS_C;
				directions[i][j] = B[i][j];
			directions[i][j + nOpt] = -B[i][j];
		}
		#endif
	}
}

// generate the Householder matrix starting with a random vector
void generateHouseholderMatrix(const rand_type v[nOpt], householder_type H[nOpt][nOpt])
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
		
			#ifndef FIXED
			H[i][j] = (-2 * v[i] * v[j]);
			#else
			H[i][j] = (v[i] * v[j]) << 1;
			H[i][j] = -H[i][j];
			#endif

		}

		// Sum with identity matrix
		H[i][i]++;
		// H[i][i] += norm;
		
		// Saturates householder matrix entries (use expC for previous code)
		for(int j = 0; j < nOpt; j++)
		{
			// Saturates householder matrix entries
			H[i][j] = (H[i][j] > expC) ? householder_type(expC) :
						((H[i][j] < -expC) ? householder_type(-expC) : H[i][j]);
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
