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
	#ifdef DEBUG_PRINT
	double H_f[nOpt][nOpt];
	double frameMeshDiff_f[nOpt];
	double B_f[nOpt][2*nOpt];
	double directions_f[nOpt][2*nOpt];
	#endif

	// Householder matrix
	householder_type H[nOpt][nOpt];

	// columns are polling directions
	direction_type B[nOpt][nOpt];

	mesh_exp_type frameMeshDiff[nOpt];

	#ifdef PRAGMAS
	/*
	 * This function runs once per MADS iteration (MADS_ITER=7 times per
	 * controller() call), far less often than costFunctionArx (called
	 * MADS_ITER*2*nOpt times), so full unrolling below is affordable here.
	 * nOpt <= a handful for any realistic control horizon, so every array
	 * is small enough to become individual registers rather than BRAM.
	 */
	#pragma HLS ARRAY_PARTITION variable=H             complete dim=0
	#pragma HLS ARRAY_PARTITION variable=B             complete dim=0
	#pragma HLS ARRAY_PARTITION variable=directions    complete dim=0
	#pragma HLS ARRAY_PARTITION variable=frameMeshDiff complete dim=1
	#endif

	// generate Householder matrix starting with a random vector
	generateHouseholderMatrix(randomVector, H);

	#ifdef DEBUG_PRINT
	for(int i = 0; i < nOpt; i++) for(int j = 0; j < nOpt; j++) H_f[i][j] = H[i][j].to_double();
	#endif

	// fill B matrix with polling directions
	for (int i = 0; i < nOpt; i++)
	{
		#ifdef PRAGMAS
		/* nOpt iterations, each independent of the others (frameMeshDiff[i]
		 * and directions[i][*] only ever depend on row i's own inputs):
		 * fully unrolling turns this into nOpt parallel direction-vector
		 * datapaths. */
		#pragma HLS UNROLL
		#endif

		frameMeshDiff[i] = frameIdx[i] - meshIdx[i];
		#ifdef DEBUG_PRINT
		frameMeshDiff_f[i] = frameMeshDiff[i].to_double();
		#endif
		
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
			/* BUG: B[i][j] = 0 at least when i = j = 0 --- should be -256 */
			// B[i][j] = (direction_type)(hls::round(mesh[i] * H[i][j]));	// old, needs mesh[nOpt] defined, don't know if it works
			// B[i][j] = H[i][j] << (frameMeshDiff[i]+MADS_C);	// doesn't work
			B[i][j] = H[i][j] * (1 << frameMeshDiff[i]+MADS_C);	// this works
			directions[i][j] = B[i][j];
			directions[i][j + nOpt] = -B[i][j];
			
			#ifdef DEBUG_PRINT
			B_f[i][j] = B[i][j].to_double();
			directions_f[i][j] = directions[i][j].to_double();
			directions_f[i][j+nOpt] = directions[i][j+nOpt].to_double();
			#endif
		}
		#endif
	}
}

// generate the Householder matrix starting with a random vector
void generateHouseholderMatrix(const rand_type v[nOpt], householder_type H[nOpt][nOpt])
{
	#ifdef DEBUG_PRINT
	double v_f[nOpt];
	double H_f[nOpt][nOpt];
	for(int i = 0; i < nOpt; i++) v_f[i] = v[i].to_double();
	#endif

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

			#ifdef DEBUG_PRINT
			H_f[i][j] = H[i][j].to_double();
			#endif
		}

		// Sum with identity matrix
		H[i][i]++;
		// H[i][i] += norm;
		#ifdef DEBUG_PRINT
		H_f[i][i] = H[i][i].to_double();
		#endif
		
		// Saturates householder matrix entries (use expC for previous code)
		for(int j = 0; j < nOpt; j++)
		{
			// Saturates householder matrix entries
			H[i][j] = (H[i][j] > expC) ? householder_type(expC) :
						((H[i][j] < -expC) ? householder_type(-expC) : H[i][j]);
			// H[i][j] /= hMax[j];
			#ifdef DEBUG_PRINT
			H_f[i][j] = H[i][j].to_double();
			#endif
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
