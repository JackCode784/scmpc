#include "myLib.h"
#include "hls_math.h"

void generatePollDirections(data_rand randomVector[N], data_meshExp frameIdx[N], data_meshExp meshIdx[N], data_dir directions[N][2*N]) {

	// Householder matrix
	data_rand H[N][N] = { 0 };

	// norms of every column of the Householder matrix
	data_rand hMax[N] = { 0 };

	// columns are polling directions
	data_dir B[N][N];

	data_meshExp frameMeshDiff[N];
	data_meshSize mesh[N];

	for (int i = 0; i < N; i++) {
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
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (H[i][j] > expC) {
				H[i][j] = expC;
			} else if (H[i][j] < -expC) {
				H[i][j] = -expC;
			}
//			H[i][j] /= hMax[j];
			B[i][j] = hls::round(mesh[i] * H[i][j]);
		}
	}
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			directions[i][j] = B[i][j];
			directions[i][j + N] = -B[i][j];
		}
	}
}

// generate the Householder matrix starting with a random vector
void generateHouseholderMatrix(data_rand v[N], data_rand H[N][N])
{
//	data_rand norm = 0;
//
//	for (int i = 0; i < N; i++) {
//		norm += v[i] * v[i];
//	}

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			H[i][j] = -2 * v[i] * v[j];
			if (i == j) {
				H[i][j]++;
//				H[i][j] += norm;
			}
		}
	}
}


//// compute the infinity norm of each column of the matrix H
//void generateHMax(data_rand hMax[N], data_rand H[N][N]) {
//	for (int j = 0; j < N; j++) {
//		for (int i = 0; i < N; i++) {
//			if (hls::fabs(H[i][j]) > hMax[j]) {
//				hMax[j] = hls::fabs(H[i][j]);
//			}
//		}
//	}
//}
