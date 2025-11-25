#include "myLib.h"

void MPCwithMADS(data_in x[NX], data_in par[NP], data_in ref[NREF], data_out uOpt[NU]) {
#pragma HLS ARRAY_PARTITION variable=uOpt dim=1 complete
#pragma HLS ARRAY_PARTITION variable=par dim=1 complete
#pragma HLS ARRAY_PARTITION variable=ref dim=1 complete
#pragma HLS ARRAY_PARTITION variable=x dim=1 complete

	// internal copies of input
	data_x x_int[NX];
	data_x par_int[NP];
	data_x ref_int[NREF];

	// flag for the initialization
	static bool firstCall = true;

	// optimum
	static data_u optimum[N];

	// internal copy of the current state
	for (int i = 0; i < NX; i++) {
		x_int[i] = x[i];
	}

	// internal copy of the parameters
	for (int i = 0; i < NP; i++) {
		par_int[i] = par[i];
	}

	// internal copy of the reference
	for (int i = 0; i < NREF; i++) {
		ref_int[i] = ref[i];
	}

	// initialization for the first call
	if (firstCall) {
		for (int i = 0; i < N; i++) {
			optimum[i] = startingPoint[i];
		}
		firstCall = false;
	}

	// run MADS algorithm to solve the optimization problem
	MADS(optimum, x_int, par_int, ref_int);

	// select the first NU elements of the optimum as the current input
	for (int i = 0; i < NU; i++) {
		uOpt[i] = optimum[i];
	}

	shiftOpt(optimum);
}
