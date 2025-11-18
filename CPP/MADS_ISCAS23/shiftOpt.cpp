#include "myLib.h"

// shift the optimum of one prediction step
void shiftOpt(data_u optimum[N]) {

	for (int i = 0; i < N - NU; i++) {
		optimum[i] = optimum[i + NU];
	}

}
