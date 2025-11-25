#include "controller.h"

// shift the optimum of one prediction step
void shiftOpt(fxd optimum[nDim_CTRL]) {

	for (int i = 0; i < nDim_CTRL - nU; i++) {
		optimum[i] = optimum[i + nU];
	}

}
