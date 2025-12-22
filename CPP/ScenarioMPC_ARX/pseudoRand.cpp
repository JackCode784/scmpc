#include "setup.h"
#ifdef DEBUG_MODE
#include <cstdlib>
#endif

// Check if correct, random vector is only 2-dimensional instead of nDim_CTRL
void pseudoRandArx(rand_type randomVector[nOpt])
{
	#ifdef DEBUG_MODE
	for(int i = 0; i < nOpt; i++)
	{
		randomVector[i] = static_cast <rand_type> (rand()) / (static_cast <rand_type> (RAND_MAX)); // random number in [0, 1]
		randomVector[i] *= 2;	// random number in [0, 2]
		randomVector[i] -= 1;	// random number in [-1, 1]
	}
	#else
    // Random number generator implementation

	#endif
}
