#include "setup.h"
#ifdef DEBUG_MODE
#include <cstdlib> // rand()
#endif

#ifdef DEBUG_MODE
static unsigned int random_state = 0xAAAAAAAAu;

inline void pseudoRandSeed(unsigned int seed)
{
	random_state = (seed == 0u) ? 1u : seed;
}

inline unsigned int xorshift32Step()
{
	unsigned int x = random_state;
	x ^= (x << 13);
	x ^= (x >> 17);
	x ^= (x << 5);
	random_state = x;
	return x;
}
#endif

// Check if correct, random vector is only 2-dimensional instead of nDim_CTRL
void pseudoRandArx(rand_type randomVector[nOpt])
{
	#ifndef DEBUG_MODE
	for(int i = 0; i < nOpt; i++)
	{
		randomVector[i] = static_cast <rand_type> (rand()) / (static_cast <rand_type> (RAND_MAX)); // random number in [0, 1]
		randomVector[i] *= 2;	// random number in [0, 2]
		randomVector[i] -= 1;	// random number in [-1, 1]
	}
	#else
    // Random number generator implementation
	for(int i = 0; i < nOpt; i++)
	{
		// #pragma HLS pipeline II=1
		unsigned int r = xorshift32Step();
		unsigned int u16 = (r >> 16) & 0xFFFFu;
		rand_type frac = rand_type(u16) / rand_type(65536u);
		randomVector[i] = (rand_type(2) * frac) - rand_type(1);
	}
	#endif
}

void pseudoRandArx(theta_type thetaRow[nTheta], theta_type thetaRange[nTheta])
{
	#ifndef DEBUG_MODE
    for (int i = 0; i < nTheta; i++)
    {
        thetaRow[i] = static_cast<theta_type>(rand()) / (static_cast<theta_type>(RAND_MAX)); // random number in [0, 1]
		thetaRow[i] *= thetaRange[i];                                                        // random vector in [0, thetaMax-thetaMin]
		thetaRow[i] += thetaMin[i];                                                          // random vector in [thetaMin, thetaMax]
    }
	#else
	for(int i = 0; i < nTheta; i++)
	{
		// #pragma HLS pipeline II=1
		unsigned int r = xorshift32Step();
		unsigned int u16 = (r >> 16) & 0xFFFFu;
		theta_type frac = theta_type(u16) / theta_type(65536u);
		thetaRow[i] = (thetaRange[i] * frac) + thetaMin[i];
	}
	#endif
}