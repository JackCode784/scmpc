#include "setup.h"
#ifdef PRNG_STDLIB
#include <cstdlib> // rand()
#endif

#ifndef PRNG_STDLIB
// 32-bit initial state
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

/* Return one random sample in [-1, 1] */
rand_type pseudoRandArx()
{
	rand_type res;
	#ifdef PRNG_STDLIB
	res = (rand_type)2 * ((rand_type)rand() / (rand_type)RAND_MAX) - (rand_type)1;
	#else
	unsigned int r = xorshift32Step();
	u16_type u16 = (r >> 16) & 0xFFFFu;
	#ifdef FIXED
	frac_type frac;
	frac.range(15,0) = u16.range(15,0);
	res = (rand_type)(frac << 1);
	res -= 1;
	#else
	frac_type frac = u16 / 65536.0; // frac in [0,1]
	res = rand_type(2) * (rand_type)(frac) - rand_type(1);
	#endif
	#endif // PRNG_STDLIB
	
	return res;
}

// Overloading for a generic [-1, 1] vector of coefficients for passive learning
void pseudoRandArx(rand_type coeffs[nGens])
{
	#ifdef PRNG_STDLIB
	for (int i = 0; i < nGens; i++)
    {
		coeffs[i] = (rand_type)2 * ((rand_type)rand() / (rand_type)RAND_MAX)- (rand_type)1;
    }
	#else
	// Random number generator implementation
	for(int i = 0; i < nGens; i++)
	{
		#ifdef PRAGMAS
		#pragma HLS pipeline II=1
		#endif
		unsigned int r = xorshift32Step();
		u16_type u16 = (r >> 16) & 0xFFFFu;
		#ifdef FIXED
		frac_type frac;
		frac.range(15,0) = u16.range(15,0);
		coeffs[i] = (rand_type)(frac) << 1;
		coeffs[i] -= 1;
		#else
		frac_type frac = u16 / 65536.0;
		coeffs[i] = (rand_type(2) * (rand_type)(frac)) - rand_type(1);
		#endif

		// #ifdef FIXED
		// 		float fracf = frac.to_float();
		// #endif
	}
	#endif
}
