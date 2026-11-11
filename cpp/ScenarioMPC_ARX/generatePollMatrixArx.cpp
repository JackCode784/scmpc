#include "setup.h"

// Generate matrix whose columns are poll points for the computation of cost function in MADS alg
void generatePollMatrixArx(const norm_input_type currU[nOpt], const mesh_exp_type frameExp[nOpt], const mesh_exp_type meshExp[nOpt], norm_input_type pollMatrix[nOpt][2 * nOpt])
{
	#ifdef PRAGMAS
	// #pragma HLS INLINE
	#endif

	#ifdef DEBUG_PRINT
	double frameExp_f[nOpt], meshExp_f[nOpt];
	double directions_f[nOpt][2*nOpt];
	double randVec_f[nOpt];
	double pollMatrix_f[nOpt][2*nOpt];
	double currU_f[nOpt];
	for(int i = 0; i < nOpt; i++) {
		frameExp_f[i] = frameExp[i].to_double(); 
		meshExp_f[i] = meshExp[i].to_double(); 
		currU_f[i]=currU[i].to_double(); }
	#endif

	// polling directions
	direction_type directions[nOpt][2 * nOpt];

	// vector for directions generation
	rand_type randomVector[nOpt];

	#ifdef PRAGMAS
	#pragma HLS ARRAY_PARTITION variable=directions   complete dim=0
	#pragma HLS ARRAY_PARTITION variable=randomVector complete dim=1
	#endif

	// generate random vector in [-1, 1]
	for(int i = 0; i < nOpt; i++)
	{
		#ifdef PRAGMAS
		/* pseudoRandArx() advances the single shared xorshift32 state
		 * (pseudoRand.cpp): draw i+1 depends on draw i, so this loop
		 * cannot be unrolled. PIPELINE II=1 still issues one draw/cycle. */
		#pragma HLS PIPELINE II=1
		#endif
		randomVector[i] = pseudoRandArx();
	}

	#ifdef DEBUG_PRINT
	for(int i = 0; i < nOpt; i++) randVec_f[i] = randomVector[i].to_double();
	#endif

	// generate poll directions starting with a random vector
	generatePollDirectionsArx(randomVector, frameExp, meshExp, directions);

	#ifdef DEBUG_PRINT
	for(int i = 0; i < nOpt; i++) for(int j = 0; j < 2*nOpt; j++) directions_f[i][j] = directions[i][j].to_double();
	#endif

	// fill the columns with polling points
	#ifndef FIXED
	for (int i = 0; i < nOpt; i++)
	{
		mesh_type mesh = 1;

		if (meshExp[i] < 0)
		{
			for (int j = 0; j < -meshExp[i]; j++)
				mesh /= 2;
		}
		else
		{
			for (int j = 0; j < meshExp[i]; j++)
				mesh *= 2;
		}

		for (int j = 0; j < 2 * nOpt; j++)
			pollMatrix[i][j] = currU[i] + (mesh * directions[i][j]);
	}
	#else /* fixed point */
	/* step = direction * 2^meshExp, as a shift in poll_step_type (types.h:
	 * shifting direction_type itself made fine steps one-sided), rounded
	 * once into norm_input_type by the addition.
	 * Every entry is independent of the others, so the loop order does not
	 * change the result. Columns j outer and pipelined, rows i inner and
	 * unrolled: nOpt copies of the 40-bit shift/round/saturate logic, reused
	 * over the 2*nOpt columns, instead of one copy per entry (2*nOpt*nOpt,
	 * ~250 LUTs each in csynth). Costs ~2*nOpt-1 cycles per call. */
	for (int j = 0; j < 2 * nOpt; j++)
	{
		#ifdef PRAGMAS
		/* One column per cycle: shares the per-row logic across columns
		 * (see above). */
		#pragma HLS PIPELINE II=1
		#endif
		for (int i = 0; i < nOpt; i++)
		{
			#ifdef PRAGMAS
			/* Rows of a column are independent: one copy of the logic per row. */
			#pragma HLS UNROLL
			#endif
			const poll_step_type step = poll_step_type(directions[i][j]) << meshExp[i];
			pollMatrix[i][j] = currU[i] + step;

			#ifdef DEBUG_PRINT
			pollMatrix_f[i][j] = pollMatrix[i][j].to_double();
			#endif
		}
	}
	#endif
}
