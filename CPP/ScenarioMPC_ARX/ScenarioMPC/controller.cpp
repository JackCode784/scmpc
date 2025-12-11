#include "controller.h"

void control(fxd_in x_in[nX], fxd_out u_opt[nU], fxd_in ref_in[nRef])
{
#pragma HLS ARRAY_PARTITION variable = x_in dim = 1 complete
#pragma HLS ARRAY_PARTITION variable = ref_in dim = 1 complete
#pragma HLS ARRAY_PARTITION variable = u_opt dim = 1 complete

	fxd x_reg[nX_CTRL];

	fxd u_reg[nU];

	fxd x_aug[nX_CTRL];

	fxd ref_reg[nX];

	fxd ref_aug[nX_CTRL];

	static fxd u_old[nU];

	fxd delta_u[nU];

	static bool firstCall = true;

	fxd z[nDim_CTRL];

	scaleX(x_in, x_reg);

	scaleRef(ref_in, ref_reg);

	if (firstCall)
	{
		for (int i = 0; i < nU; i++)
		{
#pragma HLS UNROLL
			u_old[i] = default_u[i];
		}
		firstCall = false;
	}

	float tmp;
	tmp = x_reg[0].to_float();

	augmentState(x_reg, ref_reg, x_aug, ref_aug, u_old);

	MADS(u_old, x_aug, ref_aug);

	extractU(z, u_reg, u_old);

	scaleU(u_reg, u_opt);

	shiftOpt(z);
}
