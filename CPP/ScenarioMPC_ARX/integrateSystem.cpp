#include "controller.h"

void integrateSystem(data_cost costFcn[2], const fxd A[nX][nX], const fxd B[nX][nU], const fxd G[nX], fxd point[nDim_CTRL], fxd x[nX], fxd ref[nRef])
{

	data_cost term;

	fxd xnext[nX], xcur[nX], ucur[nU], xerr[nX];

	float tmp;

	int i, j, k;

	for (i = 0; i < nU; i++)
	{
		ucur[i] = point[i];

		if (ucur[i] < UMIN || ucur[i] > UMAX)
		{
			costFcn[1] = 500;
			break;
		}
	}

	term = 0;
	for (i = 0; i < nU; i++)
		for (j = 0; j < nU; j++)
			term += ucur[i] * R[i][j] * ucur[j];

	tmp = term.to_float();

	costFcn[0] += term;

	for (i = 0; i < nX; i++)
		xcur[i] = x[i];

	for (k = 1; k < Nu; k++)
	{
		for (i = 0; i < nX; i++)
		{
			xnext[i] = 0;
			for (j = 0; j < nX; j++)
				xnext[i] += A[i][j] * xcur[j];
			for (j = 0; j < nU; j++)
				xnext[i] += B[i][j] * ucur[j];
			xnext[i] += G[i];
		}
		for (i = 0; i < nX; i++)
			xerr[i] = xnext[i] - ref[i];

		term = 0;
		for (i = 0; i < nX; i++)
			for (j = 0; j < nX; j++)
				term += xerr[i] * Q[i][j] * xerr[j];

		costFcn[0] += term;

		for (i = 0; i < nU; i++)
			ucur[i] = point[k * nU + i];

		term = 0;
		for (i = 0; i < nU; i++)
			for (j = 0; j < nU; j++)
				term += ucur[i] * R[i][j] * ucur[j];

		costFcn[0] += term;

		for (i = 0; i < nX; i++)
			xcur[i] = xnext[i];
	}

	for (k = Nu; k < N; k++)
	{
		for (i = 0; i < nX; i++)
		{
			xnext[i] = 0;
			for (j = 0; j < nX; j++)
				xnext[i] += A[i][j] * xcur[j];
			for (j = 0; j < nU; j++)
				xnext[i] += B[i][j] * ucur[j];
			xnext[i] += G[i];
		}
		for (i = 0; i < nX; i++)
			xerr[i] = xnext[i] - ref[i];

		term = 0;
		for (i = 0; i < nX; i++)
			for (j = 0; j < nX; j++)
				term += xerr[i] * Q[i][j] * xerr[j];

		costFcn[0] += term;

		for (i = 0; i < nU; i++)
			ucur[i] = 0;

		term = 0;
		for (i = 0; i < nU; i++)
			for (j = 0; j < nU; j++)
				term += ucur[i] * R[i][j] * ucur[j];

		costFcn[0] += term;

		for (i = 0; i < nX; i++)
			xcur[i] = xnext[i];
	}

	for (i = 0; i < nX; i++)
	{
		xnext[i] = 0;
		for (j = 0; j < nX; j++)
			xnext[i] += A[i][j] * xcur[j];
		for (j = 0; j < nU; j++)
			xnext[i] += B[i][j] * ucur[j];
		xnext[i] += G[i];
	}
	for (i = 0; i < nX; i++)
		xerr[i] = xnext[i] - ref[i];
	term = 0;
	for (i = 0; i < nX; i++)
		for (j = 0; j < nX; j++)
			term += xerr[i] * P[i][j] * xerr[j];
	costFcn[0] += term;

	//	if (iLMax[HORP-1] > IMAX) {
	//		violFunction += iLMax[HORP-1] - IMAX;
	//	}
	//	if (iLMin[HORP] < IMIN) {
	//		violFunction += IMIN - iLMin[HORP];
	//	}
}
