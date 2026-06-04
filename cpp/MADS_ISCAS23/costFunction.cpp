#include "boostParams.h"
#include "hls_math.h"

void costFunction(data_cost cost[2], data_u point[N], data_x x[NX], data_x par[NP], data_x ref[NREF]) {

	// cost function computed in the current point
	data_cost costFunction = 0;

	// constraints violation function computed in the current point
	data_cost violFunction = 0;

	// input voltage
	data_x vIn = par[0];

	// load resistance
	data_x res = par[1];

	// voltage reference
	data_x vOutRef = ref[0];

	data_u dTmp = vIn / vOutRef;

	// input reference
	data_u dRef = 1 - dTmp;

	data_x vTmp = vIn*res;

	// current reference
	data_x iLRef = vOutRef*vOutRef / vTmp;

	// BOOST CONVERTER DYNAMICS (continuous time):
	// x = [iL; vOut]
	// switch ON  -->  dx/dt = A0 x + b
	// switch OFF -->  dx/dt = A1 x + b

	// BOOST CONVERTER DYNAMICS (discrete time):
	// switching system --> during the k-th time step,
	// iL oscillates between iL_(min,k) and iL_(max,k),
	// vOut oscillates between vOut_(min,k) and vOut_(max,k)
	// x_k = [iL_(min,k); vOut_(max,k)]
	// x_(k+1/2) = [iL_(max,k); vOut_(min,k)]
	// switch ON  -->  x_(k+1/2) = Ad0 x_k + bd0
	// switch OFF -->  x_(k+1) = Ad1 x_(k+1/2) + bd1
	// --> the model must be discretized every time step
	// (depending on the duty cycle in that time step)

	// current duty cycle
	data_u dOn;
	data_u dOn2;
	data_u dOff;
	data_u dOff2;

	// every iL_(min,k), for k = 0,...,HORP
	data_x iLMin[HORP+1];

	// every vOut_(max,k), for k = 0,...,HORP
	data_x vOutMax[HORP+1];

	// every iL_(max,k), for k = 0,...,HORP-1
	data_x iLMax[HORP];

	// every vOut_(min,k), for k = 0,...,HORP-1
	data_x vOutMin[HORP];

	// iL_(avg,k), for k = for k = 1,...,HORP
	data_x iLAvg[HORP];

	// vOut_(avg,k), for k = for k = 1,...,HORP
	data_x vOutAvg[HORP];

	data_u dList[N];

	data_u dErr;
	data_x iErr;
	data_x vErr;

	data_mat Ad0_int[NX][NX];
	data_mat Ad1_int[NX][NX];
	data_mat bd0_int[NX];
	data_mat bd1_int[NX];

	data_mat A0T = -invC*T/res;
	data_mat A0T2 = (A0T * A0T) >> 1;
	data_mat A1T[NX][NX];
	data_mat A1T2[NX][NX];
	data_mat bT = vIn*T*invL;
	data_mat A1bT;

	A1T[0][0] = 0;
	A1T[0][1] = -invL * T;
	A1T[1][0] = invC * T;
	A1T[1][1] = A0T;
	A1T2[0][0] = (A1T[0][1]*A1T[1][0]) >> 1;
	A1T2[0][1] = (A1T[0][1]*A1T[1][1]) >> 1;
	A1T2[1][0] = (A1T[1][0]*A1T[1][1]) >> 1;
	A1T2[1][1] = (A1T[0][1]*A1T[1][0] + A1T[1][1]*A1T[1][1]) >> 1;
	A1bT = (A1T[1][0]*bT) >> 1;

	Ad0_int[0][0] = 1;

	for (int i = 0; i < N; i++) {
		#pragma HLS UNROLL
		dList[i] = point[i];
	}

	// iL_(min,k) is the current measured current
	iLMin[0] = x[0];

	// iL_(min,k) is the current measured current
	vOutMax[0] = x[1];

	// integrate the system along the control horizon
	for (int i = 0; i < HORC; i++) {
		#pragma HLS PIPELINE

		dOn = dList[i];
		dOn2 = dOn*dOn;
		dOff = 1 - dOn;
		dOff2 = 1 + dOn2 - (dOn << 1);

		// discretize the system model
		Ad0_int[1][1] = 1 + A0T*dOn + A0T2*dOn2;
		bd0_int[0] = bT*dOn;
		Ad1_int[0][0] = 1 + A1T2[0][0]*dOff2;
		Ad1_int[0][1] = A1T[0][1]*dOff + A1T2[0][1]*dOff2;
		Ad1_int[1][0] = A1T[1][0]*dOff + A1T2[1][0]*dOff2;
		Ad1_int[1][1] = 1 + A1T[1][1]*dOff + A1T2[1][1]*dOff2;
		bd1_int[0] = bT*dOff;
		bd1_int[1] = A1bT*dOff2;

		// predict the system behavior
		iLMax[i] = Ad0_int[0][0]*iLMin[i] + bd0_int[0];
		vOutMin[i] = Ad0_int[1][1]*vOutMax[i];
		iLMin[i+1] = Ad1_int[0][0]*iLMax[i] + Ad1_int[0][1]*vOutMin[i] + bd1_int[0];
		vOutMax[i+1] = Ad1_int[1][0]*iLMax[i] + Ad1_int[1][1]*vOutMin[i] + bd1_int[1];
		iLAvg[i] = ((iLMax[i] + iLMin[i+1]) >> 1);
		vOutAvg[i] = ((vOutMin[i] + vOutMax[i+1]) >> 1);

		// add cost terms for current prediction
		iErr = iLAvg[i] - iLRef;
		vErr = vOutAvg[i] - vOutRef;
		dErr = dList[i] - dRef;
		costFunction += iErr * iErr * Q[0][0];
		costFunction += vErr * vErr * Q[1][1];
		costFunction += dErr * dErr * R;

//		// add violation terms for current prediction
//		if (iLMax[i] > IMAX) {
//			violFunction += iLMax[i] - IMAX;
//		}
//		if (iLMin[i+1] < IMIN) {
//			violFunction += IMIN - iLMin[i+1];
//		}

	}

	// if the prediction horizon is longer than the control horizon, use the
	// last predicted control for the rest of the horizon --> the system
	// matrices remain constants
	for (int i = HORC; i < HORP; i++) {
		#pragma HLS PIPELINE

		// predict the system behavior
		iLMax[i] = Ad0_int[0][0]*iLMin[i] + bd0_int[0];
		vOutMin[i] = Ad0_int[1][1]*vOutMax[i];
		iLMin[i+1] = Ad1_int[0][0]*iLMax[i] + Ad1_int[0][1]*vOutMin[i] + bd1_int[0];
		vOutMax[i+1] = Ad1_int[1][0]*iLMax[i] + Ad1_int[1][1]*vOutMin[i] + bd1_int[1];
		iLAvg[i] = ((iLMax[i] + iLMin[i+1]) >> 1);
		vOutAvg[i] = ((vOutMin[i] + vOutMax[i+1]) >> 1);

		// add cost terms for current prediction
		iErr = iLAvg[i] - iLRef;
		vErr = vOutAvg[i] - vOutRef;
		costFunction += iErr * iErr * Q[0][0];
		costFunction += vErr * vErr * Q[1][1];

//		// add violation terms for current prediction
//		if (iLMax[i] > IMAX) {
//			violFunction += iLMax[i] - IMAX;
//		}
//		if (iLMin[i+1] < IMIN) {
//			violFunction += IMIN - iLMin[i+1];
//		}

	}

//	// predict the system behavior
//	iLMax[HORP-1] = Ad0_int[0][0]*iLMin[HORP-1] + bd0_int[0];
//	vOutMin[HORP-1] = Ad0_int[1][1]*vOutMax[HORP-1];
//	iLMin[HORP] = Ad1_int[0][0]*iLMax[HORP-1] + Ad1_int[0][1]*vOutMin[HORP-1] + bd1_int[0];
//	vOutMax[HORP] = Ad1_int[1][0]*iLMax[HORP-1] + Ad1_int[1][1]*vOutMin[HORP-1] + bd1_int[1];
//	iLAvg[HORP-1] = ((iLMax[HORP-1] + iLMin[HORP]) >> 1);
//	vOutAvg[HORP-1] = ((vOutMin[HORP-1] + vOutMax[HORP]) >> 1);
//
//	// add cost terms for current prediction
//	iErr = iLAvg[HORP-1] - iLRef;
//	vErr = vOutAvg[HORP-1] - vOutRef;
//	costFunction += iErr * iErr * P00 + iErr * vErr * P01_10 + vErr * vErr * P11;
//
//	// add violation terms for current prediction
//	if (iLMax[HORP-1] > IMAX) {
//		violFunction += iLMax[HORP-1] - IMAX;
//	}
//	if (iLMin[HORP] < IMIN) {
//		violFunction += IMIN - iLMin[HORP];
//	}

	costFunction += dErr * dErr * R * (HORP - HORC);

	for (int i = 0; i < N; i++) {
		if ((dList[i] < DMIN) || (dList[i] > DMAX)) {
			violFunction = 500;
			break;
		}
	}

	cost[0] = costFunction;
	cost[1] = violFunction;

}
