#include "controller.h"
#include "math.h"

// Check if correct, random vector is only 2-dimensional instead of nDim_CTRL
void pseudoRand(data_rand randomVector[N])
{

	data_rand v[N];

	static ap_uint<25> R1 = 37027;
	static ap_uint<25> R2 = 317217;
	//	static ap_uint<25> R3 = 110139;

	static ap_ufixed<45, 25, AP_TRN, AP_WRAP> R1shift;
	static ap_ufixed<45, 25, AP_TRN, AP_WRAP> R2shift;
	//	static ap_ufixed<45, 25, AP_TRN, AP_WRAP> R3shift;

	R1.to_float();
	R1shift.to_float();
	v[0].to_float();

	ap_uint<5> a = 27;
	ap_uint<21> m = 1048576; // 2^20

	//	for (int i = 0; i < N; i++) {

	R1 = a * R1;
	R1 = R1 % m;
	R1shift = R1;
	v[0] = (R1shift >> 19) - 1;

	R2 = a * R2;
	R2 = R2 % m;
	R2shift = R2;
	v[1] = (R2shift >> 19) - 1;

	//		R3 = a * R3;
	//		R3 = R3 % m;
	//		R3shift = R3;
	//		v[2] = (R3shift >> 19) - 1;

	//	}

	for (int i = 0; i < N; i++)
	{
		randomVector[i] = v[i];
	}
}
