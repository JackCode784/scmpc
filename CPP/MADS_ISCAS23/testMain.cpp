#include "myLib.h"
#include "math.h"

int main() {

	int fail = 0;

	float x[NX] = { 0, 0 };
	float par[NP] = { 10, 4 };
	float ref[NREF] = { 14.2857 };
	float uOpt[NU];

	data_in x_fxd[NX];
	data_in par_fxd[NP];
	data_in ref_fxd[NREF];
	data_out uOpt_fxd[NU];

	data_x zz;
	zz.to_float();
	data_u uu;
	uu.to_float();
	data_cost cc;
	cc.to_float();

	int simStep = 100;

	// time for the switch to be ON during a PWM period
	float timeOn;
	float timeOn2;

	// time for the switch to be OFF during a PWM period
	float timeOff;
	float timeOff2;

	float d;
	float dList[simStep];
	float iLMin[simStep+1];
	float vOutMax[simStep+1];
	float iLMax[simStep];
	float vOutMin[simStep];
	float iLAvg[simStep];
	float vOutAvg[simStep];

	float A0f[N][N];
	float A1f[N][N];
	float bf[N];
	float Ad0f[N][N];
	float bd0f[N];
	float Ad1f[N][N];
	float bd1f[N];

	// file su cui scrivo stato e controllo del sistema ad ogni passo
	FILE *fp;
	// sequenza dello stato ad ogni passo
	float xSeq[simStep * NX + NX];
	// sequenza dello stato ad ogni passo
	float xPeakSeq[simStep * NX];
	// sequenza del controllo ad ogni passo
	float uSeq[simStep * NU];

	double error[simStep];

	iLMin[0] = x[0];
	vOutMax[0] = x[1];

	A0f[0][0] = 0;
	A0f[0][1] = 0;
	A0f[1][0] = 0;
	A0f[1][1] = 0;

	A1f[0][0] = 0;
	A1f[0][1] = -invL;
	A1f[1][0] = invC;
	A1f[1][1] = 0;

	bf[0] = 0;
	bf[1] = 0;

	Ad0f[0][0] = 1;
	Ad0f[0][1] = 0;
	Ad0f[1][0] = 0;
	Ad0f[1][1] = 0;

	Ad1f[0][0] = 1;
	Ad1f[0][1] = 0;
	Ad1f[1][0] = 0;
	Ad1f[1][1] = 0;

	bd0f[0] = 0;
	bd0f[1] = 0;

	bd1f[0] = 0;
	bd1f[1] = 0;

	A0f[1][1] = -invC.to_float()/par[1];
	A1f[1][1] = -invC.to_float()/par[1];
	bf[0] = par[0]*invL.to_float();

	xSeq[0] = x[0];
	xSeq[1] = x[1];

	// integrate the system along the control horizon
	for (int i = 0; i < simStep; i++) {

		if (i == 50) {
			ref[0] = 20;
		}

		for (int j = 0; j < NX; j++) {
			x_fxd[j] = x[j];
		}
		for (int j = 0; j < NP; j++) {
			par_fxd[j] = par[j];
		}
		for (int j = 0; j < NREF; j++) {
			ref_fxd[j] = ref[j];
		}

		MPCwithMADS(x_fxd, par_fxd, ref_fxd, uOpt_fxd);

		d = uOpt_fxd[0].to_float();
		dList[i] = d;

		if ((d < DMIN.to_float()) || (d > DMAX.to_float())) {
			fail = 1;
			break;
		}

		// discretize the system model
		timeOn = d*T.to_float();
		timeOff = T.to_float() - timeOn;
		timeOn2 = timeOn*timeOn;
		timeOff2 = timeOff*timeOff;
		Ad0f[1][1] = 1 + A0f[1][1]*timeOn + A0f[1][1]*A0f[1][1]*timeOn2/2;
		bd0f[0] = bf[0]*timeOn;
		Ad1f[0][0] = 1 + A1f[0][0]*timeOff + A1f[0][1]*A1f[1][0]*timeOff2/2;
		Ad1f[0][1] = A1f[0][1]*timeOff + A1f[0][1]*A1f[1][1]*timeOff2/2;
		Ad1f[1][0] = A1f[1][0]*timeOff + A1f[1][0]*A1f[1][1]*timeOff2/2;
		Ad1f[1][1] = 1 + A1f[1][1]*timeOff + A1f[0][1]*A1f[1][0]*timeOff2/2 + A1f[1][1]*A1f[1][1]*timeOff2/2;
		bd1f[0] = bf[0]*timeOff;
		bd1f[1] = bf[0]*A1f[1][0]*timeOff2/2;

		// predict the system behavior
		iLMax[i] = Ad0f[0][0]*iLMin[i] + bd0f[0];
		vOutMin[i] = Ad0f[1][1]*vOutMax[i];
		iLMin[i+1] = Ad1f[0][0]*iLMax[i] + Ad1f[0][1]*vOutMin[i] + bd1f[0];
		vOutMax[i+1] = Ad1f[1][0]*iLMax[i] + Ad1f[1][1]*vOutMin[i] + bd1f[1];
		iLAvg[i] = (iLMax[i] + iLMin[i+1])/2;
		vOutAvg[i] = (vOutMin[i] + vOutMax[i+1])/2;

		x[0] = iLMin[i+1];
		x[1] = vOutMax[i+1];

		// salvo lo stato attuale in xSeq
		for (int j = i * NX; j < (i + 1) * NX; j++) {
			xSeq[NX + j] = x[j - (i * NX)];
		}
		// salvo il controllo attuale in uSeq
		for (int j = i * NU; j < (i + 1) * NU; j++) {
			uSeq[j] = d;
		}
		// salvo lo stato attuale in xSeq
		for (int j = i * NX; j < (i + 1) * NX; j+=2) {
			xPeakSeq[j] = iLMax[i];
			xPeakSeq[j+1] = vOutMin[i];
		}

		error[i] = fabs(x[1] - ref[0]);

//		// add violation terms for current prediction
//		if (iLMax[i] > IMAX.to_float() + 0.1) {
//			fail = 1;
//			break;
//		}
//		if (iLMin[i+1] < IMIN.to_float() - 0.1) {
//			fail = 1;
//			break;
//		}

	}

	// scrivo sul file xSeq.dat la sequenza di stati ottenuta durante la simulazione
	fp = fopen("xSeq.dat","w");
	for (int i = 0; i < simStep * NX + NX; i++)
	{
		fprintf(fp, "%f\n", xSeq[i]);
	}
	fclose(fp);

	// scrivo sul file uSeq.dat la sequenza di controlli ottenuta durante la simulazione
	fp = fopen("uSeq.dat","w");
	for (int i = 0; i < simStep * NU; i++)
	{
		fprintf(fp, "%f\n", uSeq[i]);
	}
	fclose(fp);

	// scrivo sul file xSeq.dat la sequenza di stati ottenuta durante la simulazione
	fp = fopen("xPeakSeq.dat","w");
	for (int i = 0; i < simStep * NX; i++)
	{
		fprintf(fp, "%f\n", xPeakSeq[i]);
	}
	fclose(fp);

	return fail;

}
