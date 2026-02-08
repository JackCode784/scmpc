#ifndef _MADS_H_
#define _MADS_H_

// Library for fixed point number representation
#include <ap_fixed.h>

// Fixed point data representation for the circuit input
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT> fxd_in;

// Fixed point data representation for the circuit output
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT> fxd_out;

typedef ap_ufixed<32, 9, AP_RND_CONV, AP_SAT> data_cost;
typedef ap_fixed<18, 3, AP_TRN, AP_WRAP> data_rand;
typedef ap_ufixed<36, 12, AP_TRN, AP_WRAP> data_meshSize;
typedef ap_int<6> data_meshExp;
typedef ap_int<12> data_dir;

// number of MADS iterations
#define K 7

// Fixed point data representation for the data to convert input
// from ADC range to actual (model range) and from actual range
// to DAC range
typedef ap_fixed<32, 14, AP_RND_CONV, AP_SAT> fxd_conv;

// Fixed point data representation for every signal inside the algorithm
typedef ap_fixed<18, 5> fxd;

// Number of system states (x_k)
#define nX 3

// Number of system inputs (u_k)
#define nU 1

// Number of system parameters (p_k)
#define nP 0

// Number of system unmeasurable inputs (d_k)
#define nD 0

// Prediction horizon
#define N 5

// Control horizon
#define Nu 3

// Number of reference inputs
#define nRef 1

// Number of controller inputs [x_k; u_(k-1)]
#define nX_CTRL 3

#define nX_CTRL_sc 23

#define nDim_CTRL 3

// Maximum number of iterations
#define max_iter 20

#define TAU 1
static const ap_ufixed<1, 0, AP_TRN, AP_WRAP> expC = 0.5; //2^-c
#define C 1
// initial frame size
static const data_meshExp D[nX_CTRL] = { -8, -8 };
// Index of the input to be tracked
static const fxd ref_idx[nRef] = {0};


// DA AGGIUNGERE MATLAB
static const fxd UMIN[nU] = {
-0.300000};

static const fxd UMAX[nU] = {
0.300000};

static const fxd XMIN[nX] = {
-8, -0.8, -0.300000};

static const fxd XMAX[nX] = {
8, 0.8, 0.300000};

// Matrices of all scenarios
static const fxd A1[nX][nX] = {
{1.000000, 1.000000, 0.500000},
{0.000000, 1.000000, 1.000000},
{0.000000, 0.000000, 1.000000} };

static const fxd B1[nX][nU] = {
{0.500000},
{1.000000},
{1.000000} };

static const fxd G1[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A2[nX][nX] = {
{1.000000, 1.000000, 0.518340},
{0.000000, 1.000000, 1.036680},
{0.000000, 0.000000, 1.000000} };

static const fxd B2[nX][nU] = {
{0.518340},
{1.036680},
{1.000000} };

static const fxd G2[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A3[nX][nX] = {
{1.000000, 1.000000, 0.700394},
{0.000000, 1.000000, 1.400788},
{0.000000, 0.000000, 1.000000} };

static const fxd B3[nX][nU] = {
{0.700394},
{1.400788},
{1.000000} };

static const fxd G3[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A4[nX][nX] = {
{1.000000, 1.000000, 0.552123},
{0.000000, 1.000000, 1.104245},
{0.000000, 0.000000, 1.000000} };

static const fxd B4[nX][nU] = {
{0.552123},
{1.104245},
{1.000000} };

static const fxd G4[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A5[nX][nX] = {
{1.000000, 1.000000, 0.584294},
{0.000000, 1.000000, 1.168587},
{0.000000, 0.000000, 1.000000} };

static const fxd B5[nX][nU] = {
{0.584294},
{1.168587},
{1.000000} };

static const fxd G5[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A6[nX][nX] = {
{1.000000, 1.000000, 0.693453},
{0.000000, 1.000000, 1.386907},
{0.000000, 0.000000, 1.000000} };

static const fxd B6[nX][nU] = {
{0.693453},
{1.386907},
{1.000000} };

static const fxd G6[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A7[nX][nX] = {
{1.000000, 1.000000, 0.557088},
{0.000000, 1.000000, 1.114177},
{0.000000, 0.000000, 1.000000} };

static const fxd B7[nX][nU] = {
{0.557088},
{1.114177},
{1.000000} };

static const fxd G7[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A8[nX][nX] = {
{1.000000, 1.000000, 0.780555},
{0.000000, 1.000000, 1.561111},
{0.000000, 0.000000, 1.000000} };

static const fxd B8[nX][nU] = {
{0.780555},
{1.561111},
{1.000000} };

static const fxd G8[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A9[nX][nX] = {
{1.000000, 1.000000, 0.578328},
{0.000000, 1.000000, 1.156656},
{0.000000, 0.000000, 1.000000} };

static const fxd B9[nX][nU] = {
{0.578328},
{1.156656},
{1.000000} };

static const fxd G9[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A10[nX][nX] = {
{1.000000, 1.000000, 0.605939},
{0.000000, 1.000000, 1.211879},
{0.000000, 0.000000, 1.000000} };

static const fxd B10[nX][nU] = {
{0.605939},
{1.211879},
{1.000000} };

static const fxd G10[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd A11[nX][nX] = {
{1.000000, 1.000000, 0.732372},
{0.000000, 1.000000, 1.464744},
{0.000000, 0.000000, 1.000000} };

static const fxd B11[nX][nU] = {
{0.732372},
{1.464744},
{1.000000} };

static const fxd G11[nX] = {
0.000000, 0.000000, 0.000000};

static const fxd P[nX][nX] = {
{1.000000, 0.000000, 0.000000},
{0.000000, 0.000000, 0.000000},
{0.000000, 0.000000, 0.000000} };

static const fxd Q[nX][nX] = {
{1.000000, 0.000000, 0.000000},
{0.000000, 0.000000, 0.000000},
{0.000000, 0.000000, 0.000000} };

static const fxd R[nU][nU] = {
{10.000000} };

// Default control
static const fxd default_u[nU] = {0.000000};

// Arrays to transform the inputs from their circuit range
// to the actual (model) range:
// x = (x_cir - sim_x_scale_bias).*sim_x_scale_gain
static const fxd_conv sim_x_scale_bias[nX_CTRL] = {-8.000000, -0.800000};
static const fxd_conv sim_x_scale_gain[nX_CTRL] = {0.003907, 0.000391};

// Arrays to transform the outputs from their actual (model)
// range to the actual circuit range:
// u_cir = u.*sim_u_scale_gain + sim_u_scale_bias
static const fxd_conv sim_u_scale_bias[nU] = {-0.300000};
static const fxd_conv sim_u_scale_gain[nU] = {6825.000000};

// Arrays to transform the reference signals from
// their circuit range to the actual (model) range:
// xref = (xref_cir - sim_xref_scale_bias).*sim_xref_scale_gain
static const fxd_conv sim_xref_scale_bias[nRef] = {-8.000000};
static const fxd_conv sim_xref_scale_gain[nRef] = {0.003907};

void scaleX(fxd_in x_in[nX], fxd x_reg[nX]);
void scaleRef(fxd_in ref_in[nRef], fxd ref_reg[nX]);
void scaleU(fxd u_reg[nU], fxd_out u_opt[nU]);
void augmentState(fxd x_reg[nX], fxd ref_reg[nX], fxd x_aug[nX_CTRL], fxd ref_aug[nX_CTRL], fxd u_old[nU]);
void extractU(fxd z[nDim_CTRL], fxd u_reg[nU], fxd u_old[nU]);
void control(fxd_in x_in[nX], fxd_out u_opt[nU], fxd_in ref_in[nRef]);

void MADS(fxd z[nDim_CTRL], fxd x_aug[nX], fxd ref_aug[nRef]);

void shiftOpt(fxd optimum[nDim_CTRL]);

void costFunction(data_cost cost[2], fxd point[nDim_CTRL], fxd x[nX], fxd ref[nRef]);

void generatePollDirections(data_rand randomVector[nDim_CTRL], data_meshExp frameIdx[nDim_CTRL], data_meshExp meshIdx[nDim_CTRL], data_dir directions[nDim_CTRL][2*nDim_CTRL]);

void pseudoRand(data_rand randomVector[nDim_CTRL]);

void generateHouseholderMatrix(data_rand v[nDim_CTRL], data_rand H[nDim_CTRL][nDim_CTRL]);

void progressiveBarrierPolling(data_cost cost[2], fxd currentPoint[nDim_CTRL], fxd pollMatrix[nDim_CTRL][2*nDim_CTRL], data_meshExp frameSize[nDim_CTRL], fxd x[nX], fxd ref[nRef]);

void generatePollMatrix(fxd currentPoint[nDim_CTRL], data_meshExp frameIdx[nDim_CTRL], data_meshExp meshIdx[nDim_CTRL], fxd pollMatrix[nDim_CTRL][2 * nDim_CTRL]);

void integrateSystem(data_cost costFcn[1], const fxd A[nX][nX], const fxd B[nX][nU], const fxd G[nX], fxd point[nDim_CTRL], fxd x[nX], fxd ref[nRef]);


#endif
