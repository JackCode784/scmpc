#define DEBUG_MATLAB

#ifndef DEBUG_MATLAB
// Library for fixed point number representation
#include <ap_fixed.h>
#endif

#ifndef DEBUG_MATLAB
/*
    ----------------------------------------
    Data types definitions  
    ----------------------------------------    
*/

// Fixed point data representation for the circuit input
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT> input_type;

// Fixed point data representation for the circuit output
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT> output_type;

typedef ap_ufixed<32, 9, AP_RND_CONV, AP_SAT> cost_type;
typedef ap_fixed<18, 3, AP_TRN, AP_WRAP> rand_type;
typedef ap_ufixed<36, 12, AP_TRN, AP_WRAP> mesh_type;
typedef ap_int<6> mesh_exp_type;
typedef ap_int<12> direction_type;

// Fixed point data representation for the data to convert input
// from ADC range to actual (model range) and from actual range
// to DAC range
typedef ap_fixed<32, 14, AP_RND_CONV, AP_SAT> conv_type;

// Fixed point data representation for every signal inside the algorithm
typedef ap_fixed<18, 5> alg_type;
#else
/* --------------------------------------------------------------------- */
/*  Datatypes for debugging in VS Code  */
/* --------------------------------------------------------------------- */
typedef int mesh_exp_type;  // datatype for mesh exponents
typedef double rand_type;   // datatype for random vectors
typedef double alg_type;    // datatype for algorithm signals
typedef double mesh_type;   // datatype for mesh points (same as input_type)
typedef double output_type; // datatype for output samples
typedef double input_type;  // datatype for input samples
typedef double cost_type;   // datatype for cost function value
typedef float weights_type; // datatype for weights matrices
typedef float theta_type;   // datatype for theta ARX parameters
typedef short hzn_type;     // datatype for horizons values
typedef int direction_type; // datatype for directions matrix
typedef double conv_type;   // datatype for DAC
typedef double err_type;    // datatype for output-reference difference
#endif

/*
    ----------------------------------------
    State-space system params  
    ----------------------------------------    
*/
// Number of system states (x_k)
// #define nx 2

// Number of system inputs (u_k)
#define nu 1    // ARX assumption: SISO system

// Rewritten for ARX: number of system output
#define ny 1    // ARX assumption: SISO system

// Number of system parameters (p_k)
// #define nP 0

// Number of system unmeasurable inputs (d_k)
// #define nD 0

// Number of reference inputs
#define ny_ref 1

/*
    ----------------------------------------
    MPC params  
    ----------------------------------------    
*/
// Prediction horizon
#define Nhor 5

// Control horizon
#define NhorU 3

// Number of scenarios
#define Nscen 2

// MPC saturation constraints
// static const fxd UMIN[nU] = {
// -0.300000};
static const alg_type UMIN[nu] = {-0.3}; // rewritten for arx

// static const fxd UMAX[nU] = {
// 0.300000};
static const alg_type UMAX[nu] = {0.3}; // rewritten for arx

// static const fxd XMIN[nX] = {
// -8, -0.8, -0.300000};
static const alg_type YMIN[ny] = {0}; // rewritten for arx

// static const fxd XMAX[nX] = {
// 8, 0.8, 0.300000};
static const alg_type YMAX[ny] = {8}; // rewritten for arx

// Number of optimization variables (nu * NhorU)
static const int nOpt = nu * NhorU;

// Number of controller inputs [x_k; u_(k-1)]
// #define nX_CTRL 3

/*
    ----------------------------------------
    MADS params  
    ----------------------------------------    
*/
#define MADS_ITER 7
#define TAU 1
#define C 1
#ifndef DEBUG_MATLAB
static const ap_ufixed<1, 0, AP_TRN, AP_WRAP> expC = 0.5; //2^-c
#else
static const input_type expC = 0.5;
#endif

// initial frame size
static const mesh_exp_type D[nOpt] = {-8, -8, -8};

/*
    ----------------------------------------
    State space-representation  
    ----------------------------------------    
*/
// static const float doubleIntegratorA[nx][nx] = {
//     {0, 1e3},
//     {0, 0}
// };

// static const float doubleIntegratorB[nx][nu] = {
//     {0},
//     {1e3}
// };

// static const float doubleIntegratorC[ny][nx] = {
//     {1, 0}
// };

// static const float doubleIntegratorD[ny][nu] = {
//     {0}
// };

/*
    ----------------------------------------
    ARX parameters  
    ----------------------------------------    
*/
static const int na = 2;
static const int nb = 1;
static const int nd = 2;
static const int nTheta = na + nb;

static theta_type thetaNominal[nTheta] = {2, -1, 1};

// Ranges for other scenarios' theta
static const theta_type thetaMax[nTheta] = {2, -1, 1.5};
static const theta_type thetaMin[nTheta] = {2, -1, 0.5};

/*
    ----------------------------------------
    Matrices for cost function evaluation  
    ----------------------------------------    
*/
// static const fxd P[nX][nX] = {
// {1.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000} };
static const weights_type P[ny][ny] = {
    {1}
};

// static const fxd Q[nX][nX] = {
// {1.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000} };
static const weights_type Q[ny][ny] = {
    {1}
};

// static const fxd R[nU][nU] = {
// {10.000000} };
static const weights_type R[nu][nu] = {
    {10}
};

// Default control
// static const fxd default_u[nU] = {0.000000};
static const input_type default_u[nOpt] = {0.0};

// Arrays to transform the inputs from their circuit range
// to the actual (model) range:
// x = (x_cir - sim_x_scale_bias).*sim_x_scale_gain
// static const fxd_conv sim_x_scale_bias[nX_CTRL] = {-8.000000, -0.800000};
// static const fxd_conv sim_x_scale_gain[nX_CTRL] = {0.003907, 0.000391};

// Arrays to transform the outputs from their actual (model)
// range to the actual circuit range:
// u_cir = u.*sim_u_scale_gain + sim_u_scale_bias
// static const fxd_conv sim_u_scale_bias[nU] = {-0.300000};
// static const fxd_conv sim_u_scale_gain[nU] = {6825.000000};

// Arrays to transform the reference signals from
// their circuit range to the actual (model) range:
// xref = (xref_cir - sim_xref_scale_bias).*sim_xref_scale_gain
// static const fxd_conv sim_xref_scale_bias[nRef] = {-8.000000};
// static const fxd_conv sim_xref_scale_gain[nRef] = {0.003907};

// void scaleX(fxd_in x_in[nX], fxd x_reg[nX]);
// void scaleRef(fxd_in ref_in[nRef], fxd ref_reg[nX]);
// void scaleU(fxd u_reg[nU], fxd_out u_opt[nU]);
// void augmentState(fxd x_reg[nX], fxd ref_reg[nX], fxd x_aug[nX_CTRL], fxd ref_aug[nX_CTRL], fxd u_old[nU]);
// void extractU(fxd z[nDim_CTRL], fxd u_reg[nU], fxd u_old[nU]);
// void control(fxd_in x_in[nX], fxd_out u_opt[nU], fxd_in ref_in[nRef]);

// void MADS(fxd z[nDim_CTRL], fxd x_aug[nX], fxd ref_aug[nRef]);

// void shiftOpt(fxd optimum[nDim_CTRL]);

// void costFunction(data_cost cost[2], fxd point[nDim_CTRL], fxd x[nX], fxd ref[nRef]);

// void generatePollDirections(data_rand randomVector[nDim_CTRL], data_meshExp frameIdx[nDim_CTRL], data_meshExp meshIdx[nDim_CTRL], data_dir directions[nDim_CTRL][2*nDim_CTRL]);

// void pseudoRand(data_rand randomVector[nDim_CTRL]);

// void generateHouseholderMatrix(data_rand v[nDim_CTRL], data_rand H[nDim_CTRL][nDim_CTRL]);

// void progressiveBarrierPolling(data_cost cost[2], fxd currentPoint[nDim_CTRL], fxd pollMatrix[nDim_CTRL][2*nDim_CTRL], data_meshExp frameSize[nDim_CTRL], fxd x[nX], fxd ref[nRef]);

// void generatePollMatrix(fxd currentPoint[nDim_CTRL], data_meshExp frameIdx[nDim_CTRL], data_meshExp meshIdx[nDim_CTRL], fxd pollMatrix[nDim_CTRL][2 * nDim_CTRL]);

// void integrateSystem(data_cost costFcn[1], const fxd A[nX][nX], const fxd B[nX][nU], const fxd G[nX], fxd point[nDim_CTRL], fxd x[nX], fxd ref[nRef]);

/*
    ----------------------------------------
    Functions prototypes  
    ----------------------------------------    
*/
void computeArxOutput(output_type yRes[], const output_type yPast[na], const input_type uSamples[nb+nd], const theta_type theta[nTheta]);
void costFunctionArx(cost_type cost[2], const output_type yPast[na], const input_type currU[nOpt], const input_type uPast[nb + nd - 1], const output_type yref[ny], hzn_type predictionHzn, hzn_type controlHzn, const theta_type thetaScenarios[Nscen + 1][nTheta]);
void generatePollDirectionsArx(const rand_type randomVector[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], direction_type directions[nOpt][2 * nOpt]);
void generatePollMatrixArx(const input_type currU[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], input_type pollMatrix[nOpt][2 * nOpt]);
void generateScenarios(theta_type thetaScenarios[Nscen + 1][nTheta]);
void generateSCMPCControl(input_type uOpt[NhorU], theta_type thetaScenarios[Nscen+1][nTheta], const output_type yInit[na], const input_type uInit[nb+nd-1], const output_type yref[ny], hzn_type predictionHzn, hzn_type controlHzn);
void MADSARX(input_type uOpt[nOpt], const input_type uInit[nb+nd-1], const output_type yInit[na], const output_type yref[ny], const theta_type thetaScenarios[Nscen][nTheta], hzn_type predictionHzn, hzn_type controlHzn);
void progressiveBarrierPollingArx(cost_type bestCost[2], input_type bestPoint[nOpt], const output_type yPast[na], const input_type uPast[nb + nd - 1], const output_type yref[ny], hzn_type predictionHzn, hzn_type controlHzn, const input_type pollMatrix[nOpt][2 * nOpt], mesh_exp_type frameSize[nOpt], const theta_type thetaScenarios[Nscen + 1][nTheta]);
void pseudoRandArx(rand_type randomVector[nOpt]);
void updateConstraintViolation(cost_type cost[2], const output_type currY[]);
