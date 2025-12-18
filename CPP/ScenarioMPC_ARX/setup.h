#define DEBUG_MODE

#ifdef DEBUG_MODE
#define CONVERSIONS_MODE
#else
// Library for fixed point number representation
#include <ap_fixed.h>
#endif

#ifndef DEBUG_MODE
/*
    ----------------------------------------
    Data types definitions  
    ----------------------------------------    
*/

typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT> digital_input_type;  // Fixed point data representation for the circuit input
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT> digital_output_type; // Fixed point data representation for the circuit output 
typedef ap_ufixed<32, 9, AP_RND_CONV, AP_SAT> cost_type;
typedef ap_fixed<18, 3, AP_TRN, AP_WRAP> rand_type;
typedef ap_ufixed<36, 12, AP_TRN, AP_WRAP> mesh_type;
typedef ap_int<6> mesh_exp_type;
typedef ap_int<12> direction_type;

// Fixed point data representation for the data to convert input
// from ADC range to actual (model range) and from actual range
// to DAC range
typedef ap_fixed<32, 14, AP_RND_CONV, AP_SAT> conv_type;
typedef ap_fixed<18, 5> alg_type; // Fixed point data representation for every signal inside the algorithm

// The following are alg_type in fixed point
typedef alg_type output_type;
typedef alg_type input_type;
typedef alg_type weights_type;
typedef alg_type theta_type;
typedef alg_type err_type;

#else
/* --------------------------------------------------------------------- */
/*  Datatypes for debugging in VS Code  */
/* --------------------------------------------------------------------- */
typedef int digital_input_type;     // datatype for digital input {0, ..., 2^12-1}
typedef int digital_output_type;    // datatype for digital output {0, ..., 2^12-1}
typedef double cost_type;   // datatype for cost function value
typedef double rand_type;   // datatype for random vectors
typedef double mesh_type;   // datatype for mesh points (same as input_type)
typedef int mesh_exp_type;  // datatype for mesh exponents
typedef int direction_type; // datatype for directions matrix

typedef float conv_type;    // WIP
typedef double alg_type;    // datatype for algorithm signals

// The following are alg_type in fixed point
typedef double output_type; // datatype for output samples
typedef double input_type;  // datatype for input samples
typedef float weights_type; // datatype for weights matrices
typedef float theta_type;   // datatype for theta ARX parameters
typedef double err_type;    // datatype for output-reference difference

// To be deleted?
// typedef short hzn_type;     // datatype for horizons values, unused
#endif

/*
    ----------------------------------------
    State-space system params  
    ----------------------------------------    
*/
// Number of system states (x_k)
// #define nx 2

// Number of system inputs (u_k)
// #define nu 1    // ARX assumption: SISO system

// Rewritten for ARX: number of system output
// #define ny 1    // ARX assumption: SISO system

// Number of system parameters (p_k)
// #define nP 0

// Number of system unmeasurable inputs (d_k)
// #define nD 0

// Number of reference outputs
// #define ny_ref 1

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
static const input_type UMIN = -0.3; // rewritten for arx
static const input_type UMAX = 0.3; // rewritten for arx

static const output_type YMIN = 0; // rewritten for arx
static const output_type YMAX = 8; // rewritten for arx

// Number of optimization variables
#define nOpt NhorU

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
#ifndef DEBUG_MODE
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
#define na 2
#define nb 1
#define nd 2
#define nTheta (na + nb)

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
static const weights_type P = 1;

// static const fxd Q[nX][nX] = {
// {1.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000} };
static const weights_type Q = 1;

// static const fxd R[nU][nU] = {
// {10.000000} };
static const weights_type R = 10;

// #ifndef DEBUG_MODE
#define ADCMax 4095
#define ADCMin 0
#define ADCRange (ADCMax - ADCMin)

#define YADCGain (ADCRange / (YMAX - YMIN))
#define YDACGain (1 / YADCGain)
#define YBias 0
#define UADCGain (ADCRange / (UMAX - UMIN))
#define UDACGain (1 / UADCGain)
#define UBias 2048
// #endif

// Default control
// static const fxd default_u[nU] = {0.000000};
// static const input_type default_u[nOpt] = {0.0};

// Arrays to transform the outputs from their circuit range
// to the actual (model) range:
// x = (x_cir - sim_x_scale_bias).*sim_x_scale_gain
// static const conv_type sim_y_scale_bias[ny] = {0};
// static const conv_type sim_y_scale_gain[ny] = {0};

// Arrays to transform the inputs from their actual (model)
// range to the actual circuit range:
// u_cir = u.*sim_u_scale_gain + sim_u_scale_bias
// static const conv_type sim_u_scale_bias[NhorU] = {-0.300000};
// static const conv_type sim_u_scale_gain[NhorU] = {6825.000000};

// Arrays to transform the reference signals from
// their circuit range to the actual (model) range:
// xref = (xref_cir - sim_xref_scale_bias).*sim_xref_scale_gain
// static const conv_type sim_yref_scale_bias[ny_ref] = {-8.000000};
// static const conv_type sim_yref_scale_gain[ny_ref] = {0.003907};

/*
    ----------------------------------------
    Functions prototypes  
    ----------------------------------------    
*/
output_type computeArxOutput(const output_type yPast[na], const input_type uSamples[nb+nd], const theta_type theta[nTheta]);
void costFunctionArx(cost_type cost[2], const output_type yPast[na], const input_type currU[nOpt], const input_type uPast[nb + nd - 1], const output_type yref, const theta_type thetaScenarios[Nscen + 1][nTheta]);
void generatePollDirectionsArx(const rand_type randomVector[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], direction_type directions[nOpt][2 * nOpt]);
void generatePollMatrixArx(const input_type currU[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], input_type pollMatrix[nOpt][2 * nOpt]);
void generateScenarios(theta_type thetaScenarios[Nscen + 1][nTheta]);
void generateSCMPCControl(input_type uOpt[NhorU], theta_type thetaScenarios[Nscen+1][nTheta], const output_type yInit[na], const input_type uInit[nb+nd-1], const output_type yref);
void MADSARX(input_type uOpt[nOpt], const input_type uInit[nb+nd-1], const output_type yInit[na], const output_type yref, const theta_type thetaScenarios[Nscen][nTheta]);
void progressiveBarrierPollingArx(cost_type bestCost[2], input_type bestPoint[nOpt], const output_type yPast[na], const input_type uPast[nb + nd - 1], const output_type yref, const input_type pollMatrix[nOpt][2 * nOpt], mesh_exp_type frameExp[nOpt], const theta_type thetaScenarios[Nscen + 1][nTheta]);
void pseudoRandArx(rand_type randomVector[nOpt]);
void updateConstraintViolation(cost_type cost[2], output_type currY);

// Conversions
digital_output_type ADConvertY(output_type yAn);
output_type DAConvertY(digital_output_type yDig);
digital_input_type ADConvertU(input_type uAn);
input_type DAConvertU(digital_input_type uDig);

// Complete function
void controller(digital_input_type uOptDig[NhorU], theta_type thetaScenarios[Nscen + 1][nTheta], const digital_output_type yInitDig[na], const digital_input_type uInitDig[nb + nd - 1], const digital_output_type yrefDig);

// void extractU(fxd z[nDim_CTRL], fxd u_reg[nU], fxd u_old[nU]);
// void shiftOpt(fxd optimum[nDim_CTRL]);
// void pseudoRand(data_rand randomVector[nDim_CTRL]);
