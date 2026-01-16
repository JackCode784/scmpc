/* setup.h */
/*
*   Everything to properly setup SCMPC for an ARX model.
*   FIXED               : fixed points are used (simulations run in Vitis HLS)
*   PL                  : use passive learning technique
*   PRAGMAS             : use pragmas in certain part of code to optimize synthesis
                          (if not, let Vitis take care of it fully automatically)
*   DEBUG_MODE          : use rand() for random values generation
*   CONVERSIONS_MODE    : use AD and DA conversion functions
*/

// (Un)comment the following lines accordingly with the intended use

// #define FIXED
#define PL

#ifdef FIXED
#define PRAGMAS
#else
#define DEBUG_MODE
#define CONVERSIONS_MODE
#endif

#ifdef FIXED
// Library for fixed point number representation
#include <ap_fixed.h>
#endif

#ifdef FIXED
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
typedef ap_ufixed<32, 14, AP_RND_CONV, AP_SAT> conv_type;
typedef ap_fixed<18, 5> alg_type;   // Fixed point data representation for every signal inside the algorithm
typedef ap_ufixed<18, 2> frac_type; // pseudorand specific type
typedef ap_ufixed<32, 16> u16_type; // pseudorand specific types

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
typedef int digital_input_type;  // datatype for digital input {0, ..., 2^12-1}
typedef int digital_output_type; // datatype for digital output {0, ..., 2^12-1}
typedef double cost_type;        // datatype for cost function value
typedef double rand_type;        // datatype for random vectors
typedef double mesh_type;        // datatype for mesh points (same as input_type)
typedef int mesh_exp_type;       // datatype for mesh exponents
typedef int direction_type;      // datatype for directions matrix

typedef float conv_type; // WIP
typedef double alg_type; // datatype for algorithm signals

// The following are alg_type in fixed point
typedef double output_type; // datatype for output samples
typedef double input_type;  // datatype for input samples
typedef float weights_type; // datatype for weights matrices
typedef float theta_type;   // datatype for theta ARX parameters
typedef double err_type;    // datatype for output-reference difference
typedef double frac_type;   // datatype for pseudorand function
typedef unsigned int u16_type;

#endif

/*
    ----------------------------------------
    MPC params
    ----------------------------------------
*/
// Prediction horizon
#define Nhor 5

// Control horizon <= prediction horizon
#define NhorU 3

// Number of scenarios (convenient if it's a power of 2 minus one)
#define Nscen 2

// Passive learning only
#ifdef PL
#define EPSILON 4.4655
#endif

// MPC saturation constraints
static const input_type UMIN = -0.3; // rewritten for arx
static const input_type UMAX = 0.3;  // rewritten for arx

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
#ifdef FIXED
static const ap_ufixed<1, 0, AP_TRN, AP_WRAP> expC = 0.5; // 2^-c
#else
static const input_type expC = 0.5;
#endif

// initial frame size
static const mesh_exp_type D[nOpt] = {-8, -8, -8};

/*
    ----------------------------------------
    ARX parameters
    ----------------------------------------
*/

#ifndef PL
#define na 2
#define nb 1
#define nd 2
#else
#define na 3
#define nb 3
#define nd 1 // nd > 0
#endif

#define nTheta (na + nb)

static output_type yInit[na] = {0};
static input_type uSamples[nb + nd - 1] = {0}; // y(k) depends on u(k-1), ..., u(k-nb-nd+1)

#ifndef PL
static theta_type thetaNominal[nTheta] = {2, -1, 1};

// Ranges for other scenarios' theta
static const theta_type thetaMax[nTheta] = {2, -1, 1.5};
static const theta_type thetaMin[nTheta] = {2, -1, 0.5};

#else
static theta_type thetaNominal[nTheta] = {0.7921, 0.1524, -0.1668, 0.0842, 0.0442, 0.0860};
static theta_type generators[nTheta][nTheta] = {
    {-0.8959, -0.4594, -0.0026, -0.0027, -0.0187, 0.0080},
    {1.3452, -0.1401, 0.0030, 0.0111, -0.0283, 0.0079},
    {-0.5326, 0.4275, -0.0051, 0.0289, -0.0362, 0.0077},
    {-0.0082, 0.0028, 0.0524, 0.0788, 0.0367, 0.0085},
    {0.0859, 0.0502, -0.1015, -0.0126, 0.0271, 0.0086},
    {0.0021, 0.1180, 0.0538, -0.0985, 0.0126, 0.0086}
};
#endif

/*
    ----------------------------------------
    Matrices for cost function evaluation
    ----------------------------------------
*/
// static const fxd P[nX][nX] = {
// {1.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000} };
static const weights_type P = 1; // 1

// static const fxd Q[nX][nX] = {
// {1.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000},
// {0.000000, 0.000000, 0.000000} };
static const weights_type Q = 1; // 1

// static const fxd R[nU][nU] = {
// {10.000000} };
static const weights_type R = 10; // 10

/*
 *   ADC and DAC converters parameters
 */

#ifndef FIXED
#define ADCMax 4095
#define ADCMin 0
#define ADCRange (ADCMax - ADCMin)

#define YADCGain (ADCRange / (YMAX - YMIN))
#define YDACGain (1 / YADCGain)
#define YBias 0
#define UADCGain (ADCRange / (UMAX - UMIN))
#define UDACGain (1 / UADCGain)
#define UBias 2048
#else
static const conv_type YADCGain = 4095 / 8;
static const conv_type YDACGain = 0.0019536019536019536019536019536;
static const conv_type YBias = 0;

static const conv_type UADCGain = 4095 / 0.6;
static const conv_type UDACGain = 0.6 / 4095;
static const conv_type UBias = 2048;
#endif

/*
    ----------------------------------------
    Functions prototypes
    ----------------------------------------
*/
output_type computeArxOutput(const output_type yPast[na], const input_type uSamples[nb+nd-1], const theta_type theta[nTheta]);
void costFunctionArx(cost_type cost[2], const output_type yPast[na], const input_type currU[nOpt], const input_type uPast[nb + nd - 2], const output_type yref, const theta_type thetaScenarios[Nscen + 1][nTheta]);
void generatePollDirectionsArx(const rand_type randomVector[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], direction_type directions[nOpt][2 * nOpt]);
void generatePollMatrixArx(const input_type currU[nOpt], const mesh_exp_type frameIdx[nOpt], const mesh_exp_type meshIdx[nOpt], input_type pollMatrix[nOpt][2 * nOpt]);
void generateScenarios(theta_type thetaScenarios[Nscen + 1][nTheta]);
void generateSCMPCControl(input_type uOpt[NhorU], theta_type thetaScenarios[Nscen + 1][nTheta], const output_type yInit[na], const input_type uInit[nb + nd - 1], const output_type yref);
void MADSARX(input_type uOpt[nOpt], const input_type uInit[nb+nd-2], const output_type yInit[na], const output_type yref, const theta_type thetaScenarios[Nscen][nTheta]);
void progressiveBarrierPollingArx(cost_type bestCost[2], input_type bestPoint[nOpt], const output_type yPast[na], const input_type uPast[nb + nd - 2], const output_type yref, const input_type pollMatrix[nOpt][2 * nOpt], mesh_exp_type frameExp[nOpt], const theta_type thetaScenarios[Nscen + 1][nTheta]);
void pseudoRandArx(rand_type randomVector[nOpt]);
void pseudoRandArx(theta_type thetaRow[nTheta], theta_type thetaRange[nTheta]);
void pseudoRandArx(rand_type coeffs[], const int nGens);
void updateConstraintViolation(cost_type cost[2], output_type currY);

// Conversions
digital_output_type ADConvertY(output_type yAn);
output_type DAConvertY(digital_output_type yDig);
digital_input_type ADConvertU(input_type uAn);
input_type DAConvertU(digital_input_type uDig);

// Complete function
void controller(digital_input_type uOptDig[NhorU], const digital_output_type yCurrDig, const digital_output_type yrefDig);

// Passive learning
#ifdef PL
void intervalHull(theta_type newCenter[nTheta], theta_type newGens[nTheta][nTheta], const theta_type oldCenter[nTheta], const theta_type oldGens[nTheta][nTheta+1]);
void boundStripZonotopeIntersection(const output_type yCurr, const output_type yPast[na], const input_type uSamples[nb + nd - 1], const theta_type oldCenter[nTheta],
                                    const theta_type oldGens[nTheta][nTheta], theta_type newCenter[nTheta], theta_type newGens[nTheta][nTheta + 1]);

#endif
