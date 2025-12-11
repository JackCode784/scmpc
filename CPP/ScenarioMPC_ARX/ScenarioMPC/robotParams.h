#ifndef _ROBOTPARAMS_HPP_
#define _ROBOTPARAMS_HPP_

// PARAMETERS AND FUNCTIONS DECLARED IN THIS HEADER DEPEND ON THE APPLICATION

// DATA TYPES:

// library for fixed point number representation
#include <ap_fixed.h>

//typedef float data_in;
//typedef float data_out;
//typedef float data_x;
//typedef float data_u;
//typedef float data_meshSize;
//typedef int data_meshExp;
//typedef float data_rand;
//typedef float data_cost;
//typedef int data_dir;
typedef ap_fixed<12, 7, AP_TRN, AP_WRAP> data_in;
typedef ap_ufixed<12, 0, AP_TRN, AP_WRAP> data_out;
typedef ap_fixed<18, 7, AP_TRN, AP_WRAP> data_x;
typedef ap_ufixed<24, 0, AP_TRN, AP_WRAP> data_u;
typedef ap_ufixed<32, 14, AP_TRN, AP_WRAP> data_meshSize;
typedef ap_int<6> data_meshExp;
typedef ap_fixed<18, 3, AP_TRN, AP_WRAP> data_rand;
typedef ap_fixed<32, 12, AP_TRN, AP_WRAP> data_cost;
typedef ap_int<12> data_dir;
typedef ap_fixed<18, 3, AP_TRN, AP_WRAP> data_mat;


// MPC PARAMETERS:

// number of state variables
#define NX 2

// number of input variables
#define NU 1

// number of system parameters
#define NP 2

// number of reference inputs
#define NREF 1

// prediction horizon
#define HORP 5

// control horizon
#define HORC 2

// number of optimization variables (NU*HORC)
#define N 2

// MPC state weights
//static const ap_ufixed<18, 18, AP_TRN, AP_WRAP> Q[NX][NX] = { { 180, 0, 0 }, { 0, 180, 0 }, { 0, 0, 0 } };

// MPC input weight
//static const ap_ufixed<32, 0, AP_TRN, AP_WRAP> R[NU][NU] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
//static const float R[NU][NU] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };


// MADS PARAMETERS:

// number of MADS iterations
#define K 10

// initial frame size
//static const data_mesh D = 0.1;
static const data_meshExp D[N] = { -1, -1, -6 };

// frame size update
// successful iteration --> frameSize = frameSize / (2*TAU)
// unsuccessful iteration --> frameSize = frameSize * (2*TAU)
// TAU must be a power of 2 for ease of implementation
#define TAU 1

// MADS starting point
static const data_u startingPoint[N] = { 0, 0, 0 };


// ROBOT PARAMETERS:

// T
//static const float T  = 0.01;

// T*r/2
static const ap_ufixed<26, 0, AP_TRN, AP_WRAP> par1 = 0.00045;
//static const float par1 = 0.00045;

// T*r/(2*b)
static const ap_ufixed<13, 0, AP_TRN, AP_WRAP> par2 = 0.0018;
//static const float par2 = 0.0018;

// duty cycle constraints
static const data_u U1MIN = 0;
static const data_u U1MAX = 20;
static const data_u U2MIN = 0;
static const data_u U2MAX = 20;
static const data_u U3MIN = -0.4;
static const data_u U3MAX = 0.4;


// FUNCTION DEFINITIONS

void costFunction(data_cost cost[2], data_u point[N], data_x x[NX], data_x par[NP], data_x ref[NREF]);


#endif
