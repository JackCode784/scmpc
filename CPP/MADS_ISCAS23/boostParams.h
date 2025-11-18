#ifndef _BOOSTPARAMS_HPP_
#define _BOOSTPARAMS_HPP_

// PARAMETERS AND FUNCTIONS DECLARED IN THIS HEADER DEPEND ON THE APPLICATION

// DATA TYPES:

// library for fixed point number representation
#include <ap_fixed.h>

typedef ap_fixed<12, 7, AP_TRN, AP_WRAP> data_in;
typedef ap_fixed<12, 2, AP_TRN, AP_WRAP> data_out;
typedef ap_fixed<18, 7, AP_TRN, AP_WRAP> data_x;
typedef ap_fixed<18, 2, AP_TRN, AP_WRAP> data_u;
typedef ap_ufixed<36, 12, AP_TRN, AP_WRAP> data_meshSize;
typedef ap_int<6> data_meshExp;
typedef ap_fixed<18, 3, AP_TRN, AP_WRAP> data_rand;
typedef ap_ufixed<32, 9, AP_TRN, AP_WRAP> data_cost;
typedef ap_int<12> data_dir;
typedef ap_fixed<18, 3, AP_TRN, AP_WRAP> data_mat;


// MPC PARAMETERS:

// number of state variables
// [inductor current; output voltage]
#define NX 2

// number of input variables
// PWM duty cycle
#define NU 1

// number of system parameters
// [input voltage; load resistance]
#define NP 2

// number of reference inputs
// reference output voltage
#define NREF 1

// state element to track
// second state element --> 1
#define REFIDX 1

// prediction horizon
#define HORP 2

// control horizon
#define HORC 2

// number of optimization variables (NU*HORC)
#define N 2

#define C 1
static const ap_ufixed<1, 0, AP_TRN, AP_WRAP> expC = 0.5; //2^-c

// MPC state weights
static const ap_ufixed<3, 3, AP_TRN, AP_WRAP> Q[NX][NX] = { { 5, 0 }, { 0, 1 } };

// MPC final state weights
//static const data_in P00 = 4.2135;
//static const data_in P01_10 = 3.0248;
//static const data_in P11 = 1.7147;

// MPC input weight
static const ap_ufixed<13, 0, AP_TRN, AP_WRAP> R = 0.1;


// MADS PARAMETERS:

// number of MADS iterations
#define K 7

// initial frame size
static const data_meshExp D[N] = { -8, -8 };

// frame size update
// successful iteration --> frameSize = frameSize / (2*TAU)
// unsuccessful iteration --> frameSize = frameSize * (2*TAU)
// TAU must be a power of 2 for ease of implementation
#define TAU 1

// MADS starting point
static const data_u startingPoint[N] = { 0.5, 0.5 };


// BOOST CONVERTER PARAMETERS:

// capacitance
//#define C 20e-6f
static const ap_fixed<17, 17, AP_TRN, AP_WRAP> invC = 5e4;

// inductance
//#define L 100e-6f
static const ap_fixed<15, 15, AP_TRN, AP_WRAP> invL = 1e4;

// PWM period (1/f, with f = 50 kHz)
static const ap_ufixed<30, 0, AP_TRN, AP_WRAP> T  = 2e-5;

// duty cycle constraints
static const data_out DMIN = 0;
static const data_out DMAX = 1;

// current constraints
//static const data_in IMIN = 0;
//static const data_in IMAX = 15;

// BOOST CONVERTER DYNAMICS:
// x = [iL; vOut]
// switch ON  -->  dx/dt = A0 x + b
// switch OFF -->  dx/dt = A1 x + b
// constant matrices elements
//static data_mat A0[NX][NX] = { { 0, 0 }, { 0, 0 } };
//static data_mat A1[NX][NX] = { { 0, -invL }, { invC, 0 } };
//static data_mat b[NX] = { 0, 0 };

// BOOST CONVERTER DYNAMICS (discrete time):
// switching system --> during the k-th time step,
// iL oscillates between iL_(min,k) and iL_(max,k),
// vOut oscillates between vOut_(min,k) and vOut_(max,k)
// x_k = [iL_(min,k); vOut_(max,k)]
// x_(k+1/2) = [iL_(max,k); vOut_(min,k)]
// switch ON  -->  x_(k+1/2) = Ad0 x_k + bd0
// switch OFF -->  x_(k+1) = Ad1 x_(k+1/2) + bd1
// constant matrices elements
//static data_mat Ad0[NX][NX] = { { 1, 0 }, { 0, 0 } };
//static data_mat Ad1[NX][NX] = { { 0, 0 }, { 0, 0 } };
//static data_mat bd0[NX] = { 0, 0 };
//static data_mat bd1[NX] = { 0, 0 };


// FUNCTION DEFINITIONS

void costFunction(data_cost cost[2], data_u point[N], data_x x[NX], data_x par[NP], data_x ref[NREF]);


#endif
