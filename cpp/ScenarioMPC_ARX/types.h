/**
 * @file  types.h
 * @brief All data-type definitions for SCMPC on an ARX system.
 *
 * Two compilation targets are supported:
 *
 *  Software (FIXED not defined)
 *    Standard C++ types (int, double, ...) for PC-side simulation and
 *    debugging.  Using double keeps quantisation errors negligible so
 *    that software vs. hardware differences can be attributed solely
 *    to fixed-point effects.
 *
 *  Hardware (FIXED defined)
 *    Xilinx ap_fixed<> types for Vitis HLS 2021.1 synthesis.
 *    Bit-widths match the original design; revise if signal ranges change.
 *
 * RULE: every other file must use the semantic aliases at the bottom
 * (output_type, input_type, theta_type, ...) rather than raw types like
 * double or ap_fixed<18,5>.  This confines all type changes to this file
 * and makes the intent of every variable unambiguous at the call site.
 *
 * Do NOT put constants or function prototypes here (setup.h).
 */

#pragma once

/* ======================================================================
   Feature flags - defined externally; do NOT edit here.
   ======================================================================
   FIXED            : ap_fixed<> types   -> Vitis HLS synthesis target
   PRNG_STDLIB      : use rand() for prng (software builds only)
   CONVERSIONS_MODE : compile ADC/DAC conversion functions (SW only)
   Mutual-exclusion logic is enforced in setup.h.
   ====================================================================== */

#ifdef FIXED
/* ---------------------------------------------------------------------- */
/*  Vitis HLS fixed-point types                                           */
/* ---------------------------------------------------------------------- */
#include <ap_fixed.h>

/** System-dependent data type (based on I/O dynamic range)
 * Taken care of by MATLAB?
 */
#if ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS || ACTIVE_SYSTEM == SYSTEM_BUCK
typedef ap_ufixed<8,4> output_type;
typedef ap_ufixed<8,1> input_type;
   #ifndef NRMLZ
   typedef ap_fixed<18,5> theta_type; // System-dependent theta dynamic range
      /* Coefficients types for conversions functions */
      #ifdef CONVERSIONS_MODE
      /* WIP */
      typedef ap_ufixed<10,9> output_adc_coeff_type; // positive gain & needs 9 bits for 409.5
      typedef ap_ufixed<12,12> input_adc_coeff_type; // positive gain & needs 12 bits for 4095.
      typedef ap_ufixed<20,0> output_dac_coeff_type; // 2.442442...e-3
      typedef ap_ufixed<12,0> input_dac_coeff_type; // 2.442442...e-4
      typedef output_dac_coeff_type dig2ctrl_type; // corresponds to YDACGain
      typedef input_adc_coeff_type ctrl2dig_type; // corresponds to UADCGain
      #else
      /* WIP */
      typedef ap_ufixed<1,1> dig2ctrl_type; // 1
      typedef ap_ufixed<1,1> ctrl2dig_type; // 0
      #endif
   #else
   typedef ap_fixed<18,2, AP_SAT> theta_type;
   typedef ap_ufixed<23,3> strip_coeff_type;
   typedef ap_ufixed<23,3> strip_q_coeff_type;
   typedef ap_fixed<22,3> strip_coeff_c0_type;
   typedef ap_ufixed<21,0> strip_q_coeff_c0_type;
   /* y/u_norm_coeff_type depend on system's constraints i.e. on the specific system */
   typedef ap_fixed<16,0> y_norm_coeff_type; // 0.2 = 0.00110011...
   typedef ap_fixed<2,2> u_norm_coeff_type;  // 2 = 10.0...
   typedef ap_fixed<1,1> u_norm_inv_coeff_type;  // 0.5 = 0.10...
      /* Coefficients types for conversions functions */
      #ifdef CONVERSIONS_MODE
      /* output_adc_coeff_type, input_adc_coeff_type same as before */
      /* output_dac_coeff_type, input_dac_coeff_type same as before */
      typedef ap_ufixed<10,9> output_adc_coeff_type; // 9 bits for 409.5
      typedef ap_ufixed<12,12> input_adc_coeff_type; // 12 bits for 4095.
      typedef ap_ufixed<20,0> output_dac_coeff_type; // 2.442442...e-3
      typedef ap_ufixed<12,0> input_dac_coeff_type; // 2.442442...e-4
      typedef ap_ufixed<11,0,AP_RND_CONV,AP_SAT> dig2ctrl_type; // 4.884884...e-4
      typedef ap_ufixed<12,11,AP_RND_CONV,AP_SAT> ctrl2dig_type; // 2047.5
      #else
      /* WIP */
      typedef y_norm_coeff_type dig2ctrl_type;
      typedef u_norm_inv_coeff_type ctrl2dig_type;
      #endif
   #endif 
#else
#error "Unrecognized ACTIVE_SYSTEM."
#endif

/** Cost accumulator; wide enough to prevent saturation over the horizon. */
typedef ap_ufixed<32,  9, AP_RND_CONV, AP_SAT>  cost_type;

/** Random numbers and direction-vector coefficients for the MADS poll step. */
typedef ap_fixed<18,2,AP_TRN,AP_SAT>  rand_type;

/** Mesh-point coordinates (same range as input_type, unsigned). */
typedef ap_fixed<36, 12, AP_TRN,      AP_WRAP>  mesh_type;

/** Signed log2 frame-size exponent (small integer). */
typedef ap_int<6>                                 mesh_exp_type;

/** Elements of the poll-direction matrix D (values in {-1, 0, +1}). */
typedef ap_int<12>                                direction_type;

/**
 * Primary algorithmic type.
 * Format: ap_fixed<18, 5> - 18 total bits, 5 integer bits.
 * Range ~ [-16, +16), resolution ~ 7.6 × 10⁻⁵.
 * Increase the integer-bit count if signal ranges exceed +/-16.
 */
typedef ap_fixed <18,  5>                         alg_type;

/** Fractional type used inside the pseudo-random number generator. */
typedef ap_ufixed<16,0>                         frac_type;

/** Unsigned 32-bit helper for intermediate products in pseudorand. */
// NOTE: Vitis may want 32 bits to compute shift and mask correctly
// typedef ap_ufixed<32,32> u16_type;
typedef ap_uint<16>                         u16_type;

/* Data type for Q, P = 4 weight matrices */
typedef ap_ufixed<2,2> output_weight_type;
#else
/* ---------------------------------------------------------------------- */
/*  Floating point types for PC simulation / debugging                          */
/* ---------------------------------------------------------------------- */
typedef double       y_norm_coeff_type;
typedef double       u_norm_coeff_type;
typedef double       u_norm_inv_coeff_type;
typedef double       output_adc_coeff_type;
typedef double       input_adc_coeff_type;
typedef double       output_dac_coeff_type;
typedef double       input_dac_coeff_type;
typedef float        input_weight_type;
typedef float        output_weight_type;
typedef double       strip_coeff_type;
typedef double       strip_q_coeff_type;
typedef double       strip_coeff_c0_type;
typedef double       strip_q_coeff_c0_type;
typedef double       cost_type;
typedef double       rand_type;
typedef double       mesh_type;
typedef int          mesh_exp_type;
typedef int          direction_type;
typedef double       dig2ctrl_type;
typedef double       ctrl2dig_type;
typedef double       phi_type;
typedef double       strip_center_type;
typedef double       alg_type;
typedef double       output_type;
typedef double       input_type;
typedef double       theta_type;
typedef double       frac_type;
typedef unsigned int u16_type;
#endif  /* FIXED */

/* ======================================================================
   Semantic aliases.
   All resolve to alg_type; the distinct names document the ROLE of each
   variable at every call site.  Use these everywhere - never use
   alg_type, double, or ap_fixed<> directly outside this file.
   ====================================================================== */
typedef alg_type  err_type;      /**< Tracking error  e(k) = y(k) - y_ref. */
typedef alg_type  norm_conv_type;

/* Only derives digital_input_type and digital_output_type */
#ifdef CONVERSIONS_MODE
#ifdef FIXED
/** Raw 12-bit ADC/DAC sample, integer range {0, ..., 4095}. */
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT>  digital_input_type;
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT>  digital_output_type;
#else
typedef int          digital_input_type;   /**< ADC raw integer {0,...,4095} */
typedef int          digital_output_type;
#endif
#else // they're the same because no conversion is done
typedef input_type   digital_input_type;
typedef output_type  digital_output_type;
#endif

/* Only derives norm_input_type and norm_output_type */
#ifdef NRMLZ
#ifdef FIXED
/** Normalized quantities types */
typedef ap_fixed<18,2,AP_RND_CONV,AP_SAT> norm_output_type;
typedef ap_fixed<18,2,AP_RND_CONV,AP_SAT> norm_input_type;
typedef ap_fixed<18,4,AP_TRN,AP_SAT> phi_type;
typedef ap_fixed<18,2,AP_SAT> strip_center_type;
typedef ap_ufixed<3,0> input_weight_type; /* R = 0.125 = 2^(-3) */
#else
typedef double norm_output_type;
typedef double norm_input_type;
#endif
#else
typedef output_type norm_output_type;
typedef input_type norm_input_type;
typedef output_type phi_type; // output_type likely "larger" than input_type
typedef output_type strip_center_type; // output_type likely "larger" than input_type
#ifdef FIXED
typedef ap_ufixed<5,4> input_weight_type; /* R = RBaseline = 12.5 */
#endif
#endif

typedef norm_output_type  err_type;      /**< Tracking error  e(k) = y(k) - y_ref. */