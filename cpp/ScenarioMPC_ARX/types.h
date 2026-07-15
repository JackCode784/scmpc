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
      typedef ap_ufixed<28,4,AP_RND_CONV,AP_SAT> dig2ctrl_type;
      typedef ap_ufixed<28,24,AP_RND_CONV,AP_SAT> ctrl2dig_type;
      #else
      /* WIP */
      typedef ap_ufixed<28,4,AP_RND_CONV,AP_SAT> dig2ctrl_type;
      typedef ap_ufixed<28,24,AP_RND_CONV,AP_SAT> ctrl2dig_type;
      #endif
   #else
   typedef ap_fixed<18,2, AP_SAT> theta_type;
      /* Coefficients types for conversions functions */
      #ifdef CONVERSIONS_MODE
      typedef ap_ufixed<28,4,AP_RND_CONV,AP_SAT> dig2ctrl_type;
      typedef ap_ufixed<28,24,AP_RND_CONV,AP_SAT> ctrl2dig_type;
      #else
      /* WIP */
      typedef ap_ufixed<28,4,AP_RND_CONV,AP_SAT> dig2ctrl_type;
      typedef ap_ufixed<28,24,AP_RND_CONV,AP_SAT> ctrl2dig_type;
      #endif
   #endif 
#else
#error "Unrecognized ACTIVE_SYSTEM."
#endif

/** Cost accumulator; wide enough to prevent saturation over the horizon. */
typedef ap_ufixed<32,  9, AP_RND_CONV, AP_SAT>  cost_type;

/** Random numbers and direction-vector coefficients for the MADS poll step. */
typedef ap_fixed <18,  3, AP_TRN,      AP_WRAP>  rand_type;

/** Mesh-point coordinates (same range as input_type, unsigned). */
typedef ap_fixed<36, 12, AP_TRN,      AP_WRAP>  mesh_type;

/** Signed log2 frame-size exponent (small integer). */
typedef ap_int<6>                                 mesh_exp_type;

/** Elements of the poll-direction matrix D (values in {-1, 0, +1}). */
typedef ap_int<12>                                direction_type;

/** Intermediate type for ADC/DAC conversion arithmetic. */
typedef ap_ufixed<32, 14, AP_RND_CONV, AP_SAT>   conv_type;

/**
 * Primary algorithmic type.
 * Format: ap_fixed<18, 5> - 18 total bits, 5 integer bits.
 * Range ~ [-16, +16), resolution ~ 7.6 × 10⁻⁵.
 * Increase the integer-bit count if signal ranges exceed +/-16.
 */
typedef ap_fixed <18,  5>                         alg_type;

/** Fractional type used inside the pseudo-random number generator. */
typedef ap_ufixed<18,  2>                         frac_type;

/** Unsigned 32-bit helper for intermediate products in pseudorand. */
typedef ap_ufixed<32, 16>                         u16_type;

#else
/* ---------------------------------------------------------------------- */
/*  Floating point types for PC simulation / debugging                          */
/* ---------------------------------------------------------------------- */
typedef double       cost_type;
typedef double       rand_type;
typedef double       mesh_type;
typedef int          mesh_exp_type;
typedef int          direction_type;
typedef double       conv_type;
typedef double       dig2ctrl_type;
typedef double       ctrl2dig_type;
typedef double       phi_type;
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
typedef alg_type  weights_type;  /**< MPC cost-weight matrix entry.        */
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
typedef ap_fixed<18,2, AP_SAT> norm_output_type;
typedef ap_fixed<18,2, AP_SAT> norm_input_type;
typedef ap_fixed<18,4,AP_TRN,AP_SAT> phi_type;
#else
typedef double norm_output_type;
typedef double norm_input_type;
#endif
#else
typedef output_type norm_output_type;
typedef input_type norm_input_type;
typedef output_type phi_type; // output_type likely "larger" than input_type
#endif

typedef norm_output_type  err_type;      /**< Tracking error  e(k) = y(k) - y_ref. */