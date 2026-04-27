/**
 * @file  types.h
 * @brief All data-type definitions for SCMPC on an ARX system.
 *
 * Two compilation targets are supported:
 *
 *  Software (FIXED not defined)
 *    Standard C++ types (int, double, …) for PC-side simulation and
 *    debugging.  Using double keeps quantisation errors negligible so
 *    that software vs. hardware differences can be attributed solely
 *    to fixed-point effects.
 *
 *  Hardware (FIXED defined)
 *    Xilinx ap_fixed<> types for Vitis HLS 2021.1 synthesis.
 *    Bit-widths match the original design; revise if signal ranges change.
 *
 * RULE: every other file must use the semantic aliases at the bottom
 * (output_type, input_type, theta_type, …) rather than raw types like
 * double or ap_fixed<18,5>.  This confines all type changes to this file
 * and makes the intent of every variable unambiguous at the call site.
 *
 * Do NOT put constants or function prototypes here (→ setup.h).
 * Do NOT put system-specific data here (→ system_configs.h).
 */

#pragma once

/* ======================================================================
   Feature flags — defined externally; do NOT edit here.
   ======================================================================
   FIXED            : ap_fixed<> types   → Vitis HLS synthesis target
   DEBUG_MODE       : enable printf debug output (software builds only)
   CONVERSIONS_MODE : compile ADC/DAC conversion functions (SW only)
   Mutual-exclusion logic is enforced in setup.h.
   ====================================================================== */

#ifdef FIXED
/* ---------------------------------------------------------------------- */
/*  Vitis HLS fixed-point types                                           */
/* ---------------------------------------------------------------------- */
#include <ap_fixed.h>

/** Raw 12-bit ADC/DAC sample, integer range {0, …, 4095}. */
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT>  digital_input_type;
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT>  digital_output_type;

/** Cost accumulator; wide enough to prevent saturation over the horizon. */
typedef ap_ufixed<32,  9, AP_RND_CONV, AP_SAT>  cost_type;

/** Random direction-vector coefficients for the MADS poll step. */
typedef ap_fixed <18,  3, AP_TRN,      AP_WRAP>  rand_type;

/** Mesh-point coordinates (same range as input_type, unsigned). */
typedef ap_ufixed<36, 12, AP_TRN,      AP_WRAP>  mesh_type;

/** Signed log₂ frame-size exponent (small integer). */
typedef ap_int<6>                                 mesh_exp_type;

/** Elements of the poll-direction matrix D (values in {−1, 0, +1}). */
typedef ap_int<12>                                direction_type;

/** Intermediate type for ADC/DAC conversion arithmetic. */
typedef ap_ufixed<32, 14, AP_RND_CONV, AP_SAT>   conv_type;

/**
 * Primary algorithmic type.
 * Format: ap_fixed<18, 5> — 18 total bits, 5 integer bits.
 * Range ≈ [−16, +16), resolution ≈ 7.6 × 10⁻⁵.
 * Increase the integer-bit count if signal ranges exceed ±16.
 */
typedef ap_fixed <18,  5>                         alg_type;

/** Fractional type used inside the pseudo-random number generator. */
typedef ap_ufixed<18,  2>                         frac_type;

/** Unsigned 32-bit helper for intermediate products in pseudorand. */
typedef ap_ufixed<32, 16>                         u16_type;

#else
/* ---------------------------------------------------------------------- */
/*  Software types for PC simulation / debugging                          */
/* ---------------------------------------------------------------------- */
typedef int          digital_input_type;   /**< ADC raw integer {0,…,4095} */
typedef int          digital_output_type;
typedef double       cost_type;
typedef double       rand_type;
typedef double       mesh_type;
typedef int          mesh_exp_type;
typedef int          direction_type;
typedef double       conv_type;
typedef double       alg_type;
typedef double       frac_type;
typedef unsigned int u16_type;
#endif  /* FIXED */

/* ======================================================================
   Semantic aliases.
   All resolve to alg_type; the distinct names document the ROLE of each
   variable at every call site.  Use these everywhere — never use
   alg_type, double, or ap_fixed<> directly outside this file.
   ====================================================================== */
typedef alg_type  output_type;   /**< Plant output sample  y(k).            */
typedef alg_type  input_type;    /**< Control input sample u(k).            */
typedef alg_type  weights_type;  /**< MPC cost-weight matrix entry.         */
typedef alg_type  theta_type;    /**< ARX parameter vector entry.           */
typedef alg_type  err_type;      /**< Tracking error  e(k) = y(k) − y_ref. */
