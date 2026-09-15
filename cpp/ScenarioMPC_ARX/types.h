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
typedef ap_fixed<18,5,AP_RND_CONV,AP_SAT> output_type; // [YMIN-SIGMA_UNNORM, YMAX+SIGMA_UNNORM] = [-0.02, 10.02]
typedef ap_ufixed<18,1,AP_RND_CONV,AP_SAT> input_type; // [UMIN, UMAX] = [0, 1]
typedef ap_fixed<18,0> noise_type; // [-SIGMA_UNNORM, SIGMA_UNNORM]
   #ifndef NRMLZ
   /* No normalization */
   typedef ap_fixed<18,5,AP_RND_CONV,AP_SAT> theta_type; // System-dependent theta dynamic range
   typedef ap_fixed<18,5> output_strip_offset_type; // System-dependent output strip offset [SIGMA_UNNORM - YMAX, SIGMA_UNNORM+YMAX]=[-9.98, 10.02]
   typedef ap_fixed<18,9,AP_RND_CONV,AP_SAT> proj_type; // |cproj| <= nTheta*..., same for |gproj|
   typedef ap_fixed<18,9,AP_RND_CONV,AP_SAT> proj_inv_type; // inverse of proj_type
   typedef ap_fixed<18,9,AP_RND_CONV,AP_SAT> support_strip_offset_type; // sum of nGen + 1 proj_type variables
   typedef ap_fixed<18,9> tight_strip_center_type; // difference of tight strip offset
   typedef ap_ufixed<18,9> vol_type; // could be anything without normalization
   typedef ap_fixed<18,10> det_type; // could be anything without normalization
   typedef ap_fixed<18,9,AP_RND_CONV,AP_SAT> elim_type; // possibly almost-singular matrix, factor variable, pivotAbs in matDet
      /* Coefficients types for conversions functions */
      #ifdef CONVERSIONS_MODE
      /* No normalization, yes ADC/DAC conversions */
      typedef output_dac_coeff_type dig2ctrl_type; // corresponds to YDACGain
      typedef input_adc_coeff_type ctrl2dig_type; // corresponds to UADCGain
      #else
      /* No normalization, no ADC/DAC conversions */
      typedef ap_ufixed<1,1> dig2ctrl_type; // 1
      typedef ap_ufixed<1,1> ctrl2dig_type; // 0
      #endif
   #else
   typedef ap_fixed<18,2,AP_RND_CONV,AP_SAT> theta_type;
   /* MATLAB-computed offline normalization constants */
   typedef ap_ufixed<23,3> strip_coeff_type;       // myInvDmDg
   typedef ap_ufixed<23,3> strip_q_coeff_type;     // qmyInvDmDg
   typedef ap_fixed<22,3> strip_coeff_c0_type;     // myInvDmc0
   typedef ap_ufixed<21,0> strip_q_coeff_c0_type;  // qmyInvDmc0

   typedef ap_fixed<18,-2> norm_noise_type; // [-my*SIGMA_UNNORM, my*SIGMA_UNNORM]
   typedef ap_fixed<18,4,AP_RND_CONV,AP_SAT> output_strip_offset_type; // [-my*SIGMA_UNNORM-YNORMMAX,my*SIGMA_UNNORM+YNORMMAX]
   typedef ap_fixed<18,2,AP_RND_CONV,AP_SAT> proj_type; // |cproj| <= nTheta*0.44, |gproj| <=  
   typedef ap_fixed<18,11,AP_RND_CONV,AP_SAT> proj_inv_type; // inverse of proj_type
   typedef ap_fixed<18,9,AP_RND_CONV,AP_SAT> support_strip_offset_type; // sum of nGen + 1 proj_type variables
   typedef ap_fixed<19,10,AP_RND_CONV,AP_SAT> tight_strip_center_type; // difference of tight strip offset
   typedef ap_ufixed<18,3+1,AP_RND_CONV,AP_SAT> vol_type; // with normalization, it surely is smaller than 2^nTheta (max possible initial zonotope volume)
   typedef ap_fixed<18,3+2,AP_RND_CONV,AP_SAT> det_type; // with normalization, it can be proven that det is in [-2^(nTheta-1),2^(nTheta-1)]
   typedef ap_fixed<18,9,AP_RND_CONV,AP_SAT> elim_type; // possibly almost-singular matrix, factor variable, pivotAbs in matDet
   /* y/u_norm_coeff_type depend on system's constraints i.e. on the specific system */
   typedef ap_ufixed<16,0> y_norm_coeff_type; // 0.2 = 0.00110011...
   typedef ap_ufixed<2,2> u_norm_coeff_type;  // 2 = 10.0...
   typedef ap_ufixed<1,0> u_norm_inv_coeff_type;  // 0.5 = 0.10...
      #ifdef CONVERSIONS_MODE
      /* Yes normalization & ADC/DAC conversions */
      typedef ap_ufixed<11,-9,AP_RND_CONV,AP_SAT> dig2ctrl_type; // 4.884884...e-4
      typedef ap_ufixed<12,11,AP_RND_CONV,AP_SAT> ctrl2dig_type; // 2047.5
      #else
      /* Yes normalization, no ADC/DAC conversions */
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
// typedef ap_fixed<36, 12, AP_TRN,      AP_WRAP>  mesh_type;

/** Signed log2 frame-size exponent (small integer). */
typedef ap_int<6>                                 mesh_exp_type;

/** Elements of the poll-direction matrix (integer values). */
typedef ap_fixed<24,13+1,AP_RND_CONV,AP_SAT> direction_type;

/* Householder matrix entries data types in [-4,3] */
typedef ap_fixed<18,3,AP_TRN,AP_SAT> householder_type;

/*
 * Primary algorithmic type.
 * Increase the integer-bit count if signal ranges exceed +/-16.
 */
// typedef ap_fixed <18,  5>                         alg_type;

/** Fractional type used inside the pseudo-random number generator. */
typedef ap_ufixed<16,0>                         frac_type;

/** Unsigned 32-bit helper for intermediate products in pseudorand. */
// NOTE: Vitis may want 32 bits to compute shift and mask correctly
// typedef ap_ufixed<32,32> u16_type;
typedef ap_uint<16>                         u16_type;

/* Data type for outputWeight, terminalOutputWeight = 4 weight matrices */
typedef ap_ufixed<3,3> output_weight_type;
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
typedef double       noise_type;
typedef double       output_strip_offset_type;
typedef double       proj_type;
typedef double       support_strip_offset_type;
typedef double       tight_strip_center_type;
typedef double       tight_strip_radius_type;
typedef double       vol_type;
typedef double       det_type;
typedef double       elim_type;
typedef double       proj_inv_type;
typedef double       householder_type;
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

/* Only derives digital_input_type and digital_output_type */
#ifdef CONVERSIONS_MODE
#ifdef FIXED
/** Raw 12-bit ADC/DAC sample, integer range {0, ..., 4095}. */
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT>  digital_input_type;
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT>  digital_output_type;
/* Conversions constants */
typedef ap_ufixed<10,9> output_adc_coeff_type; // 9 bits for 409.5
typedef ap_ufixed<12,12> input_adc_coeff_type; // 12 bits for 4095.
typedef ap_ufixed<20,0> output_dac_coeff_type; // 2.442442...e-3
typedef ap_ufixed<12,-10> input_dac_coeff_type; // 2.442442...e-4
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
typedef ap_fixed<18,1,AP_TRN,AP_SAT> phi_type;
typedef ap_fixed<18,2,AP_RND_CONV,AP_SAT> strip_center_type;
typedef ap_ufixed<3,0> input_weight_type; /* R = 0.125 = 2^(-3) */
#else
typedef double norm_noise_type;
typedef double norm_output_type;
typedef double norm_input_type;
#endif
#else
typedef output_type norm_output_type;
typedef input_type norm_input_type;
typedef noise_type norm_noise_type;
typedef output_type phi_type; // output_type likely "larger" than input_type
typedef output_type strip_center_type; // output_type likely "larger" than input_type
#ifdef FIXED
typedef ap_ufixed<5,4> input_weight_type; /* R = RBaseline = 12.5 */
#endif
#endif

/* Aliases */
typedef support_strip_offset_type tight_strip_offset_type; // conservative, intersection of output and support strip
typedef tight_strip_center_type tight_strip_radius_type; // they are sum/diff of same things
typedef norm_output_type  err_type;      // Tracking error  e(k) = y(k) - y_ref