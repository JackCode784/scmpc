/**
 * @file  types.h
 * @brief System-independent data types for SCMPC on an ARX system.
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
 *    Xilinx ap_fixed<> types for Vitis HLS synthesis.
 *
 * WHERE EACH FIXED-POINT TYPE IS DEFINED
 *   - In the system's file (systems/system_*.h, #ifdef FIXED): every type
 *     whose range depends on the system - the physical signals
 *     (output_type, input_type, noise_type), the strip/normalization
 *     constants (parameter magnitudes), the cost accumulator (weights,
 *     horizon), the converter gains, the input weight, and every type
 *     without NRMLZ (nothing is normalized then). A system without such a
 *     block is floating point only.
 *   - Here: the types that are the same for every system - those sized by
 *     the normalization (|x| <= 1 up to small factors), the 12-bit
 *     converter codes, and the MADS internals.
 *   system_configs.h includes the system's file BEFORE this one, so the
 *   aliases at the bottom can refer to the system's types.
 *
 * RULE: every other file must use the semantic aliases (output_type,
 * input_type, theta_type, ...) rather than raw types like double or
 * ap_fixed<18,5>.  This keeps all type choices in the type files and
 * makes the intent of every variable unambiguous at the call site.
 *
 * Do NOT put constants or function prototypes here (setup.h).
 */

#pragma once

/* ======================================================================
   Feature flags - defined externally (setup.h); do NOT edit here.
   ======================================================================
   FIXED            : ap_fixed<> types   -> Vitis HLS synthesis target
   NRMLZ            : normalized controller variables
   CONVERSIONS_MODE : 12-bit ADC/DAC codes at the controller's ports
   ====================================================================== */

#ifdef FIXED
/* ---------------------------------------------------------------------- */
/*  Vitis HLS fixed-point types, the same for every system                */
/*  (<ap_fixed.h> and WORD_LENGTH: system_configs.h)                      */
/* ---------------------------------------------------------------------- */

/** Random numbers and direction-vector coefficients for the MADS poll step. */
typedef ap_fixed<WORD_LENGTH,2,AP_TRN,AP_SAT>  rand_type;

/** Signed log2 frame-size exponent (small integer). */
typedef ap_int<6>                                 mesh_exp_type;

/** Elements of the poll-direction matrix (integer values). */
typedef ap_fixed<24,13+1,AP_RND_CONV,AP_SAT> direction_type;

/* Householder matrix entries data types in [-4,3] */
typedef ap_fixed<WORD_LENGTH,3,AP_TRN,AP_SAT> householder_type;

/** Fractional type used inside the pseudo-random number generator. */
typedef ap_ufixed<16,0>                         frac_type;

/** Unsigned 16-bit helper for intermediate products in pseudorand. */
typedef ap_uint<16>                         u16_type;

/* Data type for weightQ, weightP. FIXED applies them as
 * shifts by log2Q/log2P (costFunctionArx), so they must be powers of two.
 * 8 bits so that the value itself (e.g. 16) is held
 * and can be checked: the former ap_ufixed<3,3> wrapped 16 around to 0. */
typedef ap_ufixed<8,8> output_weight_type;

#ifdef NRMLZ
/* Normalized quantities: the same ranges for every system. */
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> theta_type;    // normalized zonotope, |theta| <= 1
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> norm_output_type;
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> norm_input_type;
typedef ap_fixed<WORD_LENGTH,-2> norm_noise_type; // [-my*SIGMA_UNNORM, my*SIGMA_UNNORM]
typedef ap_fixed<WORD_LENGTH,1,AP_TRN,AP_SAT> phi_type;
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> strip_center_type;

/* Passive learning (strip intersection, zonotope volume) */
typedef ap_fixed<WORD_LENGTH,4,AP_RND_CONV,AP_SAT> output_strip_offset_type; // [-my*SIGMA_UNNORM-YNORMMAXPHYS,my*SIGMA_UNNORM+YNORMMAXPHYS]
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> proj_type; // |cproj| <= nTheta*0.44, |gproj| <=
typedef ap_fixed<WORD_LENGTH,11,AP_RND_CONV,AP_SAT> proj_inv_type; // inverse of proj_type
typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> support_strip_offset_type; // sum of nGen + 1 proj_type variables
typedef ap_fixed<19,10,AP_RND_CONV,AP_SAT> tight_strip_center_type; // difference of tight strip offset
typedef ap_ufixed<WORD_LENGTH,3+1,AP_RND_CONV,AP_SAT> vol_type; // with normalization, it surely is smaller than 2^nTheta (max possible initial zonotope volume)
typedef ap_fixed<WORD_LENGTH,3+2,AP_RND_CONV,AP_SAT> det_type; // with normalization, it can be proven that det is in [-2^(nTheta-1),2^(nTheta-1)]
typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> elim_type; // possibly almost-singular matrix, factor variable, pivotAbs in matDet

/**
 * Effective ARX coefficients and offset in normalized I/O coordinates,
 * computed once per controller() call by computeArxCoeffs() (see
 * computeArxOutput.cpp for the derivation):
 *   coeff[i] = myInvDmDg[i]*theta[i] + myInvDmc0[i]
 *   offset   = sum_i qmyInvDmDg[i]*theta[i] + qmyInvDmc0 + yNormOffset
 * so that a prediction is just  y = offset + sum_i sample[i]*coeff[i].
 *
 * Range: |coeff[i]| <= |myInvDmc0[i]| + myInvDmDg[i]*|theta[i]|, which
 * for SYSTEM_BUCK_LOSS is at most 1.818 + 0.222*2 = 2.26 even at
 * theta_type's own +-2 limit (2.04 for |theta| <= 1, the normalized
 * zonotope's actual extent), and smaller for SYSTEM_BUCK_ALBERTO; |offset|
 * stays below 1. 3 integer bits ([-4, 4)) covers both with margin.
 * Width 25: the DSP48E1 multiplier is 25x18, so sample (18 bits) x coeff
 * (25 bits) is still ONE DSP per product, and the extra bits over 18 are
 * free precision (22 fractional bits, 6 more than norm_output_type).
 */
typedef ap_fixed<25,3,AP_RND_CONV,AP_SAT> arx_coeff_type;

/**
 * Accumulator for  offset + sum_i sample[i]*coeff[i]  in computeArxOutput.
 * Each 43-bit product (18x25) is TRUNCATED to 21 fractional bits before
 * being added - dropping low bits is free wiring, and it keeps the three
 * adders 26 bits wide instead of ~45. Worst-case truncation error is
 * nTheta*2^-21 < 1.5e-6, i.e. under 0.1 LSB of norm_output_type (2^-16);
 * the single AP_RND_CONV+AP_SAT cast to norm_output_type happens once, at
 * the end. AP_WRAP (no saturation logic) is safe here: |sample| < 2 and
 * |coeff| < 4 bound the true sum by 2*3*2.26 + 1 < 16, inside this type's
 * 5 integer bits ([-16, 16)), so no partial sum can ever wrap.
 */
typedef ap_fixed<26,5,AP_TRN,AP_WRAP> arx_acc_type;

  #ifdef CONVERSIONS_MODE
  /* Normalization + conversion products are system-independent:
   * 2/4095 (code -> normalized) and 4095/2 (normalized -> code). */
  typedef ap_ufixed<11,-9,AP_RND_CONV,AP_SAT> dig2ctrl_type; // 4.884884...e-4
  typedef ap_ufixed<12,11,AP_RND_CONV,AP_SAT> ctrl2dig_type; // 2047.5
  #endif
#endif /* NRMLZ */

#ifdef CONVERSIONS_MODE
/** Raw 12-bit ADC/DAC sample, integer range {0, ..., 4095}. */
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT>  digital_input_type;
typedef ap_ufixed<12, 12, AP_RND_CONV, AP_SAT>  digital_output_type;
#endif

#else
/* ---------------------------------------------------------------------- */
/*  Floating point types for PC simulation / debugging                    */
/* ---------------------------------------------------------------------- */
typedef double       output_adc_coeff_type;
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
typedef double       output_type;
typedef double       input_type;
typedef double       theta_type;
typedef double       frac_type;
typedef unsigned int u16_type;
#ifdef CONVERSIONS_MODE
typedef int          digital_input_type;   /**< ADC raw integer {0,...,4095} */
typedef int          digital_output_type;
#endif
#ifdef NRMLZ
typedef double norm_noise_type;
typedef double norm_output_type;
typedef double norm_input_type;
typedef double arx_coeff_type;
typedef double arx_acc_type;
#endif
#endif  /* FIXED */

/* ======================================================================
   Aliases, in every target
   ====================================================================== */
#ifndef CONVERSIONS_MODE
/* no conversion: the controller's ports carry the physical values */
typedef input_type   digital_input_type;
typedef output_type  digital_output_type;
#endif

#ifndef NRMLZ
/* no normalization: the controller works on the physical values */
typedef output_type norm_output_type;
typedef input_type norm_input_type;
typedef noise_type norm_noise_type;
typedef output_type phi_type; // output_type likely "larger" than input_type
typedef output_type strip_center_type; // output_type likely "larger" than input_type
/* Without NRMLZ the effective coefficients ARE theta and the offset is 0 */
typedef theta_type arx_coeff_type;
#endif

typedef support_strip_offset_type tight_strip_offset_type; // conservative, intersection of output and support strip
typedef tight_strip_center_type tight_strip_radius_type; // they are sum/diff of same things
typedef norm_output_type  err_type;      // Tracking error  e(k) = y(k) - y_ref
