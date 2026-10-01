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
constexpr int WORD_LENGTH = 18;

/** System-dependent data type (based on I/O dynamic range)
 * Taken care of by MATLAB?
 */
#if ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS || ACTIVE_SYSTEM == SYSTEM_BUCK || ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
/*
 * output_type holds the PHYSICAL output, so it must cover the sensor range
 * [YMINPHYS-SIGMA_UNNORM, YMAXPHYS+SIGMA_UNNORM] (the constraints YMIN/YMAX
 * lie inside it). Signed ap_fixed<W,I> covers [-2^(I-1), 2^(I-1)).
 */
#if ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
typedef ap_fixed<WORD_LENGTH,8,AP_RND_CONV,AP_SAT> output_type; // [YMINPHYS-SIGMA_UNNORM, YMAXPHYS+SIGMA_UNNORM] = [-0.2, 100.2] within [-128, 128)
#else
typedef ap_fixed<WORD_LENGTH,5,AP_RND_CONV,AP_SAT> output_type; // [YMINPHYS-SIGMA_UNNORM, YMAXPHYS+SIGMA_UNNORM] = [-0.02, 10.02] within [-16, 16)
#endif
typedef ap_ufixed<WORD_LENGTH,1,AP_RND_CONV,AP_SAT> input_type; // [UMINPHYS, UMAXPHYS] = [0, 1]
typedef ap_fixed<WORD_LENGTH,0> noise_type; // [-SIGMA_UNNORM, SIGMA_UNNORM]
   #ifndef NRMLZ
   /* No normalization */
   typedef ap_fixed<WORD_LENGTH,5,AP_RND_CONV,AP_SAT> theta_type; // System-dependent theta dynamic range
   typedef ap_fixed<WORD_LENGTH,5> output_strip_offset_type; // System-dependent output strip offset [SIGMA_UNNORM - YMAXPHYS, SIGMA_UNNORM+YMAXPHYS]=[-9.98, 10.02]
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> proj_type; // |cproj| <= nTheta*..., same for |gproj|
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> proj_inv_type; // inverse of proj_type
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> support_strip_offset_type; // sum of nGen + 1 proj_type variables
   typedef ap_fixed<WORD_LENGTH,9> tight_strip_center_type; // difference of tight strip offset
   typedef ap_ufixed<WORD_LENGTH,9> vol_type; // could be anything without normalization
   typedef ap_fixed<WORD_LENGTH,10> det_type; // could be anything without normalization
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> elim_type; // possibly almost-singular matrix, factor variable, pivotAbs in matDet
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
   typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> theta_type;
   /* MATLAB-computed offline normalization constants */
   typedef ap_ufixed<23,3> strip_coeff_type;       // myInvDmDg
   typedef ap_ufixed<23,3> strip_q_coeff_type;     // qmyInvDmDg
   typedef ap_fixed<22,3> strip_coeff_c0_type;     // myInvDmc0
   typedef ap_ufixed<21,0> strip_q_coeff_c0_type;  // qmyInvDmc0

   typedef ap_fixed<WORD_LENGTH,-2> norm_noise_type; // [-my*SIGMA_UNNORM, my*SIGMA_UNNORM]
   typedef ap_fixed<WORD_LENGTH,4,AP_RND_CONV,AP_SAT> output_strip_offset_type; // [-my*SIGMA_UNNORM-YNORMMAXPHYS,my*SIGMA_UNNORM+YNORMMAXPHYS]
   typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> proj_type; // |cproj| <= nTheta*0.44, |gproj| <=  
   typedef ap_fixed<WORD_LENGTH,11,AP_RND_CONV,AP_SAT> proj_inv_type; // inverse of proj_type
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> support_strip_offset_type; // sum of nGen + 1 proj_type variables
   typedef ap_fixed<19,10,AP_RND_CONV,AP_SAT> tight_strip_center_type; // difference of tight strip offset
   typedef ap_ufixed<WORD_LENGTH,3+1,AP_RND_CONV,AP_SAT> vol_type; // with normalization, it surely is smaller than 2^nTheta (max possible initial zonotope volume)
   typedef ap_fixed<WORD_LENGTH,3+2,AP_RND_CONV,AP_SAT> det_type; // with normalization, it can be proven that det is in [-2^(nTheta-1),2^(nTheta-1)]
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> elim_type; // possibly almost-singular matrix, factor variable, pivotAbs in matDet
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
/** Cost accumulator; wide enough to prevent saturation over the horizon. */
typedef ap_ufixed<32,  9, AP_RND_CONV, AP_SAT>  cost_type;

#elif ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
/*
 * Inverted pendulum: angle y in [YMINPHYS, YMAXPHYS] = [-0.9, 0.9] rad,
 * torque u in [UMINPHYS, UMAXPHYS] = [-3, 3]. Sized for NRMLZ (with or
 * without CONVERSIONS_MODE), CTRL_MODE_SCMPC - see the notes below.
 */
#ifndef NRMLZ
#error "SYSTEM_INVERTED_PENDULUM in FIXED needs NRMLZ: unnormalized, b = 7.5e-4 next to a1 = 2 would need ~25-bit parameters everywhere."
#endif
/* Physical angle. The sensor (ADC) only spans +-0.9 rad, but a run that
 * violates the constraints can go further, and the test bench logs it in
 * this type: +-4 rad covers a fall past +-pi. 15 fractional bits
 * (3.1e-5 rad) are finer than one ADC step (1.8/4095 = 4.4e-4 rad). */
typedef ap_fixed<WORD_LENGTH,3,AP_RND_CONV,AP_SAT> output_type;   // [-4, 4) rad
/* Physical torque: SIGNED (the buck converters' input_type is unsigned). */
typedef ap_fixed<WORD_LENGTH,3,AP_RND_CONV,AP_SAT> input_type;    // [-4, 4) for [-3, 3]
typedef ap_fixed<WORD_LENGTH,0> noise_type;                       // SIGMA_UNNORM = 0
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> theta_type;    // normalized zonotope, |theta| <= 1
/*
 * Strip/normalization constants (setup.h's computeStripCoeffs), here:
 *   myInvDmDg = [1.0e-6, 3.78e-4, 1.52e-3]    qmyInvDmDg = 0
 *   myInvDmc0 = [2, -0.998821, 2.51e-3]       qmyInvDmc0 = 0
 * (offsets are 0: the ranges are symmetric). The pendulum's uncertainty is
 * tiny next to its coefficients - a2 = -0.998821 sits only 1.2e-3 from the
 * marginally stable -1 - so these need far more fractional bits than the
 * buck converters': the buck types (20/19 fractional bits) would carry
 * myInvDmDg[1] at 0.25% and a2 at ~1e-6, i.e. ~0.1% of that margin.
 */
typedef ap_ufixed<24,-9> strip_coeff_type;      // myInvDmDg  < 2^-9 = 1.95e-3, 33 fractional bits
typedef ap_ufixed<24,-9> strip_q_coeff_type;    // qmyInvDmDg (0 here)
typedef ap_fixed<25,3> strip_coeff_c0_type;     // myInvDmc0: 2.0 needs 3 integer bits; 22 fractional bits (2.4e-7)
typedef ap_ufixed<21,0> strip_q_coeff_c0_type;  // qmyInvDmc0 (0 here)
typedef ap_fixed<WORD_LENGTH,-2> norm_noise_type;
/*
 * PL/AL-only types: copied from the buck converters so the design
 * compiles in every CTRL_MODE, but NOT sized or validated for the
 * pendulum (its normalized regressor phi = sample*myInvDmDg is ~1e-3, far
 * below what phi_type/proj_type resolve well). CTRL_MODE_SCMPC never uses
 * them.
 */
typedef ap_fixed<WORD_LENGTH,4,AP_RND_CONV,AP_SAT> output_strip_offset_type;
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> proj_type;
typedef ap_fixed<WORD_LENGTH,11,AP_RND_CONV,AP_SAT> proj_inv_type;
typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> support_strip_offset_type;
typedef ap_fixed<19,10,AP_RND_CONV,AP_SAT> tight_strip_center_type;
typedef ap_ufixed<WORD_LENGTH,3+1,AP_RND_CONV,AP_SAT> vol_type;
typedef ap_fixed<WORD_LENGTH,3+2,AP_RND_CONV,AP_SAT> det_type;
typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> elim_type;
/* Normalization gains: 2/(YMAXPHYS-YMINPHYS), 2/(UMAXPHYS-UMINPHYS) and
 * its inverse (the buck types hold 0.2, 2 and 0.5 and cannot hold these). */
typedef ap_ufixed<WORD_LENGTH,1> y_norm_coeff_type;      // yNormGain = 1.1111
typedef ap_ufixed<WORD_LENGTH,0> u_norm_coeff_type;      // uNormGain = 0.3333
typedef ap_ufixed<WORD_LENGTH,2> u_norm_inv_coeff_type;  // uNormGainInverse = 3
      #ifdef CONVERSIONS_MODE
      /* Normalization+conversion products are system-independent (see the
       * buck block): 2/4095 and 4095/2. */
      typedef ap_ufixed<11,-9,AP_RND_CONV,AP_SAT> dig2ctrl_type; // 4.884884...e-4
      typedef ap_ufixed<12,11,AP_RND_CONV,AP_SAT> ctrl2dig_type; // 2047.5
      #else
      typedef y_norm_coeff_type dig2ctrl_type;
      typedef u_norm_inv_coeff_type ctrl2dig_type;
      #endif
/* Cost accumulator: measured up to ~1600 over 20 random runs (float,
 * scenarios on; weights 16) - the buck's 9 integer bits (max 512) would
 * saturate it and make MADS candidates indistinguishable. 12 integer bits
 * (max 4096), 20 fractional. */
typedef ap_ufixed<32, 12, AP_RND_CONV, AP_SAT>  cost_type;
#else
#error "Unrecognized ACTIVE_SYSTEM."
#endif

/** Random numbers and direction-vector coefficients for the MADS poll step. */
typedef ap_fixed<WORD_LENGTH,2,AP_TRN,AP_SAT>  rand_type;

/** Mesh-point coordinates (same range as input_type, unsigned). */
// typedef ap_fixed<36, 12, AP_TRN,      AP_WRAP>  mesh_type;

/** Signed log2 frame-size exponent (small integer). */
typedef ap_int<6>                                 mesh_exp_type;

/** Elements of the poll-direction matrix (integer values). */
typedef ap_fixed<24,13+1,AP_RND_CONV,AP_SAT> direction_type;

/* Householder matrix entries data types in [-4,3] */
typedef ap_fixed<WORD_LENGTH,3,AP_TRN,AP_SAT> householder_type;

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

/* Data type for outputWeight, terminalOutputWeight = 2^log2Q, 2^log2P
 * (setup.h). FIXED applies them as shifts (costFunctionArx), so the type
 * only has to hold the values for testMain's check: 2^-8 ... 2^7. */
typedef ap_ufixed<16,8> output_weight_type;
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
#if ACTIVE_SYSTEM == SYSTEM_INVERTED_PENDULUM
/* ADC/DAC over [-0.9, 0.9] rad and [-3, 3]: the buck types overflow on
 * YADCGain = 2275 (max 511) and UDACGain = 1.47e-3 (max 9.8e-4), and
 * would round UADCGain = 682.5 to an integer (a 0.07% gain error that
 * goes straight into uConvCoeff). */
typedef ap_ufixed<16,12> output_adc_coeff_type; // YADCGain = 4095/1.8 = 2275
typedef ap_ufixed<14,10> input_adc_coeff_type;  // UADCGain = 4095/6 = 682.5 (exact)
typedef ap_ufixed<20,-11> output_dac_coeff_type; // YDACGain = 1.8/4095 = 4.396e-4 < 2^-11, 31 fractional bits
typedef ap_ufixed<20,-9> input_dac_coeff_type;   // UDACGain = 6/4095 = 1.465e-3 < 2^-9, 29 fractional bits
#else
typedef ap_ufixed<10,9> output_adc_coeff_type; // 9 bits for 409.5
typedef ap_ufixed<12,12> input_adc_coeff_type; // 12 bits for 4095.
typedef ap_ufixed<20,0> output_dac_coeff_type; // 2.442442...e-3
typedef ap_ufixed<12,-10> input_dac_coeff_type; // 2.442442...e-4
#endif
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
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> norm_output_type;
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> norm_input_type;
typedef ap_fixed<WORD_LENGTH,1,AP_TRN,AP_SAT> phi_type;
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> strip_center_type;
/* Input weight R = 2^log2R in normalized units (setup.h). FIXED applies it
 * as a shift by log2R (costFunctionArx.cpp), so this type only has to
 * hold R for DEBUG_PRINT: 2^-20 ... 2^3. */
typedef ap_ufixed<24,4> input_weight_type;

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
#else
typedef double norm_noise_type;
typedef double norm_output_type;
typedef double norm_input_type;
typedef double arx_coeff_type;
typedef double arx_acc_type;
#endif
#else
typedef output_type norm_output_type;
typedef input_type norm_input_type;
typedef noise_type norm_noise_type;
typedef output_type phi_type; // output_type likely "larger" than input_type
typedef output_type strip_center_type; // output_type likely "larger" than input_type
/* Without NRMLZ the effective coefficients ARE theta and the offset is 0 */
typedef theta_type arx_coeff_type;
#ifdef FIXED
typedef ap_ufixed<5,4> input_weight_type; /* R = RBaseline = 12.5 */
#endif
#endif

/* Aliases */
typedef support_strip_offset_type tight_strip_offset_type; // conservative, intersection of output and support strip
typedef tight_strip_center_type tight_strip_radius_type; // they are sum/diff of same things
typedef norm_output_type  err_type;      // Tracking error  e(k) = y(k) - y_ref