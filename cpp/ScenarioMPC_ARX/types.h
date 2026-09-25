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
/*
 * WORD_LENGTH=18 matches the FPGA's native DSP48E1 hard-multiplier
 * operand width: an ap_fixed<18,...> value fits in ONE multiplier input
 * port, so multiplying two such values maps to a single DSP48E1. A
 * wider operand (>18 bits) forces Vitis to cascade two DSP48E1s plus
 * LUT-built glue logic to combine their partial products instead - see
 * arx_partial_type's comment further down for a concrete, measured
 * example of exactly this penalty. This is a hardware fact, not a
 * per-system choice, so it stays a single project-wide constant.
 */
constexpr int WORD_LENGTH = 18;

/*
 * Automatic integer-bit-count derivation for FIXED-mode types whose
 * safe range is a closed-form function of quantities already known at
 * compile time (system I/O bounds, noise amplitude, ...) - as opposed
 * to types whose range depends on what an iterative, data-dependent
 * algorithm (e.g. the PL/AL-mode zonotope/strip-intersection math, or
 * matDet's Gaussian elimination) can produce, which would need a real
 * interval-arithmetic proof or empirical range-profiling to size
 * correctly, not a formula - those are intentionally NOT covered here
 * and remain hand-set.
 *
 * GUARD_BITS is the margin knob: raise it to add headroom (in whole
 * bits, i.e. each +1 doubles the safety margin) to every type sized via
 * neededIntBits() below, in one place.
 */
constexpr int GUARD_BITS = 0;

/**
 * Smallest integer-bit count I such that a signed ap_fixed<_,I> can
 * represent maxMagnitude (with guardBits of extra headroom) without
 * overflowing, i.e. the smallest I with 2^(I-1) > maxMagnitude.
 *
 * Deliberately NOT using std::log2/std::ceil (<cmath>): those aren't
 * reliably constexpr across compilers, and I is being computed for use
 * as an ap_fixed<> TEMPLATE PARAMETER, which must be a genuine integral
 * constant expression at the point of the typedef - a plain doubling
 * loop needs nothing but +, *, comparisons and a loop, all of which
 * have been valid in a C++14 constexpr function (this file's own
 * documented C++ standard - see setup.h's header comment) since C++14
 * relaxed the constexpr-function rules, and none of it touches
 * ap_fixed<> at all, so it evaluates identically under plain C++
 * (host/software builds) and Vitis HLS's C++ front end.
 *
 * A caller should static_assert(neededIntBits(...) < WORD_LENGTH, ...)
 * at the point of use: if the needed range ever eats the ENTIRE word,
 * that should be a loud, named compile error (a system whose dynamic
 * range doesn't fit in WORD_LENGTH bits at all), not a silently
 * zero/negative-fractional-bit type.
 */
constexpr int neededIntBits(double maxMagnitude, int guardBits = GUARD_BITS)
{
    int i = 1;
    double bound = 1.0; // 2^(i-1) for i=1
    while (bound <= maxMagnitude) { bound *= 2.0; i++; }
    return i + guardBits;
}

/** System-dependent data type (based on I/O dynamic range) */
#if ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS || ACTIVE_SYSTEM == SYSTEM_BUCK || ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO
/*
 * output_type must represent [YMINPHYS-SIGMA_UNNORM, YMAXPHYS+SIGMA_UNNORM]
 * - the actual physical signal, which can genuinely approach the
 * sensor's physical limit during a real constraint excursion, not just
 * the (possibly tighter) YMIN/YMAX control constraint. Both tails are
 * checked explicitly since YMINPHYS need not be >= 0 for every system.
 */
constexpr double outputMaxMag =
    (YMINPHYS - SIGMA_UNNORM < 0.0 ? -(YMINPHYS - SIGMA_UNNORM) : (YMINPHYS - SIGMA_UNNORM)) >
    (YMAXPHYS + SIGMA_UNNORM < 0.0 ? -(YMAXPHYS + SIGMA_UNNORM) : (YMAXPHYS + SIGMA_UNNORM))
    ? (YMINPHYS - SIGMA_UNNORM < 0.0 ? -(YMINPHYS - SIGMA_UNNORM) : (YMINPHYS - SIGMA_UNNORM))
    : (YMAXPHYS + SIGMA_UNNORM < 0.0 ? -(YMAXPHYS + SIGMA_UNNORM) : (YMAXPHYS + SIGMA_UNNORM));
constexpr int outputIntBits = neededIntBits(outputMaxMag);
static_assert(outputIntBits < WORD_LENGTH,
    "output_type's required dynamic range leaves no fractional bits within WORD_LENGTH - "
    "this system's I/O range genuinely does not fit an 18-bit fixed-point word at any useful precision.");
typedef ap_fixed<WORD_LENGTH,outputIntBits,AP_RND_CONV,AP_SAT> output_type; // [YMINPHYS-SIGMA_UNNORM, YMAXPHYS+SIGMA_UNNORM], integer bits derived automatically
typedef ap_ufixed<WORD_LENGTH,1,AP_RND_CONV,AP_SAT> input_type; // [UMIN, UMAX] = [0, 1]
typedef ap_fixed<WORD_LENGTH,0> noise_type; // [-SIGMA_UNNORM, SIGMA_UNNORM]
   #ifndef NRMLZ
   /* No normalization */
   typedef ap_fixed<WORD_LENGTH,5,AP_RND_CONV,AP_SAT> theta_type; // System-dependent theta dynamic range
   typedef ap_fixed<WORD_LENGTH,5> output_strip_offset_type; // System-dependent output strip offset [SIGMA_UNNORM - YMAX, SIGMA_UNNORM+YMAX]=[-9.98, 10.02]
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
   typedef ap_fixed<WORD_LENGTH,4,AP_RND_CONV,AP_SAT> output_strip_offset_type; // [-my*SIGMA_UNNORM-YNORMMAX,my*SIGMA_UNNORM+YNORMMAX]
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
#else
#error "Unrecognized ACTIVE_SYSTEM."
#endif

/** Cost accumulator; wide enough to prevent saturation over the horizon. */
typedef ap_ufixed<32,  9, AP_RND_CONV, AP_SAT>  cost_type;

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
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> norm_output_type;
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> norm_input_type;
typedef ap_fixed<WORD_LENGTH,1,AP_TRN,AP_SAT> phi_type;
typedef ap_fixed<WORD_LENGTH,2,AP_RND_CONV,AP_SAT> strip_center_type;
typedef ap_ufixed<3,0> input_weight_type; /* R = 0.125 = 2^(-3) */

/**
 * computeArxOutput.cpp's NRMLZ branch chains two multiplies:
 * ((sample) * theta[i]) * myInvDmDg[i]. With no intermediate type, C++/
 * ap_fixed give (sample * theta[i]) - a product of two ap_fixed<18,2> -
 * its lossless result width: 18+18=36 total bits, 2+2=4 integer bits.
 * A 36-bit operand does not fit a single DSP48E1's native 25x18 signed
 * multiplier, so Vitis cascades two DSP48s plus LUT-built glue to
 * combine their partial products - confirmed in costFunctionArx's own
 * synthesis report as 24 instances of a "mul_36s_..." pattern, each
 * costing 2 DSP + 85 LUT (48 DSP / 2040 LUT total), the dominant single
 * contributor to CTRL_MODE_SCMPC's PRAGMA_PROFILE_LATENCY exceeding
 * 100% LUT utilisation by 414 LUT (see costFunctionArx.cpp).
 *
 * arx_partial_type is that intermediate's EXPLICIT type, sized to break
 * the chain into two single-DSP multiplies instead of one 2-DSP one:
 *   - 4 integer bits, matching the lossless requirement above, so the
 *     first multiply cannot silently overflow before AP_SAT can act.
 *   - 20 fractional bits: 4 more than norm_output_type's 16, enough
 *     guard precision that rounding here should not be distinguishable
 *     from rounding at the FINAL cast this expression already performs
 *     (see below) - not a precision concession, since the 36-bit
 *     intermediate's "extra" bits beyond what norm_output_type can hold
 *     are already discarded by that existing final cast; this only
 *     moves WHERE they get discarded, from after the second multiply to
 *     before it.
 * AP_RND_CONV+AP_SAT match every other rounding-sensitive type in this
 * file, rather than introducing a different convention here.
 */
typedef ap_fixed<24,4,AP_RND_CONV,AP_SAT> arx_partial_type;
#else
typedef double norm_noise_type;
typedef double norm_output_type;
typedef double norm_input_type;
typedef double arx_partial_type;
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