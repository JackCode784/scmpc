/* Header file for SYSTEM_BUCK and SYSTEM_BUCK_LOSS (contents: system_configs.h) */
#pragma once
#if ACTIVE_SYSTEM == SYSTEM_BUCK || ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
constexpr int na      = 2;
constexpr int nb      = 1;
constexpr int nd      = 2;
constexpr int nGens   = 3;
constexpr int nTheta = na + nb;

constexpr double UMINPHYS = 0;
constexpr double UMAXPHYS = 1;
constexpr double YMINPHYS = 0;
constexpr double YMAXPHYS = 10;
constexpr double UMIN = UMINPHYS;
constexpr double UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = YMAXPHYS;
constexpr double Ts = 1e-5;    // Sampling time, logged in output files for printing time in seconds

#if ACTIVE_SYSTEM == SYSTEM_BUCK
/* No rate limit (this system predates the rate constraint): the whole
 * output range, which no step inside [YMINPHYS, YMAXPHYS] can exceed. */
constexpr double DELTAY = YMAXPHYS - YMINPHYS;
constexpr double SIGMA_UNNORM = 0.2;

constexpr double thetaNominalInit[nTheta] = { 1.8889, -0.9333, 0.4444};
constexpr double generatorsInit[nTheta][nGens] = {
        { 0.0625,        0,         0},
        {      0,   0.0606,         0},
        {      0,        0,    0.2500}};
constexpr double thetaTrueInit[nTheta] = {1.8612, -0.9276, 0.6732};
constexpr double yHistInit[na] = {0, 0};
constexpr double uHistInit[nb+nd-1] = {0, 0};
#endif

#if ACTIVE_SYSTEM == SYSTEM_BUCK_LOSS
constexpr double DELTAY = 0.1;
constexpr double SIGMA_UNNORM = 0.02;

constexpr double thetaNominalInit[nTheta] = {1.817972144631566,  -0.871463786797535,   0.457543557236585};
constexpr double generatorsInit[nTheta][nGens] = {
        {-0.093616869443703,   0.124811247863353,   0.003362744341625},
        {0.057591158572667,  -0.117838849718767,   0.003770097120654},
        {0.271071396611763,   0.068140402887871,   0.000360367556716}};
constexpr double thetaTrueInit[nTheta] = {1.857831352244614, -0.933661885378246, 0.683201544134083};
constexpr double yHistInit[na] = {0, 0};
constexpr double uHistInit[nb+nd-1] = {0, 0};
#endif

#ifdef FIXED
/* ======================================================================
   Fixed-point types of the buck converters (the ones that depend on the
   system; the others are in types.h)
   ====================================================================== */
/*
 * output_type holds the PHYSICAL output, so it must cover the sensor range
 * [YMINPHYS-SIGMA_UNNORM, YMAXPHYS+SIGMA_UNNORM] (the constraints YMIN/YMAX
 * lie inside it). Signed ap_fixed<W,I> covers [-2^(I-1), 2^(I-1)).
 */
typedef ap_fixed<WORD_LENGTH,5,AP_RND_CONV,AP_SAT> output_type; // [YMINPHYS-SIGMA_UNNORM, YMAXPHYS+SIGMA_UNNORM] = [-0.2, 10.2] within [-16, 16)
typedef ap_ufixed<WORD_LENGTH,1,AP_RND_CONV,AP_SAT> input_type; // [UMINPHYS, UMAXPHYS] = [0, 1]
typedef ap_fixed<WORD_LENGTH,0> noise_type; // [-SIGMA_UNNORM, SIGMA_UNNORM]

/** Cost accumulator; wide enough to prevent saturation over the horizon. */
typedef ap_ufixed<32,  9, AP_RND_CONV, AP_SAT>  cost_type;

   #ifdef NRMLZ
   /* Strip/normalization constants (setup.h's computeStripCoeffs) */
   typedef ap_ufixed<23,3> strip_coeff_type;       // myInvDmDg
   typedef ap_ufixed<23,3> strip_q_coeff_type;     // qmyInvDmDg
   typedef ap_fixed<22,3> strip_coeff_c0_type;     // myInvDmc0
   typedef ap_ufixed<21,0> strip_q_coeff_c0_type;  // qmyInvDmc0

   /* Input weight weightR in normalized units.
    * FIXED multiplies by it (costFunctionArx), like floating point does. */
   typedef ap_ufixed<3,0> input_weight_type; /* weightR = 2^log2R */

      #ifndef CONVERSIONS_MODE
      /* Yes normalization, no ADC/DAC conversions: the gains themselves */
      typedef ap_ufixed<16,0> dig2ctrl_type;  // yNormGain = 2/10 = 0.2 = 0.00110011...
      typedef ap_ufixed<1,0> ctrl2dig_type;   // uNormGainInverse = 1/2 = 0.10...
      #endif
   #else
   /* No normalization: physical ranges everywhere */
   typedef ap_fixed<WORD_LENGTH,5,AP_RND_CONV,AP_SAT> theta_type; // System-dependent theta dynamic range
   typedef ap_fixed<WORD_LENGTH,5> output_strip_offset_type; // System-dependent output strip offset [SIGMA_UNNORM - YMAXPHYS, SIGMA_UNNORM+YMAXPHYS]=[-9.98, 10.02]
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> proj_type; // |cproj| <= nTheta*..., same for |gproj|
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> proj_inv_type; // inverse of proj_type
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> support_strip_offset_type; // sum of nGen + 1 proj_type variables
   typedef ap_fixed<WORD_LENGTH,9> tight_strip_center_type; // difference of tight strip offset
   typedef ap_ufixed<WORD_LENGTH,9> vol_type; // could be anything without normalization
   typedef ap_fixed<WORD_LENGTH,10> det_type; // could be anything without normalization
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> elim_type; // possibly almost-singular matrix, factor variable, pivotAbs in matDet
   typedef ap_ufixed<5,4> input_weight_type; /* weightR = RBaseline = 12.5 */
      #ifdef CONVERSIONS_MODE
      /* No normalization, yes ADC/DAC conversions: the converter gains */
      typedef ap_ufixed<20,0> dig2ctrl_type;  // yDACGain = 10/4095 = 2.442442...e-3
      typedef ap_ufixed<12,12> ctrl2dig_type; // uADCGain = 4095
      #else
      /* No normalization, no ADC/DAC conversions */
      typedef ap_ufixed<1,1> dig2ctrl_type; // 1
      typedef ap_ufixed<1,1> ctrl2dig_type; // 0
      #endif
   #endif

   #ifdef CONVERSIONS_MODE
   /* Test-bench ADC/DAC model (conversions.cpp) */
   typedef ap_ufixed<10,9> output_adc_coeff_type;  // yADCGain = 4095/10 = 409.5
   typedef ap_ufixed<12,-10> input_dac_coeff_type; // uDACGain = 1/4095 = 2.442442...e-4
   #endif
#endif /* FIXED */

#endif
