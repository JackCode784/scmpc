/* Header file for SYSTEM_BUCK_ALBERTO (contents: system_configs.h) */
#pragma once
#if ACTIVE_SYSTEM == SYSTEM_BUCK_ALBERTO

constexpr int na      = 2;
constexpr int nb      = 1;
constexpr int nd      = 2;
constexpr int nGens   = 3;
constexpr int nTheta = na + nb;

constexpr double  UMINPHYS = 0;
constexpr double  UMAXPHYS = 1;
constexpr double YMINPHYS = 0;
constexpr double YMAXPHYS = 100;
constexpr double  UMIN = UMINPHYS;
constexpr double  UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = 85;
constexpr double DELTAY = 5;
constexpr double SIGMA_UNNORM = 0.2;
constexpr double Ts = 10e-3;    // Sampling time, logged in output files for printing time in seconds. Conventional value


constexpr double thetaNominalInit[nTheta] = {1.733749265658302,  -0.881912845894497,  14.298236163643255};
constexpr double generatorsInit[nTheta][nGens] = {
    {-0.068444143534821,   0.137725841350229,   0.004638817609457},
    {-0.008278303729254,  -0.132496642357720,   0.004822306118286},
    {7.403892173947610,   0.001125040669095,   0.000048274664814}};
constexpr double thetaTrueInit[nTheta] = {1.753571428571429,  -0.894444444444444,  13.888888888888886};
constexpr double yHistInit[na] ={ 0, 0};
constexpr double uHistInit[nb+nd-1] ={ 0, 0};

#ifdef FIXED
/* ======================================================================
   Fixed-point types (the ones that depend on the system; the others are
   in types.h). Those of SYSTEM_BUCK/SYSTEM_BUCK_LOSS except output_type:
   NOT sized or validated for this system's ranges (e.g. without NRMLZ,
   theta_type's +-16 cannot hold theta3 = 14.3 +- 7.4).
   ====================================================================== */
typedef ap_fixed<WORD_LENGTH,8,AP_RND_CONV,AP_SAT> output_type; // [YMINPHYS-SIGMA_UNNORM, YMAXPHYS+SIGMA_UNNORM] = [-0.2, 100.2] within [-128, 128)
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
      typedef ap_ufixed<16,0> dig2ctrl_type;  // yNormGain = 2/100 = 0.02
      typedef ap_ufixed<1,0> ctrl2dig_type;   // uNormGainInverse = 1/2 = 0.10...
      #endif
   #else
   /* No normalization: physical ranges everywhere */
   typedef ap_fixed<WORD_LENGTH,5,AP_RND_CONV,AP_SAT> theta_type;
   typedef ap_fixed<WORD_LENGTH,5> output_strip_offset_type;
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> proj_type;
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> proj_inv_type;
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> support_strip_offset_type;
   typedef ap_fixed<WORD_LENGTH,9> tight_strip_center_type;
   typedef ap_ufixed<WORD_LENGTH,9> vol_type;
   typedef ap_fixed<WORD_LENGTH,10> det_type;
   typedef ap_fixed<WORD_LENGTH,9,AP_RND_CONV,AP_SAT> elim_type;
   typedef ap_ufixed<5,4> input_weight_type;
      #ifdef CONVERSIONS_MODE
      /* No normalization, yes ADC/DAC conversions: the converter gains */
      typedef ap_ufixed<20,0> dig2ctrl_type;  // yDACGain = 100/4095 = 2.442442...e-2
      typedef ap_ufixed<12,12> ctrl2dig_type; // uADCGain = 4095
      #else
      /* No normalization, no ADC/DAC conversions */
      typedef ap_ufixed<1,1> dig2ctrl_type; // 1
      typedef ap_ufixed<1,1> ctrl2dig_type; // 0
      #endif
   #endif

   #ifdef CONVERSIONS_MODE
   /* Test-bench ADC/DAC model (conversions.cpp) */
   typedef ap_ufixed<10,9> output_adc_coeff_type;  // yADCGain = 4095/100 = 40.95
   typedef ap_ufixed<12,-10> input_dac_coeff_type; // uDACGain = 1/4095 = 2.442442...e-4
   #endif
#endif /* FIXED */

#endif
