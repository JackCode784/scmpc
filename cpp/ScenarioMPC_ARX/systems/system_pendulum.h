/* Header file for pendulum system */
#ifndef SYSTEM_TESTBENCH_PASS
constexpr int   na    = 2;
constexpr int   nb    = 1;
constexpr int   nd    = 2;
constexpr int   nGens = 3;
constexpr int   nTheta = na + nb;

/** None of these quantities are in the controller's 
 * datapath, so they can all be double - these are used 
 * to compute constants in setup.h */
constexpr double UMINPHYS = -3;
constexpr double UMAXPHYS = 3;
constexpr double YMINPHYS = -0.9;
constexpr double YMAXPHYS = 0.9;
constexpr double UMIN = UMINPHYS;
constexpr double UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = YMAXPHYS;
constexpr double DELTAY = 10e-3;
constexpr double SIGMA_UNNORM = 0.00;

/* The plant derives l, m from Ts, changing it silently changes
    since they are derived from theta params and Ts (l = Ts^2 * g / (theta[1] + 1)) */
constexpr double Ts = 10e-3;    // Sampling time, logged in output files for printing time in seconds

constexpr double thetaNominalInit[nTheta] = {2.000000000000000, -0.998821001683502, 0.000751434382693};
constexpr double generatorsInit[nTheta][nGens] = {
    {-0.000001, 0, 0},
    {0, 1e-3 * -0.281338543972914, 1e-4 * -0.961862828825779},
    {0, 1e-3 * -0.386439223972734, 1e-4 * 0.700262993444492}};
static constexpr const double (&thetaTrueInit)[nTheta] = thetaNominalInit;
//2.0000000000,   -0.998637500000000,   0.001205632716049
//2.0, -0.999091666666667, 0.0003572245084590763
//2.000000000000000,   -0.998637500000000,   0.001205632716049
constexpr double yHistInit[na] = {-0.78, -0.78};
constexpr double uHistInit[nb+nd-1] = {0, 0};

#ifdef FIXED
/* ======================================================================
   Fixed-point types (the ones that depend on the system; the others are
   in types.h). Sized for NRMLZ (with or without CONVERSIONS_MODE),
   CTRL_MODE_SCMPC - see the notes below.
   ====================================================================== */
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
/*
 * Strip/normalization constants (setup.h's computeStripCoeffs), here:
 *   myInvDmDg = [1.0e-6, 3.78e-4, 1.52e-3]    qmyInvDmDg = 0
 *   myInvDmc0 = [2, -0.998821, 2.51e-3]       qmyInvDmc0 = 0
 * (offsets are 0: the ranges are symmetric). The pendulum's uncertainty is
 * tiny next to its coefficients - a2 = -0.998821 sits only 1.2e-3 from the
 * marginally stable -1 - so these need far more fractional bits than the
 * buck converters': the buck types (20/19 fractional bits) would carry
 * myInvDmDg[1] at 0.25% and a2 at ~1e-6, i.e. ~0.1% of that margin.
 * Constants only, rounded once at initialization: AP_RND_CONV/AP_SAT cost no
 * hardware and keep the error within 1/2 LSB (AP_TRN: 1 LSB, biased).
 */
typedef ap_ufixed<24,-9,AP_RND_CONV,AP_SAT> strip_coeff_type;      // myInvDmDg  < 2^-9 = 1.95e-3, 33 fractional bits
typedef ap_ufixed<24,-9,AP_RND_CONV,AP_SAT> strip_q_coeff_type;    // qmyInvDmDg (0 here)
typedef ap_fixed<25,3,AP_RND_CONV,AP_SAT> strip_coeff_c0_type;     // myInvDmc0: 2.0 needs 3 integer bits; 22 fractional bits (2.4e-7)
typedef ap_ufixed<21,0,AP_RND_CONV,AP_SAT> strip_q_coeff_c0_type;  // qmyInvDmc0 (0 here)
/*
 * The passive-learning types in types.h (phi_type, proj_type, ...) are the
 * buck converters' ones: they compile in every CTRL_MODE, but are NOT
 * sized or validated for the pendulum (its normalized regressor
 * phi = sample*myInvDmDg is ~1e-3, far below what phi_type/proj_type
 * resolve well). CTRL_MODE_SCMPC never uses them.
 */
#ifndef CONVERSIONS_MODE
/* Normalization gains 2/(YMAXPHYS-YMINPHYS) and (UMAXPHYS-UMINPHYS)/2
 * (the buck types hold 0.2 and 0.5 and cannot hold these). */
typedef ap_ufixed<WORD_LENGTH,1> dig2ctrl_type;  // yNormGain = 1.1111
typedef ap_ufixed<WORD_LENGTH,2> ctrl2dig_type;  // uNormGainInverse = 3
#else
/* ADC/DAC over [-0.9, 0.9] rad and [-3, 3] (test-bench converter model):
 * the buck types overflow on yADCGain = 2275 (max 511) and
 * uDACGain = 1.47e-3 (max 9.8e-4). */
typedef ap_ufixed<16,12> output_adc_coeff_type; // yADCGain = 4095/1.8 = 2275
typedef ap_ufixed<20,-9> input_dac_coeff_type;  // uDACGain = 6/4095 = 1.465e-3 < 2^-9, 29 fractional bits
#endif
/* Input weight weightR in normalized units.
 * FIXED multiplies by it (costFunctionArx), like floating point does. */
typedef ap_ufixed<WORD_LENGTH,3> input_weight_type; /* weightR = 2^log2R */
/* Cost accumulator: measured up to ~1600 over 20 random runs (float,
 * scenarios on; weights 16) - the buck's 9 integer bits (max 512) would
 * saturate it and make MADS candidates indistinguishable. 12 integer bits
 * (max 4096), 20 fractional. */
typedef ap_ufixed<32, 12, AP_RND_CONV, AP_SAT>  cost_type;
#endif /* FIXED */
#else /* SYSTEM_MODEL_DONE */
/* ----------------------------------------------------------------------
   Inverted pendulum: thetaddot = g/l*sin(theta) + u/(m*l^2), y = theta.
   ----------------------------------------------------------------------
   The controller's ARX model is the forward-Euler discretization, with
   step Ts, of the LINEARIZED equation (sin(theta) ~ theta):
     y(k) = 2*y(k-1) + (-1 + Ts^2*g/l)*y(k-2) + Ts^2/(m*l^2)*u(k-2)
   i.e. theta = [2, -1 + Ts^2*g/l, Ts^2/(m*l^2)] (na=2, nb=1, nd=2). The
   nominal l, m are recovered from thetaNominalInit through this map
   (l ~ 0.83, m ~ 0.19 for the current values in system_configs.h), so there is a
   single source of truth.

   A random true plant draws l and m uniformly in nominal*(1 +- REL_UNC);
   with PLANT_MODEL == PLANT_MODEL_PHYSICS its output comes from the non linear model
   with those l, m.

   The nonlinear equation is integrated with PENDULUM_EULER_SUBSTEPS
   forward-Euler sub-steps per sample, input held constant over the
   sample (zero-order hold). Besides sin(theta), this differs from the ARX
   model in a second, smaller way: fine integration lets u(k-1) already
   affect y(k), which the one-step Euler model (nd = 2) cannot represent -
   a genuine model mismatch the robust controller has to absorb.
   ---------------------------------------------------------------------- */
constexpr double PEND_G     = 9.81;   /* [m/s^2]                               */
constexpr double PEND_REL_UNC = 0.2;  /* l, m drawn in nominal*(1 +- 0.2)      */
constexpr int    PENDULUM_EULER_SUBSTEPS = 100;

static_assert(na == 2 && nb == 1 && nd == 2,
    "plant.h's pendulum map assumes the ARX structure na=2, nb=1, nd=2.");

struct PendulumPlant : ArxPlant {double l, m, x[2]; };

inline void pendulumThetaFromPhys(double l, double m, double theta[nTheta])
{
    theta[0] = 2.0;
    theta[1] = -1.0 + Ts * Ts * PEND_G / l;
    theta[2] = Ts * Ts / (m * l * l);
}

inline void pendulumPhysFromTheta(const double theta[nTheta], double &l, double &m)
{
    l = Ts * Ts * PEND_G / (theta[1] + 1.0);
    m = Ts * Ts / (theta[2] * l * l);
}

inline void plantSampleTrue(PendulumPlant &p, TbRng &rng, bool random)
{
    if (random)
    {
        double lNom, mNom;
        pendulumPhysFromTheta(thetaNominalInit, lNom, mNom);
        p.l = lNom * (1.0 + PEND_REL_UNC * rng.uniform());
        p.m = mNom * (1.0 + PEND_REL_UNC * rng.uniform());
        pendulumThetaFromPhys(p.l, p.m, p.theta);
    }
    else
    {
        /* thetaTrueInit exactly for the ARX model; l, m derived from it
         * for the nonlinear one */
        for (int i = 0; i < nTheta; i++) p.theta[i] = thetaTrueInit[i];
        pendulumPhysFromTheta(thetaTrueInit, p.l, p.m);
    }
}

inline void plantReset(PendulumPlant &p)
{
    plantReset(static_cast<ArxPlant&>(p));
    /* State consistent with the output history: angle y(-1), velocity
     * from the last two samples. */
    p.x[0] = yHistInit[0];
    p.x[1] = (yHistInit[0] - yHistInit[1]) / Ts;
    p.uEarly = uHistInit[0];
    p.fEarly = 0.0;
}

inline double plantNextOutput(PendulumPlant &p)
{
    const double h = Ts / PENDULUM_EULER_SUBSTEPS;
    /* Zero-order hold of uEarly for the first fEarly, then uLate
     * of the interval (controller latency, testbench_settings::computeDelayFraction) */
    const int nEarly = int(std::lround(p.fEarly * PENDULUM_EULER_SUBSTEPS));
    for (int s = 0; s < PENDULUM_EULER_SUBSTEPS; s++)
    {
        const double uHold = (s < nEarly) ? p.uEarly : p.uLate;
        const double angle = p.x[0], vel = p.x[1];
        p.x[0] = angle + h * vel;
        p.x[1] = vel + h * (PEND_G / p.l * std::sin(angle) + uHold / (p.m * p.l * p.l));
    }
    return p.x[0];
}

inline void plantPrintParams(const PendulumPlant &p, FILE *f, bool header)
{
    if (header) { for (int i = 0; i < nTheta; i++) fprintf(f, " theta%d", i + 1); fprintf(f, " l m"); }
    else        { for (int i = 0; i < nTheta; i++) fprintf(f, " %.12g", p.theta[i]); fprintf(f, " %.6g %.6g", p.l, p.m); }
}

#if PLANT_MODEL == PLANT_MODEL_PHYSICS
using Plant = PendulumPlant;
#define PLANT_TAG "nl"
#endif
#endif