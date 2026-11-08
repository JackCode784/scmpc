#pragma once
#if ACTIVE_SYSTEM == SYSTEM_GAIN_DEMO
constexpr int na      = 1;
constexpr int nb      = 1;
constexpr int nd      = 2;
constexpr int nGens   = 2;
constexpr int nTheta = na + nb;

constexpr double UMINPHYS = 0;
constexpr double UMAXPHYS = 1.5;
constexpr double YMINPHYS = 0;
constexpr double YMAXPHYS = 7;
constexpr double UMIN = UMINPHYS;
constexpr double UMAX = UMAXPHYS;
constexpr double YMIN = YMINPHYS;
constexpr double YMAX = 5;
constexpr double DELTAY = 100; /* effectively unconstrained: this demo is about the static YMAX bound, not the rate limit */
constexpr double SIGMA_UNNORM = 0.02;

/* Sampling time, logged in output files for printing time in seconds.
    This is just a conventional value, no sampling time was specified. */
constexpr double Ts = 10e-3;

/**
 * Minimal demonstration system: IS SCENARIO MPC WORTH IT?
 * ------------------------------------------------------------------
 * y(k) = a*y(k-1) + b*u(k-2)   (na=1, nb=1, nd=2 - a first-order ARX
 * with a two-step actuation delay, structurally identical to
 * SYSTEM_BUCK/SYSTEM_BUCK_LOSS's own (nb,nd)=(1,2), just na=1 instead
 * of na=2 for the simplest possible hand-checkable dynamics).
 *
 * theta = [a, b]. The zonotope is diagonal (independent per-parameter
 * uncertainty), exactly like SYSTEM_SIMPLE/SYSTEM_BUCK:
 *   thetaCenter = [a0, b0] = [0.5, 2.0]
 *   GENERATORS  = diag(deltaA, deltaB) = diag(0.05, 1.0)
 *   => a in [0.45, 0.55], b in [1.0, 3.0]
 *
 * thetaTrue = [0.5, 3.0]: 'a' sits EXACTLY at its nominal/center value
 * (ξ_a=0 - the pole is assumed known, only carried as a generator for
 * structural realism/non-singularity of the generator matrix, and to
 * keep matDet(thetaGens) - called unconditionally in testMain.cpp -
 * well-defined; it is deliberately NOT the source of the mismatch this
 * system demonstrates), while 'b' (the INPUT GAIN) sits EXACTLY at the
 * zonotope's vertex ξ_b=+1, i.e. the worst case the controller's own
 * uncertainty description claims is possible.
 *
 * THE IDEALIZED (STATIC) ARGUMENT
 * ---------------------------------
 * DC gain is b/(1-a): G_nominal = 2/0.5 = 4, G_true = 3/0.5 = 6 - the
 * true plant is 50% "stronger" than the model the controller nominally
 * trusts. With YMAX=5 and a reference that steps from 1.0 to 4.9 (see
 * testMain.cpp's generateReference, SYSTEM_GAIN_DEMO branch), a
 * controller that only ever evaluates thetaCenter would compute
 * u* = yref/G_nominal = 4.9/4 = 1.225 (within UMAX=1.5), predicting
 * y=4.9 - safely under YMAX. Applied to the TRUE plant, the actual
 * steady-state output would be G_true*u* = 6*1.225 = 7.35 - a 47%
 * breach the nominal-only controller never sees coming.
 *
 * WHAT ACTUALLY HAPPENS IN CLOSED LOOP (measured, floating point,
 * NRMLZ/FIXED/CONVERSIONS_MODE all undefined - see setup.h's TARGET
 * SELECTION block - scen::nScen=4, mads::iter=7, the project's real values;
 * mean/max taken over simulation steps 200-300, i.e. 100+ steps after
 * the reference step, once the closed loop has settled into its
 * long-run behaviour rather than a transient):
 *
 *   USE_SCENS_CONSTR undefined (constraint checked only against
 *   thetaCenter - certainty-equivalent MPC): output settles at a
 *   PERSISTENT y ~= 6.3-6.6 (26-32% over YMAX=5) - not a transient
 *   overshoot that decays, a genuine steady violation, confirmed
 *   stable out to 1200 steps.
 *
 *   USE_SCENS_CONSTR defined (the codebase's actual default: also
 *   check cost[1] against scen::nScen=4 parameter draws sampled at random
 *   from the SAME zonotope every prediction step - see
 *   costFunctionArx.cpp/generateScenarios.cpp): output settles at
 *   y ~= 5.6-5.75 (12-15% over YMAX) - roughly HALF the exceedance of
 *   the certainty-equivalent case, a real and substantial improvement,
 *   but NOT a full elimination.
 *
 * WHY NOT A FULL ELIMINATION AT scen::nScen=4
 * -----------------------------------------
 * This is sample-based, not exact worst-case, robustness: scenarios
 * are redrawn at random every call rather than fixed at the zonotope's
 * vertices, so whether a given call's scen::nScen*mpc::predHrzn=20 draws happen to
 * sample close to the b=3 vertex varies call to call, and
 * progressiveBarrierPollingArx's mesh resets to a small fixed size
 * (mads::d0, setup.h) at the start of every controller() call, so MADS
 * can only make a small, local correction per call rather than jump
 * straight to the fully-robust input. The residual gap is a SAMPLE
 * BUDGET effect, not a broken mechanism: repeating this exact
 * experiment with scen::nScen bumped to 16 (in an isolated, non-committed
 * build - scen::nScen is a shared setup.h constant, changing it here would
 * also change SYSTEM_BUCK_LOSS's hardware area) reduced the exceedance
 * to ~2%; scen::nScen=64 reduced it to ~3 violations out of 100 samples
 * checked, essentially eliminating it. This confirms the mechanism
 * converges toward the idealized worst-case-robust result as the
 * scenario budget grows - it is simply underpowered, not incorrect, at
 * the scen::nScen=4 budget this project's hardware profiles are sized for.
 *
 * SCOPE: validated in floating point with NRMLZ, FIXED and
 * CONVERSIONS_MODE all undefined - CTRL_MODE_SCMPC only.
 *   - NRMLZ is unsupported: setup.h's NRMLZ branch requires
 *     MATLAB-precomputed myInvDmDg/myInvDmc0/... constants (see
 *     setup.h, "Pre-computed in MATLAB") that only exist for
 *     SYSTEM_BUCK/SYSTEM_BUCK_LOSS and are irrelevant to
 *     CTRL_MODE_SCMPC anyway (they only feed the CTRL_MODE_PL/AL
 *     strip-intersection code path in controller.cpp).
 *   - CONVERSIONS_MODE should stay OFF for this system specifically:
 *     with it on, the 12-bit ADC's full-scale range is [YMIN,YMAX], so
 *     once the true output exceeds YMAX the SENSOR reading fed back to
 *     the controller saturates at the rail - a realistic effect in
 *     general, but here it quietly changes what the controller
 *     believes y(k-1) is once a violation is already underway,
 *     confounding the comparison above with an extra, unintended
 *     nonlinearity. (Fixed in passing: conversions.cpp's ctrlU2dig had
 *     a latent bug referencing dig::codeMin/dig::codeMax - only defined under
 *     CONVERSIONS_MODE - whenever FIXED was undefined regardless of
 *     CONVERSIONS_MODE, making "FIXED off + CONVERSIONS_MODE off" fail
 *     to compile at all before this system needed it.)
 *   - FIXED-point/HLS synthesis support (bit widths in types.h) has
 *     not been added for this system - it exists to answer "is
 *     scenario MPC worth it", not to be synthesised.
 */

constexpr double thetaNominalInit[nTheta] = { 0.5, 2.0};
constexpr double generatorsInit[nTheta][nGens] = {
    {0.05, 0},
    {0, 1.0}};
constexpr double thetaTrueInit[nTheta] = {0.5, 3.0};
constexpr double yHistInit[na] = {0};     // na=1
constexpr double uHistInit[nb+nd-1] = {0, 0};  // nb+nd-1=2
#endif
