/**
 * @file  unit_tests.cpp
 * @brief Unit tests for the controller functions of cpp/ScenarioMPC_ARX.
 *
 * Built by the harness (scmpc_test unit) against the controller sources,
 * without testMain.cpp, in several configurations (float and fixed point,
 * with and without normalization, passive learning). By hand, from
 * cpp/ScenarioMPC_ARX:
 *
 *   g++ -std=c++14 -O2 -I . -o unit_tests ../../tests/unit/unit_tests.cpp \
 *       $(ls *.cpp | grep -v testMain.cpp) && ./unit_tests
 *
 * Each test checks a property the function must have, against an
 * independent computation in double. In floating point the two must agree
 * to rounding (relative 1e-9). In fixed point the tolerance is derived
 * from the LSB of the types involved, never tuned to make a test pass;
 * the largest error seen is printed, so a growing error is visible before
 * it fails.
 *
 * Where the implementation deliberately quantizes an intermediate value
 * (e.g. the tracking error to err_type), the reference applies the same
 * conversion: the test then checks the STRUCTURE of the computation
 * (indices, horizons, weights, which terms are included), which is what
 * refactors touch, and only the remaining accumulation rounding is left
 * to the tolerance.
 */
#include "setup.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <algorithm>

/* Scenario switches of setup.h as constants, usable in ordinary code */
#ifdef USE_SCENS_COST
static const bool USE_COST = true;
#else
static const bool USE_COST = false;
#endif
#ifdef USE_SCENS_CONSTR
static const bool USE_CONSTR = true;
#else
static const bool USE_CONSTR = false;
#endif
static const bool USE_SCENS_ANY = USE_COST || USE_CONSTR;

/* ======================================================================
   Minimal test framework
   ====================================================================== */
static int g_failed = 0;     /* failed checks, whole program */
static int t_failed = 0;     /* failed checks, current test  */
static int t_checks = 0;
static int t_printed = 0;    /* failure messages printed, current test */

#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        t_checks++;                                                        \
        if (!(cond)) {                                                     \
            t_failed++; g_failed++;                                        \
            if (t_printed++ < 5) {                                         \
                printf("      FAIL line %d: ", __LINE__);                  \
                printf(__VA_ARGS__);                                       \
                printf("\n");                                              \
            }                                                              \
        }                                                                  \
    } while (0)

static void beginTest() { t_failed = t_checks = t_printed = 0; }
static void endTest(const char *name, const char *detail)
{
    printf("    %-34s %s  (%d checks)%s%s\n", name, t_failed ? "FAIL" : "ok  ", t_checks,
           detail[0] ? "  " : "", detail);
}
#define SKIP(name, why) printf("    %-34s skip  (%s)\n", name, why)

/* LSB of a fixed-point type; 0 in floating point (types are double). */
#ifdef FIXED
template <class T> double lsb() { return std::ldexp(1.0, -(T::width - T::iwidth)); }
static const bool isFixed = true;
#else
template <class T> double lsb() { return 0.0; }
static const bool isFixed = false;
#endif

/* Test-local random numbers: std::mt19937's raw output is fixed by the
 * standard, so the inputs are the same on every compiler and OS. */
static std::mt19937 rng(12345u);
static double uni(double a, double b) { return a + (b - a) * (double(rng()) / 4294967296.0); }

static double absd(double x) { return x < 0 ? -x : x; }

/* Solve A x = b (n x n, Gaussian elimination with partial pivoting). */
template <int N>
static bool solve(double A[N][N], double b[N], double x[N])
{
    for (int c = 0; c < N; c++)
    {
        int p = c;
        for (int r = c + 1; r < N; r++) if (absd(A[r][c]) > absd(A[p][c])) p = r;
        if (absd(A[p][c]) < 1e-300) return false;
        for (int k = 0; k < N; k++) std::swap(A[c][k], A[p][k]);
        std::swap(b[c], b[p]);
        for (int r = c + 1; r < N; r++)
        {
            const double f = A[r][c] / A[c][c];
            for (int k = c; k < N; k++) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int r = N - 1; r >= 0; r--)
    {
        double s = b[r];
        for (int k = r + 1; k < N; k++) s -= A[r][k] * x[k];
        x[r] = s / A[r][r];
    }
    return true;
}

/* Determinant of an n x n matrix (in double). */
template <int N>
static double det(double A[N][N])
{
    double M[N][N], d = 1.0;
    for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) M[i][j] = A[i][j];
    for (int c = 0; c < N; c++)
    {
        int p = c;
        for (int r = c + 1; r < N; r++) if (absd(M[r][c]) > absd(M[p][c])) p = r;
        if (M[p][c] == 0.0) return 0.0;
        if (p != c) { for (int k = 0; k < N; k++) std::swap(M[c][k], M[p][k]); d = -d; }
        d *= M[c][c];
        for (int r = c + 1; r < N; r++)
        {
            const double f = M[r][c] / M[c][c];
            for (int k = c; k < N; k++) M[r][k] -= f * M[c][k];
        }
    }
    return d;
}

/* Scenario coefficients as the controller computes them, from its current
 * zonotope (the power-on one after resetControllerState()). */
static void realCoefficients(arx_coeff_type coeff[Nscen + 1][nTheta], arx_coeff_type offset[Nscen + 1],
                             theta_type thetaScen[Nscen][nUnc])
{
    generateScenarios(thetaScen, thetaCenter, thetaGens);
    computeArxCoeffs(thetaScen, thetaCenter, coeff, offset);
}

/* The regressor entry i of an ARX prediction: y(k-1-i) for i < na, then
 * u(k-nd-(i-na)) = uSamples[i-na+nd-1] (uSamples[0] = u(k)). */
template <class Y, class U>
static double regressor(int i, const Y yPast[na], const U uSamples[nb + nd - 1])
{
    return (i < na) ? double(yPast[i]) : double(uSamples[i - na + nd - 1]);
}

/* ======================================================================
   1. computeArxOutput: y = offset + coeff^T * regressor
   ====================================================================== */
static void testComputeArxOutput()
{
    beginTest();
    resetControllerState();
    pseudoRandReset();
    arx_coeff_type coeff[Nscen + 1][nTheta], offset[Nscen + 1];
    theta_type thetaScen[Nscen][nUnc];
    realCoefficients(coeff, offset, thetaScen);

    /* fixed: each product rounds into the accumulator and the sum rounds
     * once into norm_output_type */
    const double tol = isFixed ? (nTheta + 1) * lsb<norm_output_type>() : 0.0;
    double maxErr = 0.0;
    int skipped = 0;
    for (int trial = 0; trial < 2000; trial++)
    {
        norm_output_type yPast[na];
        norm_input_type uSamples[nb + nd - 1];
        for (int i = 0; i < na; i++) yPast[i] = uni(-0.4, 0.4);
        for (int i = 0; i < nb + nd - 1; i++) uSamples[i] = uni(-0.9, 0.9);
        const int l = int(uni(0, Nscen + 1 - 1e-9));

        double ref = double(offset[l]);
        for (int i = 0; i < nTheta; i++) ref += double(coeff[l][i]) * regressor(i, yPast, uSamples);
        if (isFixed && absd(ref) > 1.9) { skipped++; continue; }   /* would saturate */

        const double got = double(computeArxOutput(yPast, uSamples, coeff[l], offset[l]));
        const double err = absd(got - ref);
        if (err > maxErr) maxErr = err;
        CHECK(err <= tol + 1e-12 * (1.0 + absd(ref)), "row %d: got %.12g, expected %.12g", l, got, ref);
    }
    char d[96];
    snprintf(d, sizeof d, "max error %.3g (tol %.3g), %d saturating cases skipped", maxErr, tol, skipped);
    endTest("computeArxOutput", d);
}

/* ======================================================================
   2. computeArxCoeffs: the normalized prediction is the normalization of
      the physical prediction with theta = c0 + D_G * theta_normalized
   ====================================================================== */
static void testComputeArxCoeffs()
{
    beginTest();
    resetControllerState();
    pseudoRandReset();
    arx_coeff_type coeff[Nscen + 1][nTheta], offset[Nscen + 1];
    theta_type thetaScen[Nscen][nUnc];
    realCoefficients(coeff, offset, thetaScen);

#ifdef NRMLZ
    const double my = double(yNormGain), qy = double(yNormOffset);
    const double mu = double(uNormGain), qu = double(uNormOffset);
    /* fixed: every coefficient and the offset are rounded to arx_coeff_type */
    const double tol = isFixed ? (nTheta + 1) * lsb<arx_coeff_type>() * 2.0 : 0.0;
#else
    const double tol = isFixed ? (nTheta + 1) * lsb<arx_coeff_type>() : 0.0;
#endif
    double maxErr = 0.0;
    for (int l = 0; l < Nscen + 1; l++)
    {
        /* this row's physical parameters */
        double theta[nTheta];
        for (int i = 0; i < nTheta; i++)
        {
            const int k = UNC_MAP.red[i];
            const double tn = (k < 0) ? 0.0 : double((l < Nscen) ? thetaScen[l][k] : thetaCenter[k]);
#ifdef NRMLZ
            theta[i] = thetaNominalInit[i] + DgInit.Dg[i] * tn;
#else
            theta[i] = (k < 0) ? thetaNominalInit[i] : tn;
#endif
        }
        for (int trial = 0; trial < 50; trial++)
        {
            double yn[na], un[nb + nd - 1];
            for (int i = 0; i < na; i++) yn[i] = uni(-1, 1);
            for (int i = 0; i < nb + nd - 1; i++) un[i] = uni(-1, 1);

            double got = double(offset[l]);
            for (int i = 0; i < nTheta; i++) got += double(coeff[l][i]) * regressor(i, yn, un);
#ifdef NRMLZ
            double yp[na], up[nb + nd - 1];
            for (int i = 0; i < na; i++) yp[i] = (yn[i] - qy) / my;
            for (int i = 0; i < nb + nd - 1; i++) up[i] = (un[i] - qu) / mu;
            double phys = 0.0;
            for (int i = 0; i < nTheta; i++) phys += theta[i] * regressor(i, yp, up);
            const double expected = my * phys + qy;
#else
            double expected = 0.0;
            for (int i = 0; i < nTheta; i++) expected += theta[i] * regressor(i, yn, un);
#endif
            const double err = absd(got - expected);
            if (err > maxErr) maxErr = err;
            CHECK(err <= tol + 1e-9 * (1.0 + absd(expected)), "row %d: got %.12g, expected %.12g", l, got, expected);
        }
    }
    char d[64];
    snprintf(d, sizeof d, "max error %.3g (tol %.3g)", maxErr, tol);
    endTest("computeArxCoeffs (normalization)", d);
}

/* ======================================================================
   3. addConstraintViolation: 0 inside the bounds, the exact excess outside
   ====================================================================== */
static void testConstraintViolation()
{
    beginTest();
    const double lo = double(YNORMMIN), hi = double(YNORMMAX), dy = double(DELTAYNORM);
    /* values on the grid of every type involved: multiples of 2^-8 */
    const double q = 1.0 / 256;
    struct Case { double y, yPrev, expected; };
    const double mid = std::round((lo + hi) / 2 / q) * q;
    const Case cases[] = {
        { mid, mid, 0.0 },                                           /* inside, no step       */
        { hi + 4 * q, hi + 4 * q, 4 * q },                            /* above the box         */
        { lo - 8 * q, lo - 8 * q, 8 * q },                            /* below the box         */
        { mid + dy + 2 * q, mid, 2 * q },                             /* step up too large     */
        { mid - dy - 2 * q, mid, 2 * q },                             /* step down too large   */
        { hi + 4 * q, hi + 4 * q - dy - 2 * q, 4 * q + 2 * q },       /* both: the terms add   */
    };
    for (const Case &c : cases)
    {
        if (c.y < -1.99 || c.y > 1.99 || c.yPrev < -1.99 || c.yPrev > 1.99) continue;
        cost_type acc = 0;
        const norm_output_type y = c.y, yPrev = c.yPrev;
        addConstraintViolation(acc, y, yPrev);
        /* the expected value with the bounds as the types hold them */
        double e = 0.0;
        const double yq = double(y), pq = double(yPrev);
        if (yq > hi) e += yq - hi; else if (yq < lo) e += lo - yq;
        if (yq - pq > dy) e += yq - pq - dy; else if (yq - pq < -dy) e += pq - yq - dy;
        CHECK(absd(double(acc) - e) <= 1e-12, "y=%g yPrev=%g: got %.12g, expected %.12g", yq, pq, double(acc), e);
        CHECK((c.expected == 0.0) == (double(acc) == 0.0), "y=%g yPrev=%g: violation %g, expected %s", yq, pq,
              double(acc), c.expected == 0.0 ? "none" : "some");
    }
    endTest("addConstraintViolation", "");
}

/* ======================================================================
   4. costFunctionArx against a reference implementation of its cost
   ====================================================================== */
static void referenceCost(double out[2], const norm_output_type yPast[na], const norm_input_type currU[nOpt],
                          const norm_input_type uPast[nUKnown], const norm_output_type yref,
                          const arx_coeff_type coeff[Nscen + 1][nTheta], const arx_coeff_type offset[Nscen + 1])
{
    norm_input_type uS[nb + nd - 1];
    for (int i = 0; i < nUKnown; i++) uS[i + 1 - DC_STEPS] = uPast[i];
    norm_output_type rows[Nscen + 1][na];
    for (int l = 0; l < Nscen + 1; l++) for (int i = 0; i < na; i++) rows[l][i] = yPast[i];

    double viol = 0.0, track = 0.0;
    for (int i = 0; i < nOpt; i++)
        if (double(currU[i]) < double(UNORMMIN) || double(currU[i]) > double(UNORMMAX)) viol = 512;

    /* input term: R * (Delta u)^2 over the nOpt decision variables, the
     * first one against the last known input uPast[0] */
    const double R = double(weightR);
    for (int j = 0; j < nOpt; j++)
    {
        const double d = double(currU[j]) - double(j == 0 ? uPast[0] : currU[j - 1]);
        track += R * d * d;
    }

    const bool scen = USE_SCENS_ANY;
    for (int k = 0; k < Nhor; k++)
    {
        if (k >= DC_STEPS && k - DC_STEPS < NhorU) uS[0] = currU[k - DC_STEPS];
        double scenSum = 0.0, eNom = 0.0;
        for (int l = scen ? 0 : Nscen; l < Nscen + 1; l++)
        {
            const norm_output_type yNext = computeArxOutput(rows[l], uS, coeff[l], offset[l]);
            const double y = double(yNext), yPrev = double(rows[l][0]);
            double v = 0.0;
            if (y > double(YNORMMAX)) v += y - double(YNORMMAX); else if (y < double(YNORMMIN)) v += double(YNORMMIN) - y;
            if (y - yPrev > double(DELTAYNORM)) v += y - yPrev - double(DELTAYNORM);
            else if (y - yPrev < -double(DELTAYNORM)) v += yPrev - y - double(DELTAYNORM);
            const double e = double(err_type(yNext - yref));   /* as the implementation quantizes it */
            if (l < Nscen)
            {
                if (USE_COST) scenSum += e * e;
                if (USE_CONSTR) viol += v;
            }
            else { viol += v; eNom = e; }
            for (int i = na - 1; i > 0; i--) rows[l][i] = rows[l][i - 1];
            rows[l][0] = yNext;
        }
        const double w = (k < Nhor - 1) ? double(weightQ) : double(weightP);
        track += w * (eNom * eNom + (USE_COST ? scenSum / Nscen : 0.0));
        for (int i = nb + nd - 2; i > 0; i--) uS[i] = uS[i - 1];
    }
    out[0] = track;
    out[1] = viol;
}

static void testCostFunction()
{
    beginTest();
    resetControllerState();
    pseudoRandReset();
    arx_coeff_type coeff[Nscen + 1][nTheta], offset[Nscen + 1];
    theta_type thetaScen[Nscen][nUnc];
    realCoefficients(coeff, offset, thetaScen);

    /* fixed: every cost_type addition rounds (<= 1/2 LSB), the scenario
     * mean is a right shift (<= 1 LSB) and the weights multiply what is
     * left; the violation sum is exact, then clamped to cost_type */
    const double tolTrack = isFixed ? double(weightQ) * (Nscen + 4) * (Nhor + nOpt) * lsb<cost_type>() : 0.0;
    double maxTrack = 0.0, maxViol = 0.0;
    int outOfBounds = 0;
    for (int trial = 0; trial < 300; trial++)
    {
        norm_output_type yPast[na];
        norm_input_type currU[nOpt], uPast[nUKnown];
        const double y0 = uni(-0.5, 0.5);
        for (int i = 0; i < na; i++) yPast[i] = y0 + uni(-0.01, 0.01);
        for (int i = 0; i < nOpt; i++) currU[i] = uni(-1.05, 1.05);      /* sometimes out of bounds */
        for (int i = 0; i < nUKnown; i++) uPast[i] = uni(-1, 1);
        const norm_output_type yref = uni(-0.8, 0.8);

        cost_type cost[2];
        costFunctionArx(cost, yPast, currU, uPast, yref, coeff, offset);
        double ref[2];
        referenceCost(ref, yPast, currU, uPast, yref, coeff, offset);
        if (ref[1] >= 512) outOfBounds++;

        /* cost_type saturates: compare with the clamped reference */
        const double maxCost = isFixed ? double(cost_type(1e30)) : 1e300;
        const double r0 = ref[0] < maxCost ? ref[0] : maxCost, r1 = ref[1] < maxCost ? ref[1] : maxCost;
        const double e0 = absd(double(cost[0]) - r0), e1 = absd(double(cost[1]) - r1);
        if (e0 > maxTrack) maxTrack = e0;
        if (e1 > maxViol) maxViol = e1;
        CHECK(e0 <= tolTrack + 1e-9 * (1 + r0), "cost[0] = %.12g, reference %.12g", double(cost[0]), r0);
        CHECK(e1 <= 1e-9 * (1 + r1), "cost[1] = %.12g, reference %.12g", double(cost[1]), r1);
    }
    char d[128];
    snprintf(d, sizeof d, "max error cost[0] %.3g (tol %.3g), cost[1] %.3g; %d of 300 with inputs out of bounds",
             maxTrack, tolTrack, maxViol, outOfBounds);
    endTest("costFunctionArx vs reference", d);
}

/* ======================================================================
   5. generateScenarios: every scenario lies in the zonotope; the same
      PRNG state gives the same scenarios
   ====================================================================== */
static void testGenerateScenarios()
{
    beginTest();
    resetControllerState();
    double maxXi = 0.0;
    /* fixed: theta = c + G*xi rounds each of the nGens products and the sum */
    const double tol = isFixed ? (nGens + 1) * lsb<theta_type>() : 0.0;
    for (int rep = 0; rep < 20; rep++)
    {
        theta_type s[Nscen][nUnc];
        generateScenarios(s, thetaCenter, thetaGens);
        for (int l = 0; l < Nscen; l++)
        {
            /* interval hull: |theta_k - c_k| <= sum_j |G_kj| */
            for (int k = 0; k < nUnc; k++)
            {
                double r = 0.0;
                for (int j = 0; j < nGens; j++) r += absd(double(thetaGens[k][j]));
                const double dev = absd(double(s[l][k]) - double(thetaCenter[k]));
                CHECK(dev <= r + tol + 1e-12, "scenario %d, parameter %d: |theta-c| = %g > %g", l, k, dev, r);
            }
            /* square generator matrix: the coefficients xi themselves */
            if (nUnc == nGens)
            {
                double A[nUnc][nUnc], b[nUnc], xi[nUnc];
                for (int k = 0; k < nUnc; k++)
                {
                    for (int j = 0; j < nUnc; j++) A[k][j] = double(thetaGens[k][j]);
                    b[k] = double(s[l][k]) - double(thetaCenter[k]);
                }
                if (solve<nUnc>(A, b, xi))
                    for (int j = 0; j < nUnc; j++)
                    {
                        if (absd(xi[j]) > maxXi) maxXi = absd(xi[j]);
                        if (!isFixed) CHECK(absd(xi[j]) <= 1.0 + 1e-9, "scenario %d: xi[%d] = %g", l, j, xi[j]);
                    }
            }
        }
    }
    /* determinism */
    theta_type a[Nscen][nUnc], b[Nscen][nUnc];
    pseudoRandReset(); generateScenarios(a, thetaCenter, thetaGens);
    pseudoRandReset(); generateScenarios(b, thetaCenter, thetaGens);
    bool same = true;
    for (int l = 0; l < Nscen; l++) for (int k = 0; k < nUnc; k++) same = same && (double(a[l][k]) == double(b[l][k]));
    CHECK(same, "same PRNG state, different scenarios");
    char d[64];
    snprintf(d, sizeof d, "max |xi| %.6f", maxXi);
    endTest("generateScenarios", nUnc == nGens ? d : "");
}

/* ======================================================================
   6. pseudoRandArx: range, reproducibility, first two moments of U[-1,1]
   ====================================================================== */
static void testPseudoRand()
{
    beginTest();
    pseudoRandReset();
    const int n = 20000;
    double sum = 0.0, sum2 = 0.0, first[8];
    for (int i = 0; i < n; i++)
    {
        const double r = double(pseudoRandArx());
        if (i < 8) first[i] = r;
        CHECK(r >= -1.0 && r <= 1.0, "sample %d = %g outside [-1, 1]", i, r);
        sum += r; sum2 += r * r;
    }
    const double mean = sum / n, var = sum2 / n - mean * mean;
    /* U[-1,1]: mean 0, variance 1/3; with n = 20000 the standard error of
     * the mean is 0.004, so 0.02 is 5 standard errors */
    CHECK(absd(mean) < 0.02, "mean %g", mean);
    CHECK(absd(var - 1.0 / 3.0) < 0.02, "variance %g (1/3 expected)", var);
    pseudoRandReset();
    for (int i = 0; i < 8; i++) CHECK(double(pseudoRandArx()) == first[i], "sequence differs after reset at %d", i);
    char d[64];
    snprintf(d, sizeof d, "mean %.4f, variance %.4f", mean, var);
    endTest("pseudoRandArx", d);
}

/* ======================================================================
   7. zonotopeVolume: |det G| for a square generator matrix
   ====================================================================== */
static void testZonotopeVolume()
{
    beginTest();
    if (nUnc != nGens) { SKIP("zonotopeVolume", "generator matrix not square"); return; }
    double maxRel = 0.0, maxAbs = 0.0;
    for (int trial = 0; trial < 500; trial++)
    {
        theta_type G[nUnc][nGens];
        double A[nUnc][nUnc];
        for (int i = 0; i < nUnc; i++)
            for (int j = 0; j < nGens; j++) { G[i][j] = uni(-0.6, 0.6); A[i][j] = double(G[i][j]); }
        const double ref = absd(det<nUnc>(A));
        if (ref < 0.01) continue;      /* nearly singular: relative error meaningless */
        const double got = double(zonotopeVolume(G));
        const double rel = absd(got - ref) / ref, ab = absd(got - ref);
        if (rel > maxRel) maxRel = rel;
        if (ab > maxAbs) maxAbs = ab;
        /* fixed: Gaussian elimination in elim_type rounds every factor and
         * update to its LSB; nUnc^2 of them enter the pivots' product */
        const double tolAbs = isFixed ? nUnc * nUnc * lsb<elim_type>() : 0.0;
        CHECK(ab <= tolAbs + 1e-9 * ref, "volume %.9g, |det| %.9g", got, ref);
    }
    char d[128];
    snprintf(d, sizeof d, "max error %.3g absolute, %.3g relative (elim_type LSB %.3g)", maxAbs, maxRel, lsb<elim_type>());
    endTest("zonotopeVolume", d);
}

/* ======================================================================
   8. Conversions: physical -> ADC code -> normalized (output path) and
      normalized -> DAC code -> physical (input path), against the exact
      affine maps, within an error budget built from what the types store
   ====================================================================== */
static void testConversions()
{
    beginTest();
#if defined(NRMLZ) && defined(CONVERSIONS_MODE)
    /* Exact maps, from the (as stored) physical ranges:
     *   ADC:  code = A*(y - Ymin),          A = ADC_RANGE / (Ymax - Ymin)
     *   ctrl: yn   = C*code - 1,            C = 2 / ADC_RANGE
     *   ctrl: code = E*(un + 1),            E = ADC_RANGE / 2
     *   DAC:  u    = Umin + G*code,         G = (Umax - Umin) / ADC_RANGE */
    const double yMin = double(YMINPHYS), yMax = double(YMAXPHYS);
    const double uMin = double(UMINPHYS), uMax = double(UMAXPHYS);
    const double A = ADC_RANGE / (yMax - yMin), B = -A * yMin, C = 2.0 / ADC_RANGE, D = -1.0;
    const double E = ADC_RANGE / 2.0, F = ADC_RANGE / 2.0;
    const double G = (uMax - uMin) / ADC_RANGE, UB = -uMin / G;

    /* What the stored constants differ by, in ADC codes:
     *   output path: code error |dA|*max|y| + |dB|, then |dC|*ADC_RANGE + |dD| in normalized units
     *   input path:  |dE| + |dF| codes, then |dUB| codes and |dG|*ADC_RANGE physical */
    const double yAbs = std::max(absd(yMin), absd(yMax));
    const double adcCodes   = absd(double(YADCGain) - A) * yAbs + absd(double(YBias) - B);
    const double ctrlYNorm  = absd(double(yConvCoeff) - C) * ADC_RANGE + absd(double(yConvOffset) - D);
    const double ctrlUCodes = absd(double(uConvCoeff) - E) + absd(double(uConvOffset) - F);
    const double dacPhys    = absd(double(UBias) - UB) * G + absd(double(UDACGain) - G) * ADC_RANGE;
    /* code quantization: fixed rounds (1/2 code), float stores codes in an int (truncates: 1 code) */
    const double q = isFixed ? 0.5 : 1.0;
    const double tolY = C * (q + adcCodes) + ctrlYNorm + 2 * lsb<norm_output_type>();
    const double tolU = G * (q + ctrlUCodes) + dacPhys + 2 * lsb<input_type>();

    double maxY = 0.0, maxU = 0.0;
    for (int t = 0; t < 4000; t++)
    {
        const output_type y = uni(yMin, yMax);
        const double ey = absd(double(dig2ctrlY(ADConvertY(y))) - (-1.0 + C * A * (double(y) - yMin)));
        if (ey > maxY) maxY = ey;
        CHECK(ey <= tolY * (1 + 1e-9), "y = %.9g: error %.3g > budget %.3g", double(y), ey, tolY);

        const norm_input_type un = uni(-0.999, 0.999);
        const double eu = absd(double(DAConvertU(ctrlU2dig(un))) - (uMin + G * E * (double(un) + 1.0)));
        if (eu > maxU) maxU = eu;
        CHECK(eu <= tolU * (1 + 1e-9), "u_norm = %.9g: error %.3g > budget %.3g", double(un), eu, tolU);
    }
    char d[300];
    snprintf(d, sizeof d,
             "max error y %.2f codes (budget %.2f), u %.2f codes (budget %.2f); stored constants off by: "
             "ADC %.2f codes, y-ctrl %.2f codes at full scale, u-ctrl %.2f codes, DAC %.2f codes",
             maxY / C, tolY / C, maxU / G, tolU / G, adcCodes, ctrlYNorm / C, ctrlUCodes, dacPhys / G);
    endTest("conversions (ADC/DAC chains)", d);
#else
    SKIP("conversions (ADC/DAC chains)", "needs NRMLZ and CONVERSIONS_MODE");
#endif
}

/* ======================================================================
   9. MADSARX never returns a worse point than its start, in the
      progressive barrier's order: lower violation first, then lower cost
   ====================================================================== */
static void testMadsNotWorse()
{
    beginTest();
    resetControllerState();
    pseudoRandReset();
    arx_coeff_type coeff[Nscen + 1][nTheta], offset[Nscen + 1];
    theta_type thetaScen[Nscen][nUnc];
    realCoefficients(coeff, offset, thetaScen);
    int improved = 0;
    for (int trial = 0; trial < 60; trial++)
    {
        norm_output_type yPast[na];
        norm_input_type uOpt[nOpt], start[nOpt], uPast[nUKnown];
        const double y0 = uni(-0.5, 0.5);
        for (int i = 0; i < na; i++) yPast[i] = y0;
        for (int i = 0; i < nOpt; i++) start[i] = uOpt[i] = uni(-0.8, 0.8);
        for (int i = 0; i < nUKnown; i++) uPast[i] = start[0];
        const norm_output_type yref = uni(-0.7, 0.7);

        cost_type c0[2], c1[2];
        costFunctionArx(c0, yPast, start, uPast, yref, coeff, offset);
        MADSARX(uOpt, uPast, yPast, yref, coeff, offset);
        costFunctionArx(c1, yPast, uOpt, uPast, yref, coeff, offset);
        const double v0 = double(c0[1]), v1 = double(c1[1]), j0 = double(c0[0]), j1 = double(c1[0]);
        CHECK(v1 < v0 || (v1 == v0 && j1 <= j0), "start (viol %g, cost %g) -> result (viol %g, cost %g)", v0, j0, v1, j1);
        if (v1 < v0 || j1 < j0) improved++;
    }
    char d[64];
    snprintf(d, sizeof d, "%d of 60 starts improved", improved);
    endTest("MADSARX not worse than its start", d);
}

/* ======================================================================
   10. Passive learning, closed loop: the true parameters never leave the
       zonotope, and the zonotope does not grow
   ====================================================================== */
static void testPassiveLearningContainment()
{
#if CTRL_MODE == CTRL_MODE_PL && !defined(FIXED) && !defined(CONVERSIONS_MODE)
    beginTest();
    if (nUnc != nGens) { SKIP("passive learning containment", "generator matrix not square"); return; }
    /* Equation-error ARX plant (the model the strip assumes):
     *   y(k) = sum_i a_i y(k-i) + sum_i b_i u(k-nd-i) + e(k),  |e(k)| <= SIGMA_UNNORM
     * The strip at step k uses phi(k) and y(k), so theta_true satisfies
     * |y(k) - phi(k)^T theta_true| <= sigma, and Bravo et al.'s update
     * (outer bound of zonotope intersect strip) must keep it inside. */
    const double sigma = double(SIGMA_UNNORM);
    double worstXi = 0.0, volRatio = 0.0, distRatio = 0.0, vol0First = 0.0;
    const int runs = 5, steps = 300;
    for (int run = 0; run < runs; run++)
    {
        resetControllerState();
        pseudoRandReset();
        /* a true theta inside the initial zonotope (certain parameters exact) */
        double thetaTrue[nTheta];
        double xiTrue[nGens];
        for (int j = 0; j < nGens; j++) xiTrue[j] = uni(-0.9, 0.9);
        for (int i = 0; i < nTheta; i++)
        {
            thetaTrue[i] = thetaNominalInit[i];
            for (int j = 0; j < nGens; j++) thetaTrue[i] += generatorsInit[i][j] * xiTrue[j];
        }
        double yH[na], uH[nb + nd - 1];
        for (int i = 0; i < na; i++) yH[i] = yHistInit[i];
        for (int i = 0; i < nb + nd - 1; i++) uH[i] = uHistInit[i];
        const double yref = 0.5 * (double(YMINPHYS) + double(YMAXPHYS));

        auto normTrue = [&](int k) {   /* theta_true of uncertain parameter k, normalized */
            const int i = UNC_MAP.full[k];
            return (thetaTrue[i] - thetaNominalInit[i]) / DgInit.Dg[i];
        };
        auto dist = [&]() {
            double s = 0.0;
            for (int k = 0; k < nUnc; k++) { const double d = normTrue(k) - double(thetaCenter[k]); s += d * d; }
            return std::sqrt(s);
        };
        const double vol0 = double(zonotopeVolume(thetaGens)), dist0 = dist();
        if (run == 0) vol0First = vol0;
        double volPrev = vol0;
        int stepsOutside = 0;
        for (int k = 0; k < steps; k++)
        {
            double y = uni(-sigma, sigma);
            for (int i = 0; i < nTheta; i++) y += thetaTrue[i] * regressor(i, yH, uH);
            const double u = double(controller(y, yref));      /* no conversions: physical in/out */
            for (int i = na - 1; i > 0; i--) yH[i] = yH[i - 1];
            yH[0] = y;
            for (int i = nb + nd - 2; i > 0; i--) uH[i] = uH[i - 1];
            uH[0] = u;

            /* theta_true in Z(k): solve G xi = theta_true - c, need |xi| <= 1 */
            double A[nUnc][nUnc], b[nUnc], xi[nUnc];
            for (int r = 0; r < nUnc; r++)
            {
                for (int j = 0; j < nUnc; j++) A[r][j] = double(thetaGens[r][j]);
                b[r] = normTrue(r) - double(thetaCenter[r]);
            }
            double m = 1e300;
            if (solve<nUnc>(A, b, xi)) { m = 0.0; for (int j = 0; j < nUnc; j++) if (absd(xi[j]) > m) m = absd(xi[j]); }
            if (m > worstXi) worstXi = m;
            if (m > 1.0 + 1e-7) stepsOutside++;

            const double vol = double(zonotopeVolume(thetaGens));
            CHECK(vol <= volPrev * (1.0 + 1e-9), "run %d, step %d: volume grew %g -> %g", run, k, volPrev, vol);
            volPrev = vol;
        }
        CHECK(stepsOutside == 0, "run %d: theta_true outside the zonotope at %d of %d steps", run, stepsOutside, steps);
        volRatio += volPrev / vol0 / runs;
        distRatio += dist() / dist0 / runs;
    }
    char d[192];
    snprintf(d, sizeof d, "max |xi_true| %.6f; after %d steps: volume %.3g -> x%.3g, centre-to-truth distance x%.3g (mean of %d runs)",
             worstXi, steps, vol0First, volRatio, distRatio, runs);
    endTest("passive learning containment", d);
#else
    SKIP("passive learning containment", "needs CTRL_MODE_PL, floating point, no CONVERSIONS_MODE");
#endif
}

/* ======================================================================
   main
   ====================================================================== */
int main()
{
    printf("    [%s, %s%s%s, system %d, ctrl mode %d]\n", isFixed ? "fixed" : "float",
           NRMLZ_PRINT[0] == 't' ? "NRMLZ " : "", CONVERSIONS_MODE_PRINT[0] == 't' ? "CONVERSIONS " : "",
           DC_STEPS ? "DELAY_COMPENSATION" : "", ACTIVE_SYSTEM, CTRL_MODE);
    testComputeArxOutput();
    testComputeArxCoeffs();
    testConstraintViolation();
    testCostFunction();
    testGenerateScenarios();
    testPseudoRand();
    testZonotopeVolume();
    testConversions();
    testMadsNotWorse();
    testPassiveLearningContainment();
    printf("    => %s\n", g_failed ? "FAILED" : "all passed");
    return g_failed ? 1 : 0;
}
