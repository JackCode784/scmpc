#include "setup.h"

/* computeArxOutput
One-step-ahead ARX prediction from past output/input samples:
    y = offset + sum_i sample[i] * coeff[i]
where coeff/offset are the effective coefficients computed once per
controller() call by computeArxCoeffs() below.
*/
norm_output_type computeArxOutput(const norm_output_type yPast[na],
                                  const norm_input_type  uSamples[nb+nk-1],
                                  const arx_coeff_type   coeff[nTheta],
                                  const arx_coeff_type   offset)
{
    #ifdef PRAGMAS
    #pragma HLS INLINE
    #pragma HLS ARRAY_PARTITION variable=coeff dim=1 complete
    #pragma HLS ARRAY_PARTITION variable=yPast dim=1 complete
    #pragma HLS ARRAY_PARTITION variable=uSamples dim=1 complete
    #endif

    #ifndef NRMLZ
    /* Without normalization coeff == theta and offset == 0: unchanged
     * from the original  yRes += sample * theta  formulation. */
    norm_output_type yRes = offset;
    for(int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        yRes += ((i < na) ? yPast[i] : uSamples[i-na+nk-1]) * coeff[i];
    }
    return yRes;
    #else
    /* See arx_acc_type in types.h: products truncated to 21 fractional
     * bits, narrow wrap-around adds, ONE rounding/saturating cast at the end. */
    arx_acc_type acc = (arx_acc_type)offset;
    for(int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        acc += (arx_acc_type)(((i < na) ? yPast[i] : uSamples[i-na+nk-1]) * coeff[i]);
    }
    return (norm_output_type)acc;
    #endif
}

/* computeArxCoeffs
Fold the normalization into the ARX coefficients, once per controller()
call, for every parameter vector costFunctionArx will predict with: the
Nscen scenarios (rows 0..Nscen-1) and the zonotope centre (row Nscen).

The normalized prediction used to be evaluated, per term, as
    sample*theta*myInvDmDg + qmyInvDmDg*theta + sample*myInvDmc0
(3-4 multiplies, 2 wide adds and 2 rounding/saturating casts per term,
inside every prediction step of every costFunctionArx call). Grouping by
sample:
    sample*(myInvDmDg*theta + myInvDmc0)  +  qmyInvDmDg*theta
          \_______ coeff[i] ___________/     \_ part of offset _/
Both brackets depend only on theta, which is fixed for the whole
controller() call, so they are computed here once and the prediction
itself reduces to one multiply per term. Mathematically identical; in
fixed point only the rounding points move (see arx_coeff_type/arx_acc_type
in types.h for the precision budget).

The scenario/centre vectors only hold the nUnc UNCERTAIN parameters (the
zonotope's own dimension, setup.h's UNC_MAP); each certain parameter i is
re-inserted here as its known value: THETA_NOMINAL_UNNORM[i] without
NRMLZ, 0 with it (its normalized value; myInvDmDg[i] = 0 then makes
coeff[i] = myInvDmc0[i], its exact nominal coefficient). The i-loop is
fully unrolled, so UNC_MAP.red[i] is a constant and the choice costs no
hardware.
*/
void computeArxCoeffs(const theta_type thetaScenarios[Nscen][nUnc],
                      const theta_type thetaNominal  [nUnc],
                      arx_coeff_type   coeff         [Nscen+1][nTheta],
                      arx_coeff_type   offset        [Nscen+1])
{
    #ifdef PRAGMAS
    #pragma HLS INLINE
    #endif

    /* Scenario rows are only meaningful (and thetaScenarios only
     * initialized) when scenarios are in use; the centre row always is. */
    #if defined(USE_SCENS_COST) || defined(USE_SCENS_CONSTR)
    constexpr int firstRow = 0;
    #else
    constexpr int firstRow = Nscen;
    #endif

    /*
     * Row loop deliberately left rolled: this runs once per controller()
     * call (vs. 43 costFunctionArx calls), so sharing one row's worth of
     * multipliers across the Nscen+1 rows costs a few tens of cycles in
     * total, while unrolling would add (Nscen+1)*2*nTheta = 30 DSPs.
     */
    for(int l = firstRow; l < Nscen+1; l++)
    {
        #ifndef NRMLZ
        for(int i = 0; i < nTheta; i++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            const int k = UNC_MAP.red[i];
            coeff[l][i] = (k < 0)      ? theta_type(THETA_NOMINAL_UNNORM_TABLE[i]) :
                          (l < Nscen)  ? thetaScenarios[l][k] : thetaNominal[k];
        }
        offset[l] = 0;
        #else
        arx_coeff_type off = STRIP_qmyInvDmc0 + yNormOffset;
        for(int i = 0; i < nTheta; i++)
        {
            #ifdef PRAGMAS
            #pragma HLS UNROLL
            #endif
            const int k = UNC_MAP.red[i];
            theta_type th = (k < 0)     ? theta_type(0) :
                            (l < Nscen) ? thetaScenarios[l][k] : thetaNominal[k];
            coeff[l][i] = STRIP_myInvDmDg(i) * th + STRIP_myInvDmc0(i);
            off += STRIP_qmyInvDmDg(i) * th;
        }
        offset[l] = off;
        #endif
    }
}
