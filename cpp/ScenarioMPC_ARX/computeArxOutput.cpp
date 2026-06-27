#include "setup.h"

/* computeArxOutput     
Computes ARX model output using past output and input samples.
ARX output is the scalar product between past output/input samples and theta
parameters 
*/
output_type computeArxOutput(const output_type yPast[na], const input_type uSamples[nb+nk-1], const theta_type theta[nTheta])
{
    #ifdef PRAGMAS
    // #pragma HLS INLINE
    #endif
    output_type yRes = 0;


    #ifdef PRAGMAS
    #pragma HLS array_partition variable=theta dim=1 complete
    #pragma HLS array_partition variable=yPast dim=1 complete
    #pragma HLS array_partition variable=uSamples dim=1 complete
    #endif 

    for(int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #endif
        
        #ifndef NRMLZ
        yRes += ((i < na) ? yPast[i] : uSamples[i-na+nk-1]) * theta[i];
        #else
        /* Compute normalized output from normalized I/O samples */
        yRes += ((i < na) ? yPast[i] : uSamples[i-na+nk-1]) * theta[i] * myInvDmDg[i] +
                qmyInvDmDg[i] * theta[i] +
                ((i < na) ? yPast[i] : uSamples[i-na+nk-1]) * myInvDmc0[i];
        #endif
    }
    #ifdef NRMLZ
    yRes += qmyInvDmc0 + yNormOffset;
    #endif

    return yRes;
}