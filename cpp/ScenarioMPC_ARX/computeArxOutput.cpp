#include "setup.h"

/* computeArxOutput     
Computes ARX model output using past output and input samples.
ARX output is the scalar product between past output/input samples and theta
parameters 
*/
norm_output_type computeArxOutput(const norm_output_type yPast[na], const norm_input_type uSamples[nb+nk-1], const theta_type theta[nTheta])
{
    #ifdef PRAGMAS
    #pragma HLS INLINE
    #pragma HLS ARRAY_PARTITION variable=theta dim=1 complete
    #pragma HLS ARRAY_PARTITION variable=yPast dim=1 complete
    #pragma HLS ARRAY_PARTITION variable=uSamples dim=1 complete
    #endif

    norm_output_type yRes = 0;


    for(int i = 0; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #pragma HLS UNROLL
        #endif
        
        #ifndef NRMLZ
        yRes += ((i < na) ? yPast[i] : uSamples[i-na+nk-1]) * theta[i];
        #else
        /*
         * Compute normalized output from normalized I/O samples.
         *
         * sampleTheta breaks the ((sample)*theta[i])*myInvDmDg[i] chain
         * at an explicit, appropriately-sized intermediate type instead
         * of letting it grow losslessly to 36 bits - see arx_partial_type
         * in types.h for the full reasoning and the synthesis evidence
         * that motivated this (introduced to fix CTRL_MODE_SCMPC's
         * PRAGMA_PROFILE_LATENCY exceeding 100% LUT utilisation).
         */
        arx_partial_type sampleTheta = (arx_partial_type)(((i < na) ? yPast[i] : uSamples[i-na+nk-1]) * theta[i]);
        yRes += (norm_output_type)(sampleTheta * myInvDmDg[i] +
                qmyInvDmDg[i] * theta[i] +
                ((i < na) ? yPast[i] : uSamples[i-na+nk-1]) * myInvDmc0[i]);
        #endif
    }
    #ifdef NRMLZ
    yRes += (norm_output_type)(qmyInvDmc0 + yNormOffset);
    #endif

    return yRes;
}