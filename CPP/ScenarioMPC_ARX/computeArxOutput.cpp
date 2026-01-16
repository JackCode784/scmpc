#include "setup.h"

// computeArxOutput     
// Computes ARX ***nominal*** model output using past output and input samples.
// ARX output is the scalar product between past output/input samples and theta
// parameters
output_type computeArxOutput(const output_type yPast[na], const input_type uSamples[nb+nd-1], const theta_type theta[nTheta])
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

    for(int i = 0; i < na; i++)
    {
        #ifdef PRAGMAS
        #endif
        yRes += yPast[i] * theta[i];
    }

    for(int i = na; i < nTheta; i++)
    {
        #ifdef PRAGMAS
        #endif
        yRes += uSamples[i - na + nd - 1] * theta[i];
    }

    return yRes;
}
