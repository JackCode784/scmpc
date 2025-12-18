#include "setup.h"

// computeArxOutput     
// Computes ARX ***nominal*** model output using past output and input samples.
// ARX output is the scalar product between past output/input samples and theta
// parameters
output_type computeArxOutput(const output_type yPast[na], const input_type uSamples[nb+nd], const theta_type theta[nTheta])
{
    output_type yRes = 0;

    for(int i = 0; i < na; i++)
        yRes += yPast[i] * theta[i];

    for(int i = na; i < nTheta; i++)
        yRes += uSamples[i-na+nd] * theta[i];

    return yRes;
}
