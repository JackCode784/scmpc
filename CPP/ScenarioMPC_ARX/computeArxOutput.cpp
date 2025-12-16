#include "setup.h"

// computeArxOutput     
// Computes ARX ***nominal*** model output using past output and input samples.
// ARX output is the scalar product between past output/input samples and theta
// parameters
void computeArxOutput(output_type yRes[], const output_type yPast[na], const input_type uSamples[nb+nd], const theta_type theta[nTheta])
{
    yRes[0] = 0;

    for(int i = 0; i < na; i++)
        yRes[0] += yPast[i] * theta[i];

    for(int i = na; i < nTheta; i++)
        yRes[0] += uSamples[i-na+nd] * theta[i];
}
