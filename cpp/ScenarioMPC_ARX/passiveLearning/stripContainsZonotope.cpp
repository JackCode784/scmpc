#include "setup.h"

int stripContainsZonotope( const output_type yHist[na],
                            const input_type uHist[nb + nk - 1], 
                            const alg_type stripoffset, 
                            const theta_type thetaCenter[nTheta],
                            const theta_type gens[nTheta][nGens])
{
    int contained = 1;
    alg_type cproj = 0;
    alg_type gproj = 0;

    for(int i = 0; i < nTheta; i++) {
        cproj += ((i < na) ? (alg_type)yHist[i] : (alg_type)uHist[i-na+nk-1]) * (alg_type)thetaCenter[i];
    }

    if (gproj + cproj > stripoffset || gproj - cproj < -stripoffset)
        contained = 0;

    return contained;
}