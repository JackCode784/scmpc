#include "setup.h"
#ifdef PL

// Bounds the intersection of a strip and a zonotope with another zonotope,
// the size of which is optimized with an analytic formula for a lambda parameter
// THERE IS ONE DIVISION HERE!!!
void boundStripZonotopeIntersection(const output_type yCurr, const output_type yPast[na], const input_type uSamples[nb + nd - 1], const theta_type oldCenter[nTheta],
                                    const theta_type oldGens[nTheta][nTheta], theta_type newCenter[nTheta], theta_type newGens[nTheta][nTheta + 1])
{
    // Information operations
    // output_type d[2] = {yCurr + EPSILON, -yCurr + EPSILON};

    /*
     * Zonotope update
     */

    // Lambda computation
    alg_type lambda[nTheta];
    alg_type tmp[nTheta][nTheta];
    alg_type div = EPSILON * EPSILON;

    for (int i = 0; i < nTheta; i++)
    {
        // newCenter[i] = oldCenter[i]; // useful afterwards
        lambda[i] = 0;

        for (int j = 0; j < nTheta; j++)
        {
            tmp[i][j] = 0;

            for (int k = 0; k < nTheta; k++)
                tmp[i][j] += oldGens[i][k] * oldGens[j][k];
        }

        for (int j = 0; j < nTheta; j++)
        {
            // From most recent to oldest sample
            lambda[i] += tmp[i][j] * ((j < na) ? yPast[j] : uSamples[j - na + nd - 1]);
        }
    }

    for (int i = 0; i < nTheta; i++)
    {
        div += ((i < na) ? yPast[i] : uSamples[i - na + nd - 1]) * lambda[i];
    }

    for (int i = 0; i < nTheta; i++)
    {
        lambda[i] /= div;
    }

    // New center
    alg_type term = yCurr;
    for (int i = 0; i < nTheta; i++)
    {
        term -= ((i < na) ? yPast[i] : uSamples[i - na + nd - 1]) * oldCenter[i];
    }

    for (int i = 0; i < nTheta; i++)
        newCenter[i] = lambda[i] * term + oldCenter[i];

    // New generators
    for (int i = 0; i < nTheta; i++)
    {
        newGens[i][nTheta] = lambda[i] * EPSILON;

        for (int j = 0; j < nTheta; j++)
        {
            newGens[i][j] = 0; // preparation for upcoming for loop
            tmp[i][j] = (i == j) ? 1 : 0;
            tmp[i][j] -= lambda[i] * ((j < na) ? yPast[j] : uSamples[j - na + nd - 1]);
        }

        for (int j = 0; j < nTheta; j++)
        {
            for (int k = 0; k < nTheta; k++)
            {
                newGens[i][j] += tmp[i][k] * oldGens[k][j];
            }
        }
    }

    return;
}
#endif
