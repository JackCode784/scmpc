#include "setup.h"

// Computes input zonotope order one interval hull
// no const in input arguments to give be able to modify input zonotope and return the same variables
void intervalHull(theta_type newCenter[nTheta], theta_type newGens[nTheta][nTheta], const theta_type oldCenter[nTheta], const theta_type oldGens[nTheta][nTheta+1])
{
    for(int i = 0; i < nTheta; i++)
    {
        newCenter[i] = oldCenter[i];

        for(int j = 0;  j < nTheta; j++)
        {   
            newGens[i][j] = 0;
            newGens[i][i] += (oldGens[i][j] > 0) ? oldGens[i][j] : -oldGens[i][j];
        }

        // Last term missing
        newGens[i][i] += (oldGens[i][nTheta] > 0) ? oldGens[i][nTheta] : -oldGens[i][nTheta];
    }
    return;
}
