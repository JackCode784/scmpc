#include "../setup.h"

void reduceOrderPCA(const theta_type oldCenter[nTheta], const theta_type** oldGens, theta_type newCenter[nTheta], theta_type** newGens)
{
    theta_type X[nTheta][2*nTheta];
    theta_type cov[nTheta][nTheta];
    theta_type U[nTheta][nTheta];
    theta_type S[nTheta][nTheta];
    theta_type V[nTheta][nTheta];

    for(int i = 0; i < nTheta; i++)
    {
        for(int j = 0; j < nTheta; j++)
        {
            cov[i][j] = 0;
            X[i][j] = oldGens[i][j];
            X[i][j+nTheta] = -oldGens[i][j];
        }
    }

    // Compute covariance matrix
    for(int i = 0; i < nTheta; i++)
    {
        for(int j = 0; j < nTheta; j++)
        {
            for(int k = 0; k < nTheta; k++)
            {
                cov[i][j] += X[i][k] * X[j][k] + X[i][k+nTheta] * X[j][k+nTheta];
            }
        }
    }

    // SVD decomposition of covariance matrix
    // SVD(cov, nTheta, nTheta, U, S, V);

    // Reduced zonotope


    return;
}