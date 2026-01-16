#include "../setup.h"

void transpose(const theta_type** M, int nRows, int nCols, theta_type** MT)
{
    for(int i = 0; i < nRows; i++)
    {
        for(int j = 0; j < nCols; j++)
        {
            MT[i][j] = M[j][i];
        }
    }
    return;
}