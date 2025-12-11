#ifndef _MYLIB_HPP_
#define _MYLIB_HPP_

// library including parameters depending on the application
#include "boostParams.h"

void MPCwithMADS(data_in x[NX], data_in par[NP], data_in ref[NREF], data_out uOpt[NU]);
void shiftOpt(data_u optimum[N]);
void MADS(data_u optimum[N], data_x x[NX], data_x par[NP], data_x ref[NREF]);
void generatePollMatrix(data_u currentPoint[N], data_meshExp frameSize[N], data_meshExp meshSize[N], data_u pollMatrix[N][2 * N]);
void generatePollDirections(data_rand randomVector[N], data_meshExp frameSize[N], data_meshExp meshSize[N], data_dir directions[N][2*N]);
void generateHouseholderMatrix(data_rand v[N], data_rand H[N][N]);
//void generateHMax(data_rand hMax[N], data_rand H[N][N]);
void pseudoRand(data_rand randomVector[N]);
void progressiveBarrierPolling(data_cost cost[2], data_u currentPoint[N], data_u pollMatrix[N][2*N], data_meshExp frameSize[N], data_x x[NX], data_x par[NP], data_x ref[NREF]);

#endif
