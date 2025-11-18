#include "controller.h"

void quadraticTerm(data_cost term, fxd x[], fxd M[][], int dim) {

	int i,j;
	for (i=0;i<dim;i++)
		for (j=0;j<dim;j++)
			term+=x[i]*M[i][j]*x[j];

}
