#include "controller.h"

int main() {

	float tmp;

	fxd_in x_in[nX] = { 2047, 2047 };
	fxd_out u_opt[1];
	fxd_in ref_in[nRef] = { 3328 };

	tmp = x_in[0].to_float();

	control(x_in, u_opt, ref_in);

	FILE *fp;
	fp = fopen("output.txt","w");
	fprintf(fp,"%f %f %f %f",x_in[0].to_float(),x_in[1].to_float(),ref_in[0].to_float(),u_opt[0].to_float());
	fclose(fp);

	return 0;

}
