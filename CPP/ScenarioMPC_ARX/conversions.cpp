#include "setup.h"

// #ifndef DEBUG_MATLAB
/*
 *	Each of these convert an input/output sample to digital or analogue
 */
digital_output_type ADConvertY(output_type yAn)
{
	digital_output_type yDig = YADCGain * yAn + YBias;
	return yDig;
}

output_type DAConvertY(digital_output_type yDig)
{
	output_type yAn = (yDig - YBias) * YDACGain;
	return yAn;
}

digital_input_type ADConvertU(input_type uAn)
{
	digital_input_type uDig = UADCGain * uAn + UBias;
	return uDig;
}

input_type DAConvertU(digital_input_type uDig)
{
	input_type uAn = (uDig - UBias) * UDACGain;
	return uAn;
}
// #endif
