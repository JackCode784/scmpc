#include "setup.h"

/*
 *	Each of these convert an input/output sample to digital or analogue
 */
digital_output_type ADConvertY(const output_type yAn)
{
// #ifdef FIXED
// 	float YADCGainf = YADCGain.to_float();
// 	float yAnf = yAn.to_float();
// 	float YBiasf = YBias.to_float();
// #endif
	digital_output_type yDig = YADCGain * yAn + YBias;
	
	/* Saturate output if outside ADC values */
	yDig = (yDig < ADC_MIN) ? ADC_MIN :
			(yDig > ADC_MAX) ? ADC_MAX :
			yDig;
	return yDig;
}

output_type DAConvertY(const digital_output_type yDig)
{
//  	#ifdef FIXED
//  	float YDACGainf = YDACGain.to_float();
//  	float yDigf = yDig.to_float();
//  	float YBiasf = YBias.to_float();
//  #endif
	output_type yAn = (yDig - YBias) * YDACGain;
	return yAn;
}

digital_input_type ADConvertU(const input_type uAn)
{
// 	#ifdef FIXED
// 	float UADCGainf = UADCGain.to_float();
// 	float uAnf = uAn.to_float();
// 	float UBiasf = UBias.to_float();
// #endif
	digital_input_type uDig = UADCGain * uAn + UBias;
	return uDig;
}

input_type DAConvertU(const digital_input_type uDig)
{
// 	#ifdef FIXED
// 	float UDACGainf = UDACGain.to_float();
// 	float UDigf = uDig.to_float();
// 	float UBiasf = UBias.to_float();
// #endif
	input_type uAn = (uDig - UBias) * UDACGain;
	return uAn;
}
