#include "setup.h"

/*
 *	Each of these convert an input/output sample to digital or analog
 */
digital_output_type ADConvertY(const output_type yAn)
{
// #ifdef FIXED
// 	float YADCGainf = YADCGain.to_float();
// 	float yAnf = yAn.to_float();
// 	float YBiasf = YBias.to_float();
// #endif
	digital_output_type yDig = YADCGain * yAn + YBias;
	
	#ifndef FIXED
	/* Saturate output if outside ADC values */
	yDig = (yDig < (digital_output_type)ADC_MIN) ? (digital_output_type)ADC_MIN :
			(yDig > (digital_output_type)ADC_MAX) ? (digital_output_type)ADC_MAX :
			yDig;
	#endif
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

	#ifdef NRMLZ
	// Internal normalization
	yAn = yNormGain * yAn + yNormOffset;
	#endif

	return yAn;
}

digital_input_type ADConvertU(const input_type uAn)
{
// 	#ifdef FIXED
// 	float UADCGainf = UADCGain.to_float();
// 	float uAnf = uAn.to_float();
// 	float UBiasf = UBias.to_float();
// #endif
	#ifdef NRMLZ
	digital_input_type uDig = (uAn - uNormOffset) * uNormGainInverse;
	uDig = UADCGain * uAn + UBias;
	#else
	digital_input_type uDig = UADCGain * uAn + UBias;
	#endif
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
