#include "setup.h"

#ifdef CONVERSIONS_MODE
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
	/* Emulate DAC saturation when using floating point */
	yDig = (yDig < ADC_MIN) ? ADC_MIN : 
			(yDig > ADC_MAX) ? ADC_MAX : yDig;
	#endif
	return yDig;
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
#endif

/* Convert digital output sample to output type used in controller */
norm_output_type dig2ctrlY(const digital_output_type yDig)
{
	#ifdef PRAGMAS
	#pragma hls inline
	#endif
	norm_output_type yCtrl = (norm_output_type)(yConvCoeff*yDig) + yConvOffset;
	return yCtrl;
}

/* Convert input sample used in controller to digital */
digital_input_type ctrlU2dig(const norm_input_type uCtrl)
{
	#ifdef PRAGMAS
	#pragma hls inline
	#endif
	digital_input_type uDig = (digital_input_type)(uConvCoeff*uCtrl) + uConvOffset;
	#ifndef FIXED
	/* Emulate ADC saturation when using floating point */
	uDig = (uDig < ADC_MIN) ? ADC_MIN : 
			(uDig > ADC_MAX) ? ADC_MAX : 
			uDig;
	#endif
	return uDig;
}


#ifdef NRMLZ
/** The following functions normalize (denormalize) output (input) samples
 * They're used iff NRMLZ is defined.
 */
norm_output_type normalizeY(output_type yAn)
{
	norm_output_type yNorm = yNormGain * yAn + yNormOffset;
	return yNorm;
}

input_type denormalizeU(norm_input_type uNorm)
{
	input_type uAn = (uNorm - uNormOffset) * uNormGainInverse;
	return uAn;
}
#endif

/* Not used on field, only for debugging */
// output_type DAConvertY(const digital_output_type yDig)
// {
// 	//  	#ifdef FIXED
// 	//  	float YDACGainf = YDACGain.to_float();
// 	//  	float yDigf = yDig.to_float();
// 	//  	float YBiasf = YBias.to_float();
// 	//  #endif
// 	output_type yAn = (yDig - YBias) * YDACGain;
// 	return yAn;
// }

// /* Not used on field, only for debugging */
// digital_input_type ADConvertU(const input_type uAn)
// {
// 	// 	#ifdef FIXED
// 	// 	float UADCGainf = UADCGain.to_float();
// 	// 	float uAnf = uAn.to_float();
// 	// 	float UBiasf = UBias.to_float();
// 	// #endif
// 	digital_input_type uDig = UADCGain * uAn + UBias;
// 	return uDig;
// }