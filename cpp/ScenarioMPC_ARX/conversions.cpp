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
#endif

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

/* Convert directly from digital (normalized) output (input) 
	samples to normalized (digital) output (input) samples 
	
	This way the controller never sees variables in their original,
	system-dependent domain
*/
#if defined(NRMLZ) && defined(CONVERSIONS_MODE)
norm_output_type dig2normY(digital_output_type yDig)
{
	norm_output_type yNorm = yDig * yDANormGain + yDANormOffset;
	return yNorm;
}

digital_input_type norm2digU(norm_input_type uNorm)
{
	digital_input_type uDig = uNorm*uNormADGain + uNormADOffset;
	return uDig;
}
#endif