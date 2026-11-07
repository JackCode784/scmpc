#include "setup.h"

#ifdef CONVERSIONS_MODE
/*
*	Each of these convert an input/output sample to digital or analog
*/
digital_output_type ADConvertY(const output_type yAn)
{
	digital_output_type yDig = dig::yADCGain * yAn + dig::yBias;
	#ifndef FIXED
	/* Emulate DAC saturation when using floating point */
	yDig = (yDig < dig::codeMin) ? dig::codeMin : 
	(yDig > dig::codeMax) ? dig::codeMax : yDig;
	#endif

	#ifdef DEBUG_PRINT
	float YADCGain_f = dig::yADCGain.to_float();
	float yAn_f = yAn.to_float();
	float YBias_f = dig::yBias.to_float();
	float yDig_f = yDig.to_float();
	float YADCGAINF = float(dig::codeRange) / (YMAXPHYS - YMINPHYS).to_float(); /* cross-check of dig::yADCGain: the ADC spans the PHYSICAL range */
	#endif

	return yDig;
}

input_type DAConvertU(const digital_input_type uDig)
{
	input_type uAn = (uDig - dig::uBias) * dig::uDACGain;
	
	#ifdef DEBUG_PRINT
	float UDACGainf = dig::uDACGain.to_float();
	float UDigf = uDig.to_float();
	float UBiasf = dig::uBias.to_float();
	float uAn_f = uAn.to_float();
	#endif

	return uAn;
}
#endif

/* Convert digital output sample to output type used in controller */
norm_output_type dig2ctrlY(const digital_output_type yDig)
{
	#ifdef PRAGMAS
	/* Single multiply-add, called once per controller() call: inlining
	 * removes the function-call RTL boundary (and its own tiny FSM)
	 * so the operation merges into whatever schedule surrounds the call. */
	#pragma HLS INLINE
	#endif
	norm_output_type yCtrl = conv::yConvCoeff*yDig + conv::yConvOffset;

	#ifdef DEBUG_PRINT
	float yDig_f = yDig.to_float();
	float yConvCoeff_f = conv::yConvCoeff.to_float();
	float yConvOffset_f = conv::yConvOffset.to_float();
	float yCtrl_f = yCtrl.to_float();
	#endif
	
	return yCtrl;
}

/* Convert input sample used in controller to digital */
digital_input_type ctrlU2dig(const norm_input_type uCtrl)
{
	#ifdef PRAGMAS
	#pragma HLS INLINE
	#endif
	digital_input_type uDig = conv::uConvCoeff*uCtrl + conv::uConvOffset;
	#if !defined(FIXED) && defined(CONVERSIONS_MODE)
	/* Emulate ADC saturation when using floating point. dig::codeMin/dig::codeMax
	 * only exist under CONVERSIONS_MODE (setup.h); without it,
	 * digital_input_type == input_type (types.h) and there is no ADC
	 * code to saturate - the old #ifndef FIXED alone made "FIXED off +
	 * CONVERSIONS_MODE off" (pure floating-point, no ADC/DAC modelling)
	 * fail to compile with "dig::codeMin was not declared in this scope". */
	uDig = (uDig < dig::codeMin) ? dig::codeMin :
			(uDig > dig::codeMax) ? dig::codeMax :
			uDig;
	#endif

	#ifdef DEBUG_PRINT
	float uConvCoeff_f = conv::uConvCoeff.to_float();
	float uCtrl_f = uCtrl.to_float();
	float uConvOffset_f = conv::uConvOffset.to_float();
	float uDig_f = uDig.to_float();
	#endif

	return uDig;
}


#ifdef NRMLZ
/** The following functions normalize (denormalize) output (input) samples
 * They're used iff NRMLZ is defined.
 */
norm_output_type normalizeY(output_type yAn)
{
	norm_output_type yNorm = nrmlz::yNormGain * yAn + nrmlz::yNormOffset;
	return yNorm;
}

input_type denormalizeU(norm_input_type uNorm)
{
	input_type uAn = (uNorm - nrmlz::uNormOffset) * nrmlz::uNormGainInverse;
	return uAn;
}
#endif

/* Not used on field, only for debugging */
// output_type DAConvertY(const digital_output_type yDig)
// {
// 	//  	#ifdef FIXED
// 	//  	float YDACGainf = dig::yDACGain.to_float();
// 	//  	float yDigf = yDig.to_float();
// 	//  	float YBiasf = dig::yBias.to_float();
// 	//  #endif
// 	output_type yAn = (yDig - dig::yBias) * dig::yDACGain;
// 	return yAn;
// }

// /* Not used on field, only for debugging */
// digital_input_type ADConvertU(const input_type uAn)
// {
// 	// 	#ifdef FIXED
// 	// 	float UADCGainf = dig::uADCGain.to_float();
// 	// 	float uAnf = uAn.to_float();
// 	// 	float UBiasf = dig::uBias.to_float();
// 	// #endif
// 	digital_input_type uDig = dig::uADCGain * uAn + dig::uBias;
// 	return uDig;
// }
