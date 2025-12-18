#include "setup.h"

// Take digital inputs, convert them to fixed point, find optimal input, return it as digital
void controller(digital_input_type uOptDig[NhorU], theta_type thetaScenarios[Nscen + 1][nTheta], const digital_output_type yInitDig[na], const digital_input_type uInitDig[nb + nd - 1], const digital_output_type yrefDig)
{
    // Convert digital inputs to fixed point/float
    input_type uOpt[NhorU];
    input_type uInit[nb + nd - 1];
    output_type yInit[na];
    output_type yref;

    for(int i = 0; i < na; i++)
        yInit[i] = DAConvertY(yInitDig[i]);

    for(int i = 0; i < nb + nd - 1; i++)
        uInit[i] = DAConvertU(uInitDig[i]);

    for(int i = 0; i < NhorU; i++)
        uOpt[i] = DAConvertU(uOptDig[i]);

    yref = DAConvertY(yrefDig);

    // Call generateSCMPCControl
    generateSCMPCControl(uOpt, thetaScenarios, yInit, uInit, yref);

    // Convert uOpt back to digital
    for(int i = 0; i < NhorU; i++)
        uOptDig[i] = ADConvertU(uOpt[i]);

}