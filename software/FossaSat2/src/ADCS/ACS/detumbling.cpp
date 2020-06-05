/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: detumbling.c
   04/30/20

   This file drives the detumbling procedure commands
*/

/************************* Headers **************************/
#include <math.h>
#include "../ADCS/adcs.h"

/*********************** Main function ***********************/
void ACS_BdotFunction(const float omega[], const float mag[], const float orbitalInclination,  const float orbitalPeriod, float intensity[]) {
    // Constants definition
    // Module of the magnetic field intensity
    const double B_module = ADCS_VectorNorm(mag) + adcsParams.BmodTol;
    FOSSASAT_DEBUG_PRINT(F("B_module = \t"));
    FOSSASAT_DEBUG_PRINTLN(B_module, 4);

    // Controller gain constant
    const float gainConstant = 4.0*M_PI * (1.0 + sin(orbitalInclination)) * adcsParams.minInertialMoment/orbitalPeriod;
    FOSSASAT_DEBUG_PRINT(F("gainConstant = "));
    FOSSASAT_DEBUG_PRINTLN(gainConstant, 4);

    // General gain
    const float gainGeneral = gainConstant/pow(B_module, 2);
    FOSSASAT_DEBUG_PRINT(F("gainGeneral = \t"));
    FOSSASAT_DEBUG_PRINTLN(gainGeneral, 4);

    // Coil magnetic characteristics
    // TODO configurable?
    const double coilChar[3][3] = { {1.88 * pow(10, 5), 0,                 0},
                                    {0,                 6.1 * pow(10, 5),  0},
                                    {0,                 0,                 5.96 * pow(10, 5)} };

    // Generate the control law by means of a vector product: m = a*(B x omega) = (K/B_module)*(B x omega)
    float controlLaw[3];
    controlLaw[0] = gainGeneral*(omega[1]*mag[2] - omega[2]*mag[1]);
    controlLaw[1] = gainGeneral*(omega[2]*mag[0] - omega[0]*mag[2]);
    controlLaw[2] = gainGeneral*(omega[0]*mag[1] - omega[1]*mag[0]);

    // Magnetic moment needed at the axes: calculated by minimization of a least squares problem
    float magMoment[3];
    magMoment[0] = (controlLaw[2]*mag[1] - mag[2]*controlLaw[1])/B_module;
    magMoment[1] = (controlLaw[0]*mag[2] - mag[0]*controlLaw[2])/B_module;
    magMoment[2] = (controlLaw[1]*mag[0] - mag[1]*controlLaw[0])/B_module;

    // Definition of intensity output -solving the equation: A*I = m-
    intensity[0] = magMoment[0]/coilChar[0][0];
    intensity[1] = magMoment[1]/coilChar[1][1];
    intensity[2] = magMoment[2]/coilChar[2][2];
}
