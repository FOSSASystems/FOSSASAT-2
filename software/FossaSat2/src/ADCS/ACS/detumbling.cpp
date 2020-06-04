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
    const double B_module = pow((pow(mag[0], 2) + pow(mag[1], 2) + mag[2]), 0.5);

    // Controller gain constant
    const float gainConstant = 4.0*M_PI * (1.0 + sin(orbitalInclination)) * adcsParams.minInertialMoment/orbitalPeriod;

    // General gain
    const int gainGeneral = gainConstant/B_module;

    // Coil magnetic characteristics
    // TODO configurable?
    const double coilChar[3][3] = { {1.88 * pow(10, 5), 0,               0},
                                    {0,                 6.1*pow(10, 5),  0},
                                    {0,                 0,               5.96*pow(10, 5)} };

    // Generate the control law by means of a vector product: m = a*(B x omega) = (K/B_module)*(B x omega)
    float m[3];
    m[0] = gainGeneral*(omega[1]*mag[2] - omega[2]*mag[1]);
    m[1] = gainGeneral*(omega[2]*mag[0] - omega[0]*mag[2]);
    m[2] = gainGeneral*(omega[0]*mag[1] - omega[1]*mag[0]);

    // Definition of intensity output -solving the equation: A*I = m-
    intensity[0] = m[0]/coilChar[0][0];
    intensity[1] = m[1]/coilChar[1][1];
    intensity[2] = m[2]/coilChar[2][2];
}
