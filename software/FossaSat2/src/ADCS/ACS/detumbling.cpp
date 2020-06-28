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
void ACS_BdotFunction(const ADCS_CALC_TYPE omega[ADCS_NUM_AXES], const ADCS_CALC_TYPE mag[ADCS_NUM_AXES], const ADCS_CALC_TYPE coilChar[ADCS_NUM_AXES][ADCS_NUM_AXES],
                      const ADCS_CALC_TYPE meanOrbitalMotion, const ADCS_CALC_TYPE orbInclination, const ADCS_CALC_TYPE minInertialMoment,
                      ADCS_CALC_TYPE intensity[ADCS_NUM_AXES]) {
    // Constants definition
    // Module of the magnetic field intensity
    const ADCS_CALC_TYPE B_module = ADCS_Add_Tolerance(ADCS_VectorNorm(mag), 0);
    FOSSASAT_DEBUG_PRINT(F("B_module = \t"));
    FOSSASAT_DEBUG_PRINTLN(B_module, 4);

    // Controller gain constant
    const ADCS_CALC_TYPE gainConstant = 2.0*meanOrbitalMotion*(1.0 + sin(orbInclination)) * minInertialMoment;
    FOSSASAT_DEBUG_PRINT(F("gainConstant = "));
    FOSSASAT_DEBUG_PRINTLN(gainConstant, 4);

    // General gain
    const ADCS_CALC_TYPE gainGeneral = gainConstant/pow(B_module, 2);
    FOSSASAT_DEBUG_PRINT(F("gainGeneral = \t"));
    FOSSASAT_DEBUG_PRINTLN(gainGeneral, 4);

    // Generate the control law by means of a vector product: m = a*(B x omega) = (K/B_module)*(B x omega)
    ADCS_CALC_TYPE controlLaw[ADCS_NUM_AXES];
    controlLaw[0] = gainGeneral*(omega[1]*mag[2] - omega[2]*mag[1]);
    controlLaw[1] = gainGeneral*(omega[2]*mag[0] - omega[0]*mag[2]);
    controlLaw[2] = gainGeneral*(omega[0]*mag[1] - omega[1]*mag[0]);
    FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(controlLaw, ADCS_NUM_AXES);

    // Magnetic moment needed at the axes: calculated by minimization of a least squares problem
    ADCS_CALC_TYPE magMoment[ADCS_NUM_AXES];
    magMoment[0] = (controlLaw[2]*mag[1] - mag[2]*controlLaw[1])/pow(B_module,2);
    magMoment[1] = (controlLaw[0]*mag[2] - mag[0]*controlLaw[2])/pow(B_module,2);
    magMoment[2] = (controlLaw[1]*mag[0] - mag[1]*controlLaw[0])/pow(B_module,2);
    FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(magMoment, ADCS_NUM_AXES);

    // Definition of intensity output -solving the equation: A*I = m-
    intensity[0] = magMoment[0]*coilChar[0][0] + magMoment[1]*coilChar[0][1] + magMoment[2]*coilChar[0][2];
    intensity[1] = magMoment[0]*coilChar[1][0] + magMoment[1]*coilChar[1][1] + magMoment[2]*coilChar[1][2];
    intensity[2] = magMoment[0]*coilChar[2][0] + magMoment[1]*coilChar[2][1] + magMoment[2]*coilChar[2][2];
}
