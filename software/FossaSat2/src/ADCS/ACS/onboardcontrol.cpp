/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: onboardcontrol.c
   04/30/20

   This file drives the main active control algorithm and its calculations
*/

/******************** Headers **********************/
#include <math.h>
#include "../ADCS/adcs.h"

/****************** Main function *********************/
void ACS_OnboardControl(const ADCS_CALC_TYPE stateVars[ADCS_STATE_DIM], const ADCS_CALC_TYPE mag[ADCS_NUM_AXES], const float gain[ADCS_NUM_AXES][ADCS_STATE_DIM],
                        const ADCS_CALC_TYPE coilChar[ADCS_NUM_AXES][ADCS_NUM_AXES], ADCS_CALC_TYPE intensity[ADCS_NUM_AXES], ADCS_CALC_TYPE controlLaw[ADCS_NUM_AXES]) {
  // Module of the magnetic field intensity
  const ADCS_CALC_TYPE B_module = ADCS_Add_Tolerance(ADCS_VectorNorm(mag), 0);

  // Generation of the control law by means of a matrix product Lc = -Kx
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    ADCS_CALC_TYPE controlLawAux = 0;
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      controlLawAux += (ADCS_CALC_TYPE)gain[i][j] * stateVars[j];
    }
    controlLaw[i] = -1.0*controlLawAux;
  }
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(controlLaw, ADCS_NUM_AXES);

  // Calculation of magnetic dipole applied on the coils
  ADCS_CALC_TYPE magMoment[ADCS_NUM_AXES];
  magMoment[0] = (controlLaw[2]*mag[1] - mag[2]*controlLaw[1])/pow(B_module, 2);
  magMoment[1] = (controlLaw[0]*mag[2] - mag[0]*controlLaw[2])/pow(B_module, 2);
  magMoment[2] = (controlLaw[1]*mag[0] - mag[1]*controlLaw[0])/pow(B_module, 2);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(magMoment, ADCS_NUM_AXES);

  // Definition of intensity output -solving the equation: A*I = m-
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    intensity[i] = 0;
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
        intensity[i] += coilChar[i][j] * magMoment[j];
    }
  }
}
