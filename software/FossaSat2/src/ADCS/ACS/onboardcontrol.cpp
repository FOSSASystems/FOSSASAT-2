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
void ACS_OnboardControl(ADCS_CALC_TYPE state[], ADCS_CALC_TYPE mag[], float gain[][2*ADCS_NUM_AXES], ADCS_CALC_TYPE intensity[]) {
  // Module of the magnetic field intensity
  const ADCS_CALC_TYPE B_module = ADCS_VectorNorm(mag) + adcsParams.calcTol;

  // Variables initialization
  ADCS_CALC_TYPE controlLaw[ADCS_NUM_AXES];
  ADCS_CALC_TYPE controlLawAux = 0;

  // Generation of the control law by means of a matrix product Lc = K*M
  for(uint8_t i = 0; i <= ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j <= 2*ADCS_NUM_AXES; j++) {
      controlLawAux += (ADCS_CALC_TYPE)gain[i][j] * state[j];
      controlLaw[i] = controlLawAux;
    }
  }

  // Calculation of magnetic dipole applied on the coils
  ADCS_CALC_TYPE magMoment[3];
  magMoment[0] = (controlLaw[2]*mag[1] - mag[2]*controlLaw[1])/pow(B_module, 2);
  magMoment[1] = (controlLaw[0]*mag[2] - mag[0]*controlLaw[2])/pow(B_module, 2);
  magMoment[2] = (controlLaw[1]*mag[0] - mag[1]*controlLaw[0])/pow(B_module, 2);

  // Definition of intensity output -solving the equation: A*I = m-
  intensity[0] = magMoment[0]*adcsParams.coilChar[0][0] + magMoment[1]*adcsParams.coilChar[0][1] + magMoment[2]*adcsParams.coilChar[0][2];
  intensity[1] = magMoment[0]*adcsParams.coilChar[1][0] + magMoment[1]*adcsParams.coilChar[1][1] + magMoment[2]*adcsParams.coilChar[1][2];
  intensity[2] = magMoment[0]*adcsParams.coilChar[2][0] + magMoment[1]*adcsParams.coilChar[2][1] + magMoment[2]*adcsParams.coilChar[2][2];
}
