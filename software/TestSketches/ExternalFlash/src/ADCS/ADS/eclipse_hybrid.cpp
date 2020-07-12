/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: ads_determination
   File: eclipse_hybrid.c
   04/24/20

   This file drives the angular state determination from the magnetometer
*/

/*********************** Headers *****************************/
#include <math.h>
#include "../ADCS/adcs.h"

/******************** Main function ***************************/
void ADS_Eclipse_Hybrid(const ADCS_CALC_TYPE magData[ADCS_NUM_AXES], const ADCS_CALC_TYPE magEphe[ADCS_NUM_AXES],
                        ADCS_CALC_TYPE rotationMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES]) {
  // Variables declaration
  ADCS_CALC_TYPE bodyFrame[ADCS_NUM_AXES];
  ADCS_CALC_TYPE lvFrame[ADCS_NUM_AXES];

  // Unitary frame vectors
  const ADCS_CALC_TYPE dataGain = 1.0 / ADCS_Add_Tolerance(ADCS_VectorNorm(magData), 0);
  const ADCS_CALC_TYPE epheGain = 1.0 / ADCS_Add_Tolerance(ADCS_VectorNorm(magEphe), 0);

  // Unitary vector calculation in the body frame
  bodyFrame[0] = dataGain * magData[0];
  bodyFrame[1] = dataGain * magData[1];
  bodyFrame[2] = dataGain * magData[2];

  // Unitary vector calculation in the LV frame
  lvFrame[0] = epheGain * magEphe[0];
  lvFrame[1] = epheGain * magEphe[1];
  lvFrame[2] = epheGain * magEphe[2];

  // Euler angles matrix -dot product between the two unitary vectors-
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      rotationMatrix[i][j] = lvFrame[j] * bodyFrame[i];
    }
  }
}
