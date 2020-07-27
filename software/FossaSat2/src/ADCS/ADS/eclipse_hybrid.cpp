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

  // Dot product between the two (cos(alpha) == a dot b)
  ADCS_CALC_TYPE alphaDot = bodyFrame[0]*lvFrame[0]+bodyFrame[1]*lvFrame[1]+bodyFrame[2]*lvFrame[2];

  // Cross product between the two (sin(alpha) == norm(a cross b))
  ADCS_CALC_TYPE rotationAxe[ADCS_NUM_AXES];
  rotationAxe[0] = bodyFrame[2]*lvFrame[1]-bodyFrame[1]*lvFrame[2];
  rotationAxe[1] = bodyFrame[0]*lvFrame[2]-bodyFrame[2]*lvFrame[0];
  rotationAxe[2] = bodyFrame[1]*lvFrame[0]-bodyFrame[0]*lvFrame[1];

  ADCS_CALC_TYPE alphaCross = sqrt(pow(rotationAxe[0],2)+pow(rotationAxe[1],2)+pow(rotationAxe[2],2));

  // Euler angles matrix -Rodrigues' formula-
  ADCS_CALC_TYPE rotationMatrixAux1[ADCS_NUM_AXES][ADCS_NUM_AXES] = {{alphaDot, 0, 0},
                                                                     {0, alphaDot, 0},
                                                                     {0, 0, alphaDot}};
  ADCS_CALC_TYPE rotationMatrixAux2[ADCS_NUM_AXES][ADCS_NUM_AXES];
  for(uint32_t i = 0; i < ADCS_NUM_AXES; i++){
    for(uint32_t j = 0; j < ADCS_NUM_AXES; j++){
        rotationMatrixAux2[i][j] = (1.0-alphaDot)*(rotationAxe[i]*rotationAxe[j]);
    }
  }

  ADCS_CALC_TYPE rotationMatrixAux3[ADCS_NUM_AXES][ADCS_NUM_AXES] = {{0, -1.0*rotationAxe[2], rotationAxe[1]},
                                                                     {rotationAxe[2], 0, -1.0*rotationAxe[0]},
                                                                     {-1.0*rotationAxe[1], rotationAxe[0], 0}};
  for(uint32_t i = 0; i < ADCS_NUM_AXES; i++){
    for(uint32_t j = 0; j < ADCS_NUM_AXES; j++){
        rotationMatrixAux3[i][j] *= -1.0*alphaCross;
    }
  }

  // Final computation
  for(uint32_t i = 0; i < ADCS_NUM_AXES; i++){
    for(uint32_t j = 0; j < ADCS_NUM_AXES; j++){
        rotationMatrix[i][j] = rotationMatrixAux1[i][j]+rotationMatrixAux2[i][j]+rotationMatrixAux3[i][j];
    }
  }

}
