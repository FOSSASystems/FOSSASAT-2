/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: ads.h
   File: rotation_verification.c
   04/30/20

   This file drives the functions that checks whether
   the rotation matrix calculation and the IMU data are similar
*/

/****************** Headers *********************/
#include "../ADCS/adcs.h"

/*************** Main function ***************/
bool ADS_Rotation_Verification(ADCS_CALC_TYPE angles1[ADCS_NUM_AXES], const ADCS_CALC_TYPE angles2[ADCS_NUM_AXES],
                               const ADCS_CALC_TYPE weightRatio, const ADCS_CALC_TYPE trigger) {
  // Constants definition
  const ADCS_CALC_TYPE weight1 = weightRatio;
  const ADCS_CALC_TYPE weight2 = 1.0 - weight1;

  // Compare both vectors and compute the norm
  ADCS_CALC_TYPE diff[ADCS_NUM_AXES];
  diff[0] = angles1[0] - angles2[0];
  diff[1] = angles1[1] - angles2[1];
  diff[2] = angles1[2] - angles2[2];

  // Arithmetic average of both measurements
  if(ADCS_VectorNorm(diff) >= trigger) {
    angles1[0] = weight1*angles1[0] + weight2*angles2[0];
    angles1[1] = weight1*angles1[1] + weight2*angles2[1];
    angles1[2] = weight1*angles1[2] + weight2*angles2[2];
    return(false);
  }

  return(true);
}
