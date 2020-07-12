/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: attitude_determination
   File: angles_determination.c
   04/18/20

   This file drives the Euler angle determination algorithm
*/

/**************** Headers ***************/
#include <math.h>
#include "../ADCS/adcs.h"

/**************** Main function *************/
void ADS_Angles_Determination(const ADCS_CALC_TYPE eulerAnglesMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES], ADCS_CALC_TYPE newAnglesVector[ADCS_NUM_AXES]) {
  // Angular determination and desambiguation
  ADCS_CALC_TYPE psi = atan2(eulerAnglesMatrix[1][2], eulerAnglesMatrix[2][2]);
  ADCS_CALC_TYPE theta = atan2(eulerAnglesMatrix[0][1], eulerAnglesMatrix[0][0]);
  ADCS_CALC_TYPE phi = atan2(-1.0 * eulerAnglesMatrix[0][2] * sin(psi), eulerAnglesMatrix[1][2]);

  // New variables
  newAnglesVector[0] = psi;     // X rotation -Roll-
  newAnglesVector[1] = phi;     // Y rotation -Pitch-
  newAnglesVector[2] = theta;   // Z rotation -Yaw-
}
