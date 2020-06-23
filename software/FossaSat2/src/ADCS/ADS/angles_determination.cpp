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
void ADS_Angles_Determination(const ADCS_CALC_TYPE eulerAnglesMatrix[][ADCS_NUM_AXES], ADCS_CALC_TYPE newAnglesVector[]) {
  // Angular determination and desambiguation
  ADCS_CALC_TYPE psi = atan2(eulerAnglesMatrix[1][2], eulerAnglesMatrix[2][2]);
  ADCS_CALC_TYPE theta = atan2(eulerAnglesMatrix[0][1], eulerAnglesMatrix[0][0]);
  ADCS_CALC_TYPE phi = asin(-1.0 * eulerAnglesMatrix[0][2]);
  if(sin(phi) < 0.0) {
    if(cos(phi) > 0.0) {
      phi = M_PI + phi;
    }
  } else {
    if(cos(phi) < 0.0) {
      phi = M_PI - phi;
    }
  }

  // New variables
  newAnglesVector[0] = psi;         // X rotation -Roll-
  newAnglesVector[1] = phi;         // Y rotation -Pitch-
  newAnglesVector[2] = theta;       // Z rotation -Yaw-
}
