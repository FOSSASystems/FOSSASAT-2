/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: attitude_determination
   File:
   04/18/20

   This file drives the Euler integrator scheme.
*/

/**************** Headers *****************/
#include "../ADCS/adcs.h"

/********** Main function **************/
void ADS_Euler_Integrator(const ADCS_CALC_TYPE omega[], const ADCS_CALC_TYPE currentAngles[], ADCS_CALC_TYPE nextAngles[], const ADCS_CALC_TYPE delta_t) {
  ADCS_CALC_TYPE p = omega[0];
  ADCS_CALC_TYPE q = omega[1];
  ADCS_CALC_TYPE r = omega[2];
  ADCS_CALC_TYPE psi = currentAngles[0];
  ADCS_CALC_TYPE phi = currentAngles[1];
  ADCS_CALC_TYPE theta = currentAngles[2];

  // New Euler angles at time t+1
  nextAngles[0] = psi + delta_t*(p + (q*sin(psi) + r*cos(psi))*tan(phi));
  nextAngles[1] = phi + delta_t*(q*cos(psi) - r*sin(psi));

  // cos(phi) == -calcTol check
  ADCS_CALC_TYPE cosPhi = cos(phi);
  if(cosPhi == (-1.0 * adcsParams.calcTol)) {
    nextAngles[2] = theta + delta_t*((q*sin(psi) + r*cos(psi))*(1.0/(cosPhi - adcsParams.calcTol)));
  } else {
    nextAngles[2] = theta + delta_t*((q*sin(psi) + r*cos(psi))*(1.0/(cosPhi + adcsParams.calcTol)));
  }
}
