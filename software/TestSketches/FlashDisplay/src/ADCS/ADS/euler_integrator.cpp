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
void ADS_Euler_Integrator(const ADCS_CALC_TYPE omega[ADCS_NUM_AXES], const ADCS_CALC_TYPE currentAngles[ADCS_NUM_AXES],
                          ADCS_CALC_TYPE nextAngles[ADCS_NUM_AXES], const ADCS_CALC_TYPE delta_t) {
  ADCS_CALC_TYPE p = omega[0];
  ADCS_CALC_TYPE q = omega[1];
  ADCS_CALC_TYPE r = omega[2];
  ADCS_CALC_TYPE psi = currentAngles[0];
  ADCS_CALC_TYPE phi = currentAngles[1];
  ADCS_CALC_TYPE theta = currentAngles[2];

  // convert to SI units
  ADCS_CALC_TYPE delta_t_sec = delta_t / 1000.0;

  // New Euler angles at time t+1
  nextAngles[0] = psi + delta_t_sec*(p + (q*sin(psi) + r*cos(psi))*ADCS_Add_Tolerance(tan(phi), M_PI/2.0));
  nextAngles[1] = phi + delta_t_sec*(q*cos(psi) - r*sin(psi));
  nextAngles[2] = theta + delta_t_sec*((q*sin(psi) + r*cos(psi))*(1.0/(ADCS_Add_Tolerance(cos(phi), 0))));
}
