/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: intensities_rectifier.c
   04/30/20

   This file drives the intensity rectifier filter to implement a pulse-modulated
   control law by means of a modified trapezoidal rule integration scheme
*/

/************** Headers ****************/
#include "../ADCS/adcs.h"

/**************** Main function ***********/
float ACS_IntensitiesRectifier(const ADCS_CALC_TYPE intensity1, const ADCS_CALC_TYPE intensity2, const uint32_t delta_t) {
  // Energy approximation of the signal by the trapezoidal rule
  ADCS_CALC_TYPE intensityDiff = intensity2 - intensity1;
  ADCS_CALC_TYPE energy = (pow(intensity1, 2.0) + (pow(intensityDiff, 2.0) / 3.0) + intensity1*intensityDiff) * (ADCS_CALC_TYPE)delta_t;

  // Pulse time calculation
  ADCS_CALC_TYPE duration = energy/pow(adcsParams.pulseAmplitude,2);
  return(duration);
}
