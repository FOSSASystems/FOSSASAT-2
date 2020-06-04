/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: intensities_rectifier.c
   04/30/20

   This file drives the intensity rectifier filter to implement a pulse-modulated
   control law by means of a modified trapezoidal rule integration scheme
*/

/***************** Headers ****************/
#include "../ADCS/adcs.h"

/**************** Main function ***********/
float ACS_IntensitiesRectifier(const float intensity1, const float intensity2, const int delta_t) {
  // Energy approximation of the signal by the trapezoidal rule
  float intensityDiff = intensity2 - intensity1;
  float energy = (pow(intensity1, 2) + (1.0/3.0)*pow(intensityDiff, 2) + intensity1*(intensityDiff)) * delta_t;

  //Pulse time calculation
  float duration = energy/adcsParams.pulseAmplitude;
  return(duration);
}
