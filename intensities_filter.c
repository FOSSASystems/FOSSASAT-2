/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: intensities_rectifier.c
   04/30/20

   This file drives the intensity rectifier filter to implement a pulse-modulated
   control law by means of a modified trapezoidal rule integration scheme
*/

/***************** Headers ****************/
#include "adcs.h"

/**************** Main function ***********/
float intensities_rectifier(const float intensity1, const float intensity2, const int delta_t)
{
  // Amplitude of the pulse
  const float PULSE_AMPLITUDE = 1;      // To be modify

  // Energy approximation of the signal by the trapezoidal rule
  float E = (intensity1*intensity1+(float(1)/3)*(intensity2-intensity1)*(intensity2-intensity1)
            + intensity1*(intensity2-intensity1))*delta_t;

  // Pulse time calculation
  float duration = E/pow(PULSE_AMPLITUDE,2);

  return duration;
}
