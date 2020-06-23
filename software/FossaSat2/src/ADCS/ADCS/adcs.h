/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs
   File: adcs.h
   04/30/20

   This file drives the function declaration file
*/

#ifndef ADCS_H_INCLUDED
#define ADCS_H_INCLUDED

/***************** Headers ****************/
#include <math.h>

#include "../../../FossaSat2.h"

#define ADCS_CALC_TYPE                                  double
#define ADCS_NUM_AXES                                   3

#include "../ACS/acs.h"
//#include "../ADS/ads.h"
// TODO no global vars inside aux functions

/*********** Functions declaration ************/
// Main structure
void ADCS_Main(const uint8_t controlFlags, const uint32_t detumbleDuration, const uint32_t activeDuration,
               const uint8_t position[], const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion);

ADCS_CALC_TYPE ADCS_VectorNorm(const ADCS_CALC_TYPE dim[]);

void ADCS_Detumble_Init(const uint32_t detumbleDuration, const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion);
void ADCS_Detumble_Update();
void ADCS_Detumble_Finish(bool startActiveControl);


void ADCS_Finish();
void ADCS_Update_Bridges();

// Auxiliary functions
/*void bdot_function(const double* omega, const double* B,
                   const float T, const float i, double* I);                        //Detumbling procedure
float intensities_rectifier(const float intensity1, const float intensity2,
                            const int delta_t);                                     // Intensities rectifier function
void onboardcontrol(double* M, double* B, double** K, double* I, double* t);        // Controller function*/

#endif // ADCS_H_INCLUDED
