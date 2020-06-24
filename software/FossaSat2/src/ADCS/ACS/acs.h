#ifndef ACS_H_INCLUDED
#define ACS_H_INCLUDED

/***************** Headers ****************/
#include <math.h>

/*********** Functions declaration ************/
//Detumbling procedure
void ACS_BdotFunction(const ADCS_CALC_TYPE omega[], const ADCS_CALC_TYPE mag[], ADCS_CALC_TYPE intensity[]);

// Intensities rectifier function
float ACS_IntensitiesRectifier(const ADCS_CALC_TYPE intensity1, const ADCS_CALC_TYPE intensity2, const uint32_t delta_t);

// Controller function
void ACS_OnboardControl(ADCS_CALC_TYPE state[], ADCS_CALC_TYPE mag[], float gain[][2*ADCS_NUM_AXES], ADCS_CALC_TYPE intensity[]);

#endif // ADCS_H_INCLUDED
