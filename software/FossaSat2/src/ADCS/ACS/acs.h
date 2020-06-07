#ifndef ACS_H_INCLUDED
#define ACS_H_INCLUDED

/***************** Headers ****************/
#include <math.h>

/*********** Functions declaration ************/
//Detumbling procedure
void ACS_BdotFunction(const ADCS_CALC_TYPE omega[], const ADCS_CALC_TYPE mag[], const ADCS_CALC_TYPE orbitalInclination,
                      const ADCS_CALC_TYPE meanOrbitalMotion, ADCS_CALC_TYPE intensity[]);

// Intensities rectifier function
float ACS_IntensitiesRectifier(const ADCS_CALC_TYPE intensity1, const ADCS_CALC_TYPE intensity2, const uint32_t delta_t);

//void onboardcontrol(double* M, double* B, double** K, double* I, double* t);        // Controller function

#endif // ADCS_H_INCLUDED
