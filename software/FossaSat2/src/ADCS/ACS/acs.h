#ifndef ACS_H_INCLUDED
#define ACS_H_INCLUDED

/***************** Headers ****************/
#include <math.h>

/*********** Functions declaration ************/
//Detumbling procedure
void ACS_BdotFunction(const float omega[], const float mag[], const float orbitalInclination, const float orbitalPeriod, float intensity[]);

// Intensities rectifier function
float ACS_IntensitiesRectifier(const float intensity1, const float intensity2, const int delta_t);

//void onboardcontrol(double* M, double* B, double** K, double* I, double* t);        // Controller function

#endif // ADCS_H_INCLUDED
