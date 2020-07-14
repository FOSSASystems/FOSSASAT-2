#ifndef ACS_H_INCLUDED
#define ACS_H_INCLUDED

/***************** Headers ****************/
#include <math.h>

/*********** Functions declaration ************/
//Detumbling procedure
void ACS_BdotFunction(const ADCS_CALC_TYPE omega[ADCS_NUM_AXES], const ADCS_CALC_TYPE mag[ADCS_NUM_AXES], const ADCS_CALC_TYPE coilChar[ADCS_NUM_AXES][ADCS_NUM_AXES],
                      const ADCS_CALC_TYPE meanOrbitalMotion, const ADCS_CALC_TYPE orbInclination, const ADCS_CALC_TYPE minInertialMoment,
                      ADCS_CALC_TYPE intensity[ADCS_NUM_AXES]);

// Intensities rectifier function
ADCS_CALC_TYPE ACS_IntensitiesRectifier(const ADCS_CALC_TYPE intensity1, const ADCS_CALC_TYPE intensity2, const uint32_t delta_t, const ADCS_CALC_TYPE amplitude);

// Controller function
void ACS_OnboardControl(const ADCS_CALC_TYPE stateVars[ADCS_STATE_DIM], const ADCS_CALC_TYPE mag[ADCS_NUM_AXES], const float gain[ADCS_NUM_AXES][ADCS_STATE_DIM],
                        const ADCS_CALC_TYPE coilChar[ADCS_NUM_AXES][ADCS_NUM_AXES], ADCS_CALC_TYPE intensity[ADCS_NUM_AXES], ADCS_CALC_TYPE controlLaw[ADCS_NUM_AXES]);

#endif // ADCS_H_INCLUDED
