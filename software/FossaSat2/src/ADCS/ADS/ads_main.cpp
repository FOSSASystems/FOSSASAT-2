/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: ads_determination
   File: ads_main.c
   04/30/20

   This file drives the main ADS subsystem structure
*/

/****************** Headers *********************/

#include <math.h>
#include "../ADCS/adcs.h"

/*****************  Auxiliary functions implementation *********************/
bool ADS_Eclipse_Decision(const ADCS_CALC_TYPE luxData[]) {
  ADCS_CALC_TYPE sum = 0;
  for(uint8_t i = 0; i < ADCS_NUM_PANELS; i++) {
    sum += luxData[i];
  }

  if(sum < adcsParams.eclipseThreshold) {
    return(true);
  }

  return(false);
}

/**************** Main structure ****************/
void ADS_Main(ADCS_CALC_TYPE omega[], ADCS_CALC_TYPE magData[], ADCS_CALC_TYPE stateVars[], ADCS_CALC_TYPE controlVector[], ADCS_CALC_TYPE P[][2*ADCS_NUM_AXES], ADCS_CALC_TYPE solarEphe[], ADCS_CALC_TYPE magEphe[], ADCS_CALC_TYPE filtered_y[], ADCS_CALC_TYPE newAnglesVector[]) {
  // Constants declaration

  // Relevant variables declaration
  ADCS_CALC_TYPE anglesInt[ADCS_NUM_AXES];                      // Angular determination variables
  ADCS_CALC_TYPE rotationMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES];  // Rotation matrix
  ADCS_CALC_TYPE solarEphBody[ADCS_NUM_AXES];                   // Solar ephemerides in the body frame
  ADCS_CALC_TYPE redundantSolarEph[ADCS_NUM_AXES];              // Redundant solar ephemerides in the body frame

  // Previous state of the system from t = t_1-1
  ADCS_CALC_TYPE prevAngles[ADCS_NUM_AXES];
  prevAngles[0] = stateVars[0];
  prevAngles[1] = stateVars[1];
  prevAngles[2] = stateVars[2];

  // Measurements from sensors
  ADCS_CALC_TYPE luxData[ADCS_NUM_PANELS];
  // TODO solar measurements

  // Decide whether the satellite is in eclipse situation or under light
  if(ADS_Eclipse_Decision(luxData)) {
    // Generation of the measurements
    ADS_Euler_Integrator(omega, prevAngles, anglesInt, adcsParams.timeStep);
    ADS_Eclipse_Hybrid(magData, magEphe, rotationMatrix);

  } else {
    // Generation of the measurements
    ADS_Euler_Integrator(omega, prevAngles, anglesInt, adcsParams.timeStep);
    ADS_Solar_Determination(luxData, solarEphBody, redundantSolarEph);
    ADS_Measurement_Hybrid(solarEphBody, redundantSolarEph, magData, solarEphe, solarEphe, magEphe, rotationMatrix);

  }

  // Processing of the measurements
  ADS_Angles_Determination(rotationMatrix, newAnglesVector);
  // TODO rotation verification debug-only?
  ADS_Rotation_Verification(newAnglesVector, anglesInt);

  // Generation of the state point
  ADCS_CALC_TYPE measurements[2*ADCS_NUM_AXES];
  measurements[0] = newAnglesVector[0];
  measurements[1] = newAnglesVector[1];
  measurements[2] = newAnglesVector[2];
  measurements[3] = omega[0];
  measurements[4] = omega[1];
  measurements[5] = omega[2];

  // Filtering of the signal
  ADS_Kalman_Filter(adcsParams.disturbCovariance, adcsParams.noiseCovariance, adcsParams.timeStep, stateVars, measurements, controlVector, P, filtered_y);
}
