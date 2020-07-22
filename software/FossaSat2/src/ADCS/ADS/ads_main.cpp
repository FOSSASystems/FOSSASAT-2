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
bool ADS_Eclipse_Decision(const ADCS_CALC_TYPE luxData[ADCS_NUM_PANELS], const ADCS_CALC_TYPE threshold) {

  ADCS_CALC_TYPE sum = 0;
  // skip luxData[0] - not solar cell power
  for(uint8_t i = 1; i < ADCS_NUM_PANELS; i++) {
    sum += luxData[i];
  }

  if(sum < threshold) {
    return(true);
  }

  return(false);
}

/**************** Main structure ****************/
void ADS_Main(const ADCS_CALC_TYPE omega[ADCS_NUM_AXES], const ADCS_CALC_TYPE magData[ADCS_NUM_AXES], const ADCS_CALC_TYPE stateVars[ADCS_STATE_DIM],
              const ADCS_CALC_TYPE controlVector[ADCS_STATE_DIM], ADCS_CALC_TYPE matrixP[ADCS_STATE_DIM][ADCS_STATE_DIM], const ADCS_CALC_TYPE solarEphe[ADCS_STATE_DIM],
              const ADCS_CALC_TYPE magEphe[ADCS_STATE_DIM], ADCS_CALC_TYPE filtered_y[ADCS_STATE_DIM], ADCS_CALC_TYPE newAnglesVector[ADCS_NUM_AXES]) {

  // Relevant variables declaration
  ADCS_CALC_TYPE anglesInt[ADCS_NUM_AXES];                      // Angular determination variables
  ADCS_CALC_TYPE rotationMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES];  // Rotation matrix

  // Previous state of the system from t = t_1-1
  ADCS_CALC_TYPE prevAngles[ADCS_NUM_AXES];
  prevAngles[0] = stateVars[0];
  prevAngles[1] = stateVars[1];
  prevAngles[2] = stateVars[2];

  // Measurements from sensors
  ADCS_CALC_TYPE luxData[ADCS_NUM_PANELS];
  luxData[0] = Sensors_Read_Light(lightSensorPanelY);
  luxData[1] = Sensors_Current_ReadPower(currSensorXA);
  luxData[2] = Sensors_Current_ReadPower(currSensorZA);
  luxData[3] = Sensors_Current_ReadPower(currSensorY);
  luxData[4] = Sensors_Current_ReadPower(currSensorXB);
  luxData[5] = Sensors_Current_ReadPower(currSensorZB);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(luxData, ADCS_NUM_PANELS);

  // Decide whether the satellite is in eclipse situation or under light
  if(ADS_Eclipse_Decision(luxData, adcsParams.eclipseThreshold)) {
    // Generation of the measurements
    FOSSASAT_DEBUG_PRINTLN(F("eclipse"));
    ADS_Euler_Integrator(omega, prevAngles, anglesInt, adcsParams.timeStep);
    ADS_Eclipse_Hybrid(magData, magEphe, rotationMatrix);

  } else {
    FOSSASAT_DEBUG_PRINTLN(F("no eclipse"));
    ADCS_CALC_TYPE solarEphBody[ADCS_NUM_AXES];                   // Solar ephemerides in the body frame

    // Generation of the measurements
    ADS_Euler_Integrator(omega, prevAngles, anglesInt, adcsParams.timeStep);
    ADS_Solar_Determination(luxData, solarEphBody);
    ADS_Measurement_Hybrid(solarEphe, magEphe, solarEphBody, magData, rotationMatrix);
  }

  // Processing of the measurements
  ADS_Angles_Determination(rotationMatrix, newAnglesVector);
  ADS_Rotation_Verification(newAnglesVector, anglesInt, adcsParams.rotationWeightRatio, adcsParams.rotationTrigger);

  // Generation of the state point
  ADCS_CALC_TYPE measurements[ADCS_STATE_DIM];
  measurements[0] = newAnglesVector[0];
  measurements[1] = newAnglesVector[1];
  measurements[2] = newAnglesVector[2];
  measurements[3] = omega[0];
  measurements[4] = omega[1];
  measurements[5] = omega[2];

  // Filtering of the signal
  ADS_Kalman_Filter(adcsParams.disturbCovariance, adcsParams.noiseCovariance, adcsParams.timeStep, stateVars, measurements, controlVector, adcsParams.inertiaTensor, matrixP, filtered_y);
}
