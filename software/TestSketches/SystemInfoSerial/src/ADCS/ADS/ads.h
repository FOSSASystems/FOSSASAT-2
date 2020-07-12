/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: ads.h
   File: ads.h
   04/30/20

   This file drives the ADS functions declaration
*/

#ifndef ADS_H_INCLUDED
#define ADS_H_INCLUDED

/********** Functions declaration *************/
bool ADS_Eclipse_Decision(const ADCS_CALC_TYPE luxData[ADCS_NUM_PANELS], const ADCS_CALC_TYPE threshold);

// Main ADS structure
void ADS_Main(ADCS_CALC_TYPE omega[ADCS_NUM_AXES], ADCS_CALC_TYPE magData[ADCS_NUM_AXES], ADCS_CALC_TYPE stateVars[ADCS_STATE_DIM],
              ADCS_CALC_TYPE controlVector[ADCS_STATE_DIM], ADCS_CALC_TYPE matrixP[ADCS_STATE_DIM][ADCS_STATE_DIM], ADCS_CALC_TYPE solarEphe[ADCS_STATE_DIM],
              ADCS_CALC_TYPE magEphe[ADCS_STATE_DIM], ADCS_CALC_TYPE filtered_y[ADCS_STATE_DIM], ADCS_CALC_TYPE newAnglesVector[ADCS_NUM_AXES]);

// Determination of the Euler angles
void ADS_Angles_Determination(const ADCS_CALC_TYPE eulerAnglesMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES], ADCS_CALC_TYPE newAnglesVector[ADCS_NUM_AXES]);

// Rotation matrix calculation in eclipse situation
void ADS_Eclipse_Hybrid(const ADCS_CALC_TYPE magData[ADCS_NUM_AXES], const ADCS_CALC_TYPE magEphe[ADCS_NUM_AXES],
                        ADCS_CALC_TYPE rotationMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES]);

// Forward Euler integration scheme
void ADS_Euler_Integrator(const ADCS_CALC_TYPE omega[ADCS_NUM_AXES], const ADCS_CALC_TYPE currentAngles[ADCS_NUM_AXES],
                          ADCS_CALC_TYPE nextAngles[ADCS_NUM_AXES], const ADCS_CALC_TYPE delta_t);

// Filtering function
void ADS_Kalman_Filter(const ADCS_CALC_TYPE Q, const ADCS_CALC_TYPE R, const ADCS_CALC_TYPE delta_t,
                       const ADCS_CALC_TYPE x0[ADCS_STATE_DIM], const ADCS_CALC_TYPE y0[ADCS_STATE_DIM],
                       const ADCS_CALC_TYPE u0[ADCS_NUM_AXES], const ADCS_CALC_TYPE invI[ADCS_NUM_AXES][ADCS_NUM_AXES],
                       ADCS_CALC_TYPE P[][ADCS_STATE_DIM], ADCS_CALC_TYPE filtered_y[]);

// Rotation matrix calculation in not eclipse situation
void ADS_Measurement_Hybrid(const ADCS_CALC_TYPE v_1[ADCS_NUM_AXES], const ADCS_CALC_TYPE v_2[ADCS_NUM_AXES], const ADCS_CALC_TYPE v_3[ADCS_NUM_AXES],
                            const ADCS_CALC_TYPE m_1[ADCS_NUM_AXES], const ADCS_CALC_TYPE m_2[ADCS_NUM_AXES], const ADCS_CALC_TYPE m_3[ADCS_NUM_AXES],
                            ADCS_CALC_TYPE eulerAnglesMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES]);

// Verify both magnetic and IMU data coincide
bool ADS_Rotation_Verification(ADCS_CALC_TYPE angles1[ADCS_NUM_AXES], ADCS_CALC_TYPE angles2[ADCS_NUM_AXES],
                               const ADCS_CALC_TYPE weightRatio, const ADCS_CALC_TYPE trigger);

// Solar data processing
void ADS_Solar_Determination(const ADCS_CALC_TYPE luxData[ADCS_NUM_PANELS], ADCS_CALC_TYPE solarEph[ADCS_NUM_AXES],
                             ADCS_CALC_TYPE redundantSolarEph[ADCS_NUM_AXES]);

#endif // ADS_H_INCLUDED
