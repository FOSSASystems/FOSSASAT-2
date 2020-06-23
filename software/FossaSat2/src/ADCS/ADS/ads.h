/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: ads.h
   File: ads.h
   04/30/20

   This file drives the ADS functions declaration
*/

#ifndef ADS_H_INCLUDED
#define ADS_H_INCLUDED

// TODO measure_generation unused?

#define ADCS_STATE_DIM          (2*ADCS_NUM_AXES)

/********** Functions declaration *************/
bool ADS_Eclipse_Decision(const ADCS_CALC_TYPE luxData[]);

// Main ADS structure
void ADS_Main(ADCS_CALC_TYPE state[], ADCS_CALC_TYPE controlVector[], ADCS_CALC_TYPE P[][ADCS_NUM_AXES], ADCS_CALC_TYPE ephemerides[], ADCS_CALC_TYPE filtered_y[]);

// Rotation matrix calculation in eclipse situation
void ADS_Eclipse_Hybrid(const ADCS_CALC_TYPE magData[], const ADCS_CALC_TYPE magEphe[], ADCS_CALC_TYPE rotationMatrix[][ADCS_NUM_AXES]);

// Determination of the Euler angles
void ADS_Angles_Determination(const ADCS_CALC_TYPE eulerAnglesMatrix[][ADCS_NUM_AXES], ADCS_CALC_TYPE newAnglesVector[]);

// Forward Euler integration scheme
void ADS_Euler_Integrator(const ADCS_CALC_TYPE p, const ADCS_CALC_TYPE q, const ADCS_CALC_TYPE r, const ADCS_CALC_TYPE currentAngles[], ADCS_CALC_TYPE nextAngles[], const ADCS_CALC_TYPE delta_t);

// Verify both magnetic and IMU data coincide
bool ADS_Rotation_Verification(ADCS_CALC_TYPE angles1[], ADCS_CALC_TYPE angles2[]);

// Solar data processing
void ADS_Solar_Determination(ADCS_CALC_TYPE luxData[], ADCS_CALC_TYPE solarEph[], ADCS_CALC_TYPE redundantSolarEph[]);

// Rotation matrix calculation in not eclipse situation
void ADS_Measurement_Hybrid(ADCS_CALC_TYPE v_1[], ADCS_CALC_TYPE v_2[], ADCS_CALC_TYPE v_3[],
                            ADCS_CALC_TYPE m_1[], ADCS_CALC_TYPE m_2[], ADCS_CALC_TYPE m_3[],
                            ADCS_CALC_TYPE eulerAnglesMatrix[][ADCS_NUM_AXES]);

// Filtering function
void ADS_Kalman_Filter(const ADCS_CALC_TYPE Q, const ADCS_CALC_TYPE R, const ADCS_CALC_TYPE delta_t,
                       const ADCS_CALC_TYPE x0[], const ADCS_CALC_TYPE y0[], const ADCS_CALC_TYPE u0[],
                       ADCS_CALC_TYPE P[][ADCS_STATE_DIM], ADCS_CALC_TYPE filtered_y[]);

#endif // ADS_H_INCLUDED
