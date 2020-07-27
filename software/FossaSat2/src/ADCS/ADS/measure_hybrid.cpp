/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: attitude_determination
   File:
   04/18/20

   This file drives the measurements hybridation algorithm (TRIAD)
*/

/*********************** Headers ***********************/
#include "../ADCS/adcs.h"

/********************* Main function *********************/
void ADS_Measurement_Hybrid(const ADCS_CALC_TYPE v_1[ADCS_NUM_AXES], const ADCS_CALC_TYPE v_2[ADCS_NUM_AXES], const ADCS_CALC_TYPE m_1[ADCS_NUM_AXES],
                            const ADCS_CALC_TYPE m_2[ADCS_NUM_AXES], ADCS_CALC_TYPE eulerAnglesMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES]) {

  // Obtain both ephemeris and measurements vectors
  ADCS_CALC_TYPE m1[ADCS_NUM_AXES];
  memcpy(m1, m_1, ADCS_NUM_AXES*sizeof(ADCS_CALC_TYPE));
  ADCS_CALC_TYPE v1[ADCS_NUM_AXES];
  memcpy(v1, v_1, ADCS_NUM_AXES*sizeof(ADCS_CALC_TYPE));

  ADCS_CALC_TYPE m1_norm = ADCS_Add_Tolerance(ADCS_VectorNorm(m1),0);
  ADCS_CALC_TYPE v1_norm = ADCS_Add_Tolerance(ADCS_VectorNorm(v1),0);
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    m1[i] *= (1.0/m1_norm);
    v1[i] *= (1.0/v1_norm);
  }

  ADCS_CALC_TYPE m2[ADCS_NUM_AXES];
  m2[0] = m_1[1]*m_2[2]-m_1[2]*m_2[1];
  m2[1] = m_1[2]*m_2[0]-m_1[0]*m_2[2];
  m2[2] = m_1[0]*m_2[1]-m_1[1]*m_2[0];

  ADCS_CALC_TYPE v2[ADCS_NUM_AXES];
  v2[0] = v_1[1]*v_2[2]-v_1[2]*v_2[1];
  v2[1] = v_1[2]*v_2[0]-v_1[0]*v_2[2];
  v2[2] = v_1[0]*v_2[1]-v_1[1]*v_2[0];

  ADCS_CALC_TYPE m2_norm = ADCS_Add_Tolerance(ADCS_VectorNorm(m2),0);
  ADCS_CALC_TYPE v2_norm = ADCS_Add_Tolerance(ADCS_VectorNorm(v2),0);
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    m2[i] *= (1.0/m2_norm);
    v2[i] *= (1.0/v2_norm);
  }

  ADCS_CALC_TYPE m3[ADCS_NUM_AXES];
  m3[0] = m_1[1]*m2[2]-m_1[2]*m2[1];
  m3[1] = m_1[2]*m2[0]-m_1[0]*m2[2];
  m3[2] = m_1[0]*m2[1]-m_1[1]*m2[0];

  ADCS_CALC_TYPE v3[ADCS_NUM_AXES];
  v3[0] = v_1[1]*v2[2]-v_1[2]*v2[1];
  v3[1] = v_1[2]*v2[0]-v_1[0]*v2[2];
  v3[2] = v_1[0]*v2[1]-v_1[1]*v2[0];

  ADCS_CALC_TYPE m3_norm = ADCS_Add_Tolerance(ADCS_VectorNorm(m3),0);
  ADCS_CALC_TYPE v3_norm = ADCS_Add_Tolerance(ADCS_VectorNorm(v3),0);
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    m3[i] *= (1.0/m3_norm);
    v3[i] *= (1.0/v3_norm);
  }

  // Measurements and ephemerides matrix
  ADCS_CALC_TYPE measMaxtrix[ADCS_NUM_AXES][ADCS_NUM_AXES];
  ADCS_CALC_TYPE epheMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES];
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    measMaxtrix[i][0] = m1[i];
    measMaxtrix[i][1] = m2[i];
    measMaxtrix[i][2] = m3[i];
    epheMatrix[i][0] = v1[i];
    epheMatrix[i][1] = v2[i];
    epheMatrix[i][2] = v3[i];
  }

  // Transpose of the ephemerides matrix
  ADCS_CALC_TYPE invEpheMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES];
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
        invEpheMatrix[i][j] = epheMatrix[j][i];
    }
  }

  // Euler angles matrix (matrices multiplication)
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      eulerAnglesMatrix[i][j] = 0;
      for(uint8_t k = 0; k < ADCS_NUM_AXES; k++) {
        eulerAnglesMatrix[i][j] += (measMaxtrix[i][k] * invEpheMatrix[k][j]);
      }
    }
  }

}

