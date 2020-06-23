/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: attitude_determination
   File:
   04/18/20

   This file drives the measurements hybridation algorithm
*/

/*********************** Headers ***********************/
#include "../ADCS/adcs.h"

/********************* Main function *********************/
void ADS_Measurement_Hybrid(ADCS_CALC_TYPE v_1[], ADCS_CALC_TYPE v_2[], ADCS_CALC_TYPE v_3[],
                            ADCS_CALC_TYPE m_1[], ADCS_CALC_TYPE m_2[], ADCS_CALC_TYPE m_3[],
                            ADCS_CALC_TYPE eulerAnglesMatrix[][ADCS_NUM_AXES]) {

  // Normalize both ephemeris and measurements vectors
  ADCS_CALC_TYPE m1_norm = ADCS_VectorNorm(m_1);
  ADCS_CALC_TYPE m2_norm = ADCS_VectorNorm(m_2);
  ADCS_CALC_TYPE m3_norm = ADCS_VectorNorm(m_3);

  ADCS_CALC_TYPE v1_norm = ADCS_VectorNorm(v_1);
  ADCS_CALC_TYPE v2_norm = ADCS_VectorNorm(v_2);
  ADCS_CALC_TYPE v3_norm = ADCS_VectorNorm(v_3);

  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
      m_1[i] *= (1.0/m1_norm);
      m_2[i] *= (1.0/m2_norm);
      m_3[i] *= (1.0/m3_norm);

      v_1[i] *= (1.0/v1_norm);
      v_2[i] *= (1.0/v2_norm);
      v_3[i] *= (1.0/v3_norm);
  }

  // Measurements matrix
  ADCS_CALC_TYPE measMaxtrix[ADCS_NUM_AXES][ADCS_NUM_AXES];
  measMaxtrix[0][0] = m_1[0];
  measMaxtrix[0][1] = m_2[0];
  measMaxtrix[0][2] = m_3[0];
  measMaxtrix[1][0] = m_1[1];
  measMaxtrix[1][1] = m_2[1];
  measMaxtrix[1][2] = m_3[1];
  measMaxtrix[2][0] = m_1[2];
  measMaxtrix[2][1] = m_2[2];
  measMaxtrix[2][2] = m_3[2];

  // Inverse theoretical matrix
  ADCS_CALC_TYPE invV[ADCS_NUM_AXES][ADCS_NUM_AXES];
  const ADCS_CALC_TYPE det_V = (  (v_1[0]*v_2[1]*v_3[2]) - (v_1[2]*v_2[1]*v_3[0]) + (v_1[1]*v_2[2]*v_3[0])
                                + (v_1[2]*v_2[0]*v_3[1]) - (v_1[1]*v_2[0]*v_3[2]) - (v_1[0]*v_2[2]*v_3[1])  );

  // det_V == -calcTol check
  ADCS_CALC_TYPE gain;
  if(det_V == (-1.0 * adcsParams.calcTol)) {
    gain = 1.0/(det_V - adcsParams.calcTol);
  } else {
    gain = 1.0/(det_V + adcsParams.calcTol);
  }

  invV[0][0] = gain * (v_2[1]*v_3[2] - v_2[2]*v_3[1]);
  invV[0][1] = gain * (v_2[2]*v_3[0] - v_2[0]*v_3[2]);
  invV[0][2] = gain * (v_2[0]*v_3[1] - v_2[1]*v_3[0]);
  invV[1][0] = gain * (v_1[2]*v_3[1] - v_1[1]*v_3[2]);
  invV[1][1] = gain * (v_1[0]*v_3[2] - v_1[2]*v_3[0]);
  invV[1][2] = gain * (v_1[1]*v_3[0] - v_1[0]*v_3[1]);
  invV[2][0] = gain * (v_1[1]*v_2[2] - v_1[2]*v_2[1]);
  invV[2][1] = gain * (v_1[2]*v_2[0] - v_1[0]*v_2[2]);
  invV[2][2] = gain * (v_1[0]*v_2[1] - v_1[1]*v_2[0]);

  // Euler angles matrix (matrices multiplication)
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      for(uint8_t k = 0; k < ADCS_NUM_AXES; k++) {
        eulerAnglesMatrix[i][j] += (measMaxtrix[i][k] * invV[k][j]);
      }
    }
  }
}
