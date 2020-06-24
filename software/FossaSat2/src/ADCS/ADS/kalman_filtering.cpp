/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: attitude_determination
   File: kalman_filtering.c
   04/18/20

   This file drives the Kalman filtering processing
*/

/***************** Headers ********************/
#include "../ADCS/adcs.h"


/*************** Auxiliary functions implementation *****************/
void ADS_Inverse_Matrix(ADCS_CALC_TYPE matrix[][ADCS_STATE_DIM]) {
	ADCS_CALC_TYPE temp;

	// Create the augmented matrix
	for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
		for(uint8_t j = 0; j < 2 * ADCS_STATE_DIM; j++) {
			if(j == (i + ADCS_STATE_DIM)) {
        matrix[i][j] = 1;
      }
		}
	}

	// Interchange the rows of matrix from the end
  ADCS_CALC_TYPE tempArr[ADCS_STATE_DIM];
	for (uint8_t i = ADCS_STATE_DIM - 1; i > 0; i--) {
		if(matrix[i - 1][0] < matrix[i][0]) {
      for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
        tempArr[j] = matrix[i][j];
      }
      for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
        matrix[i][j] = matrix[i - 1][j];
      }
      for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
        matrix[i - 1][j] = tempArr[j];
      }
		}
	}

	// Replace a row by sum of itself and a
	// constant multiple of another row of the matrix
	for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
		for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
			if (j != i) {
				temp = matrix[j][i] / matrix[i][i];
				for(uint8_t k = 0; k < 2 * ADCS_STATE_DIM; k++) {
					matrix[j][k] -= (matrix[i][k] * temp);
				}
			}
		}
	}

	// Multiply each row by a nonzero integer.
	// Divide row element by the diagonal element
	for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
		temp = matrix[i][i];
		for(uint8_t j = 0; j < 2 * ADCS_STATE_DIM; j++) {
			matrix[i][j] = matrix[i][j] / temp;
		}
	}
}

/*************** Main function ******************/
void ADS_Kalman_Filter(const ADCS_CALC_TYPE Q, const ADCS_CALC_TYPE R, const ADCS_CALC_TYPE delta_t,
                       const ADCS_CALC_TYPE x0[], const ADCS_CALC_TYPE y0[], const ADCS_CALC_TYPE u0[],
                       ADCS_CALC_TYPE P[][ADCS_STATE_DIM], ADCS_CALC_TYPE filtered_y[]) {

  // Constants and variables declarations and preliminary computations
  const ADCS_CALC_TYPE u[ADCS_STATE_DIM] = {u0[0], u0[1], u0[2]};                      // Total control vector
  const ADCS_CALC_TYPE invI[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{}, {}, {}};            // Inertia tensor inverse
  ADCS_CALC_TYPE M_aux1[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{0},{0},{0},{0},{0},{0}};   // Auxiliary matrix
  ADCS_CALC_TYPE M_aux2[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{0},{0},{0},{0},{0},{0}};   // Auxiliary matrix
  ADCS_CALC_TYPE M_aux3[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{0},{0},{0},{0},{0},{0}};   // Auxiliary matrix

  ADCS_CALC_TYPE S[ADCS_STATE_DIM][ADCS_STATE_DIM];                                          // Partial solution of the CARE
  ADCS_CALC_TYPE invS[ADCS_STATE_DIM][ADCS_STATE_DIM];                                       // Inverse of the matrix
  ADCS_CALC_TYPE kalman_gain[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{0},{0},{0},{0},{0},{0}};    // Proper Kalman filter

  // Initialization of the algorithm in the general case
  ADCS_CALC_TYPE x_k[ADCS_STATE_DIM] = {0};                                 // Filtered state of the satellite
  ADCS_CALC_TYPE y_k[ADCS_STATE_DIM] = {0};                                 // Filtered measurements

  // Noise and disturbances covariance matrices
  const ADCS_CALC_TYPE Q_m[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{Q,0,0,0,0,0},{0,Q,0,0,0,0},{0,0,Q,0,0,0},{0,0,0,Q,0,0},{0,0,0,0,Q,0},{0,0,0,0,0,Q}};
  const ADCS_CALC_TYPE R_m[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{R,0,0,0,0,0},{0,R,0,0,0,0},{0,0,R,0,0,0},{0,0,0,R,0,0},{0,0,0,0,R,0},{0,0,0,0,0,R}};

  // State space linearised matrices
  const ADCS_CALC_TYPE C[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
  const ADCS_CALC_TYPE A[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{1,0,0,delta_t,0,0},{0,1,0,0,delta_t,0},{0,0,1,0,0,delta_t},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};

  ADCS_CALC_TYPE A_trans[ADCS_STATE_DIM][ADCS_STATE_DIM];
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      A_trans[i][j] = A[j][i];
    }
  }

  ADCS_CALC_TYPE B[ADCS_STATE_DIM][ADCS_STATE_DIM] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  for(uint8_t i = ADCS_STATE_DIM/2; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = ADCS_STATE_DIM/2; j < ADCS_STATE_DIM; j++) {
      B[i][j] = invI[(i-ADCS_STATE_DIM/2)][(j-ADCS_STATE_DIM/2)]*delta_t;
    }
  }

  // Main structure and filtering algorithm
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      x_k[i] += A[i][j]*x0[j]+B[i][j]*u[j];
    }
    y_k[i] = y0[i]-x_k[i];
  }

  // Compute the expected-error matrix
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      for(uint8_t k = 0; k < ADCS_STATE_DIM; k++) {
        M_aux1[i][j] += P[i][k]*A_trans[k][j];
        M_aux2[i][j] += A[i][k]*M_aux1[k][j];
      }
    }
  }

  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      P[i][j] = M_aux2[i][j]+Q_m[i][j];
    }
  }

  // Inverse S matrix
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      S[i][j] = P[i][j]+R_m[i][j];
    }
  }

  ADS_Inverse_Matrix(S);

  // Compute the Kalman filter matrix
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      for(uint8_t k = 0; k < ADCS_STATE_DIM; k++) {
        kalman_gain[i][j] += P[i][k]*invS[k][j];
      }
    }
  }

  // Kalman transfer function and processing
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    filtered_y[i] = 0;
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      filtered_y[i] += kalman_gain[i][j]*y_k[j];
      filtered_y[i] += x0[i];
    }
  }

  // Next expected-error matrix
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      for(uint8_t k = 0; k < ADCS_STATE_DIM; k++) {
        M_aux3[i][j] += (C[i][k]-kalman_gain[i][k])*P[k][j];
      }
    }
  }

  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      P[i][j] = M_aux3[i][j];
    }
  }
}
