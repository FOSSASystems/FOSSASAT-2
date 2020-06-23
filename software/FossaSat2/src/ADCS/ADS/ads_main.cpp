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
void ADS_Main(ADCS_CALC_TYPE state[], ADCS_CALC_TYPE controlVector[], ADCS_CALC_TYPE P[][ADCS_NUM_AXES], ADCS_CALC_TYPE ephemerides[], ADCS_CALC_TYPE filtered_y[]) {
  
}

/*void ads_main(double state_variables[], double control_vector[], float P[][], float ephemerides[], float filtered_y[])
{
    // Constants declaration
    const float Q = ;                                   // Disturbances covariance matrix
    const float R = ;                                   // Noise covariance matrix
    float dt = ;                                        // Time step between measurements

    // Relevant variables declaration
    double lux_data[NUMBER_PANELS];                     // Solar measurements vector declaration
    double magnetic_data[STATE_DIM/2];                  // Magnetic measurements vector declaration

    double r_s1[STATE_DIM/2];                           // Solar ephemerides in the body frame
    double r_s2[STATE_DIM/2];                           // Redundant solar ephemerides in the body frame

    double angles_int[STATE_DIM/2];                     // Angular determination variables
    double rotation_matrix[STATE_DIM/2][STATE_DIM/2];   // Rotation matrix
    double new_angles_vector[STATE_DIM/2];              // Angles at time t+1
    double measurements[STATE_DIM];                     // Controller input

    // Previous state of the system from t = t_1-1
    double phi_0 = state_variables[0];
    double psi_0 = state_variables[1];
    double theta_0 = state_variables[2];

    // Theoretical LVHV values
    double solar_ephe[STATE_DIM/2] = {ephemerides[0], ephemerides[1], ephemerides[2]};
    double magnetic_ephe[STATE_DIM/2] = {ephemerides[3], ephemerides[4], ephemerides[5]};

    // Measurements from sensors
    double p = ;        // IMU angular velocity data calling
    double q = ;
    double r = ;

    lux_data[] = ;      // Solar panels data calling
    magn_data[0] = //Call the magnetometer (x direction);
    magn_data[1] = //Call the magnetometer (y direction);
    magn_data[2] = //Call the magnetometer (z direction);

    // Decide whether the satellite is in eclipse situation or under light
    bool eclipse = eclipse_decision(lux_data, NUMBER_PANELS);
    if (eclipse == TRUE)
    {
        // Generation of the measurements
            euler_integrator(angles_int, p, q, r, theta_0, phi_0, psi_0);
            eclipse_hybrid(magnetic_data, magnetic_ephe, rotation_matrix);

        // Processing of the measurements
            angles_determination(rotation_matrix, new_angles_vector);
            rotation_verification(new_angles_vector, angles_int);
    }
    else if (eclipse == FALSE)
    {
        // Generation of the measurements
            euler_integrator(angles_int, p, q, r, theta_0, phi_0, psi_0);
            solar_determination(lux_data, solar_ephe, r_s1, r_s2);
            measure_hybrid(r_s1, r_s2, magnetic_data, solar_ephe, solar_ephe, magnetic_ephe, rotation_matrix)
            angles_determination(rotation_matrix, new_angles_vector);
            rotation_verification(new_angles_vector, angles_int);
    }

    // Generation of the state point
    measurements[0] = new_angles_vector[0];             // X rotation angle -roll-
    measurements[1] = new_angles_vector[1];             // Y rotation angle -pitch-
    measurements[2] = new_angles_vector[2];             // Z rotation angle -yaw-
    measurements[3] = p;                                //IMU ANGULAR VELOCITY p;
    measurements[4] = q;                                //IMU ANGULAR VELOCITY q;
    measurements[5] = r;                                //IMU ANGULAR VELOCITY r;

    // Filtering of the signal
    kalman_filtering(Q, R, dt, state_variables, measurements,
                     control_vector, filtered_y, P);

  return;
}*/
