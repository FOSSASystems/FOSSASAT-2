/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: adcs_main.c
   04/30/20

   This file drives the whole ADCS structure
*/

/************** Headers *****************/
#include "adcs.h"

/********* Auxiliary functions *******/
//void active_controlling(const int active_time, const int position, float pulse_times[][3]);

/*********** Main function ***************/
void ADCS_Main(const uint8_t controlFlags, const uint32_t detumbleDuration, const uint32_t activeDuration,
               const uint8_t position[], const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion) {

  // save control flags
  adcsParams.control.val = controlFlags;

  // set science mode flag
  scienceModeActive = true;

  // always detumble first
  ADCS_Detumble_Init(detumbleDuration, orbitalInclination, meanOrbitalMotion);

  if(!adcsParams.control.bits.detumbleOnly) {
    // Active controlling loop
    //active_controlling(active_time-detumbleTime, position, pulse_times);
  }
}

ADCS_CALC_TYPE ADCS_VectorNorm(const ADCS_CALC_TYPE dim[]) {
  return(sqrt(pow(dim[0], 2) + pow(dim[1], 2) + pow(dim[2], 2)));
}

/************ Auxiliary functions implementation ***********/
void ADCS_Detumble_Init(const uint32_t detumbleDuration, const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion) {
    // cache parameters
    adcsParams.maxPulseInt = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_MAX_INTENSITY);     // Maximum applicable intensity
    adcsParams.maxPulseLen = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_MAX_LENGTH);    // Maximum pulse time available by energy reasons
    adcsParams.omegaTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_OMEGA_TOLERANCE);   // Angular velocity tolerance to interrupt the detumbling for safety reasons
    adcsParams.timeStep = PersistentStorage_Get<uint32_t>(FLASH_ADCS_TIME_STEP);           // Time step between to calculation instants
    adcsParams.minInertialMoment = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_MIN_INERTIAL_MOMENT); // Minimum inertial moment
    adcsParams.pulseAmplitude = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_AMPLITUDE);  // Amplitude of pulse for ACS_IntensitiesRectifier
    adcsParams.calcTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_CALCULATION_TOLERANCE);
    adcsParams.orbInclination = orbitalInclination;
    adcsParams.meanOrbitalMotion = meanOrbitalMotion;
    adcsParams.detumbleLen = detumbleDuration;
    adcsParams.bridgeTimerUpdatePeriod = PersistentStorage_Get<uint32_t>(FLASH_ADCS_BRIDGE_TIMER_UPDATE_PERIOD);
    adcsParams.bridgeOutputHigh = PersistentStorage_Get<int8_t>(FLASH_ADCS_BRIDGE_OUTPUT_HIGH);
    adcsParams.bridgeOutputLow = PersistentStorage_Get<int8_t>(FLASH_ADCS_BRIDGE_OUTPUT_LOW);

    // Coil magnetic characteristics -shall be introduced already inversed-.
    const uint8_t coilCharLen = ADCS_NUM_AXES*ADCS_NUM_AXES*sizeof(ADCS_CALC_TYPE);
    uint8_t coilCharBuff[coilCharLen];
    PersistentStorage_Read(FLASH_ADCS_COIL_CHAR_MATRIX, coilCharBuff, coilCharLen);
    memcpy(adcsParams.coilChar, coilCharBuff, coilCharLen);

    // print parameters for debugging
    FOSSASAT_DEBUG_PRINT(F("timeStep="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.timeStep);
    FOSSASAT_DEBUG_PRINT(F("maxPulseInt="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.maxPulseInt, 4);
    FOSSASAT_DEBUG_PRINT(F("maxPulseLen="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.maxPulseLen, 4);
    FOSSASAT_DEBUG_PRINT(F("omegaTol="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.omegaTol, 4);
    FOSSASAT_DEBUG_PRINT(F("minInertialMoment="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.minInertialMoment, 4);
    FOSSASAT_DEBUG_PRINT(F("pulseAmplitude="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.pulseAmplitude, 4);
    FOSSASAT_DEBUG_PRINT(F("orbInclination="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.orbInclination, 4);
    FOSSASAT_DEBUG_PRINT(F("meanOrbitalMotion="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.meanOrbitalMotion, 8);
    FOSSASAT_DEBUG_PRINTLN(F("coilChar="));
    for(uint8_t row = 0; row < ADCS_NUM_AXES; row++) {
      for(uint8_t col = 0; col < ADCS_NUM_AXES; col++) {
        FOSSASAT_DEBUG_PRINT(adcsParams.coilChar[row][col], 4);
        FOSSASAT_DEBUG_PRINT('\t');
      }
      FOSSASAT_DEBUG_PRINTLN();
    }
    FOSSASAT_DEBUG_PRINTLN();
    FOSSASAT_DEBUG_PRINT(F("bridgeTimerUpdatePeriod="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.bridgeTimerUpdatePeriod);
    FOSSASAT_DEBUG_PRINT(F("bridgeOutputHigh="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.bridgeOutputHigh);
    FOSSASAT_DEBUG_PRINT(F("bridgeOutputLow="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.bridgeOutputLow);

    // wake up IMU
    adcsState.active = true;
    Sensors_IMU_Sleep(false);
    delay(50);

    // set initial state
    for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
      adcsState.prevIntensity[i] = 0;
    }

    // get initial IMU data
    Sensors_IMU_Update();
    ADCS_CALC_TYPE omega[ADCS_NUM_AXES];
    omega[0] = Sensors_IMU_CalcGyro(imu.gx);
    omega[1] = Sensors_IMU_CalcGyro(imu.gy);
    omega[2] = Sensors_IMU_CalcGyro(imu.gz);
    adcsState.prevOmegaNorm = ADCS_VectorNorm(omega);

    FOSSASAT_DEBUG_PRINT(F("omega= \t"));
    FOSSASAT_DEBUG_PRINT(omega[0], 4); FOSSASAT_DEBUG_PRINT('\t');
    FOSSASAT_DEBUG_PRINT(omega[1], 4); FOSSASAT_DEBUG_PRINT('\t');
    FOSSASAT_DEBUG_PRINTLN(omega[2], 4);
    FOSSASAT_DEBUG_PRINT(F("omegaNorm=\t"));
    FOSSASAT_DEBUG_PRINTLN(adcsState.prevOmegaNorm, 4);

    // configure H bridge timer
    HbridgeTimer->setOverflow(adcsParams.bridgeTimerUpdatePeriod * (uint32_t)1000, MICROSEC_FORMAT);
    HbridgeTimer->attachInterrupt(ADCS_Update_Bridges);
    adcsState.bridgeStateX.outputHigh = false;
    adcsState.bridgeStateY.outputHigh = false;
    adcsState.bridgeStateZ.outputHigh = false;

    // configure ADCS timer
    AdcsTimer->setOverflow(adcsParams.timeStep * (uint32_t)1000, MICROSEC_FORMAT);
    AdcsTimer->attachInterrupt(ADCS_Detumble_Update);

    // start timers
    adcsState.start = millis();
    adcsState.bridgeStateX.lastUpdate = adcsState.start;
    adcsState.bridgeStateY.lastUpdate  = adcsState.start;
    adcsState.bridgeStateZ.lastUpdate  = adcsState.start;
    HbridgeTimer->resume();
    AdcsTimer->resume();
}

void ADCS_Detumble_Update() {
  // TODO add ADCS result reporting

  // check detumbling length
  if(millis() - adcsState.start > adcsParams.detumbleLen) {
    // time limit reached
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling done (time limit reached)"));
    return;
  }

  // check battery voltage or LP mode
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (battery too low)"));
    return;
  }
  #endif

  // check Hbridge faults
  uint8_t fault = bridgeX.getFault();
  if((!adcsParams.control.bits.overrideFaultX) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (X axis fault)"));
    return;
  }

  fault = bridgeY.getFault();
  if((!adcsParams.control.bits.overrideFaultY) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (Y axis fault)"));
    return;
  }

  fault = bridgeZ.getFault();
  if((!adcsParams.control.bits.overrideFaultZ) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (Z axis fault)"));
    return;
  }

  // check abort command
  if(abortExecution) {
    abortExecution = false;
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (aborted)"));
    return;
  }

  // Call sensors data
  Sensors_IMU_Update();

  // Call for the magnetometer raw data
  ADCS_CALC_TYPE mag[ADCS_NUM_AXES];
  mag[0] = Sensors_IMU_CalcMag(imu.mx);
  mag[1] = Sensors_IMU_CalcMag(imu.my);
  mag[2] = Sensors_IMU_CalcMag(imu.mz);

  // Call the IMU angular velocity data
  ADCS_CALC_TYPE omega[ADCS_NUM_AXES];
  omega[0] = Sensors_IMU_CalcGyro(imu.gx);
  omega[1] = Sensors_IMU_CalcGyro(imu.gy);
  omega[2] = Sensors_IMU_CalcGyro(imu.gz);

  ADCS_CALC_TYPE omegaNorm = ADCS_VectorNorm(omega);
  ADCS_CALC_TYPE intensity[ADCS_NUM_AXES];

  FOSSASAT_DEBUG_PRINT(F("mag=\t\t"));
  FOSSASAT_DEBUG_PRINT(mag[0], 4); FOSSASAT_DEBUG_PRINT('\t');
  FOSSASAT_DEBUG_PRINT(mag[1], 4); FOSSASAT_DEBUG_PRINT('\t');
  FOSSASAT_DEBUG_PRINTLN(mag[2], 4);
  FOSSASAT_DEBUG_PRINT(F("omega= \t"));
  FOSSASAT_DEBUG_PRINT(omega[0], 4); FOSSASAT_DEBUG_PRINT('\t');
  FOSSASAT_DEBUG_PRINT(omega[1], 4); FOSSASAT_DEBUG_PRINT('\t');
  FOSSASAT_DEBUG_PRINTLN(omega[2], 4);
  FOSSASAT_DEBUG_PRINT(F("omegaNorm=\t"));
  FOSSASAT_DEBUG_PRINTLN(omegaNorm, 4);

  // check tolerance
  if ((adcsParams.control.bits.overrideDetumbleTol) || (abs(omegaNorm - adcsState.prevOmegaNorm) >= adcsParams.omegaTol)) {
      // Control law generation
      ACS_BdotFunction(omega, mag, intensity);
      ADCS_CALC_TYPE intensityNorm = ADCS_VectorNorm(intensity);
      ADCS_CALC_TYPE pulseLength[ADCS_NUM_AXES];
      FOSSASAT_DEBUG_PRINT(F("intensityNorm=\t"));
      FOSSASAT_DEBUG_PRINTLN(intensityNorm, 4);

      if (intensityNorm < adcsParams.maxPulseInt) {
          // Calculate the intensities
          pulseLength[0] = ACS_IntensitiesRectifier(adcsState.prevIntensity[0], intensity[0], adcsParams.timeStep);
          pulseLength[1] = ACS_IntensitiesRectifier(adcsState.prevIntensity[1], intensity[1], adcsParams.timeStep);
          pulseLength[2] = ACS_IntensitiesRectifier(adcsState.prevIntensity[2], intensity[2], adcsParams.timeStep);

      } else {
          // Max intensity application
          pulseLength[0] = adcsParams.maxPulseLen;
          pulseLength[1] = adcsParams.maxPulseLen;
          pulseLength[2] = adcsParams.maxPulseLen;
      }

      // update H-bridges
      adcsState.bridgeStateX.pulseLen = (uint32_t)(pulseLength[0]);
      adcsState.bridgeStateY.pulseLen = (uint32_t)(pulseLength[1]);
      adcsState.bridgeStateZ.pulseLen = (uint32_t)(pulseLength[2]);

      FOSSASAT_DEBUG_PRINT(F("intensity=\t"));
      FOSSASAT_DEBUG_PRINT(intensity[0], 8); FOSSASAT_DEBUG_PRINT('\t');
      FOSSASAT_DEBUG_PRINT(intensity[1], 8); FOSSASAT_DEBUG_PRINT('\t');
      FOSSASAT_DEBUG_PRINTLN(intensity[2], 8);
      FOSSASAT_DEBUG_PRINT(F("pulseLength=\t"));
      FOSSASAT_DEBUG_PRINT(pulseLength[0], 8); FOSSASAT_DEBUG_PRINT('\t');
      FOSSASAT_DEBUG_PRINT(pulseLength[1], 8); FOSSASAT_DEBUG_PRINT('\t');
      FOSSASAT_DEBUG_PRINTLN(pulseLength[2], 8);
      FOSSASAT_DEBUG_PRINTLN();

  } else {
    // tolerance reached, stop detumbling
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling done (tolerance reached)"));
    return;
  }

  // save values for the next iteration
  adcsState.prevOmegaNorm = omegaNorm;
  adcsState.prevIntensity[0] = intensity[0];
  adcsState.prevIntensity[1] = intensity[1];
  adcsState.prevIntensity[2] = intensity[2];
}

void ADCS_Finish() {
  adcsState.active = false;
  bridgeX.stop();
  bridgeY.stop();
  bridgeZ.stop();
  AdcsTimer->pause();
  AdcsTimer->detachInterrupt();
  HbridgeTimer->pause();
  scienceModeActive = false;
}

void ADCS_Update_Bridges() {
  uint32_t currTime = millis();

  if(currTime - adcsState.bridgeStateX.lastUpdate >= adcsState.bridgeStateX.pulseLen) {
    adcsState.bridgeStateX.lastUpdate = currTime;
    if(adcsState.bridgeStateX.outputHigh) {
      bridgeX.drive(adcsParams.bridgeOutputHigh);
    } else {
      bridgeX.drive(adcsParams.bridgeOutputLow);
    }
    adcsState.bridgeStateX.outputHigh = !adcsState.bridgeStateX.outputHigh;
  }

  if(currTime - adcsState.bridgeStateY.lastUpdate >= adcsState.bridgeStateY.pulseLen) {
    adcsState.bridgeStateY.lastUpdate = currTime;
    if(adcsState.bridgeStateY.outputHigh) {
      bridgeY.drive(adcsParams.bridgeOutputHigh);
    } else {
      bridgeY.drive(adcsParams.bridgeOutputLow);
    }
    adcsState.bridgeStateY.outputHigh = !adcsState.bridgeStateY.outputHigh;
  }

  if(currTime - adcsState.bridgeStateZ.lastUpdate >= adcsState.bridgeStateZ.pulseLen) {
    adcsState.bridgeStateZ.lastUpdate = currTime;
    if(adcsState.bridgeStateZ.outputHigh) {
      bridgeZ.drive(adcsParams.bridgeOutputHigh);
    } else {
      bridgeZ.drive(adcsParams.bridgeOutputLow);
    }
    adcsState.bridgeStateZ.outputHigh = !adcsState.bridgeStateZ.outputHigh;
  }
}

/*
void active_controlling(const int active_time, const float T, const int position[], float pulse_times[][])
{
    // Constants declaration
    const STATE_DIM = 6;        // Number of relevant attitude variables
    const float MAX_INT = ;     // Maximum applicable intensity
    const float MAX_TIME = ;    // Maximum pulse time available
    const float OMEGA_TOL = ;   // Angular velocity tolerance to interrupt the detumbling for safety reasons
    const float ANGULAR_TOL = ; // Angular tolerance between time steps to avoid overcontrolling
    const DELTA_T = ;           // Time step between to calculation instants

    const int interval_1 = 1;   // First position interval
    const int interval_2 = 2;   // Second position interval
    const int interval_3 = 3;   // Third position interval
    const int interval_4 = 4;   // Fourth position interval

    // Controllers
    float K_1[STATE_DIM][STATE_DIM];
    float K_2[STATE_DIM][STATE_DIM];
    float K_3[STATE_DIM][STATE_DIM];
    float K_4[STATE_DIM][STATE_DIM];
    float K_I[STATE_DIM][STATE_DIM];
    controller_generation(K_1, K_2, K_3, K_4, K_I); // Call for the controller data�

    // Kalman filter gain generation
    float kalman_gain[STATE_DIM][STATE_DIM];
    kalman_filter_generation(kalman_filter);

    // Variables declaration
    int i = 0;
    bool go_on = TRUE;
    float magn_data[STATE_DIM/2];
    float state_variables[active_time][STATE_DIM];

    //Main Loop
    pulse_times[i][0] = 0;  // Initialization of the intensities
    pulse_times[i][1] = 0;
    pulse_times[i][2] = 0;
    intensity_norm[i] = 0;

    ads_main(kalman_gain, magn_data, state_variables[i]);
    omega_norm[i] = sqrt(state_variables[i][0]+state_variables[i][1]+state_variables[i][2]);
    euler_norm[i] = sqrt(state_variables[i][3]+state_variables[i][4]+state_variables[i][5]);

    i++;

    while ((i < active_time)&&(go_on == TRUE))
    {
        // ADS on
        ads_main(kalman_gain, magn_data, state_variables[i]);

        // Control structure
        if (((abs(euler_norm[i]-euler_norm[i-1]) >= ANGULAR_TOL)&&(abs(omega_norm[i]-omega_norm[i-1]) >= OMEGA_TOL))
        {
            // ACS on
            switch (position[i]) // Choose controller
            {
            case (interval_1):
                K = K_1;
            case (interval_2):
                K = K_2;
            case (interval_3):
                K = K_3;
            case (interval_4):
                K = K_4;
            default:
                K = K_I;
            }

            // Active controlling
            onboardcontrol(state_variables, magn_data, K, I[i]);
            intensity_norm[i] = sqrt(I[i][0]^2+I[i][1]^2+I[i][2]^2);
            if (intensity_norm[i] < MAX_INT)
            {
                // Calculate the intensities
                pulse_times[i][0] = intensities_rectifier(I[i-1][0], I[i][0], DELTA_T);
                pulse_times[i][1] = intensities_rectifier(I[i-1][1], I[i][1], DELTA_T);
                pulse_times[i][2] = intensities_rectifier(I[i-1][2], I[i][2], DELTA_T);
            }
            else
            {
                pulse_times[i][0] = MAX_TIME;
                pulse_times[i][1] = MAX_TIME;
                pulse_times[i][2] = MAX_TIME;
            }
        }
        else
        {
            // Do not do anything
            pulse_times[i][0] = 0;
            pulse_times[i][1] = 0;
            pulse_times[i][2] = 0;
        }
        i++;
    }

   return;
}*/
