/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: adcs_main.c
   04/30/20

   This file drives the whole ADCS structure
*/

/************** Headers *****************/
#include "adcs.h"

/*********** Main function ***************/
void ADCS_Main(const uint8_t controlFlags, const uint32_t detumbleDuration, const uint32_t activeDuration,
               const uint8_t position[], const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion) {

  // save control flags
  adcsParams.control.val = controlFlags;

  // set science mode flag
  scienceModeActive = true;

  // set active control parameters
  if(!adcsParams.control.bits.detumbleOnly) {
    adcsParams.activeLen = activeDuration;
    memcpy(adcsParams.position, position, 6*sizeof(position[0]));
  }

  // always detumble first, active control will be enabled on successful detumble finish
  ADCS_Detumble_Init(detumbleDuration, orbitalInclination, meanOrbitalMotion);
}

ADCS_CALC_TYPE ADCS_VectorNorm(const ADCS_CALC_TYPE dim[]) {
  return(sqrt(pow(dim[0], 2) + pow(dim[1], 2) + pow(dim[2], 2)));
}

/************ Auxiliary functions implementation ***********/
void ADCS_Detumble_Init(const uint32_t detumbleDuration, const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion) {
    // cache parameters
    adcsParams.maxPulseInt = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_MAX_INTENSITY);     // Maximum applicable intensity
    adcsParams.maxPulseLen = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_MAX_LENGTH);    // Maximum pulse time available by energy reasons
    adcsParams.detumbleOmegaTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_DETUMB_OMEGA_TOLERANCE);   // Angular velocity tolerance to interrupt the detumbling for safety reasons
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
    FOSSASAT_DEBUG_PRINT(F("detumbleOmegaTol="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.detumbleOmegaTol, 4);
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
  // TODO add ADCS result reporting + ADCS on-going

  // check detumbling length
  if(millis() - adcsState.start > adcsParams.detumbleLen) {
    // time limit reached
    ADCS_Detumble_Finish(true);
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling done (time limit reached)"));
    return;
  }

  // check battery voltage or LP mode
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
    ADCS_Detumble_Finish(false);
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (battery too low)"));
    return;
  }
  #endif

  // check Hbridge faults
  uint8_t fault = bridgeX.getFault();
  if((!adcsParams.control.bits.overrideFaultX) && (fault != 0) && (fault & FAULT)) {
    ADCS_Detumble_Finish(false);
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (X axis fault)"));
    return;
  }

  fault = bridgeY.getFault();
  if((!adcsParams.control.bits.overrideFaultY) && (fault != 0) && (fault & FAULT)) {
    ADCS_Detumble_Finish(false);
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (Y axis fault)"));
    return;
  }

  fault = bridgeZ.getFault();
  if((!adcsParams.control.bits.overrideFaultZ) && (fault != 0) && (fault & FAULT)) {
    ADCS_Detumble_Finish(false);
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling stopped (Z axis fault)"));
    return;
  }

  // check abort command
  if(abortExecution) {
    abortExecution = false;
    ADCS_Detumble_Finish(false);
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
  if ((adcsParams.control.bits.overrideDetumbleTol) || (abs(omegaNorm - adcsState.prevOmegaNorm) >= adcsParams.detumbleOmegaTol)) {
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
    ADCS_Detumble_Finish(true);
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling done (tolerance reached)"));
    return;
  }

  // save values for the next iteration
  adcsState.prevOmegaNorm = omegaNorm;
  adcsState.prevIntensity[0] = intensity[0];
  adcsState.prevIntensity[1] = intensity[1];
  adcsState.prevIntensity[2] = intensity[2];
}

void ADCS_Detumble_Finish(bool startActiveControl) {
  ADCS_Finish();

  if(startActiveControl && !adcsParams.control.bits.detumbleOnly) {
    // initialize active control
    ADCS_ActiveControl_Init(adcsParams.activeLen, adcsParams.position);
  }
}

void ADCS_ActiveControl_Init(const uint32_t activeDuration, const uint8_t position[]) {
  // cache parameters
  adcsParams.maxPulseInt = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_MAX_INTENSITY);     // Maximum applicable intensity
  adcsParams.maxPulseLen = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_MAX_LENGTH);    // Maximum pulse time available
  adcsParams.activeEulerTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ACTIVE_EULER_TOLERANCE);   // Angular velocity tolerance to interrupt the detumbling for safety reasons
  adcsParams.activeOmegaTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ACTIVE_OMEGA_TOLERANCE);   // Angular tolerance between time steps to avoid overcontrolling
  adcsParams.eclipseThreshold = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ECLIPSE_THRESHOLD);
  adcsParams.rotationWeightRatio = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ROTATION_WEIGHT_RATIO);
  adcsParams.rotationTrigger = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ROTATION_TRIGGER);
  adcsParams.numControllers = PersistentStorage_Get<uint8_t>(FLASH_ADCS_NUM_CONTROLLERS);
  adcsParams.activeLen = activeDuration;

  // load controllers
  const size_t controllerBuffLen = 10*FLASH_ADCS_CONTROLLER_SLOT_SIZE;
  uint8_t controllerBuff[controllerBuffLen];
  PersistentStorage_Read(FLASH_ADCS_CONTROLLERS, controllerBuff, controllerBuffLen);
  memcpy(adcsParams.controllers, controllerBuff, controllerBuffLen);

  // print parameters for debugging
  FOSSASAT_DEBUG_PRINT(F("maxPulseInt="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.maxPulseInt, 4);
  FOSSASAT_DEBUG_PRINT(F("maxPulseLen="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.maxPulseLen, 4);
  FOSSASAT_DEBUG_PRINT(F("activeEulerTol="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.activeEulerTol, 4);
  FOSSASAT_DEBUG_PRINT(F("activeOmegaTol="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.activeOmegaTol, 4);
  FOSSASAT_DEBUG_PRINT(F("eclipseThreshold="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.eclipseThreshold, 4);
  FOSSASAT_DEBUG_PRINT(F("rotationWeightRatio="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.rotationWeightRatio, 4);
  FOSSASAT_DEBUG_PRINT(F("rotationTrigger="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.rotationTrigger, 4);
  FOSSASAT_DEBUG_PRINT(F("disturbCovariance="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.disturbCovariance, 4);
  FOSSASAT_DEBUG_PRINT(F("noiseCovariance="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.noiseCovariance, 4);
  FOSSASAT_DEBUG_PRINT(F("numControllers="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.numControllers);

  // wake up IMU
  adcsState.active = true;
  Sensors_IMU_Sleep(false);
  delay(50);

  // get initial IMU data
  Sensors_IMU_Update();
  ADCS_CALC_TYPE mag[ADCS_NUM_AXES];
  mag[0] = Sensors_IMU_CalcMag(imu.mx);
  mag[1] = Sensors_IMU_CalcMag(imu.my);
  mag[2] = Sensors_IMU_CalcMag(imu.mz);
  ADCS_CALC_TYPE omega[ADCS_NUM_AXES];
  omega[0] = Sensors_IMU_CalcGyro(imu.gx);
  omega[1] = Sensors_IMU_CalcGyro(imu.gy);
  omega[2] = Sensors_IMU_CalcGyro(imu.gz);
  adcsState.prevOmegaNorm = ADCS_VectorNorm(omega);

  // set initial state
  ADCS_CALC_TYPE magneticEphe[ADCS_NUM_AXES];
  ADCS_CALC_TYPE rotationMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES];
  ADCS_CALC_TYPE newAnglesVector[ADCS_NUM_AXES];
  // TODO Select ephemerides
  //select_ephemerides(i, position, ephemerides);  // Select the initial position
  adcsState.prevControlVector[0] = 0;
  adcsState.prevControlVector[1] = 0;
  adcsState.prevControlVector[2] = 0;

  ADS_Eclipse_Hybrid(mag, magneticEphe, rotationMatrix);
  ADS_Angles_Determination(rotationMatrix, newAnglesVector);
  adcsState.prevEulerNorm = ADCS_VectorNorm(newAnglesVector);
  adcsState.prevStateVars[0] = newAnglesVector[0];
  adcsState.prevStateVars[1] = newAnglesVector[1];
  adcsState.prevStateVars[2] = newAnglesVector[2];
  adcsState.prevStateVars[3] = omega[0];
  adcsState.prevStateVars[4] = omega[1];
  adcsState.prevStateVars[5] = omega[2];
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    adcsState.prevIntensity[i] = 0;
  }
  for(uint8_t i = 0; i < 2*ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < 2*ADCS_NUM_AXES; j++) {
      adcsState.kalmanMatrixP[i][j] = 0;
    }
  }

  // configure H bridge timer
  HbridgeTimer->setOverflow(adcsParams.bridgeTimerUpdatePeriod * (uint32_t)1000, MICROSEC_FORMAT);
  HbridgeTimer->attachInterrupt(ADCS_Update_Bridges);
  adcsState.bridgeStateX.outputHigh = false;
  adcsState.bridgeStateY.outputHigh = false;
  adcsState.bridgeStateZ.outputHigh = false;

  // configure ADCS timer
  AdcsTimer->setOverflow(adcsParams.timeStep * (uint32_t)1000, MICROSEC_FORMAT);
  AdcsTimer->attachInterrupt(ADCS_ActiveControl_Update);

  // start timers
  adcsState.start = millis();
  adcsState.bridgeStateX.lastUpdate = adcsState.start;
  adcsState.bridgeStateY.lastUpdate  = adcsState.start;
  adcsState.bridgeStateZ.lastUpdate  = adcsState.start;
  HbridgeTimer->resume();
  AdcsTimer->resume();
}

void ADCS_ActiveControl_Update() {
  // TODO add ADCS result reporting + ADCS on-going

  // check active control length
  if(millis() - adcsState.start > adcsParams.activeLen) {
    // time limit reached
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Active control done (time limit reached)"));
    return;
  }

  // check battery voltage or LP mode
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Active control stopped (battery too low)"));
    return;
  }
  #endif

  // check Hbridge faults
  uint8_t fault = bridgeX.getFault();
  if((!adcsParams.control.bits.overrideFaultX) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Active control stopped (X axis fault)"));
    return;
  }

  fault = bridgeY.getFault();
  if((!adcsParams.control.bits.overrideFaultY) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Active control stopped (Y axis fault)"));
    return;
  }

  fault = bridgeZ.getFault();
  if((!adcsParams.control.bits.overrideFaultZ) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Active control stopped (Z axis fault)"));
    return;
  }

  // check abort command
  if(abortExecution) {
    abortExecution = false;
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Active control stopped (aborted)"));
    return;
  }

  // TODO Select ephemerides
  uint8_t controller = 0;
  ADCS_CALC_TYPE solarEphe[ADCS_NUM_AXES];
  ADCS_CALC_TYPE magEphe[ADCS_NUM_AXES];

  // get IMU data
  Sensors_IMU_Update();
  ADCS_CALC_TYPE omega[ADCS_NUM_AXES];
  omega[0] = Sensors_IMU_CalcGyro(imu.gx);
  omega[1] = Sensors_IMU_CalcGyro(imu.gy);
  omega[2] = Sensors_IMU_CalcGyro(imu.gz);
  ADCS_CALC_TYPE magData[ADCS_NUM_AXES];
  magData[0] = Sensors_IMU_CalcMag(imu.mx);
  magData[1] = Sensors_IMU_CalcMag(imu.my);
  magData[2] = Sensors_IMU_CalcMag(imu.mz);

  ADCS_CALC_TYPE stateVars[2*ADCS_NUM_AXES];
  ADCS_CALC_TYPE prevStateVars[2*ADCS_NUM_AXES];
  ADCS_CALC_TYPE controlVector[2*ADCS_NUM_AXES];
  ADCS_CALC_TYPE prevControlVector[2*ADCS_NUM_AXES];
  ADCS_CALC_TYPE filtered_y[2*ADCS_NUM_AXES];
  ADCS_CALC_TYPE kalmanMatrixP[2*ADCS_NUM_AXES][2*ADCS_NUM_AXES];
  ADCS_CALC_TYPE newAnglesVector[ADCS_NUM_AXES];

  // copy all the volatile previous states
  for(uint8_t i = 0; i < 2*ADCS_NUM_AXES; i++){
    prevControlVector[i] = adcsState.prevControlVector[i];
    prevStateVars[i] = adcsState.prevStateVars[i];
    for(uint8_t j = 0; j < 2*ADCS_NUM_AXES; j++) {
      kalmanMatrixP[i][j] = adcsState.kalmanMatrixP[i][j];
    }
  }

  // Main structure
  ADS_Main(omega, magData, prevStateVars, prevControlVector, kalmanMatrixP, solarEphe, magEphe, filtered_y, newAnglesVector);

  // Control structure
  ADCS_CALC_TYPE eulerNorm = ADCS_VectorNorm(newAnglesVector);
  ADCS_CALC_TYPE omegaNorm = ADCS_VectorNorm(omega);
  bool eulerToleranceReached = (abs(eulerNorm - adcsState.prevEulerNorm) >= adcsParams.activeEulerTol);
  bool omegaToleranceReached = (abs(omegaNorm - adcsState.prevOmegaNorm) >= adcsParams.activeOmegaTol);
  if ((adcsParams.control.bits.overrideEulerTol || eulerToleranceReached) && (adcsParams.control.bits.overrideOmegaTol || omegaToleranceReached)) {
    // Choose controller from ADCS parameters
    // TODO configurable controllers
    float controllerMatrix[ADCS_NUM_AXES][2*ADCS_NUM_AXES];
    memcpy(controllerMatrix, adcsParams.controllers + (controller * FLASH_ADCS_CONTROLLER_SLOT_SIZE), FLASH_ADCS_CONTROLLER_SLOT_SIZE);

    // Active controlling
    ADCS_CALC_TYPE intensity[ADCS_NUM_AXES];
    ACS_OnboardControl(stateVars, magData, controllerMatrix, intensity);

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
    ADCS_Finish();
    FOSSASAT_DEBUG_PRINTLN(F("Active control done (tolerance reached)"));
    return;
  }

  // save values for the next iteration
  adcsState.prevEulerNorm = eulerNorm;
  adcsState.prevOmegaNorm = omegaNorm;
  for(uint8_t i = 0; i < 2*ADCS_NUM_AXES; i++) {
    adcsState.prevStateVars[i] = stateVars[i];
    adcsState.prevControlVector[i] = controlVector[i];
    for(uint8_t j = 0; j < 2*ADCS_NUM_AXES; j++) {
      adcsState.kalmanMatrixP[i][j] = kalmanMatrixP[i][j];
    }
  }
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
