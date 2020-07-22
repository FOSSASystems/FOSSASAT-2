/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: adcs_main.c
   04/30/20

   This file drives the whole ADCS structure
*/

/************** Headers *****************/
#include "adcs.h"

ADCS_CALC_TYPE ADCS_VectorNorm(const ADCS_CALC_TYPE dim[ADCS_NUM_AXES]) {
  return(sqrt(pow(dim[0], 2) + pow(dim[1], 2) + pow(dim[2], 2)));
}

ADCS_CALC_TYPE ADCS_Add_Tolerance(ADCS_CALC_TYPE var, ADCS_CALC_TYPE forbiddenVal) {
  // check if variable is within the interval <forbiddenVal - tolerance, forbiddenVal + tolerance>
  ADCS_CALC_TYPE limitHigh = forbiddenVal + adcsParams.calcTol;
  ADCS_CALC_TYPE limitLow = forbiddenVal - adcsParams.calcTol;
  if((var < limitLow) || (var > limitHigh)) {
    return(var);
  }

  // variable is within range, clamp to limits
  if(var > forbiddenVal) {
    return(limitHigh);
  }
  return(limitLow);
}

/************ Auxiliary functions implementation ***********/
void ADCS_Detumble_Init(const uint8_t controlFlags, const uint32_t detumbleDuration, const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion) {
    // execute common part
    ADCS_Common_Init(controlFlags, orbitalInclination, meanOrbitalMotion);

    // cache parameters
    adcsParams.detumbleOmegaTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_DETUMB_OMEGA_TOLERANCE);   // Angular velocity tolerance to interrupt the detumbling for safety reasons
    adcsParams.detumbleLen = detumbleDuration;

    // print parameters for debugging
    FOSSASAT_DEBUG_PRINT(F("detumbleOmegaTol="));
    FOSSASAT_DEBUG_PRINTLN(adcsParams.detumbleOmegaTol, 4);

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
    Sensors_IMU_CalcGyro(imu.gz, imu.gx, -1*imu.gy, FLASH_IMU_OFFSET_GYRO_X, omega);
    adcsState.prevOmegaNorm = ADCS_VectorNorm(omega);

    FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(omega, ADCS_NUM_AXES);
    FOSSASAT_DEBUG_PRINT(F("omegaNorm=\t"));
    FOSSASAT_DEBUG_PRINTLN(adcsState.prevOmegaNorm, 4);

    // configure timers
    ADCS_Setup_Timers(ADCS_Detumble_Update);
}

void ADCS_Detumble_Update() {
  // check all conditions
  if(!ADCS_Check()) {
    return;
  }

  // Call sensors data
  Sensors_IMU_Update();

  // Call for the magnetometer raw data
  ADCS_CALC_TYPE mag[ADCS_NUM_AXES];
  Sensors_IMU_CalcMag(imu.mz, -1*imu.mx, -1*imu.my, FLASH_IMU_OFFSET_MAG_X, mag);

  // Call the IMU angular velocity data
  ADCS_CALC_TYPE omega[ADCS_NUM_AXES];
  Sensors_IMU_CalcGyro(imu.gz, imu.gx, -1*imu.gy, FLASH_IMU_OFFSET_GYRO_X, omega);

  ADCS_CALC_TYPE omegaNorm = ADCS_VectorNorm(omega);
  ADCS_CALC_TYPE intensity[ADCS_NUM_AXES];

  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(mag, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(omega, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT(F("omegaNorm=\t"));
  FOSSASAT_DEBUG_PRINTLN(omegaNorm, 4);

  // check tolerance
  if ((adcsParams.control.bits.overrideDetumbleTol) || (abs(omegaNorm - adcsState.prevOmegaNorm) >= adcsParams.detumbleOmegaTol)) {
      // Control law generation
      ACS_BdotFunction(omega, mag, adcsParams.coilChar, adcsParams.meanOrbitalMotion, adcsParams.orbInclination, adcsParams.minInertialMoment, intensity);

      // calculate and update pulse length
      ADCS_Set_Pulse_Lengths(intensity);

  } else {
    // tolerance reached, stop detumbling
    ADCS_Finish(ADCS_RES_DONE_TOL_REACHED);
    FOSSASAT_DEBUG_PRINTLN(F("Detumbling done (tolerance reached)"));
    return;
  }

  // save values for the next iteration
  adcsState.prevOmegaNorm = omegaNorm;
  adcsState.prevIntensity[0] = intensity[0];
  adcsState.prevIntensity[1] = intensity[1];
  adcsState.prevIntensity[2] = intensity[2];
}

void ADCS_ActiveControl_Init(const uint8_t controlFlags, const uint32_t activeDuration, const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion) {
  // execute common part
  ADCS_Common_Init(controlFlags, orbitalInclination, meanOrbitalMotion);

  // cache parameters
  adcsParams.activeEulerTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ACTIVE_EULER_TOLERANCE);   // Angular velocity tolerance to interrupt the detumbling for safety reasons
  adcsParams.activeOmegaTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ACTIVE_OMEGA_TOLERANCE);   // Angular tolerance between time steps to avoid overcontrolling
  adcsParams.eclipseThreshold = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ECLIPSE_THRESHOLD);
  adcsParams.rotationWeightRatio = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ROTATION_WEIGHT_RATIO);
  adcsParams.rotationTrigger = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_ROTATION_TRIGGER);
  adcsParams.numControllers = PersistentStorage_Get<uint8_t>(FLASH_ADCS_NUM_CONTROLLERS);
  adcsParams.disturbCovariance = PersistentStorage_Get<ADCS_CALC_TYPE>(ADCS_DISTURBANCE_COVARIANCE);
  adcsParams.noiseCovariance = PersistentStorage_Get<ADCS_CALC_TYPE>(ADCS_NOISE_COVARIANCE);
  adcsParams.activeLen = activeDuration;

  // load controllers
  const size_t controllerBuffLen = ADCS_MAX_NUM_CONTROLLERS*FLASH_ADCS_CONTROLLER_SLOT_SIZE;
  uint8_t controllerBuff[controllerBuffLen];
  PersistentStorage_Read(FLASH_ADCS_CONTROLLERS, controllerBuff, controllerBuffLen);
  memcpy(adcsParams.controllers, controllerBuff, controllerBuffLen);

  // print parameters for debugging
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
  FOSSASAT_DEBUG_PRINT(F("numControllers="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.numControllers);
  FOSSASAT_DEBUG_PRINT(F("disturbCovariance="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.disturbCovariance, 4);
  FOSSASAT_DEBUG_PRINT(F("noiseCovariance="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.noiseCovariance, 4);

  // wake up IMU
  adcsState.active = true;
  adcsState.currentEpheRow = 0;
  Sensors_IMU_Sleep(false);
  delay(50);

  // get initial IMU data
  Sensors_IMU_Update();
  ADCS_CALC_TYPE mag[ADCS_NUM_AXES];
  Sensors_IMU_CalcMag(imu.mz, -1*imu.mx, -1*imu.my, FLASH_IMU_OFFSET_MAG_X, mag);
  ADCS_CALC_TYPE omega[ADCS_NUM_AXES];
  Sensors_IMU_CalcGyro(imu.gz, imu.gx, -1*imu.gy, FLASH_IMU_OFFSET_GYRO_X, omega);
  adcsState.prevOmegaNorm = ADCS_VectorNorm(omega);

  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(mag, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(omega, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT(F("omegaNorm=\t"));
  FOSSASAT_DEBUG_PRINTLN(adcsState.prevOmegaNorm, 4);

  // Select ephemerides
  ADCS_CALC_TYPE solarEphe[ADCS_NUM_AXES];
  ADCS_CALC_TYPE magEphe[ADCS_NUM_AXES];
  uint32_t row = adcsState.currentEpheRow;
  ADCS_Load_Ephemerides(row, solarEphe, magEphe);
  adcsState.currentEpheRow++;

  // set initial state
  ADCS_CALC_TYPE rotationMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES];
  ADCS_CALC_TYPE newAnglesVector[ADCS_NUM_AXES];
  adcsState.prevControlVector[0] = 0;
  adcsState.prevControlVector[1] = 0;
  adcsState.prevControlVector[2] = 0;

  ADS_Eclipse_Hybrid(mag, magEphe, rotationMatrix);
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
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      adcsState.kalmanMatrixP[i][j] = 0;
    }
  }

  // configure timers
  ADCS_Setup_Timers(ADCS_ActiveControl_Update);
}

void ADCS_ActiveControl_Update() {
  // check all conditions
  if(!ADCS_Check()) {
    return;
  }

  // Select ephemerides
  ADCS_CALC_TYPE solarEphe[ADCS_NUM_AXES];
  ADCS_CALC_TYPE magEphe[ADCS_NUM_AXES];
  uint32_t row = adcsState.currentEpheRow;
  uint8_t controller = ADCS_Load_Ephemerides(row, solarEphe, magEphe);
  adcsState.currentEpheRow++;

  // get IMU data
  Sensors_IMU_Update();
  ADCS_CALC_TYPE omega[ADCS_NUM_AXES];
  Sensors_IMU_CalcGyro(imu.gz, imu.gx, -1*imu.gy, FLASH_IMU_OFFSET_GYRO_X, omega);
  ADCS_CALC_TYPE magData[ADCS_NUM_AXES];
  Sensors_IMU_CalcMag(imu.mz, -1*imu.mx, -1*imu.my, FLASH_IMU_OFFSET_MAG_X, magData);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(omega, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(magData, ADCS_NUM_AXES);

  ADCS_CALC_TYPE stateVars[ADCS_STATE_DIM];
  ADCS_CALC_TYPE intensity[ADCS_NUM_AXES];
  ADCS_CALC_TYPE prevStateVars[ADCS_STATE_DIM];
  ADCS_CALC_TYPE controlVector[ADCS_NUM_AXES];
  ADCS_CALC_TYPE kalmanMatrixP[ADCS_STATE_DIM][ADCS_STATE_DIM];
  ADCS_CALC_TYPE newAnglesVector[ADCS_NUM_AXES];

  // copy all the volatile previous states
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    controlVector[i] = adcsState.prevControlVector[i];
  }
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    prevStateVars[i] = adcsState.prevStateVars[i];
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      kalmanMatrixP[i][j] = adcsState.kalmanMatrixP[i][j];
    }
  }

  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(controlVector, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(prevStateVars, ADCS_STATE_DIM);
  FOSSASAT_DEBUG_PRINT_ADCS_MATRIX(kalmanMatrixP, ADCS_STATE_DIM, ADCS_STATE_DIM);

  // Main structure
  ADS_Main(omega, magData, prevStateVars, controlVector, kalmanMatrixP, solarEphe, magEphe, stateVars, newAnglesVector);

  // Control structure
  ADCS_CALC_TYPE eulerNorm = ADCS_VectorNorm(newAnglesVector);
  ADCS_CALC_TYPE omegaNorm = ADCS_VectorNorm(omega);
  FOSSASAT_DEBUG_PRINT(F("eulerNorm=\t"));
  FOSSASAT_DEBUG_PRINTLN(eulerNorm, 4);
  FOSSASAT_DEBUG_PRINT(F("omegaNorm=\t"));
  FOSSASAT_DEBUG_PRINTLN(omegaNorm, 4);
  bool eulerToleranceReached = (abs(eulerNorm - adcsState.prevEulerNorm) >= adcsParams.activeEulerTol);
  bool omegaToleranceReached = (abs(omegaNorm - adcsState.prevOmegaNorm) <= adcsParams.activeOmegaTol);
  if ((adcsParams.control.bits.overrideEulerTol || eulerToleranceReached) && (adcsParams.control.bits.overrideOmegaTol || omegaToleranceReached)) {
    // Choose controller from ADCS parameters
    float controllerMatrix[ADCS_NUM_AXES][ADCS_STATE_DIM];
    memcpy(controllerMatrix, adcsParams.controllers + (controller * FLASH_ADCS_CONTROLLER_SLOT_SIZE), FLASH_ADCS_CONTROLLER_SLOT_SIZE);

    // Active controlling
    ACS_OnboardControl(stateVars, magData, controllerMatrix, adcsParams.coilChar, intensity, controlVector);

    // calculate and update pulse length
    ADCS_Set_Pulse_Lengths(intensity);

  } else {
    ADCS_Finish(ADCS_RES_DONE_TOL_REACHED);
    FOSSASAT_DEBUG_PRINTLN(F("Active control done (tolerance reached)"));
    return;
  }

  // save values for the next iteration
  adcsState.prevEulerNorm = eulerNorm;
  adcsState.prevOmegaNorm = omegaNorm;
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    adcsState.prevControlVector[i] = controlVector[i];
    adcsState.prevIntensity[i] = intensity[i];
  }
  for(uint8_t i = 0; i < ADCS_STATE_DIM; i++) {
    adcsState.prevStateVars[i] = stateVars[i];
    for(uint8_t j = 0; j < ADCS_STATE_DIM; j++) {
      adcsState.kalmanMatrixP[i][j] = kalmanMatrixP[i][j];
    }
  }
}

uint8_t ADCS_Load_Ephemerides(const uint32_t row, ADCS_CALC_TYPE solarEph[ADCS_NUM_AXES], ADCS_CALC_TYPE magEph[ADCS_NUM_AXES]) {
  // get addresses
  const uint32_t rowsPerPage = (FLASH_EXT_PAGE_SIZE / FLASH_ADCS_EPHEMERIDES_SLOT_SIZE);
  uint32_t pageAddr = (row / rowsPerPage) + FLASH_ADCS_EPHEMERIDES_START;

  // ephemerides are stored in 128-byte chunks, 5 rows per chunk, followed by 3 free bytes
  uint32_t epheAddr = (row % rowsPerPage) * FLASH_ADCS_EPHEMERIDES_SLOT_SIZE;
  if((row % rowsPerPage) >= (rowsPerPage / 2)) {
    // second chunk, add 3 bytes
    epheAddr += 3;
  }

  // read the requested row
  const uint8_t epheLineLen = 6*sizeof(float) + sizeof(uint8_t);
  uint8_t epheLineBuff[epheLineLen];
  PersistentStorage_Read(pageAddr + epheAddr, epheLineBuff, epheLineLen);

  // parse the data
  float val;
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    memcpy(&val, epheLineBuff + i*sizeof(float), sizeof(float));
    solarEph[i] = (ADCS_CALC_TYPE)val;
    memcpy(&val, epheLineBuff + (i + ADCS_NUM_AXES)*sizeof(float), sizeof(float));
    magEph[i] = (ADCS_CALC_TYPE)val;
  }

  FOSSASAT_DEBUG_PRINT(F("row = "));
  FOSSASAT_DEBUG_PRINTLN(row);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(solarEph, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(magEph, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT(F("controller = "));
  FOSSASAT_DEBUG_PRINTLN(epheLineBuff[epheLineLen - 1]);

  // get the controller type
  return(epheLineBuff[epheLineLen - 1]);
}

bool ADCS_Check() {
  // check detumbling length
  if(millis() - adcsState.start > adcsParams.detumbleLen) {
    // time limit reached
    ADCS_Finish(ADCS_RES_DONE_TIME_LIMIT);
    FOSSASAT_DEBUG_PRINTLN(F("ADCS done (time limit reached)"));
    return(false);
  }

  // check battery voltage or LP mode
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
    ADCS_Finish(ADCS_RES_STOPPED_LOW_POWER);
    FOSSASAT_DEBUG_PRINTLN(F("ADCS stopped (battery too low)"));
    return(false);
  }
  #endif

  // check Hbridge faults
  uint8_t fault = bridgeX.getFault();
  if((!adcsParams.control.bits.overrideFaultX) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish(((fault & FAULT) << 3 ) | ADCS_RES_STOPPED_FAULT_X);
    FOSSASAT_DEBUG_PRINTLN(F("ADCS stopped (X axis fault)"));
    return(false);
  }

  fault = bridgeY.getFault();
  if((!adcsParams.control.bits.overrideFaultY) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish(((fault & FAULT) << 3 ) | ADCS_RES_STOPPED_FAULT_Y);
    FOSSASAT_DEBUG_PRINTLN(F("ADCS stopped (Y axis fault)"));
    return(false);
  }

  fault = bridgeZ.getFault();
  if((!adcsParams.control.bits.overrideFaultZ) && (fault != 0) && (fault & FAULT)) {
    ADCS_Finish(((fault & FAULT) << 3 ) | ADCS_RES_STOPPED_FAULT_Z);
    FOSSASAT_DEBUG_PRINTLN(F("ADCS stopped (Z axis fault)"));
    return(false);
  }

  // check abort command
  if(abortExecution) {
    abortExecution = false;
    ADCS_Finish(ADCS_RES_STOPPED_ABORT);
    FOSSASAT_DEBUG_PRINTLN(F("ADCS stopped (aborted)"));
    return(false);
  }

  return(true);
}

void ADCS_Common_Init(const uint8_t controlFlags, const ADCS_CALC_TYPE orbitalInclination, const ADCS_CALC_TYPE meanOrbitalMotion) {
  // save control flags
  adcsParams.control.val = controlFlags;

  // set science mode flag
  scienceModeActive = true;

  // reset previous ADCS result
  PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_LAST_ADCS_RESULT, ADCS_RUNNING);

  // cache common parameters
  adcsParams.maxPulseInt = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_MAX_INTENSITY);     // Maximum applicable intensity
  adcsParams.maxPulseLen = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_MAX_LENGTH);    // Maximum pulse time available
  adcsParams.timeStep = PersistentStorage_Get<uint32_t>(FLASH_ADCS_TIME_STEP);           // Time step between to calculation instants
  adcsParams.minInertialMoment = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_MIN_INERTIAL_MOMENT); // Minimum inertial moment
  adcsParams.pulseAmplitude = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_PULSE_AMPLITUDE);  // Amplitude of pulse for ACS_IntensitiesRectifier
  adcsParams.calcTol = PersistentStorage_Get<ADCS_CALC_TYPE>(FLASH_ADCS_CALCULATION_TOLERANCE);
  adcsParams.bridgeTimerUpdatePeriod = PersistentStorage_Get<uint32_t>(FLASH_ADCS_BRIDGE_TIMER_UPDATE_PERIOD);
  adcsParams.bridgeOutputHigh = PersistentStorage_Get<int8_t>(FLASH_ADCS_BRIDGE_OUTPUT_HIGH);
  adcsParams.bridgeOutputLow = PersistentStorage_Get<int8_t>(FLASH_ADCS_BRIDGE_OUTPUT_LOW);
  adcsParams.orbInclination = orbitalInclination;
  adcsParams.meanOrbitalMotion = meanOrbitalMotion;

  // Coil magnetic characteristics -shall be introduced already inversed-.
  const uint8_t coilCharLen = ADCS_NUM_AXES*ADCS_NUM_AXES*sizeof(float);
  uint8_t coilCharBuff[coilCharLen];
  PersistentStorage_Read(FLASH_ADCS_COIL_CHAR_MATRIX, coilCharBuff, coilCharLen);
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      float val = 0;
      memcpy(&val, coilCharBuff + (i*ADCS_NUM_AXES*sizeof(val) + j*sizeof(val)), sizeof(val));
      adcsParams.coilChar[i][j] = val;
    }
  }

  // inverted inertia tensor matrix for Kalman filter
  const uint8_t inertiaTensorLen = ADCS_NUM_AXES*ADCS_NUM_AXES*sizeof(float);
  uint8_t inertiaTensorBuff[inertiaTensorLen];
  PersistentStorage_Read(FLASH_ADCS_COIL_CHAR_MATRIX, inertiaTensorBuff, inertiaTensorLen);
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      float val = 0;
      memcpy(&val, inertiaTensorBuff + (i*ADCS_NUM_AXES*sizeof(val) + j*sizeof(val)), sizeof(val));
      adcsParams.inertiaTensor[i][j] = val;
    }
  }

  // print everything for debugging
  FOSSASAT_DEBUG_PRINT(F("maxPulseInt="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.maxPulseInt, 4);
  FOSSASAT_DEBUG_PRINT(F("maxPulseLen="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.maxPulseLen, 4);
  FOSSASAT_DEBUG_PRINT(F("timeStep="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.timeStep);
  FOSSASAT_DEBUG_PRINT(F("minInertialMoment="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.minInertialMoment, 4);
  FOSSASAT_DEBUG_PRINT(F("pulseAmplitude="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.pulseAmplitude, 4);
  FOSSASAT_DEBUG_PRINT(F("calcTol="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.calcTol, 4);
  FOSSASAT_DEBUG_PRINT(F("bridgeTimerUpdatePeriod="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.bridgeTimerUpdatePeriod);
  FOSSASAT_DEBUG_PRINT(F("bridgeOutputHigh="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.bridgeOutputHigh);
  FOSSASAT_DEBUG_PRINT(F("bridgeOutputLow="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.bridgeOutputLow);
  FOSSASAT_DEBUG_PRINT(F("orbitalInclination="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.orbInclination);
  FOSSASAT_DEBUG_PRINT(F("meanOrbitalMotion="));
  FOSSASAT_DEBUG_PRINTLN(adcsParams.meanOrbitalMotion);
  FOSSASAT_DEBUG_PRINT_ADCS_MATRIX(adcsParams.coilChar, ADCS_NUM_AXES, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT_ADCS_MATRIX(adcsParams.inertiaTensor, ADCS_NUM_AXES, ADCS_NUM_AXES);
}

void ADCS_Finish(uint8_t result) {
  PersistentStorage_SystemInfo_Set<uint8_t>(FLASH_LAST_ADCS_RESULT, result);
  adcsState.active = false;
  bridgeX.stop();
  bridgeY.stop();
  bridgeZ.stop();
  AdcsTimer->pause();
  AdcsTimer->detachInterrupt();
  HbridgeTimer->pause();
  scienceModeActive = false;
}

void ADCS_Set_Pulse_Lengths(ADCS_CALC_TYPE intensity[ADCS_NUM_AXES]) {
  ADCS_CALC_TYPE intensityNorm = ADCS_VectorNorm(intensity);
  FOSSASAT_DEBUG_PRINT(F("intensityNorm=\t"));
  FOSSASAT_DEBUG_PRINTLN(intensityNorm, 4);

  // Calculate the intensities
  // pulseLength vector follows solar panel reference frame, not ADCS frame!
  // solar X+ = ADCS X+
  // solar Y+ = ADCS Z+
  // solar Z+ = ADCS Y+
  ADCS_CALC_TYPE pulseLength[ADCS_NUM_AXES];
  pulseLength[0] = ACS_IntensitiesRectifier(adcsState.prevIntensity[0], intensity[0], adcsParams.timeStep, adcsParams.pulseAmplitude);
  pulseLength[1] = ACS_IntensitiesRectifier(adcsState.prevIntensity[2], intensity[2], adcsParams.timeStep, adcsParams.pulseAmplitude);
  pulseLength[2] = ACS_IntensitiesRectifier(adcsState.prevIntensity[1], intensity[1], adcsParams.timeStep, adcsParams.pulseAmplitude);

  // set directions
  if(pulseLength[0] > 0) {
    adcsState.bridgeStateX.forward = true;
  } else {
    adcsState.bridgeStateX.forward = false;
  }

  if(pulseLength[1] > 0) {
    adcsState.bridgeStateY.forward = true;
  } else {
    adcsState.bridgeStateY.forward = false;
  }

  if(pulseLength[2] > 0) {
    adcsState.bridgeStateZ.forward = true;
  } else {
    adcsState.bridgeStateZ.forward = false;
  }

  // check maximum applicable intensity
  if(intensityNorm > adcsParams.maxPulseInt) {
    pulseLength[0] = adcsParams.maxPulseLen;
    pulseLength[1] = adcsParams.maxPulseLen;
    pulseLength[2] = adcsParams.maxPulseLen;
  }

  // update H-bridges
  adcsState.bridgeStateX.pulseLen = (uint32_t)(abs(pulseLength[0]));
  adcsState.bridgeStateY.pulseLen = (uint32_t)(abs(pulseLength[1]));
  adcsState.bridgeStateZ.pulseLen = (uint32_t)(abs(pulseLength[2]));

  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(intensity, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINT_ADCS_VECTOR(pulseLength, ADCS_NUM_AXES);
  FOSSASAT_DEBUG_PRINTLN();
}

void ADCS_Setup_Timers(void (*func)(void)) {
  // configure H bridge timer
  HbridgeTimer->setOverflow(adcsParams.bridgeTimerUpdatePeriod * (uint32_t)1000, MICROSEC_FORMAT);
  HbridgeTimer->attachInterrupt(ADCS_Update_Bridges);
  adcsState.bridgeStateX.outputHigh = false;
  adcsState.bridgeStateY.outputHigh = false;
  adcsState.bridgeStateZ.outputHigh = false;

  // configure ADCS timer
  AdcsTimer->setOverflow(adcsParams.timeStep * (uint32_t)1000, MICROSEC_FORMAT);
  AdcsTimer->attachInterrupt(func);

  // start timers
  adcsState.start = millis();
  adcsState.bridgeStateX.lastUpdate = adcsState.start;
  adcsState.bridgeStateY.lastUpdate = adcsState.start;
  adcsState.bridgeStateZ.lastUpdate = adcsState.start;
  HbridgeTimer->resume();
  AdcsTimer->resume();
}

void ADCS_Update_Bridges() {
  uint32_t currTime = millis();

  // bridge X update
  if(currTime - adcsState.bridgeStateX.lastUpdate >= adcsState.bridgeStateX.pulseLen) {
    // update timestamp
    adcsState.bridgeStateX.lastUpdate = currTime;

    // set value and direction
    if(adcsState.bridgeStateX.outputHigh) {
      int8_t val = adcsParams.bridgeOutputHigh;
      if(!adcsState.bridgeStateX.forward) {
        val = adcsParams.bridgeOutputLow;
      }
      bridgeX.drive(val);
    } else {
      bridgeX.drive(0);
    }

    // flip pulse bit
    adcsState.bridgeStateX.outputHigh = !adcsState.bridgeStateX.outputHigh;
  }

  // bridge Y update
  if(currTime - adcsState.bridgeStateY.lastUpdate >= adcsState.bridgeStateY.pulseLen) {
    // update timestamp
    adcsState.bridgeStateY.lastUpdate = currTime;

    // set value and direction
    if(adcsState.bridgeStateY.outputHigh) {
      int8_t val = adcsParams.bridgeOutputHigh;
      if(!adcsState.bridgeStateY.forward) {
        val = adcsParams.bridgeOutputLow;
      }
      bridgeY.drive(val);
    } else {
      bridgeY.drive(0);
    }

    // flip pulse bit
    adcsState.bridgeStateY.outputHigh = !adcsState.bridgeStateY.outputHigh;
  }

  // bridge Z update
  if(currTime - adcsState.bridgeStateZ.lastUpdate >= adcsState.bridgeStateZ.pulseLen) {
    // update timestamp
    adcsState.bridgeStateZ.lastUpdate = currTime;

    // set value and direction
    if(adcsState.bridgeStateZ.outputHigh) {
      int8_t val = adcsParams.bridgeOutputHigh;
      if(!adcsState.bridgeStateZ.forward) {
        val = adcsParams.bridgeOutputLow;
      }
      bridgeZ.drive(val);
    } else {
      bridgeZ.drive(0);
    }

    // flip pulse bit
    adcsState.bridgeStateZ.outputHigh = !adcsState.bridgeStateZ.outputHigh;
  }
}
