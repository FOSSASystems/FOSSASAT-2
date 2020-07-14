#include "Sensors.h"

void Sensors_Temperature_Setup(wireSensor_t& sensor) {
  // set resolution and mode
  sensor.bus.beginTransmission(sensor.addr);
  sensor.bus.write(TMP_100_REG_CONFIG);
  sensor.bus.write(sensor.res | sensor.mode);
  sensor.bus.endTransmission();
}

float Sensors_Temperature_Read(wireSensor_t& sensor) {
  // start one-shot conversion
  sensor.bus.beginTransmission(sensor.addr);
  sensor.bus.write(TMP_100_REG_CONFIG);
  sensor.bus.write(TMP_100_START_ONE_SHOT | sensor.res | sensor.mode);
  sensor.bus.endTransmission();

  // set mode to temperature reading
  sensor.bus.beginTransmission(sensor.addr);
  sensor.bus.write(TMP_100_REG_TEMPERATURE);
  sensor.bus.endTransmission();

  // read data from I2C sensor
  sensor.bus.requestFrom(sensor.addr, (uint8_t)2);
  uint8_t msb = sensor.bus.read();
  uint8_t lsb = sensor.bus.read();

  // convert raw data to temperature
  int16_t tempRaw = ((msb << 8) | lsb) >> 4;
  float temp = tempRaw * TMP_100_LSB_RESOLUTION;

  return (temp);
}

uint16_t Sensors_IMU_Setup() {
  // initialize IMU
  uint16_t state = imu.begin(IMU_ACCEL_GYRO_ADDRESS, IMU_MAG_ADDRESS, IMU_BUS);

  // set to sleep by default
  Sensors_IMU_Sleep(true);

  return(state);
}

void Sensors_IMU_Update() {
  // wake up IMU
  if(!adcsState.active) {
    Sensors_IMU_Sleep(false);

    // wait for everything to wake up
    delay(10);
  }

  if (imu.gyroAvailable()) {
    imu.readGyro();
  }

  if (imu.accelAvailable()) {
    imu.readAccel();
  }

  if (imu.magAvailable()) {
    imu.readMag();
  }

  // put IMU back to sleep
  if(!adcsState.active) {
    Sensors_IMU_Sleep(true);
  }
}

void Sensors_IMU_Sleep(bool sleep) {
  imu.sleepGyro(sleep);
}

void Sensors_IMU_CalcGyro(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, double processed[ADCS_NUM_AXES]) {
  float tmp[ADCS_NUM_AXES];
  Sensors_IMU_CalcGyro(rawX, rawY, rawZ, offsetAddr, tmp);
  processed[0] = (double)tmp[0];
  processed[1] = (double)tmp[1];
  processed[2] = (double)tmp[2];
}

void Sensors_IMU_CalcAccel(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, double processed[ADCS_NUM_AXES]) {
  float tmp[ADCS_NUM_AXES];
  Sensors_IMU_CalcAccel(rawX, rawY, rawZ, offsetAddr, tmp);
  processed[0] = (double)tmp[0];
  processed[1] = (double)tmp[1];
  processed[2] = (double)tmp[2];
}

void Sensors_IMU_CalcMag(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, double processed[ADCS_NUM_AXES]) {
  float tmp[ADCS_NUM_AXES];
  Sensors_IMU_CalcMag(rawX, rawY, rawZ, offsetAddr, tmp);
  processed[0] = (double)tmp[0];
  processed[1] = (double)tmp[1];
  processed[2] = (double)tmp[2];
}

void Sensors_IMU_CalcGyro(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, float processed[ADCS_NUM_AXES]) {
  float offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr);
  processed[0] = (imu.calcGyro(rawX) * IMU_DEG_S_TO_RAD_S) + offset;
  offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr + sizeof(float));
  processed[1] = (imu.calcGyro(rawY) * IMU_DEG_S_TO_RAD_S) + offset;
  offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr + 2*sizeof(float));
  processed[2] = (imu.calcGyro(rawZ) * IMU_DEG_S_TO_RAD_S) + offset;
}

void Sensors_IMU_CalcAccel(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, float processed[ADCS_NUM_AXES]) {
  float offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr);
  processed[0] = (imu.calcAccel(rawX) * IMU_G_TO_MS2) + offset;
  offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr + sizeof(float));
  processed[1] = (imu.calcAccel(rawY) * IMU_G_TO_MS2) + offset;
  offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr + 2*sizeof(float));
  processed[2] = (imu.calcAccel(rawZ) * IMU_G_TO_MS2) + offset;
}

void Sensors_IMU_CalcMag(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, float processed[ADCS_NUM_AXES]) {
  float raw[ADCS_NUM_AXES] = {(float)rawX, (float)rawY, (float)rawZ};

  // load transformation matrix and bias vector
  uint8_t transMatrBuffer[ADCS_NUM_AXES*ADCS_NUM_AXES*sizeof(float)];
  PersistentStorage_Read(FLASH_ADCS_IMU_TRANS_MATRIX, transMatrBuffer, ADCS_NUM_AXES*ADCS_NUM_AXES*sizeof(float));
  uint8_t biasVectorBuffer[ADCS_NUM_AXES*sizeof(float)];
  PersistentStorage_Read(FLASH_ADCS_IMU_BIAS_VECTOR, biasVectorBuffer, ADCS_NUM_AXES*sizeof(float));
  float transMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES];
  float biasVector[ADCS_NUM_AXES];
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    float val = 0;
    memcpy(&val, transMatrBuffer + i*sizeof(float), sizeof(float));
    biasVector[i] = val;
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      memcpy(&val, transMatrBuffer + (i*ADCS_NUM_AXES*sizeof(float) + j*sizeof(float)), sizeof(float));
      transMatrix[i][j] = val;
    }
  }

  // perform transformation
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    raw[i] -= biasVector[i];
  }

  float tmp[ADCS_NUM_AXES] = {0, 0, 0};
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      tmp[i] += transMatrix[i][j] * raw[j];
    }
  }

  // TODO vector normalization here?

  // convert to Tesla and add static offset
  float offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr);
  processed[0] = (imu.calcMag((int16_t)tmp[0]) * IMU_GAUSS_TO_TESLA) + offset;
  offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr + sizeof(float));
  processed[1] = (imu.calcMag((int16_t)tmp[1]) * IMU_GAUSS_TO_TESLA) + offset;
  offset = PersistentStorage_SystemInfo_Get<float>(offsetAddr + 2*sizeof(float));
  processed[2] = (imu.calcMag((int16_t)tmp[2]) * IMU_GAUSS_TO_TESLA) + offset;
}

bool Sensors_Current_Setup(currentSensor_t& sensor) {
  FOSSASAT_DEBUG_PRINT(F("Current sensor 0b"));
  FOSSASAT_DEBUG_PRINT(sensor.addr, BIN);
  FOSSASAT_DEBUG_PRINT(F(" init ... "));

  // initialize the current sensor
  if (!sensor.driver->begin(sensor.addr, &(sensor.bus))) {
    FOSSASAT_DEBUG_PRINTLN(F("failed!"));
    sensor.available = false;
    return(false);
  }
  FOSSASAT_DEBUG_PRINTLN(F("success!"));

  // set sensor to sleep by default
  sensor.driver->setMode(INA260_MODE_SHUTDOWN);
  sensor.available = true;

  return(true);
}

float Sensors_Current_Read(currentSensor_t& sensor) {
  if(!(sensor.available)) {
    return(0);
  }

  // wake up the sensor
  sensor.driver->setMode(INA260_MODE_CONTINUOUS);

  // wait for full recovery
  delayMicroseconds(60);

  // read the value
  float current = sensor.driver->readCurrent();

  // set the sensor back to sleep
  sensor.driver->setMode(INA260_MODE_SHUTDOWN);

  return(current);
}

float Sensors_Current_ReadVoltage(currentSensor_t& sensor) {
  if(!(sensor.available)) {
    return(0);
  }

  // wake up the sensor
  sensor.driver->setMode(INA260_MODE_CONTINUOUS);

  // wait for full recovery
  delayMicroseconds(60);

  // read the value
  float voltage = sensor.driver->readBusVoltage();

  // set the sensor back to sleep
  sensor.driver->setMode(INA260_MODE_SHUTDOWN);

  return(voltage);
}

float Sensors_Current_ReadPower(currentSensor_t& sensor) {
  if(!(sensor.available)) {
    return(0);
  }

  // wake up the sensor
  sensor.driver->setMode(INA260_MODE_CONTINUOUS);

  // wait for full recovery
  delayMicroseconds(60);

  // read the value and convert to watts
  float power = sensor.driver->readPower() / 1000.0;

  // set the sensor back to sleep
  sensor.driver->setMode(INA260_MODE_SHUTDOWN);

  return(power);
}

bool Sensors_Setup_Light(lightSensor_t& sensor) {
  FOSSASAT_DEBUG_PRINT(F("Light sensor I2C"));
  if (&(sensor.bus) == &Wire) {
    FOSSASAT_DEBUG_PRINT('1');
  } else {
    FOSSASAT_DEBUG_PRINT('2');
  }
  FOSSASAT_DEBUG_PRINT(F(" init ... "));

  // initialize the current sensor
  if (!sensor.driver->begin(&(sensor.bus))) {
    FOSSASAT_DEBUG_PRINTLN(F("failed!"));
    sensor.available = false;
    return(false);
  } else {
    FOSSASAT_DEBUG_PRINTLN(F("success!"));
    sensor.available = true;
  }

  // set default properties
  sensor.driver->setGain(LIGHT_SENSOR_GAIN);
  sensor.driver->setIntegrationTime(LIGHT_SENSOR_INTEGRATION_TIME);
  sensor.driver->enable(false);
  return(true);
}

float Sensors_Read_Light(lightSensor_t& sensor) {
  if(!sensor.available) {
    return(0);
  }

  // wake up sensor
  sensor.driver->enable(true);

  // wait for new data
  delay(30);
  float lux = sensor.driver->readLux();

  // shut down again
  sensor.driver->enable(false);
  return(lux);
}
