#include "Sensors.h"

void Sensors_Temperature_Setup(wireSensor_t& sensor, uint8_t res) {
  // set resolution
  sensor.bus.beginTransmission(sensor.addr);
  sensor.bus.write(TMP_100_REG_CONFIG);
  sensor.bus.write(res);
  sensor.bus.endTransmission();

  // set mode back to temperature reading
  sensor.bus.beginTransmission(sensor.addr);
  sensor.bus.write(TMP_100_REG_TEMPERATURE);
  sensor.bus.endTransmission();
}

float Sensors_Temperature_Read(wireSensor_t& sensor) {
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
  return(imu.begin(IMU_ACCEL_GYRO_ADDRESS, IMU_MAG_ADDRESS, IMU_BUS));
}

void Sensors_IMU_Update() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }

  if (imu.accelAvailable()) {
    imu.readAccel();
  }

  if (imu.magAvailable()) {
    imu.readMag();
  }
}

bool Sensors_Current_Setup(Adafruit_INA260& sensor, TwoWire& wire, uint8_t addr) {
  FOSSASAT_DEBUG_PRINT(F("Current sensor 0b"));
  FOSSASAT_DEBUG_PRINT(addr, BIN);
  FOSSASAT_DEBUG_PRINT(F(" init ... "));

  // initialize the current sensor
  if (!sensor.begin(addr, &wire)) {
    FOSSASAT_DEBUG_PRINTLN(F("failed!"));
    return(false);
  }
  FOSSASAT_DEBUG_PRINTLN(F("success!"));

  // set sensor to sleep by default
  sensor.setMode(INA260_MODE_SHUTDOWN);
  
  return(true);
}

float Sensors_Current_Read(Adafruit_INA260& sensor) {
  #ifdef OVERRIDE_INA
    if(&sensor == &OVERRIDE_INA) {
      return(0);
    }
  #endif
  
  // wake up the sensor
  sensor.setMode(INA260_MODE_CONTINUOUS);

  // wait for full recovery
  delayMicroseconds(60);
  
  // read the value
  float current = sensor.readCurrent();
  
  // set the sensor back to sleep
  sensor.setMode(INA260_MODE_SHUTDOWN);

  return(current);
}

float Sensors_Current_ReadVoltage(Adafruit_INA260& sensor) {
  #ifdef OVERRIDE_INA
    if(&sensor == &OVERRIDE_INA) {
      return(0);
    }
  #endif
  
  // wake up the sensor
  sensor.setMode(INA260_MODE_CONTINUOUS);

  // wait for full recovery
  delayMicroseconds(60);

  // read the value
  float voltage = sensor.readBusVoltage();
  
  // set the sensor back to sleep
  sensor.setMode(INA260_MODE_SHUTDOWN);

  return(voltage);
}

bool Sensors_Setup_Light(Adafruit_VEML7700& sensor, TwoWire& wire) {
  FOSSASAT_DEBUG_PRINT(F("Light sensor I2C"));
  if (&wire == &Wire) {
    FOSSASAT_DEBUG_PRINT('1');
  } else {
    FOSSASAT_DEBUG_PRINT('2');
  }
  FOSSASAT_DEBUG_PRINT(F(" init ... "));

  // initialize the current sensor
  if (!sensor.begin(&wire)) {
    FOSSASAT_DEBUG_PRINTLN(F("failed!"));
    return(false);
  } else {
    FOSSASAT_DEBUG_PRINTLN(F("success!"));
  }

  // set default properties
  sensor.setGain(LIGHT_SENSOR_GAIN);
  sensor.setIntegrationTime(LIGHT_SENSOR_INTEGRATION_TIME);
  return(true);
}

float Sensors_Read_Light(Adafruit_VEML7700& sensor) {
  return(sensor.readLux());
}
