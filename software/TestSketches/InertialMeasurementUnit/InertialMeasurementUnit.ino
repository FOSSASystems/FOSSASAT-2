/*
    FossaSat2.h
*/

#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include "Debug.h"

/*
    Configuration.h
*/

// I2C
#define I2C2_SDA                                        PB11
#define I2C2_SCL                                        PB10

// IMU
#define IMU_ACCEL_GYRO_ADDRESS                          0b1101010 // SDO_A/G low
#define IMU_MAG_ADDRESS                                 0b0011100 // SDO_M low

/*
    Configuration.cpp
*/

// second I2C instance
TwoWire Wire2;

// IMU
LSM9DS1 imu;

/*
    Sensors.cpp
*/

void Sensors_Setup_IMU() {
  // set configuration
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.agAddress = IMU_ACCEL_GYRO_ADDRESS;
  imu.settings.device.mAddress = IMU_MAG_ADDRESS;
  imu.settings.device.i2c = &Wire2;

  // initialize IMU
  FOSSASAT_DEBUG_PRINT(F("IMU init ... "));
  if (!imu.begin()) {
    FOSSASAT_DEBUG_PRINTLN(F("failed!"));
    return;
  } else {
    FOSSASAT_DEBUG_PRINTLN(F("success!"));
  }
}

void Sensors_Update_IMU() {
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

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  Wire2.setSDA(I2C2_SDA);
  Wire2.setSCL(I2C2_SCL);
  Wire2.begin();

  Sensors_Setup_IMU();

  FOSSASAT_DEBUG_PRINTLN();
  FOSSASAT_DEBUG_PORT.println(F("X axis\t\t\tY axis\t\t\tZ axis\t\tomega [deg./s]\ta [m/s^2]\tB [gauss]"));
  FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
}

void loop() {
  FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gx));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.ax));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.print(imu.calcMag(imu.mx));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gy));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.ay));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.print(imu.calcMag(imu.my));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gz));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.az));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.println(imu.calcMag(imu.mz));
  
  delay(1000);
}
