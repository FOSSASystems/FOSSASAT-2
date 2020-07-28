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
#define IMU_BUS                                         Wire2
#define IMU_ACCEL_GYRO_ADDRESS                          0b1101011 // SDO_A/G pulled high internally
#define IMU_MAG_ADDRESS                                 0b0011110 // SDO_M pulled high internally

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

uint16_t Sensors_Setup_IMU() {
  // initialize IMU
  uint16_t id = imu.begin(IMU_ACCEL_GYRO_ADDRESS, IMU_MAG_ADDRESS, IMU_BUS);
  imu.setMagScale(16);
  return(id);
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

  // initialize IMU
  FOSSASAT_DEBUG_PORT.print(F("IMU init (expected 0x683D):\t0x"));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_IMU(), HEX);

  FOSSASAT_DEBUG_PRINTLN();
  FOSSASAT_DEBUG_PORT.println(F("Device\t\tomega [deg./s]\ta [m/s^2]\tB [gauss]"));
  FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
}

uint32_t lastPrint = 0;
void loop() {
  Sensors_Update_IMU();
  
  if(millis() - lastPrint > 1000) {
    FOSSASAT_DEBUG_PORT.print(F("X axis\t\t"));
    FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gx));
    FOSSASAT_DEBUG_PORT.print(F("\t\t"));
    FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.ax));
    FOSSASAT_DEBUG_PORT.print(F("\t\t"));
    FOSSASAT_DEBUG_PORT.println(imu.calcMag(imu.mx));
    FOSSASAT_DEBUG_PORT.print(F("Y axis\t\t"));
    FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gy));
    FOSSASAT_DEBUG_PORT.print(F("\t\t"));
    FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.ay));
    FOSSASAT_DEBUG_PORT.print(F("\t\t"));
    FOSSASAT_DEBUG_PORT.println(imu.calcMag(imu.my));
    FOSSASAT_DEBUG_PORT.print(F("Z axis\t\t"));
    FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gz));
    FOSSASAT_DEBUG_PORT.print(F("\t\t"));
    FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.az));
    FOSSASAT_DEBUG_PORT.print(F("\t\t"));
    FOSSASAT_DEBUG_PORT.println(imu.calcMag(imu.mz));
    FOSSASAT_DEBUG_PORT.println();
    lastPrint = millis();
  }
}
