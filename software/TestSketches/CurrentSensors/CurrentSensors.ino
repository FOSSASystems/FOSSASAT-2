/*
    FossaSat2.h
*/

#include <Wire.h>
#include <Adafruit_INA260.h>
#include "Debug.h"

/*
    Configuration.h
*/

// I2C
#define I2C1_SDA                                        PA10
#define I2C1_SCL                                        PA9
#define I2C2_SDA                                        PB11
#define I2C2_SCL                                        PB10

// X axis solar cells
#define CURR_SENSOR_X_A_BUS                             Wire
#define CURR_SENSOR_X_A_ADDRESS                         0b1000001 // A1 low, A0 high
#define CURR_SENSOR_X_B_BUS                             Wire
#define CURR_SENSOR_X_B_ADDRESS                         0b1000000 // A1 low, A0 SDA

// Z axis solar cells
#define CURR_SENSOR_Z_A_BUS                             Wire
#define CURR_SENSOR_Z_A_ADDRESS                         0b1001100 // A1 SCL, A0 low
#define CURR_SENSOR_Z_B_BUS                             Wire
#define CURR_SENSOR_Z_B_ADDRESS                         0b1000100 // A1 high, A0 low

// Y axis solar cells
#define CURR_SENSOR_Y_BUS                               Wire2
#define CURR_SENSOR_Y_ADDRESS                           0b1000000 // A1 low, A0 low

// MPPT output current
#define CURR_SENSOR_MPPT_OUTPUT_BUS                     Wire2
#define CURR_SENSOR_MPPT_OUTPUT_ADDRESS                 0b1000100 // A1 high, A0 low

/*
    Configuration.cpp
*/

// second I2C instance
TwoWire Wire2;

// current sensors
Adafruit_INA260 currSensorXA = Adafruit_INA260();
Adafruit_INA260 currSensorXB = Adafruit_INA260();
Adafruit_INA260 currSensorZA = Adafruit_INA260();
Adafruit_INA260 currSensorZB = Adafruit_INA260();
Adafruit_INA260 currSensorY = Adafruit_INA260();
Adafruit_INA260 currSensorMPPT = Adafruit_INA260();

/*
    Sensors.cpp
*/

void Sensors_Setup_Current(Adafruit_INA260& sensor, TwoWire& wire, uint8_t addr) {
  FOSSASAT_DEBUG_PRINT(F("Current sensor 0b"));
  FOSSASAT_DEBUG_PRINT(addr, BIN);
  FOSSASAT_DEBUG_PRINT(F(" init ... "));

  // initialize the current sensor
  if (!sensor.begin(addr, &wire)) {
    FOSSASAT_DEBUG_PRINTLN(F("failed!"));
    return;
  } else {
    FOSSASAT_DEBUG_PRINTLN(F("success!"));
  }
}

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  Wire.setSDA(I2C1_SDA);
  Wire.setSCL(I2C1_SCL);
  Wire.begin();

  Wire2.setSDA(I2C2_SDA);
  Wire2.setSCL(I2C2_SCL);
  Wire2.begin();

  Sensors_Setup_Current(currSensorXA, CURR_SENSOR_X_A_BUS, CURR_SENSOR_X_A_ADDRESS);
  Sensors_Setup_Current(currSensorXB, CURR_SENSOR_X_B_BUS, CURR_SENSOR_X_B_ADDRESS);
  Sensors_Setup_Current(currSensorZA, CURR_SENSOR_Z_A_BUS, CURR_SENSOR_Z_A_ADDRESS);
  Sensors_Setup_Current(currSensorZB, CURR_SENSOR_Z_B_BUS, CURR_SENSOR_Z_B_ADDRESS);
  Sensors_Setup_Current(currSensorY, CURR_SENSOR_Y_BUS, CURR_SENSOR_Y_ADDRESS);
  Sensors_Setup_Current(currSensorMPPT, CURR_SENSOR_MPPT_OUTPUT_BUS, CURR_SENSOR_MPPT_OUTPUT_ADDRESS);
  
  FOSSASAT_DEBUG_PORT.println(F("Device\t\tI [mA]\t\tV [mV]"));
  FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
}

void loop() {
  FOSSASAT_DEBUG_PORT.print(F("X panel A\t"));
  FOSSASAT_DEBUG_PORT.print(currSensorXA.readCurrent());
  FOSSASAT_DEBUG_PORT.print(F("\t\t"));
  FOSSASAT_DEBUG_PORT.println(currSensorXA.readBusVoltage());
  FOSSASAT_DEBUG_PORT.print(F("X panel B\t"));
  FOSSASAT_DEBUG_PORT.print(currSensorXB.readCurrent());
  FOSSASAT_DEBUG_PORT.print(F("\t\t"));
  FOSSASAT_DEBUG_PORT.println(currSensorXB.readBusVoltage());

  FOSSASAT_DEBUG_PORT.print(F("Z panel A\t"));
  FOSSASAT_DEBUG_PORT.print(currSensorZA.readCurrent());
  FOSSASAT_DEBUG_PORT.print(F("\t\t"));
  FOSSASAT_DEBUG_PORT.println(currSensorZA.readBusVoltage());
  FOSSASAT_DEBUG_PORT.print(F("Z panel B\t"));
  FOSSASAT_DEBUG_PORT.print(currSensorZB.readCurrent());
  FOSSASAT_DEBUG_PORT.print(F("\t\t"));
  FOSSASAT_DEBUG_PORT.println(currSensorZB.readBusVoltage());

  FOSSASAT_DEBUG_PORT.print(F("Y panel \t"));
  FOSSASAT_DEBUG_PORT.print(currSensorY.readCurrent());
  FOSSASAT_DEBUG_PORT.print(F("\t\t"));
  FOSSASAT_DEBUG_PORT.println(currSensorY.readBusVoltage());

  FOSSASAT_DEBUG_PORT.print(F("MPPT output\t"));
  FOSSASAT_DEBUG_PORT.print(currSensorMPPT.readCurrent());
  FOSSASAT_DEBUG_PORT.print(F("\t\t"));
  FOSSASAT_DEBUG_PORT.println(currSensorMPPT.readBusVoltage());
  FOSSASAT_DEBUG_PORT.println();

  delay(1000);
}
