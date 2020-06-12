/*
    FossaSat2.h
*/

#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include "Debug.h"

/*
    Configuration.h
*/

// I2C
#define I2C1_SDA                                        PA10
#define I2C1_SCL                                        PA9
#define I2C2_SDA                                        PB11
#define I2C2_SCL                                        PB10
#define CAMERA_POWER_FET                                PB5

// Light Sensors
#define LIGHT_SENSOR_GAIN                               VEML7700_GAIN_1_8
#define LIGHT_SENSOR_INTEGRATION_TIME                   VEML7700_IT_25MS
#define LIGHT_SENSOR_Y_PANEL_BUS                        Wire
#define LIGHT_SENSOR_TOP_PANEL_BUS                      Wire2

/*
    Configuration.cpp
*/

TwoWire Wire2;
Adafruit_VEML7700 lightSensorPanelY = Adafruit_VEML7700();
Adafruit_VEML7700 lightSensorTop = Adafruit_VEML7700();

/*
    Sensors.cpp
*/

void Sensors_Setup_Light(Adafruit_VEML7700& sensor, TwoWire& wire) {
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
  } else {
    FOSSASAT_DEBUG_PRINTLN(F("success!"));
    return;
  }

  // set default properties
  sensor.setGain(LIGHT_SENSOR_GAIN);
  sensor.setIntegrationTime(LIGHT_SENSOR_INTEGRATION_TIME);
}

void setup() {
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  while(!FOSSASAT_DEBUG_PORT);
  FOSSASAT_DEBUG_PORT.println();

  pinMode(CAMERA_POWER_FET, OUTPUT);
  digitalWrite(CAMERA_POWER_FET, HIGH);

  Wire.setSDA(I2C1_SDA);
  Wire.setSCL(I2C1_SCL);
  Wire.begin();

  Wire2.setSDA(I2C2_SDA);
  Wire2.setSCL(I2C2_SCL);
  Wire2.begin();

  Sensors_Setup_Light(lightSensorPanelY, LIGHT_SENSOR_Y_PANEL_BUS);
  Sensors_Setup_Light(lightSensorTop, LIGHT_SENSOR_TOP_PANEL_BUS);

  FOSSASAT_DEBUG_PRINTLN();
  FOSSASAT_DEBUG_PORT.println(F("Y panel\tTop panel\tE [lx]"));
  FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
}

void loop() {
  FOSSASAT_DEBUG_PORT.print(lightSensorPanelY.readLux());
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.println(lightSensorTop.readLux());

  delay(100);
}
