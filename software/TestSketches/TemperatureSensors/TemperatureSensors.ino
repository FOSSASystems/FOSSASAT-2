/*
    FossaSat2.h
*/

#include <Wire.h>
#include "Debug.h"

/*
    Configuration.h
*/

// I2C
#define I2C1_SDA                                        PA10
#define I2C1_SCL                                        PA9
#define I2C2_SDA                                        PB11
#define I2C2_SCL                                        PB10

// common
#define TMP_100_REG_TEMPERATURE                         0x00
#define TMP_100_REG_CONFIG                              0x01
#define TMP_100_RESOLUTION_9_BITS                       0b00000000  // 0.5 deg. C
#define TMP_100_RESOLUTION_10_BITS                      0b00100000  // 0.25 deg. C
#define TMP_100_RESOLUTION_11_BITS                      0b01000000  // 0.125 deg. C
#define TMP_100_RESOLUTION_12_BITS                      0b01100000  // 0.0625 deg. C (default)
#define TMP_100_LSB_RESOLUTION                          0.0625  // deg. C

// Y panel temperature sensor
#define TEMP_SENSOR_Y_PANEL_BUS                         Wire
#define TEMP_SENSOR_Y_PANEL_ADDRESS                     0b1001000
 // ADD1 low, ADD0 low

// top panel temperature sensor
#define TEMP_SENSOR_TOP_BUS                             Wire2
#define TEMP_SENSOR_TOP_ADDRESS                         0b1001001
 // ADD1 float, ADD0 low

// bottom panel temperature sensor
#define TEMP_SENSOR_BOTTOM_BUS                          Wire2
#define TEMP_SENSOR_BOTTOM_ADDRESS                      0b1001001
 // ADD1 low, ADD0 low

// battery temperature sensor
#define TEMP_SENSOR_BATTERY_BUS                         Wire2
#define TEMP_SENSOR_BATTERY_ADDRESS                     0b1001011
 // ADD1 low, ADD0 float

// second battery temperature sensor
#define TEMP_SENSOR_SEC_BATTERY_BUS                     Wire2
#define TEMP_SENSOR_SEC_BATTERY_ADDRESS                 0b1001010 // ADD1 low, ADD0 high

/*
    Types.h
*/
struct wireSensor_t {
  TwoWire& bus;
  uint8_t addr;
};

/*
    Configuration.cpp
*/

// second I2C instance
TwoWire Wire2;

// temperature sensors
struct wireSensor_t tempSensorPanelY = { .bus = TEMP_SENSOR_Y_PANEL_BUS, .addr = TEMP_SENSOR_Y_PANEL_ADDRESS};
struct wireSensor_t tempSensorTop = { .bus = TEMP_SENSOR_TOP_BUS, .addr = TEMP_SENSOR_TOP_ADDRESS};
struct wireSensor_t tempSensorBottom = { .bus = TEMP_SENSOR_BOTTOM_BUS, .addr = TEMP_SENSOR_BOTTOM_ADDRESS};
struct wireSensor_t tempSensorBattery = { .bus = TEMP_SENSOR_BATTERY_BUS, .addr = TEMP_SENSOR_BATTERY_ADDRESS};
struct wireSensor_t tempSensorSecBattery = { .bus = TEMP_SENSOR_SEC_BATTERY_BUS, .addr = TEMP_SENSOR_SEC_BATTERY_ADDRESS};

/*
    Sensors.cpp
*/

void Sensors_Setup_Temp(wireSensor_t& sensor, uint8_t res) {
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

float Sensors_Read_Temperature(wireSensor_t& sensor) {
  // read data from I2C sensor
  sensor.bus.requestFrom(sensor.addr, (uint8_t)2);
  uint8_t msb = sensor.bus.read();
  uint8_t lsb = sensor.bus.read();

  // convert raw data to temperature
  int16_t tempRaw = ((msb << 8) | lsb) >> 4;
  float temp = tempRaw * TMP_100_LSB_RESOLUTION;
  return (temp);
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

  Sensors_Setup_Temp(tempSensorPanelY, TMP_100_RESOLUTION_12_BITS);
  Sensors_Setup_Temp(tempSensorTop, TMP_100_RESOLUTION_12_BITS);
  Sensors_Setup_Temp(tempSensorBottom, TMP_100_RESOLUTION_12_BITS);
  Sensors_Setup_Temp(tempSensorBattery, TMP_100_RESOLUTION_12_BITS);
  Sensors_Setup_Temp(tempSensorSecBattery, TMP_100_RESOLUTION_12_BITS);

  FOSSASAT_DEBUG_PRINTLN();
  FOSSASAT_DEBUG_PORT.println(F("Y panel\tTop panel\tBottom panel\tBattery\tSecond Battery\tT [deg. C]"));
  FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
}

void loop() {
  FOSSASAT_DEBUG_PORT.print(Sensors_Read_Temperature(tempSensorPanelY));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.print(Sensors_Read_Temperature(tempSensorTop));
  FOSSASAT_DEBUG_PORT.print("\t\t");
  FOSSASAT_DEBUG_PORT.print(Sensors_Read_Temperature(tempSensorBottom));
  FOSSASAT_DEBUG_PORT.print("\t\t");
  FOSSASAT_DEBUG_PORT.print(Sensors_Read_Temperature(tempSensorBattery));
  FOSSASAT_DEBUG_PORT.print('\t');
  FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorSecBattery));

  delay(1000);
}
