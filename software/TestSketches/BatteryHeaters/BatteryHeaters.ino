//Battery Heating Mechanism
#include <Wire.h>


#define BATTERY_RESISTOR_1             PA8


//I2C
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

// battery temperature sensor
#define TEMP_SENSOR_BATTERY_BUS                         Wire2
#define TEMP_SENSOR_BATTERY_ADDRESS                     0b1001001 // ADD1 low, ADD0 float
 
// second battery temperature sensor
#define TEMP_SENSOR_SEC_BATTERY_BUS                     Wire2
#define TEMP_SENSOR_SEC_BATTERY_ADDRESS                 0b1001011 // ADD1 low, ADD0 high

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

  Serial.begin(9600);
  Serial.println("Battery Heater");

  Wire.setSDA(I2C1_SDA);
  Wire.setSCL(I2C1_SCL);
  Wire.begin();

  Wire2.setSDA(I2C2_SDA);
  Wire2.setSCL(I2C2_SCL);
  Wire2.begin();
  
  Sensors_Setup_Temp(tempSensorBattery, TMP_100_RESOLUTION_12_BITS);
  Sensors_Setup_Temp(tempSensorSecBattery, TMP_100_RESOLUTION_12_BITS);

  
  pinMode(BATTERY_RESISTOR_1, OUTPUT);
}


void loop() {

Serial.print("Battery Heater ON");
Serial.println("");
digitalWrite(BATTERY_RESISTOR_1, HIGH);

delay(5000);          

Serial.print(Sensors_Read_Temperature(tempSensorSecBattery));
Serial.print(" Cº");
Serial.println("");
Serial.print(Sensors_Read_Temperature(tempSensorBattery));
Serial.print(" Cº");
Serial.println("");
Serial.print("Battery Heater OFF");
Serial.println("");
digitalWrite(BATTERY_RESISTOR_1, LOW);
 delay(5000);    

}
