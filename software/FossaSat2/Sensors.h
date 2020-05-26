#ifndef _FOSSASAT_SENSORS_H
#define _FOSSASAT_SENSORS_H

#include "FossaSat2.h"

void Sensors_Temperature_Setup(wireSensor_t& sensor, uint8_t res);
float Sensors_Temperature_Read(wireSensor_t& sensor);

uint16_t Sensors_IMU_Setup();
void Sensors_IMU_Update();

bool Sensors_Current_Setup(Adafruit_INA260& sensor, TwoWire& wire, uint8_t addr);
float Sensors_Current_Read(Adafruit_INA260& sensor);
float Sensors_Current_ReadVoltage(Adafruit_INA260& sensor);

bool Sensors_Setup_Light(Adafruit_VEML7700& sensor, TwoWire& wire);
float Sensors_Read_Light(Adafruit_VEML7700& sensor);

#endif
