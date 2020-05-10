#ifndef _FOSSASAT_SENSORS_H
#define _FOSSASAT_SENSORS_H

#include "FossaSat2.h"

void Sensors_Setup_Temp(wireSensor_t& sensor, uint8_t res);
float Sensors_Read_Temperature(wireSensor_t& sensor);

uint16_t Sensors_Setup_IMU();
void Sensors_Update_IMU();

bool Sensors_Setup_Current(Adafruit_INA260& sensor, TwoWire& wire, uint8_t addr);

bool Sensors_Setup_Light(Adafruit_VEML7700& sensor, TwoWire& wire);
float Sensors_Read_Light(Adafruit_VEML7700& sensor);

#endif
