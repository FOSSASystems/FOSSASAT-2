#ifndef _FOSSASAT_SENSORS_H
#define _FOSSASAT_SENSORS_H

#include "FossaSat2.h"

void Sensors_Temperature_Setup(wireSensor_t& sensor);
float Sensors_Temperature_Read(wireSensor_t& sensor);

uint16_t Sensors_IMU_Setup();
void Sensors_IMU_Update(bool autoSleep = true);
void Sensors_IMU_Sleep(bool sleep);

bool Sensors_Current_Setup(currentSensor_t& sensor);
float Sensors_Current_Read(currentSensor_t& sensor);
float Sensors_Current_ReadVoltage(currentSensor_t& sensor);

bool Sensors_Setup_Light(lightSensor_t& sensor);
float Sensors_Read_Light(lightSensor_t& sensor);

#endif
