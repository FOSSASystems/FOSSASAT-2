#ifndef _FOSSASAT_SENSORS_H
#define _FOSSASAT_SENSORS_H

#include "FossaSat2.h"

void Sensors_Temperature_Setup(wireSensor_t& sensor);
float Sensors_Temperature_Read(wireSensor_t& sensor);

uint16_t Sensors_IMU_Setup();
void Sensors_IMU_Update();
void Sensors_IMU_Sleep(bool sleep);

void Sensors_IMU_CalcGyro(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, double processed[3]);
void Sensors_IMU_CalcAccel(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, double processed[3]);
void Sensors_IMU_CalcMag(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, double processed[3]);

void Sensors_IMU_CalcGyro(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, float processed[3]);
void Sensors_IMU_CalcAccel(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, float processed[3]);
void Sensors_IMU_CalcMag(int16_t rawX, int16_t rawY, int16_t rawZ, uint32_t offsetAddr, float processed[3]);

bool Sensors_Current_Setup(currentSensor_t& sensor);
float Sensors_Current_Read(currentSensor_t& sensor);
float Sensors_Current_ReadVoltage(currentSensor_t& sensor);
float Sensors_Current_ReadPower(currentSensor_t& sensor);

bool Sensors_Setup_Light(lightSensor_t& sensor);
float Sensors_Read_Light(lightSensor_t& sensor);

#endif
