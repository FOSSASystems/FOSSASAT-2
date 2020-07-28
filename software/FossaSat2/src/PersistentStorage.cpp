#include "PersistentStorage.h"

bool PersistentStorage_Check_CRC(uint8_t* buff, uint32_t crcPos) {
  // check CRC of the current page
  uint32_t currCrc = 0;
  memcpy(&currCrc, buff + crcPos, sizeof(uint32_t));
  uint32_t realCrc = CRC32_Get(buff, crcPos);
  if(currCrc != realCrc) {
    // memory error happened between last check and now, increment the counter
    uint32_t errCounter = 0;
    memcpy(&errCounter, buff + crcPos + sizeof(uint32_t), sizeof(uint32_t));
    errCounter++;
    memcpy(buff + crcPos + sizeof(uint32_t), &errCounter, sizeof(uint32_t));

    // set the new CRC
    memcpy(buff + crcPos, &realCrc, sizeof(uint32_t));
    return(false);
  }
  return(true);
}

template<typename T>
// cppcheck-suppress unusedFunction
T PersistentStorage_SystemInfo_Get(uint8_t addr) {
  T t;
  memcpy(&t, systemInfoBuffer + addr, sizeof(T));
  return(t);
}

template int8_t PersistentStorage_SystemInfo_Get<int8_t>(uint8_t);
template uint8_t PersistentStorage_SystemInfo_Get<uint8_t>(uint8_t);
template int16_t PersistentStorage_SystemInfo_Get<int16_t>(uint8_t);
template uint16_t PersistentStorage_SystemInfo_Get<uint16_t>(uint8_t);
template int32_t PersistentStorage_SystemInfo_Get<int32_t>(uint8_t);
template uint32_t PersistentStorage_SystemInfo_Get<uint32_t>(uint8_t);
template float PersistentStorage_SystemInfo_Get<float>(uint8_t);
template double PersistentStorage_SystemInfo_Get<double>(uint8_t);

template<typename T>
// cppcheck-suppress unusedFunction
void PersistentStorage_SystemInfo_Set(uint8_t addr, T t) {
  // check address is in system info
  if(addr > (FLASH_EXT_PAGE_SIZE - 1)) {
    return;
  }

  // set the new value to RAM buffer
  memcpy(systemInfoBuffer + addr, &t, sizeof(T));
}

template void PersistentStorage_SystemInfo_Set<int8_t>(uint8_t, int8_t);
template void PersistentStorage_SystemInfo_Set<uint8_t>(uint8_t, uint8_t);
template void PersistentStorage_SystemInfo_Set<int16_t>(uint8_t, int16_t);
template void PersistentStorage_SystemInfo_Set<uint16_t>(uint8_t, uint16_t);
template void PersistentStorage_SystemInfo_Set<int32_t>(uint8_t, int32_t);
template void PersistentStorage_SystemInfo_Set<uint32_t>(uint8_t, uint32_t);
template void PersistentStorage_SystemInfo_Set<float>(uint8_t, float);
template void PersistentStorage_SystemInfo_Set<double>(uint8_t, double);

template<typename T>
// cppcheck-suppress unusedFunction
T PersistentStorage_Get(uint32_t addr) {
  // create buffer large enough to fit any data type saved in external flash
  uint8_t varBuff[32];

  // read the flash
  PersistentStorage_Read(addr, varBuff, sizeof(T));

  // read the value
  T t;
  memcpy(&t, varBuff, sizeof(T));
  return(t);
}

template float PersistentStorage_Get<float>(uint32_t);
template double PersistentStorage_Get<double>(uint32_t);
template uint32_t PersistentStorage_Get<uint32_t>(uint32_t);
template int8_t PersistentStorage_Get<int8_t>(uint32_t);
template uint8_t PersistentStorage_Get<uint8_t>(uint32_t);

template<typename T>
// cppcheck-suppress unusedFunction
void PersistentStorage_Set(uint32_t addr, T t) {
  // create single page buffer
  uint8_t pageBuff[FLASH_EXT_PAGE_SIZE];

  // read the page
  uint32_t addrInPage = addr & 0x000000FF;
  uint32_t pageStartAddr = addr & 0xFFFFFF00;
  PersistentStorage_Read(pageStartAddr, pageBuff, FLASH_EXT_PAGE_SIZE);

  // check if we need to update
  uint8_t newPageBuff[FLASH_EXT_PAGE_SIZE];
  memcpy(newPageBuff, pageBuff, FLASH_EXT_PAGE_SIZE);
  memcpy(newPageBuff + addrInPage, &t, sizeof(T));
  if(memcmp(pageBuff, newPageBuff, FLASH_EXT_PAGE_SIZE) == 0) {
    // the value is already there, no need to write
    return;
  }

  // we need to update
  PersistentStorage_Write(pageStartAddr, newPageBuff, FLASH_EXT_PAGE_SIZE);
}

template void PersistentStorage_Set<float>(uint32_t, float);
template void PersistentStorage_Set<double>(uint32_t, double);
template void PersistentStorage_Set<uint32_t>(uint32_t, uint32_t);
template void PersistentStorage_Set<int8_t>(uint32_t, int8_t);

template <typename T>
// cppcheck-suppress unusedFunction
void PersistentStorage_Update_Stat(uint8_t* statBuff, uint32_t addr, T val) {
  uint32_t statAddr = addr - FLASH_STATS;
  size_t typeSize = sizeof(T);

  // get min/avg/max
  T min = 0;
  memcpy(&min, statBuff + statAddr, typeSize);
  T avg = 0;
  memcpy(&avg, statBuff + statAddr + typeSize, typeSize);
  T max = 0;
  memcpy(&max, statBuff + statAddr + 2*typeSize, typeSize);

  // update stats
  if(val < min) {
    memcpy(statBuff + statAddr, &val, typeSize);
  }

  avg = (avg + val)/((T)2);
  memcpy(statBuff + statAddr + typeSize, &avg, typeSize);

  if(val > max) {
    memcpy(statBuff + statAddr + 2*typeSize, &val, typeSize);
  }
}

template void PersistentStorage_Update_Stat<uint8_t>(uint8_t*, uint32_t, uint8_t);
template void PersistentStorage_Update_Stat<int16_t>(uint8_t*, uint32_t, int16_t);
template void PersistentStorage_Update_Stat<float>(uint8_t*, uint32_t, float);

void PersistentStorage_Update_Stats(uint8_t flags) {
  // read stats page
  const size_t statsBufferLen = 2*FLASH_EXT_PAGE_SIZE;
  uint8_t statsBuffer[statsBufferLen];
  PersistentStorage_Read(FLASH_STATS, statsBuffer, statsBufferLen);

  if(flags & STATS_FLAGS_TEMPERATURES) {
    // temperatures
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_TEMP_PANEL_Y, (int16_t)(Sensors_Temperature_Read(tempSensorPanelY) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_TEMP_TOP, (int16_t)(Sensors_Temperature_Read(tempSensorTop) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_TEMP_BOTTOM, (int16_t)(Sensors_Temperature_Read(tempSensorBottom) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_TEMP_BATTERY, (int16_t)(Sensors_Temperature_Read(tempSensorBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_TEMP_SEC_BATTERY, (int16_t)(Sensors_Temperature_Read(tempSensorSecBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_TEMP_MCU, (int16_t)(Sensors_Temperature_Read(tempSensorMCU) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER)));
  }

  if(flags & STATS_FLAGS_CURRENTS) {
    // currents
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_CURR_XA, (int16_t)(Sensors_Current_Read(currSensorXA) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_CURR_XB, (int16_t)(Sensors_Current_Read(currSensorXB) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_CURR_ZA, (int16_t)(Sensors_Current_Read(currSensorZA) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_CURR_ZB, (int16_t)(Sensors_Current_Read(currSensorZB) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_CURR_Y, (int16_t)(Sensors_Current_Read(currSensorY) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER)));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_CURR_MPPT, (int16_t)(Sensors_Current_Read(currSensorMPPT) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER)));
  }

  if(flags & STATS_FLAGS_VOLTAGES) {
    // voltages
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_VOLT_XA, (uint8_t)(Sensors_Current_ReadVoltage(currSensorXA) / (float)VOLTAGE_MULTIPLIER));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_VOLT_XB, (uint8_t)(Sensors_Current_ReadVoltage(currSensorXB) / (float)VOLTAGE_MULTIPLIER));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_VOLT_ZA, (uint8_t)(Sensors_Current_ReadVoltage(currSensorZA) / (float)VOLTAGE_MULTIPLIER));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_VOLT_ZB, (uint8_t)(Sensors_Current_ReadVoltage(currSensorZB) / (float)VOLTAGE_MULTIPLIER));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_VOLT_Y, (uint8_t)(Sensors_Current_ReadVoltage(currSensorY) / (float)VOLTAGE_MULTIPLIER));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_VOLT_MPPT, (uint8_t)(Sensors_Current_ReadVoltage(currSensorMPPT) / (float)VOLTAGE_MULTIPLIER));
  }

  if(flags & STATS_FLAGS_LIGHT) {
    // lights
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_LIGHT_PANEL_Y, Sensors_Read_Light(lightSensorPanelY));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_LIGHT_TOP, Sensors_Read_Light(lightSensorTop));
  }

  if(flags & STATS_FLAGS_IMU) {
    // IMU
    float val[3];
    Sensors_IMU_Update();
    Sensors_IMU_CalcGyro(imu.gx, imu.gy, imu.gz, FLASH_IMU_OFFSET_GYRO_X, val);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_GYRO_X, val[0]);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_GYRO_Y, val[1]);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_GYRO_Z, val[2]);
    Sensors_IMU_CalcAccel(imu.ax, imu.ay, imu.az, FLASH_IMU_OFFSET_ACCEL_X, val);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_ACCEL_X, val[0]);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_ACCEL_Y, val[1]);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_ACCEL_Z, val[2]);
    Sensors_IMU_CalcMag(imu.mx, imu.my, imu.mz, FLASH_IMU_OFFSET_MAG_X, val);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_MAG_X, val[0]);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_MAG_Y, val[1]);
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_MAG_Z, val[2]);
  }

  if(flags & STATS_FLAGS_POWER) {
    // solar cells output power
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_POWER_XA, Sensors_Current_ReadPower(currSensorXA));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_POWER_XB, Sensors_Current_ReadPower(currSensorXB));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_POWER_ZA, Sensors_Current_ReadPower(currSensorZA));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_POWER_ZB, Sensors_Current_ReadPower(currSensorZB));
    PersistentStorage_Update_Stat(statsBuffer, FLASH_STATS_POWER_Y, Sensors_Current_ReadPower(currSensorY));
  }

  // write updated page
  PersistentStorage_Write(FLASH_STATS, statsBuffer, statsBufferLen);

  FOSSASAT_DEBUG_PRINTLN(F("Stats:"));
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_STATS, FLASH_EXT_PAGE_SIZE);
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_STATS + FLASH_EXT_PAGE_SIZE, FLASH_EXT_PAGE_SIZE);
}

void PersistentStorage_Reset_Stats() {
  // build a completely new stats page
  const size_t statsBufferLen = 2*FLASH_EXT_PAGE_SIZE;
  uint8_t statsPage[statsBufferLen];

  // set everything to 0 by default
  memset(statsPage, 0, statsBufferLen);

  // get minimum and maximum values for all used data types
  int16_t intMax = 32767;
  int16_t intVal = 0;
  int16_t intMin = -1;
  uint8_t byteMax = 255;
  uint8_t byteVal = 0;
  uint8_t byteMin = 0x00;
  float floatMax = 1000000;
  float floatVal = 0;
  float floatMin = -1000000;

  // set temperatures
  memcpy(statsPage + (FLASH_STATS_TEMP_PANEL_Y - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_TEMP_TOP - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_TEMP_BOTTOM - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_TEMP_BATTERY - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_TEMP_SEC_BATTERY - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_TEMP_MCU - FLASH_STATS), &intMax, sizeof(intMax));

  intVal = Sensors_Temperature_Read(tempSensorPanelY) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_TEMP_PANEL_Y - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Temperature_Read(tempSensorTop) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_TEMP_TOP - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Temperature_Read(tempSensorBottom) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_TEMP_BOTTOM - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Temperature_Read(tempSensorBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_TEMP_BATTERY - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Temperature_Read(tempSensorSecBattery) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_TEMP_SEC_BATTERY - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Temperature_Read(tempSensorMCU) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_TEMP_MCU - FLASH_STATS) + sizeof(intVal), &intMax, sizeof(intMax));

  memcpy(statsPage + (FLASH_STATS_TEMP_PANEL_Y - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_TEMP_TOP - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_TEMP_BOTTOM - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_TEMP_BATTERY - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_TEMP_SEC_BATTERY - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_TEMP_MCU - FLASH_STATS) + 2*sizeof(intVal), &intMax, sizeof(intMax));

  // set currents
  memcpy(statsPage + (FLASH_STATS_CURR_XA - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_CURR_XB - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_CURR_ZA - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_CURR_ZB - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_CURR_Y - FLASH_STATS), &intMax, sizeof(intMax));
  memcpy(statsPage + (FLASH_STATS_CURR_MPPT - FLASH_STATS), &intMax, sizeof(intMax));

  intVal = Sensors_Current_Read(currSensorXA) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_CURR_XA - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Current_Read(currSensorXB) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_CURR_XB - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Current_Read(currSensorZA) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_CURR_ZA - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Current_Read(currSensorZB) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_CURR_ZB - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Current_Read(currSensorY) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_CURR_Y - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));
  intVal = Sensors_Current_Read(currSensorMPPT) * ((CURRENT_UNIT / 1000) / CURRENT_MULTIPLIER);
  memcpy(statsPage + (FLASH_STATS_CURR_MPPT - FLASH_STATS) + sizeof(intVal), &intVal, sizeof(intVal));

  memcpy(statsPage + (FLASH_STATS_CURR_XA - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_CURR_XB - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_CURR_ZA - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_CURR_ZB - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_CURR_Y - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));
  memcpy(statsPage + (FLASH_STATS_CURR_MPPT - FLASH_STATS) + 2*sizeof(intMin), &intMin, sizeof(intMin));

  // set voltages
  memcpy(statsPage + (FLASH_STATS_VOLT_XA - FLASH_STATS), &byteMax, sizeof(byteMax));
  memcpy(statsPage + (FLASH_STATS_VOLT_XB - FLASH_STATS), &byteMax, sizeof(byteMax));
  memcpy(statsPage + (FLASH_STATS_VOLT_ZA - FLASH_STATS), &byteMax, sizeof(byteMax));
  memcpy(statsPage + (FLASH_STATS_VOLT_ZB - FLASH_STATS), &byteMax, sizeof(byteMax));
  memcpy(statsPage + (FLASH_STATS_VOLT_Y - FLASH_STATS), &byteMax, sizeof(byteMax));
  memcpy(statsPage + (FLASH_STATS_VOLT_MPPT - FLASH_STATS), &byteMax, sizeof(byteMax));

  byteVal = Sensors_Current_ReadVoltage(currSensorXA) / (float)VOLTAGE_MULTIPLIER;
  memcpy(statsPage + (FLASH_STATS_CURR_XA - FLASH_STATS) + sizeof(byteVal), &byteVal, sizeof(byteVal));
  byteVal = Sensors_Current_ReadVoltage(currSensorXB) / (float)VOLTAGE_MULTIPLIER;
  memcpy(statsPage + (FLASH_STATS_VOLT_XB - FLASH_STATS) + sizeof(byteVal), &byteVal, sizeof(byteVal));
  byteVal = Sensors_Current_ReadVoltage(currSensorZA) / (float)VOLTAGE_MULTIPLIER;
  memcpy(statsPage + (FLASH_STATS_VOLT_ZA - FLASH_STATS) + sizeof(byteVal), &byteVal, sizeof(byteVal));
  byteVal = Sensors_Current_ReadVoltage(currSensorZB) / (float)VOLTAGE_MULTIPLIER;
  memcpy(statsPage + (FLASH_STATS_VOLT_ZB - FLASH_STATS) + sizeof(byteVal), &byteVal, sizeof(byteVal));
  byteVal = Sensors_Current_ReadVoltage(currSensorY) / (float)VOLTAGE_MULTIPLIER;
  memcpy(statsPage + (FLASH_STATS_VOLT_Y - FLASH_STATS) + sizeof(byteVal), &byteVal, sizeof(byteVal));
  byteVal = Sensors_Current_ReadVoltage(currSensorMPPT) / (float)VOLTAGE_MULTIPLIER;
  memcpy(statsPage + (FLASH_STATS_VOLT_Y - FLASH_STATS) + sizeof(byteVal), &byteVal, sizeof(byteVal));

  memcpy(statsPage + (FLASH_STATS_VOLT_XA - FLASH_STATS) + 2*sizeof(byteMin), &byteMin, sizeof(byteMin));
  memcpy(statsPage + (FLASH_STATS_VOLT_XB - FLASH_STATS) + 2*sizeof(byteMin), &byteMin, sizeof(byteMin));
  memcpy(statsPage + (FLASH_STATS_VOLT_ZA - FLASH_STATS) + 2*sizeof(byteMin), &byteMin, sizeof(byteMin));
  memcpy(statsPage + (FLASH_STATS_VOLT_ZB - FLASH_STATS) + 2*sizeof(byteMin), &byteMin, sizeof(byteMin));
  memcpy(statsPage + (FLASH_STATS_VOLT_Y - FLASH_STATS) + 2*sizeof(byteMin), &byteMin, sizeof(byteMin));
  memcpy(statsPage + (FLASH_STATS_VOLT_MPPT - FLASH_STATS) + 2*sizeof(byteMin), &byteMin, sizeof(byteMin));

  // set light sensors
  float solarMax = ADCS_SOLAR_SENSOR_MAX;
  memcpy(statsPage + (FLASH_STATS_LIGHT_PANEL_Y - FLASH_STATS), &solarMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_LIGHT_TOP - FLASH_STATS), &solarMax, sizeof(floatMax));

  floatVal = Sensors_Read_Light(lightSensorPanelY);
  memcpy(statsPage + (FLASH_STATS_LIGHT_PANEL_Y - FLASH_STATS) + sizeof(floatVal), &floatVal, sizeof(floatVal));
  floatVal = Sensors_Read_Light(lightSensorTop);
  memcpy(statsPage + (FLASH_STATS_LIGHT_TOP - FLASH_STATS) + sizeof(floatVal), &floatVal, sizeof(floatVal));

  memcpy(statsPage + (FLASH_STATS_LIGHT_PANEL_Y - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_LIGHT_TOP - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));

  // set IMU
  memcpy(statsPage + (FLASH_STATS_GYRO_X - FLASH_STATS), &floatMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_GYRO_Y - FLASH_STATS), &floatMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_GYRO_Z - FLASH_STATS), &floatMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_ACCEL_X - FLASH_STATS), &floatMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_ACCEL_Y - FLASH_STATS), &floatMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_ACCEL_Z - FLASH_STATS), &floatMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_MAG_X - FLASH_STATS), &floatMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_MAG_Y - FLASH_STATS), &floatMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_MAG_Z - FLASH_STATS), &floatMax, sizeof(floatMax));

  float val[3];
  Sensors_IMU_Update();
  Sensors_IMU_CalcGyro(imu.gx, imu.gy, imu.gz, FLASH_IMU_OFFSET_GYRO_X, val);
  memcpy(statsPage + (FLASH_STATS_GYRO_X - FLASH_STATS) + sizeof(floatVal), &val[0], sizeof(floatVal));
  memcpy(statsPage + (FLASH_STATS_GYRO_Y - FLASH_STATS) + sizeof(floatVal), &val[1], sizeof(floatVal));
  memcpy(statsPage + (FLASH_STATS_GYRO_Z - FLASH_STATS) + sizeof(floatVal), &val[2], sizeof(floatVal));
  Sensors_IMU_CalcAccel(imu.ax, imu.ay, imu.az, FLASH_IMU_OFFSET_ACCEL_X, val);
  memcpy(statsPage + (FLASH_STATS_ACCEL_X - FLASH_STATS) + sizeof(floatVal), &val[0], sizeof(floatVal));
  memcpy(statsPage + (FLASH_STATS_ACCEL_Y - FLASH_STATS) + sizeof(floatVal), &val[1], sizeof(floatVal));
  memcpy(statsPage + (FLASH_STATS_ACCEL_Z - FLASH_STATS) + sizeof(floatVal), &val[2], sizeof(floatVal));
  Sensors_IMU_CalcMag(imu.mx, imu.my, imu.mz, FLASH_IMU_OFFSET_MAG_X, val);
  memcpy(statsPage + (FLASH_STATS_MAG_X - FLASH_STATS) + sizeof(floatVal), &val[0], sizeof(floatVal));
  memcpy(statsPage + (FLASH_STATS_MAG_Y - FLASH_STATS) + sizeof(floatVal), &val[1], sizeof(floatVal));
  memcpy(statsPage + (FLASH_STATS_MAG_Z - FLASH_STATS) + sizeof(floatVal), &val[2], sizeof(floatVal));

  memcpy(statsPage + (FLASH_STATS_GYRO_X - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_GYRO_Y - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_GYRO_Z - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_ACCEL_X - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_ACCEL_Y - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_ACCEL_Z - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_MAG_X - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_MAG_Y - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_MAG_Z - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));

  // set powers
  float powerMax = ADCS_SOLAR_POWER_XZ_MAX;
  memcpy(statsPage + (FLASH_STATS_POWER_XA - FLASH_STATS), &powerMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_POWER_XB - FLASH_STATS), &powerMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_POWER_ZA - FLASH_STATS), &powerMax, sizeof(floatMax));
  memcpy(statsPage + (FLASH_STATS_POWER_ZB - FLASH_STATS), &powerMax, sizeof(floatMax));
  powerMax = ADCS_SOLAR_POWER_Y_MAX;
  memcpy(statsPage + (FLASH_STATS_POWER_Y - FLASH_STATS), &powerMax, sizeof(floatMax));

  floatVal = Sensors_Current_ReadPower(currSensorXA);
  memcpy(statsPage + (FLASH_STATS_POWER_XA - FLASH_STATS) + sizeof(floatVal), &floatVal, sizeof(floatVal));
  floatVal = Sensors_Current_ReadPower(currSensorXB);
  memcpy(statsPage + (FLASH_STATS_POWER_XB - FLASH_STATS) + sizeof(floatVal), &floatVal, sizeof(floatVal));
  floatVal = Sensors_Current_ReadPower(currSensorZA);
  memcpy(statsPage + (FLASH_STATS_POWER_ZA - FLASH_STATS) + sizeof(floatVal), &floatVal, sizeof(floatVal));
  floatVal = Sensors_Current_ReadPower(currSensorZB);
  memcpy(statsPage + (FLASH_STATS_POWER_ZB - FLASH_STATS) + sizeof(floatVal), &floatVal, sizeof(floatVal));
  floatVal = Sensors_Current_ReadPower(currSensorY);
  memcpy(statsPage + (FLASH_STATS_POWER_Y - FLASH_STATS) + sizeof(floatVal), &floatVal, sizeof(floatVal));

  memcpy(statsPage + (FLASH_STATS_POWER_XA - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_POWER_XB - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_POWER_ZA - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_POWER_ZB - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));
  memcpy(statsPage + (FLASH_STATS_POWER_Y - FLASH_STATS) + 2*sizeof(floatMin), &floatMin, sizeof(floatMin));

  // write all at once
  PersistentStorage_Write(FLASH_STATS, statsPage, statsBufferLen);
}

void PersistentStorage_Increment_Counter(uint16_t addr) {
  uint16_t counter = 0;
  memcpy(&counter, systemInfoBuffer + addr, sizeof(uint16_t));
  counter++;
  PersistentStorage_SystemInfo_Set(addr, counter);
}

void PersistentStorage_Increment_Frame_Counter(bool valid) {
  uint16_t addr = FLASH_LORA_VALID_COUNTER;
  if(currentModem == MODEM_LORA) {
    if(!valid) {
      addr += 2;
    }
  } else {
    if(valid) {
      addr += 4;
    } else {
      addr += 6;
    }
  }

  PersistentStorage_Increment_Counter(addr);
}

void PersistentStorage_Get_Callsign(char* buff, uint8_t len) {
  memcpy((uint8_t*)buff, systemInfoBuffer + FLASH_CALLSIGN, len);
  buff[len] = '\0';
}

void PersistentStorage_Set_Callsign(char* newCallsign) {
  // get length of the new callsign
  uint8_t newCallsignLen = (uint8_t)strlen(newCallsign);

  // check new callsign length
  if (newCallsignLen > MAX_STRING_LENGTH) {
    FOSSASAT_DEBUG_PRINTLN(F("New callsign too long!"));
    return;
  }

  // update callsign entries
  systemInfoBuffer[FLASH_CALLSIGN_LEN] = newCallsignLen;
  memcpy(systemInfoBuffer + FLASH_CALLSIGN, newCallsign, newCallsignLen);
}

uint32_t PersistentStorage_Get_Image_Len(uint8_t slot) {
  return(PersistentStorage_Get_Image_Property(slot, 0));
}

uint32_t PersistentStorage_Get_Image_ScanStart(uint8_t slot) {
  return(PersistentStorage_Get_Image_Property(slot, 1));
}

uint32_t PersistentStorage_Get_Image_ScanEnd(uint8_t slot) {
  return(PersistentStorage_Get_Image_Property(slot, 2));
}

uint32_t PersistentStorage_Get_Image_Property(uint8_t slot, uint8_t offset) {
  // find which sector are the properties in
  uint8_t buff[FLASH_EXT_PAGE_SIZE];
  uint32_t addr = FLASH_IMAGE_PROPERTIES + (slot/FLASH_IMAGE_PROPERTIES_SLOT_SIZE) * FLASH_SECTOR_SIZE;
  PersistentStorage_Read(addr, buff, FLASH_EXT_PAGE_SIZE);

  // get the value
  uint8_t* slotAddr = buff + (slot % FLASH_IMAGE_PROPERTIES_SLOT_SIZE) * 3*sizeof(uint32_t) + offset*sizeof(uint32_t);
  uint32_t prop;
  memcpy(&prop, slotAddr, sizeof(uint32_t));
  return(prop);
}

void PersistentStorage_Set_Image_Properties(uint8_t slot, uint32_t len, uint32_t scanStart, uint32_t scanEnd) {
  // find which sector are the properties in
  uint8_t buff[FLASH_EXT_PAGE_SIZE];
  uint32_t addr = FLASH_IMAGE_PROPERTIES + (slot/FLASH_IMAGE_PROPERTIES_SLOT_SIZE) * FLASH_SECTOR_SIZE;
  PersistentStorage_Read(addr, buff, FLASH_EXT_PAGE_SIZE);

  // update values
  uint8_t* slotAddr = buff + (slot % FLASH_IMAGE_PROPERTIES_SLOT_SIZE) * 3*sizeof(uint32_t);
  memcpy(slotAddr, &len, sizeof(uint32_t));
  memcpy(slotAddr + sizeof(uint32_t), &scanStart, sizeof(uint32_t));
  memcpy(slotAddr + 2*sizeof(uint32_t), &scanEnd, sizeof(uint32_t));

  // write it back in (will automatically erase the sector)
  PersistentStorage_Write(addr, buff, FLASH_EXT_PAGE_SIZE);
}

void PersistentStorage_Set_Buffer(uint8_t addr, uint8_t* buff, size_t len) {
  // check address is in system info
  if(addr > (FLASH_EXT_PAGE_SIZE - 1)) {
    return;
  }

  // read the current system info page
  uint8_t currSysInfoPage[FLASH_EXT_PAGE_SIZE];
  PersistentStorage_Read(FLASH_SYSTEM_INFO, currSysInfoPage, FLASH_EXT_PAGE_SIZE);

  // get current memory error counter
  uint32_t errCounter = 0;
  memcpy(&errCounter, currSysInfoPage + FLASH_MEMORY_ERROR_COUNTER, sizeof(uint32_t));

  // check CRC of the current page
  uint32_t currCrc = 0;
  memcpy(&currCrc, currSysInfoPage + FLASH_SYSTEM_INFO_CRC, sizeof(uint32_t));
  if(currCrc != CRC32_Get(currSysInfoPage, FLASH_SYSTEM_INFO_CRC)) {
    // memory error happened between last write and now, increment the counter
    FOSSASAT_DEBUG_PRINTLN(F("System info CRC check failed!"));
    errCounter++;
  }

  // check if we need to update
  uint8_t newSysInfoPage[FLASH_EXT_PAGE_SIZE];
  memcpy(newSysInfoPage, currSysInfoPage, FLASH_EXT_PAGE_SIZE);
  memcpy(newSysInfoPage + addr, buff, len);
  if(memcmp(currSysInfoPage, newSysInfoPage, FLASH_EXT_PAGE_SIZE) == 0) {
    // the value is already there, no need to write
    return;
  }

  // update CRC
  uint32_t crc = CRC32_Get(newSysInfoPage, FLASH_SYSTEM_INFO_CRC);
  memcpy(newSysInfoPage + FLASH_SYSTEM_INFO_CRC, &crc, sizeof(uint32_t));

  // update memory error counter
  memcpy(newSysInfoPage + FLASH_MEMORY_ERROR_COUNTER, &errCounter, sizeof(uint32_t));

  // we need to update
  PersistentStorage_Write(FLASH_SYSTEM_INFO, newSysInfoPage, FLASH_EXT_PAGE_SIZE);
}

void PersistentStorage_Reset_System_Info() {
  // set everything to 0 by default
  memset(systemInfoBuffer, 0, FLASH_EXT_PAGE_SIZE);

  // set non-zero defaults

  // set default transmission configuration
  systemInfoBuffer[FLASH_TRANSMISSIONS_ENABLED] = 1;

  // set default callsign length
  systemInfoBuffer[FLASH_CALLSIGN_LEN] = strlen(CALLSIGN_DEFAULT);

  // set default callsign
  memcpy(systemInfoBuffer + FLASH_CALLSIGN, CALLSIGN_DEFAULT, strlen(CALLSIGN_DEFAULT));

  // set default receive windows
  systemInfoBuffer[FLASH_FSK_RECEIVE_LEN] = FSK_RECEIVE_WINDOW_LENGTH;
  systemInfoBuffer[FLASH_LORA_RECEIVE_LEN] = LORA_RECEIVE_WINDOW_LENGTH;

  // set default RTC epoch
  rtc.setDate(RTC_WEEKDAY, RTC_DAY, RTC_MONTH, RTC_YEAR);
  rtc.setTime(RTC_HOURS, RTC_MINUTES, RTC_SECONDS);
  systemInfoBuffer[FLASH_RTC_EPOCH] = rtc.getEpoch();

  // set default low power mode configuration
  systemInfoBuffer[FLASH_LOW_POWER_MODE_ENABLED] = 1;

  // set default voltage limits
  int16_t voltageLimit = DEPLOYMENT_BATTERY_VOLTAGE_LIMIT;
  memcpy(systemInfoBuffer + FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT, &voltageLimit, sizeof(int16_t));
  voltageLimit = HEATER_BATTERY_VOLTAGE_LIMIT;
  memcpy(systemInfoBuffer + FLASH_HEATER_BATTERY_VOLTAGE_LIMIT, &voltageLimit, sizeof(int16_t));
  voltageLimit = BATTERY_CW_BEEP_VOLTAGE_LIMIT;
  memcpy(systemInfoBuffer + FLASH_BATTERY_CW_BEEP_VOLTAGE_LIMIT, &voltageLimit, sizeof(int16_t));
  voltageLimit = LOW_POWER_MODE_VOLTAGE_LIMIT;
  memcpy(systemInfoBuffer + FLASH_LOW_POWER_MODE_VOLTAGE_LIMIT, &voltageLimit, sizeof(int16_t));

  // set default temperature limits
  float tempLimit = BATTERY_HEATER_TEMP_LIMIT;
  memcpy(systemInfoBuffer + FLASH_BATTERY_HEATER_TEMP_LIMIT, &tempLimit, sizeof(float));
  tempLimit = MPPT_TEMP_LIMIT;
  memcpy(systemInfoBuffer + FLASH_MPPT_TEMP_LIMIT, &tempLimit, sizeof(float));

  // set default heater duty cycle
  uint8_t dutyCycle = BATTERY_HEATER_DUTY_CYCLE;
  memcpy(systemInfoBuffer + FLASH_BATTERY_HEATER_DUTY_CYCLE, &dutyCycle, sizeof(uint8_t));

  // set default MPPT temperature switch mode
  systemInfoBuffer[FLASH_MPPT_TEMP_SWITCH_ENABLED] = 1;

  // set default statistics transmission
  systemInfoBuffer[FLASH_AUTO_STATISTICS] = 1;

  // set default TLE
  uint8_t b = Navigation_Get_EpochYear(TLE_LINE_1);
  memcpy(systemInfoBuffer + FLASH_TLE_EPOCH_YEAR, &b, sizeof(uint8_t));
  double d = Navigation_Get_EpochDay(TLE_LINE_1);
  memcpy(systemInfoBuffer + FLASH_TLE_EPOCH_DAY, &d, sizeof(double));
  d = Navigation_Get_BallisticCoeff(TLE_LINE_1);
  memcpy(systemInfoBuffer + FLASH_TLE_BALLISTIC_COEFF, &d, sizeof(double));
  d = Navigation_Get_MeanMotion2nd(TLE_LINE_1);
  memcpy(systemInfoBuffer + FLASH_TLE_MEAN_MOTION_2ND, &d, sizeof(double));
  d = Navigation_Get_DragTerm(TLE_LINE_1);
  memcpy(systemInfoBuffer + FLASH_TLE_DRAG_TERM, &d, sizeof(double));
  d = Navigation_Get_Inclination(TLE_LINE_2);
  memcpy(systemInfoBuffer + FLASH_TLE_INCLINATION, &d, sizeof(double));
  d = Navigation_Get_RightAscension(TLE_LINE_2);
  memcpy(systemInfoBuffer + FLASH_TLE_RIGHT_ASCENTION, &d, sizeof(double));
  d = Navigation_Get_Eccentricity(TLE_LINE_2);
  memcpy(systemInfoBuffer + FLASH_TLE_ECCENTRICITY, &d, sizeof(double));
  d = Navigation_Get_PerigeeArgument(TLE_LINE_2);
  memcpy(systemInfoBuffer + FLASH_TLE_PERIGEE_ARGUMENT, &d, sizeof(double));
  d = Navigation_Get_MeanAnomaly(TLE_LINE_2);
  memcpy(systemInfoBuffer + FLASH_TLE_MEAN_ANOMALY, &d, sizeof(double));
  d = Navigation_Get_MeanMotion(TLE_LINE_2);
  memcpy(systemInfoBuffer + FLASH_TLE_MEAN_MOTION, &d, sizeof(double));
  uint32_t ul = Navigation_Get_RevolutionNumber(TLE_LINE_2);
  memcpy(systemInfoBuffer + FLASH_TLE_REVOLUTION_NUMBER, &ul, sizeof(uint32_t));

  // set default latest NMEA address
  uint32_t lastNmea = FLASH_NMEA_LOG_START;
  memcpy(systemInfoBuffer + FLASH_NMEA_LOG_LATEST_ENTRY, &lastNmea, sizeof(uint32_t));

  // set default latest NMEA fix
  uint32_t lastNmeaFix = 0;
  memcpy(systemInfoBuffer + FLASH_NMEA_LOG_LATEST_FIX, &lastNmeaFix, sizeof(uint32_t));

  // set default sleep intervals
  uint8_t numIntervals = DEFAULT_NUMBER_OF_SLEEP_INTERVALS;
  memcpy(systemInfoBuffer + FLASH_NUM_SLEEP_INTERVALS, &numIntervals, sizeof(uint8_t));
  uint8_t intervalSize = sizeof(int16_t) + sizeof(uint16_t);
  int16_t voltages[] = DEFAULT_SLEEP_INTERVAL_VOLTAGES;
  uint16_t lengths[] = DEFAULT_SLEEP_INTERVAL_LENGTHS;
  for(uint8_t i = 0; i < numIntervals; i++) {
    int16_t v = voltages[i];
    memcpy(systemInfoBuffer + FLASH_SLEEP_INTERVALS + i*intervalSize, &v, sizeof(int16_t));
    uint16_t l = lengths[i];
    memcpy(systemInfoBuffer + FLASH_SLEEP_INTERVALS + sizeof(int16_t) + i*intervalSize, &l, sizeof(uint16_t));
  }

  // set default IMU offsets
  float f = IMU_OFFSET_GYRO_X;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_GYRO_X, &f, sizeof(f));
  f = IMU_OFFSET_GYRO_Y;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_GYRO_Y, &f, sizeof(f));
  f = IMU_OFFSET_GYRO_Z;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_GYRO_Z, &f, sizeof(f));
  f = IMU_OFFSET_ACCEL_X;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_ACCEL_X, &f, sizeof(f));
  f = IMU_OFFSET_ACCEL_Y;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_ACCEL_Y, &f, sizeof(f));
  f = IMU_OFFSET_ACCEL_Z;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_ACCEL_Z, &f, sizeof(f));
  f = IMU_OFFSET_MAG_X;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_MAG_X, &f, sizeof(f));
  f = IMU_OFFSET_MAG_Y;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_MAG_Y, &f, sizeof(f));
  f = IMU_OFFSET_MAG_Z;
  memcpy(systemInfoBuffer + FLASH_IMU_OFFSET_MAG_Z, &f, sizeof(f));

  // set default FSK only flag
  systemInfoBuffer[FLASH_FSK_ONLY_ENABLED] = 1;

  // set CRC
  uint32_t crc = CRC32_Get(systemInfoBuffer, FLASH_SYSTEM_INFO_CRC);
  memcpy(systemInfoBuffer + FLASH_SYSTEM_INFO_CRC, &crc, sizeof(uint32_t));

  // write the default system info
  PersistentStorage_Write(FLASH_SYSTEM_INFO, systemInfoBuffer, FLASH_EXT_PAGE_SIZE);
}

void PersistentStorage_Reset_ADCS_Params() {
  // build a completely new page
  uint8_t adcsPage[2*FLASH_EXT_PAGE_SIZE];

  // set everything to 0 by default
  memset(adcsPage, 0, 2*FLASH_EXT_PAGE_SIZE);

  ADCS_CALC_TYPE f = ADCS_PULSE_MAX_INTENSITY;
  memcpy(adcsPage + (FLASH_ADCS_PULSE_MAX_INTENSITY - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_PULSE_MAX_LENGTH;
  memcpy(adcsPage + (FLASH_ADCS_PULSE_MAX_LENGTH - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_DETUMB_OMEGA_TOLERANCE;
  memcpy(adcsPage + (FLASH_ADCS_DETUMB_OMEGA_TOLERANCE - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_MIN_INERTIAL_MOMENT;
  memcpy(adcsPage + (FLASH_ADCS_MIN_INERTIAL_MOMENT - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_PULSE_AMPLITUDE;
  memcpy(adcsPage + (FLASH_ADCS_PULSE_AMPLITUDE - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_CALCULATION_TOLERANCE;
  memcpy(adcsPage + (FLASH_ADCS_CALCULATION_TOLERANCE - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_ACTIVE_EULER_TOLERANCE;
  memcpy(adcsPage + (FLASH_ADCS_ACTIVE_EULER_TOLERANCE - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_ACTIVE_OMEGA_TOLERANCE;
  memcpy(adcsPage + (FLASH_ADCS_ACTIVE_OMEGA_TOLERANCE - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_ECLIPSE_THRESHOLD;
  memcpy(adcsPage + (FLASH_ADCS_ECLIPSE_THRESHOLD - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_ROTATION_WEIGHT_RATIO;
  memcpy(adcsPage + (FLASH_ADCS_ROTATION_WEIGHT_RATIO - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_ROTATION_TRIGGER;
  memcpy(adcsPage + (FLASH_ADCS_ROTATION_TRIGGER - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_DISTURBANCE_COVARIANCE;
  memcpy(adcsPage + (FLASH_ADCS_DISTURBANCE_COVARIANCE - FLASH_ADCS_PARAMETERS), &f, sizeof(f));
  f = ADCS_NOISE_COVARIANCE;
  memcpy(adcsPage + (FLASH_ADCS_NOISE_COVARIANCE - FLASH_ADCS_PARAMETERS), &f, sizeof(f));

  uint32_t ul = ADCS_TIME_STEP;
  memcpy(adcsPage + (FLASH_ADCS_TIME_STEP - FLASH_ADCS_PARAMETERS), &ul, sizeof(ul));
  ul = ADCS_BRIDGE_TIMER_UPDATE_PERIOD;
  memcpy(adcsPage + (FLASH_ADCS_BRIDGE_TIMER_UPDATE_PERIOD - FLASH_ADCS_PARAMETERS), &ul, sizeof(ul));
  int8_t ss = ADCS_BRIDGE_OUTPUT_HIGH;
  memcpy(adcsPage + (FLASH_ADCS_BRIDGE_OUTPUT_HIGH - FLASH_ADCS_PARAMETERS), &ss, sizeof(ss));
  ss = ADCS_BRIDGE_OUTPUT_LOW;
  memcpy(adcsPage + (FLASH_ADCS_BRIDGE_OUTPUT_LOW - FLASH_ADCS_PARAMETERS), &ss, sizeof(ss));
  uint8_t us = ADCS_NUM_CONTROLLERS;
  memcpy(adcsPage + (FLASH_ADCS_NUM_CONTROLLERS - FLASH_ADCS_PARAMETERS), &us, sizeof(us));

  // write default coil characteristics
  float coilChar[ADCS_NUM_AXES][ADCS_NUM_AXES] = ADCS_COIL_CHARACTERISTICS;
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      float val = coilChar[i][j];
      memcpy(adcsPage + (FLASH_ADCS_COIL_CHAR_MATRIX - FLASH_ADCS_PARAMETERS) + (i*ADCS_NUM_AXES*sizeof(val) + j*sizeof(val)), &val, sizeof(val));
    }
  }

  // write default inertia tensor matrix
  float inertiaTensor[ADCS_NUM_AXES][ADCS_NUM_AXES] = ADCS_INERTIA_TENSOR;
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      float val = inertiaTensor[i][j];
      memcpy(adcsPage + (FLASH_ADCS_INERTIA_TENSOR_MATRIX - FLASH_ADCS_PARAMETERS) + (i*ADCS_NUM_AXES*sizeof(val) + j*sizeof(val)), &val, sizeof(val));
    }
  }

  // write default IMU calibration parameters
  float transMatrix[ADCS_NUM_AXES][ADCS_NUM_AXES] = ADCS_IMU_TRANS_MATRIX;
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    for(uint8_t j = 0; j < ADCS_NUM_AXES; j++) {
      float val = transMatrix[i][j];
      memcpy(adcsPage + (FLASH_ADCS_IMU_TRANS_MATRIX - FLASH_ADCS_PARAMETERS) + (i*ADCS_NUM_AXES*sizeof(val) + j*sizeof(val)), &val, sizeof(val));
    }
  }

  float biasVector[ADCS_NUM_AXES] = ADCS_IMU_BIAS_VECTOR;
  for(uint8_t i = 0; i < ADCS_NUM_AXES; i++) {
    float val = biasVector[i];
    memcpy(adcsPage + (FLASH_ADCS_IMU_BIAS_VECTOR - FLASH_ADCS_PARAMETERS) + i*sizeof(val), &val, sizeof(val));
  }

  // write the default ADCS info
  PersistentStorage_Write(FLASH_ADCS_PARAMETERS, adcsPage, 2*FLASH_EXT_PAGE_SIZE);

  // write the default ADCS controller
  memset(adcsPage, 0, FLASH_EXT_PAGE_SIZE);
  float controller[ADCS_NUM_AXES][2*ADCS_NUM_AXES] = ADCS_DEFAULT_CONTROLLER;
  memcpy(adcsPage, controller, ADCS_NUM_AXES*2*ADCS_NUM_AXES*sizeof(float));
  PersistentStorage_Write(FLASH_ADCS_CONTROLLERS, adcsPage, FLASH_EXT_PAGE_SIZE);
}

uint8_t PersistentStorage_Get_Message(uint16_t slotNum, uint8_t* buff) {
  // read the message slot
  uint8_t messageBuff[MAX_STRING_LENGTH];
  PersistentStorage_Read(FLASH_STORE_AND_FORWARD_START + slotNum*MAX_STRING_LENGTH, messageBuff, MAX_STRING_LENGTH);

  // get message length
  uint8_t messageLen = messageBuff[sizeof(uint32_t)];

  // copy the message without length and ID
  if(messageLen < MAX_STRING_LENGTH) {
    memcpy(buff, messageBuff + sizeof(uint32_t) + sizeof(uint8_t), messageLen);
  }

  return(messageLen);
}

void PersistentStorage_Set_Message(uint16_t slotNum, uint8_t* buff, uint8_t len) {
  // read the current sector
  uint8_t sectorBuff[FLASH_SECTOR_SIZE];
  uint32_t addr = FLASH_STORE_AND_FORWARD_START + slotNum*MAX_STRING_LENGTH;
  PersistentStorage_Read(addr & 0xFFFFF000, sectorBuff, FLASH_SECTOR_SIZE);

  // update buffer
  memcpy(sectorBuff + ((slotNum*MAX_STRING_LENGTH) & 0x00000FFF), buff, len);

  // write updated buffer
  PersistentStorage_Write(addr & 0xFFFFF000, sectorBuff, FLASH_SECTOR_SIZE);
  FOSSASAT_DEBUG_PRINT_FLASH(addr, FLASH_EXT_PAGE_SIZE);
}

void PersistentStorage_Set_ADCS_Ephemerides(uint32_t row, float ephemerides[2*ADCS_NUM_AXES], uint8_t controllerId) {
  // calculate address in flash
  const uint32_t rowsPerSector = (FLASH_SECTOR_SIZE / FLASH_EXT_PAGE_SIZE) * (FLASH_EXT_PAGE_SIZE / FLASH_ADCS_EPHEMERIDES_SLOT_SIZE);
  uint32_t sectorAddr = (row / rowsPerSector) + FLASH_ADCS_EPHEMERIDES_START;

  // ephemerides are stored in 128-byte chunks, 5 rows per chunk, followed by 3 free bytes
  const uint32_t rowsPerChunk = ((FLASH_EXT_PAGE_SIZE / FLASH_ADCS_EPHEMERIDES_SLOT_SIZE) / 2);
  uint32_t epheAddr = (row / rowsPerChunk) * (FLASH_EXT_PAGE_SIZE / 2) + (row % rowsPerChunk);

  // read the sector
  uint8_t sectorBuff[FLASH_SECTOR_SIZE];
  PersistentStorage_Read(sectorAddr, sectorBuff, FLASH_SECTOR_SIZE);

  // update buffer
  memcpy(sectorBuff + epheAddr, ephemerides, 2*ADCS_NUM_AXES*sizeof(float));
  sectorBuff[epheAddr + 2*ADCS_NUM_AXES*sizeof(float)] = controllerId;

  // write update buffer
  PersistentStorage_Write(sectorAddr, sectorBuff, FLASH_SECTOR_SIZE);
}

void PersistentStorage_Set_ADCS_Controller(uint8_t id, float controller[ADCS_NUM_AXES][2*ADCS_NUM_AXES]) {
  // read the controller sector
  uint8_t sectorBuff[FLASH_SECTOR_SIZE];
  PersistentStorage_Read(FLASH_ADCS_CONTROLLERS, sectorBuff, FLASH_SECTOR_SIZE);

  // update buffer
  uint32_t controllerLen = ADCS_NUM_AXES*2*ADCS_NUM_AXES*sizeof(float);
  memcpy(sectorBuff + id*controllerLen, controller, controllerLen);

  // write updated buffer
  PersistentStorage_Write(FLASH_ADCS_CONTROLLERS, sectorBuff, FLASH_SECTOR_SIZE);
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_ADCS_CONTROLLERS, FLASH_EXT_PAGE_SIZE);
}

void PersistentStorage_Read(uint32_t addr, uint8_t* buff, size_t len) {
  uint8_t cmdBuff[] = {MX25L51245G_CMD_READ, (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)};
  PersistentStorage_SPItransaction(cmdBuff, 5, false, buff, len);
}

// counter to display the number of writes to external flash
#ifdef FOSSASAT_DEBUG
uint32_t writeCtr = 0;
#endif

void PersistentStorage_Write(uint32_t addr, uint8_t* buff, size_t len, bool autoErase) {
  FOSSASAT_DEBUG_PRINT(F("Write to Flash #"));
  FOSSASAT_DEBUG_PRINT(writeCtr++);
  FOSSASAT_DEBUG_PRINT(F(", address 0x"));
  FOSSASAT_DEBUG_PRINT(addr, HEX);

  // erase requested sector
  if(autoErase) {
    FOSSASAT_DEBUG_PRINT(F(", with sector erase"));
    PersistentStorage_SectorErase(addr);
  }
  FOSSASAT_DEBUG_PRINTLN();

  // set WEL bit again
  PersistentStorage_WaitForWriteEnable();

  // check if all bytes are in the same page
  uint32_t addrInPage = (addr & 0xFF) + len;
  if(addrInPage <= FLASH_EXT_PAGE_SIZE) {
    // all bytes are in the same page, write it
    uint8_t cmdBuff[] = {MX25L51245G_CMD_PP, (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)};
    PersistentStorage_SPItransaction(cmdBuff, 5, true, buff, len);
  } else {
      // some bytes are in the following page(s)

    // get the number of bytes in the first page
    size_t firstPageLen = FLASH_EXT_PAGE_SIZE - (addr & 0xFF);

    // write the first page - might start in any position in the page
    uint32_t newAddr = addr;
    uint8_t cmdBuff[] = {MX25L51245G_CMD_PP, (uint8_t)((newAddr >> 24) & 0xFF), (uint8_t)((newAddr >> 16) & 0xFF), (uint8_t)((newAddr >> 8) & 0xFF), (uint8_t)(newAddr & 0xFF)};
    PersistentStorage_SPItransaction(cmdBuff, 5, true, buff, firstPageLen);

    // wait until page is written
    PersistentStorage_WaitForWriteInProgress();

    // set WEL bit again
    PersistentStorage_WaitForWriteEnable();

    // write the other pages - will always start at the page boundary
    uint32_t remLen = len - firstPageLen;
    uint8_t numPages = (remLen / FLASH_EXT_PAGE_SIZE) + 1;
    for(uint8_t i = 0; i < numPages; i++) {
      // get address of the next page
      newAddr = (addr & 0xFFFFFF00) + ((uint32_t)(i + 1)) * FLASH_EXT_PAGE_SIZE;
      cmdBuff[1] = (uint8_t)((newAddr >> 24) & 0xFF);
      cmdBuff[2] = (uint8_t)((newAddr >> 16) & 0xFF);
      cmdBuff[3] = (uint8_t)((newAddr >> 8) & 0xFF);
      cmdBuff[4] = (uint8_t)(newAddr & 0xFF);

      // calculate length
      uint32_t writeLen = FLASH_EXT_PAGE_SIZE;
      if(remLen < FLASH_EXT_PAGE_SIZE) {
        writeLen = remLen;
      }

      // write the buffer
      PersistentStorage_SPItransaction(cmdBuff, 5, true, buff + (len - remLen), writeLen);
      remLen -= writeLen;
    }
  }

  // wait until page is written
  PersistentStorage_WaitForWriteInProgress();
}

void PersistentStorage_SectorErase(uint32_t addr) {
  // set WEL bit
  PersistentStorage_WaitForWriteEnable();

  // erase required sector
  uint8_t cmdBuf[] = {MX25L51245G_CMD_SE, (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)};
  PersistentStorage_SPItransaction(cmdBuf, 5, false, NULL, 0);

  // wait until sector is erased
  PersistentStorage_WaitForWriteInProgress(1000);
}

void PersistentStorage_64kBlockErase(uint32_t addr) {
  // set WEL bit
  PersistentStorage_WaitForWriteEnable();

  // erase required sector
  uint8_t cmdBuf[] = {MX25L51245G_CMD_BE, (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)};
  PersistentStorage_SPItransaction(cmdBuf, 5, false, NULL, 0);

  // wait until sector is erased
  PersistentStorage_WaitForWriteInProgress(3000);
}

void PersistentStorage_WriteEnable() {
  PersistentStorage_SPItransaction(MX25L51245G_CMD_WREN);
}

void PersistentStorage_WriteDisable() {
  PersistentStorage_SPItransaction(MX25L51245G_CMD_WRDI);
}

// cppcheck-suppress unusedFunction
uint8_t PersistentStorage_ReadManufacturerID() {
  uint8_t cmdBuf[] = {MX25L51245G_CMD_REMS, 0x00, 0x00, 0x00};
  uint8_t buf[2];
  PersistentStorage_SPItransaction(cmdBuf, 4, false, buf, 2);
  return(buf[0]);
}

uint8_t PersistentStorage_ReadStatusRegister() {
  uint8_t buf[1];
  PersistentStorage_SPItransaction(MX25L51245G_CMD_RDSR, false, buf, 1);
  return(buf[0]);
}

// cppcheck-suppress unusedFunction
uint8_t PersistentStorage_ReadConfigRegister() {
  uint8_t buf[1];
  PersistentStorage_SPItransaction(MX25L51245G_CMD_RDCR, false, buf, 1);
  return(buf[0]);
}

// cppcheck-suppress unusedFunction
uint8_t PersistentStorage_ReadSecurityRegister() {
  uint8_t buf[1];
  PersistentStorage_SPItransaction(MX25L51245G_CMD_RDSCUR, false, buf, 1);
  return(buf[0]);
}

void PersistentStorage_Enter4ByteMode() {
  PersistentStorage_SPItransaction(MX25L51245G_CMD_EN4B);
}

// cppcheck-suppress unusedFunction
void PersistentStorage_Exit4ByteMode() {
  PersistentStorage_SPItransaction(MX25L51245G_CMD_EX4B);
}

void PersistentStorage_Reset() {
  pinMode(FLASH_RESET, OUTPUT);
  digitalWrite(FLASH_RESET, LOW);
  delayMicroseconds(100);
  pinMode(FLASH_RESET, INPUT);
}

// cppcheck-suppress unusedFunction
void PersistentStorage_WriteStatusRegister(uint8_t sr, uint8_t cr) {
  uint8_t buf[] = {sr, cr};
  PersistentStorage_WaitForWriteEnable();
  PersistentStorage_SPItransaction(MX25L51245G_CMD_WRSR, true, buf, 2);
  PersistentStorage_WriteDisable();
}

bool PersistentStorage_WaitForWriteEnable(uint32_t timeout) {
  // start the timer
  uint32_t start = millis();

  // repeat until WEL bit is set
  while(!(PersistentStorage_ReadStatusRegister() & MX25L51245G_SR_WEL)) {
    PersistentStorage_WriteEnable();
    delayMicroseconds(10);

    // check timeout
    if(millis() - start >= timeout) {
      return(false);
    }
  }
  return(true);
}

bool PersistentStorage_WaitForWriteInProgress(uint32_t timeout) {
  // start the timer
  uint32_t start = millis();

  // repeat as long as WIP bit is set
  while(PersistentStorage_ReadStatusRegister() & MX25L51245G_SR_WIP) {
    delayMicroseconds(10);

    // check timeout
    if(millis() - start >= timeout) {
      return(false);
    }
  }
  return(true);
}

void PersistentStorage_SPItransaction(uint8_t cmd, bool write, uint8_t* data, size_t numBytes) {
  uint8_t cmdBuf[] = {cmd};
  PersistentStorage_SPItransaction(cmdBuf, 1, write, data, numBytes);
}

void PersistentStorage_SPItransaction(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* data, size_t numBytes) {
  digitalWrite(FLASH_CS, LOW);
  FlashSPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // send command
  for(uint8_t n = 0; n < cmdLen; n++) {
    // send byte
    FlashSPI.transfer(cmd[n]);
  }

  if(data != NULL) {
    // send data
    if(write) {
      for(size_t n = 0; n < numBytes; n++) {
        // send byte
        FlashSPI.transfer(data[n]);
      }

    } else {
      for(size_t n = 0; n < numBytes; n++) {
        data[n] = FlashSPI.transfer(MX25L51245G_CMD_NOP);
      }
    }
  }

  FlashSPI.endTransaction();
  digitalWrite(FLASH_CS, HIGH);
}
