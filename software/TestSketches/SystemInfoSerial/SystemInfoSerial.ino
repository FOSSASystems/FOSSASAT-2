#include "FossaSat2.h"

// compile-time checks
#if (!defined(RADIOLIB_VERSION)) || (RADIOLIB_VERSION < 0x04000000)
  #error "Unsupported RadioLib version (< 4.0.0)!"
#endif

#if (!defined(RADIOLIB_STATIC_ONLY))
  #error "RadioLib is using dynamic memory management, make sure static only is enabled in RadioLib/src/BuildOpt.h"
#endif

void setup() {
  // initialize debug port
  FOSSASAT_DEBUG_PORT.begin(FOSSASAT_DEBUG_SPEED);
  FOSSASAT_DEBUG_PORT.println();

  // setup hardware interfaces
  Configuration_Setup();

  // initialize external flash
  PersistentStorage_Reset();
  PersistentStorage_Enter4ByteMode();

  // increment reset counter
  uint16_t restartCounter = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_RESTART_COUNTER);
  FOSSASAT_DEBUG_PORT.print(F("Restart #"));
  FOSSASAT_DEBUG_PORT.println(restartCounter++);
  PersistentStorage_SystemInfo_Set(FLASH_RESTART_COUNTER, restartCounter);

#ifdef RESET_SYSTEM_INFO
  // reset system info (first sector in external flash)
  PersistentStorage_Reset_System_Info();
  PersistentStorage_Reset_ADCS_Params();
#endif

  // print system info page
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_SYSTEM_INFO, FLASH_EXT_PAGE_SIZE);

  // initialize current sensors
  FOSSASAT_DEBUG_PORT.println(F("Current sensors init:"));
  FOSSASAT_DEBUG_PORT.print(F("XA: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Current_Setup(currSensorXA));
  FOSSASAT_DEBUG_PORT.print(F("XB: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Current_Setup(currSensorXB));
  FOSSASAT_DEBUG_PORT.print(F("ZA: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Current_Setup(currSensorZA));
  FOSSASAT_DEBUG_PORT.print(F("ZB: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Current_Setup(currSensorZB));
  FOSSASAT_DEBUG_PORT.print(F("Y: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Current_Setup(currSensorY));
  FOSSASAT_DEBUG_PORT.print(F("MPPT: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Current_Setup(currSensorMPPT));

  // initialize radio
  FOSSASAT_DEBUG_PORT.print(F("LoRa modem init: "));
  FOSSASAT_DEBUG_PORT.println(Communication_Set_Modem(MODEM_LORA));
  FOSSASAT_DEBUG_PORT.print(F("FSK modem init:\t"));
  FOSSASAT_DEBUG_PORT.println(Communication_Set_Modem(MODEM_FSK));

  // initialize camera
  digitalWrite(CAMERA_POWER_FET, POWER_FET_POLARITY_ON);
  FOSSASAT_DEBUG_PORT.print(F("Camera init:\t"));
  FOSSASAT_DEBUG_PORT.println(Camera_Init(p320x240, Auto, Saturation0, Brightness0, Contrast0, Normal));
  digitalWrite(CAMERA_POWER_FET, POWER_FET_POLARITY_OFF);

  // initialize IMU
  FOSSASAT_DEBUG_PORT.print(F("IMU init (expected 0x683D):\t0x"));
  FOSSASAT_DEBUG_PORT.println(Sensors_IMU_Setup(), HEX);

  // initialize temperature sensors
  FOSSASAT_DEBUG_PORT.println(F("Temperature sensors init:"));
  
  FOSSASAT_DEBUG_PORT.print(F("Y panel: "));
  Sensors_Temperature_Setup(tempSensorPanelY);
  FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorPanelY));

  FOSSASAT_DEBUG_PORT.print(F("Top: "));
  Sensors_Temperature_Setup(tempSensorTop);
  FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorTop));

  FOSSASAT_DEBUG_PORT.print(F("Bottom: "));
  Sensors_Temperature_Setup(tempSensorBottom);
  FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorBottom));

  FOSSASAT_DEBUG_PORT.print(F("Battery: "));
  Sensors_Temperature_Setup(tempSensorBattery);
  FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorBattery));

  FOSSASAT_DEBUG_PORT.print(F("Secondary battery: "));
  Sensors_Temperature_Setup(tempSensorSecBattery);
  FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorSecBattery));

  FOSSASAT_DEBUG_PORT.print(F("MCU: "));
  Sensors_Temperature_Setup(tempSensorMCU);
  FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorMCU));
  
  // initialize light sensors
  FOSSASAT_DEBUG_PORT.println(F("Light sensors init:"));
  FOSSASAT_DEBUG_PORT.print(F("Y: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Light(lightSensorPanelY));
  FOSSASAT_DEBUG_PORT.print(F("Top: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Light(lightSensorTop));

  // initialize H-bridges
  FOSSASAT_DEBUG_PORT.println(F("H-bridges init:"));
  bridgeX.begin();
  bridgeX.stop();
  FOSSASAT_DEBUG_PORT.print(F("X: "));
  FOSSASAT_DEBUG_PORT.println(bridgeX.getFault());
  bridgeY.begin();
  bridgeY.stop();
  FOSSASAT_DEBUG_PORT.print(F("Y: "));
  FOSSASAT_DEBUG_PORT.println(bridgeY.getFault());
  bridgeZ.begin();
  bridgeZ.stop();
  FOSSASAT_DEBUG_PORT.print(F("Z: "));
  FOSSASAT_DEBUG_PORT.println(bridgeZ.getFault());
}

void loop() {
  Communication_Send_Full_System_Info();
  PowerControl_Watchdog_Heartbeat();
}
