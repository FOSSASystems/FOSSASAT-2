#include "FossaSat2.h"

// compile-time checks
#if (!defined(RADIOLIB_VERSION)) || (RADIOLIB_VERSION < 0x03060200)
  #error "Unsupported RadioLib version (< 3.6.2)!"
#endif

#if (!defined(RADIOLIB_STATIC_ONLY))
  #error "RadioLib is using dynamic memory management, make sure static only is enabled in FossaSat2.h"
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
  uint16_t restartCounter = PersistentStorage_Get<uint16_t>(FLASH_RESTART_COUNTER);
  FOSSASAT_DEBUG_PORT.print(F("Restart #"));
  FOSSASAT_DEBUG_PORT.println(restartCounter++);
  PersistentStorage_Set(FLASH_RESTART_COUNTER, restartCounter);

#ifdef RESET_SYSTEM_INFO
  // reset system info (first sector in external flash)
  PersistentStorage_Reset_System_Info();
#endif

  // print system info page
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_SYSTEM_INFO_START, FLASH_EXT_PAGE_SIZE);

  // initialize current sensors
  FOSSASAT_DEBUG_PORT.println(F("Current sensors init:"));
  FOSSASAT_DEBUG_PORT.print(F("XA: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Current(currSensorXA, CURR_SENSOR_X_A_BUS, CURR_SENSOR_X_A_ADDRESS));
  FOSSASAT_DEBUG_PORT.print(F("XB: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Current(currSensorXB, CURR_SENSOR_X_B_BUS, CURR_SENSOR_X_B_ADDRESS));
  FOSSASAT_DEBUG_PORT.print(F("ZA: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Current(currSensorZA, CURR_SENSOR_Z_A_BUS, CURR_SENSOR_Z_A_ADDRESS));
  FOSSASAT_DEBUG_PORT.print(F("ZB: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Current(currSensorZB, CURR_SENSOR_Z_B_BUS, CURR_SENSOR_Z_B_ADDRESS));
  FOSSASAT_DEBUG_PORT.print(F("Y: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Current(currSensorY, CURR_SENSOR_Y_BUS, CURR_SENSOR_Y_ADDRESS));
  FOSSASAT_DEBUG_PORT.print(F("MPPT: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Current(currSensorMPPT, CURR_SENSOR_MPPT_OUTPUT_BUS, CURR_SENSOR_MPPT_OUTPUT_ADDRESS));

  // initialize radio
  FOSSASAT_DEBUG_PORT.print(F("LoRa modem init: "));
  FOSSASAT_DEBUG_PORT.println(Communication_Set_Modem(MODEM_LORA));
  FOSSASAT_DEBUG_PORT.print(F("FSK modem init:\t"));
  FOSSASAT_DEBUG_PORT.println(Communication_Set_Modem(MODEM_FSK));

  // initialize camera
  digitalWrite(CAMERA_POWER_FET, HIGH);
  FOSSASAT_DEBUG_PORT.print(F("Camera init:\t"));
  FOSSASAT_DEBUG_PORT.println(Camera_Init(p320x240, Auto, Saturation0, Brightness0, Contrast0, Normal));
  digitalWrite(CAMERA_POWER_FET, LOW);

  // initialize IMU
  FOSSASAT_DEBUG_PORT.print(F("IMU init (expected 0x683D):\t0x"));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_IMU(), HEX);

  // initialize temperature sensors
  FOSSASAT_DEBUG_PORT.println(F("Temperature sensors init:"));
  
  FOSSASAT_DEBUG_PORT.print(F("Y panel: "));
  Sensors_Setup_Temp(tempSensorPanelY, TMP_100_RESOLUTION_12_BITS);
  FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorPanelY));

  FOSSASAT_DEBUG_PORT.print(F("Top: "));
  Sensors_Setup_Temp(tempSensorTop, TMP_100_RESOLUTION_12_BITS);
  FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorTop));

  FOSSASAT_DEBUG_PORT.print(F("Bottom: "));
  Sensors_Setup_Temp(tempSensorBottom, TMP_100_RESOLUTION_12_BITS);
  FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorBottom));

  FOSSASAT_DEBUG_PORT.print(F("Battery: "));
  Sensors_Setup_Temp(tempSensorBattery, TMP_100_RESOLUTION_12_BITS);
  FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorBattery));

  FOSSASAT_DEBUG_PORT.print(F("Secondary battery: "));
  Sensors_Setup_Temp(tempSensorSecBattery, TMP_100_RESOLUTION_12_BITS);
  FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorSecBattery));
  
  // initialize light sensors
  FOSSASAT_DEBUG_PORT.println(F("Light sensors init:"));
  FOSSASAT_DEBUG_PORT.print(F("Y: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Light(lightSensorPanelY, LIGHT_SENSOR_Y_PANEL_BUS));
  FOSSASAT_DEBUG_PORT.print(F("Top: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Light(lightSensorTop, LIGHT_SENSOR_TOP_PANEL_BUS));

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
