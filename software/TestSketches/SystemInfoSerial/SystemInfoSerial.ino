#include "FossaSat2.h"

// Atom arduino-upload build configuration:
// STM32:stm32:Nucleo_64:pnum=NUCLEO_L452REP

// compile-time checks
#if (!defined RADIOLIB_VERSION) || (RADIOLIB_VERSION < 0x03020300)
  #error "Unsupported RadioLib version (< 3.2.3)!"
#endif

#if (!defined(RADIOLIB_STATIC_ONLY))
  #error "RadioLib is using dynamic memory management, enable static only in RadioLib/src/TypeDef.h"
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
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_SYSTEM_INFO_START, 0x50);

  // initialize radio
  FOSSASAT_DEBUG_PORT.print(F("LoRa modem init: "));
  FOSSASAT_DEBUG_PORT.println(Communication_Set_Modem(MODEM_LORA));
  FOSSASAT_DEBUG_PORT.print(F("FSK modem init:\t"));
  FOSSASAT_DEBUG_PORT.println(Communication_Set_Modem(MODEM_FSK));

  // initialize camera
  digitalWrite(CAMERA_POWER_FET, HIGH);
  FOSSASAT_DEBUG_PORT.print(F("Camera init:\t"));
  FOSSASAT_DEBUG_PORT.println(Camera_Init(OV2640_320x240, Auto, Saturation0, Brightness0, Contrast0, Normal));
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
  
  // initialize light sensors
  FOSSASAT_DEBUG_PORT.println(F("Light sensors init:"));
  FOSSASAT_DEBUG_PORT.print(F("Y: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Light(lightSensorPanelY, LIGHT_SENSOR_Y_PANEL_BUS));
  FOSSASAT_DEBUG_PORT.print(F("Top: "));
  FOSSASAT_DEBUG_PORT.println(Sensors_Setup_Light(lightSensorTop, LIGHT_SENSOR_TOP_PANEL_BUS));

  // initialize H-bridges
  FOSSASAT_DEBUG_PORT.println(F("H-bridges init:"));
  bridgeX.begin();
  FOSSASAT_DEBUG_PORT.print(F("X: "));
  FOSSASAT_DEBUG_PORT.println(bridgeX.getFault());
  bridgeY.begin();
  FOSSASAT_DEBUG_PORT.print(F("Y: "));
  FOSSASAT_DEBUG_PORT.println(bridgeY.getFault());
  bridgeZ.begin();
  FOSSASAT_DEBUG_PORT.print(F("Z: "));
  FOSSASAT_DEBUG_PORT.println(bridgeZ.getFault());
  
  // ask for RTC configuration
  FOSSASAT_DEBUG_PORT.print(F("Do you wish to update RTC time to: ("));
  FOSSASAT_DEBUG_PORT.print(RTC_WEEKDAY);
  FOSSASAT_DEBUG_PORT.print(") ");
  FOSSASAT_DEBUG_PORT.print(RTC_DAY);
  FOSSASAT_DEBUG_PORT.print(". ");
  FOSSASAT_DEBUG_PORT.print(RTC_MONTH);
  FOSSASAT_DEBUG_PORT.print(". 20");
  FOSSASAT_DEBUG_PORT.print(RTC_YEAR);
  FOSSASAT_DEBUG_PORT.print(' ');
  FOSSASAT_DEBUG_PORT.print(RTC_HOURS);
  FOSSASAT_DEBUG_PORT.print(':');
  FOSSASAT_DEBUG_PORT.print(RTC_MINUTES);
  FOSSASAT_DEBUG_PORT.print(':');
  FOSSASAT_DEBUG_PORT.print(RTC_SECONDS);
  FOSSASAT_DEBUG_PORT.println('?');
  
  FOSSASAT_DEBUG_PORT.println(F(" - 'y' (lower case) to update"));
  FOSSASAT_DEBUG_PORT.println(F(" - anything else to skip or wait 10 minutes"));
  delay(10);

  uint32_t start = millis();
  while(!FOSSASAT_DEBUG_PORT.available()) {
    PowerControl_Watchdog_Heartbeat(false);
    delay(1000);
    if(millis() - start >= (uint32_t)600000) {
      FOSSASAT_DEBUG_PORT.println(F("No input for 10 minutes, skipping"));
      break;
    }
  }

  char c = FOSSASAT_DEBUG_PORT.read();
  if(c == 'y') {
    rtc.setDate(RTC_WEEKDAY, RTC_DAY, RTC_MONTH, RTC_YEAR);
    rtc.setTime(RTC_HOURS, RTC_MINUTES, RTC_SECONDS);
    PersistentStorage_Set<uint32_t>(FLASH_RTC_EPOCH, rtc.getEpoch());
  }

  while(FOSSASAT_DEBUG_PORT.available()) {
    FOSSASAT_DEBUG_PORT.read();
  }
    
}

void loop() {
  Communication_Send_Full_System_Info();
  PowerControl_Watchdog_Heartbeat();
}
