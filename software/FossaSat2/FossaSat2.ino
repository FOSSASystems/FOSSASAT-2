#include "FossaSat2.h"

// compile-time checks
#if (!defined(RADIOLIB_VERSION)) || (RADIOLIB_VERSION < 0x03060200)
  #error "Unsupported RadioLib version (< 3.6.2)!"
#endif

#if (!defined(RADIOLIB_STATIC_ONLY))
  #error "RadioLib is using dynamic memory management, make sure static only is enabled in FossaSat2.h"
#endif

// cppcheck-suppress unusedFunction
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

#ifdef RESET_SYSTEM_INFO
  // reset stats
  PersistentStorage_Reset_Stats();
#endif

  // check deployment
#ifdef ENABLE_DEPLOYMENT_SEQUENCE
  uint8_t attemptNumber = PersistentStorage_Get<uint8_t>(FLASH_DEPLOYMENT_COUNTER);
  FOSSASAT_DEBUG_PORT.print(F("Deployment attempt #"));
  FOSSASAT_DEBUG_PORT.println(attemptNumber);

  // check number of deployment attempts
  if (attemptNumber == 0) {
    // integration, reset system info
    PersistentStorage_Reset_System_Info();
    
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
    
    // print data for integration purposes (independently of FOSSASAT_DEBUG macro!)
    start = millis();
    uint32_t lastSample = 0;
    while (millis() - start <= (uint32_t)DEPLOYMENT_DEBUG_LENGTH * (uint32_t)1000) {
      // update IMU
      Sensors_Update_IMU();

      // check if its time for next measurement
      if (millis() - lastSample >= (uint32_t)DEPLOYMENT_DEBUG_SAMPLE_PERIOD) {
        lastSample = millis();
        FOSSASAT_DEBUG_PORT.println();

        // temperature sensors
        FOSSASAT_DEBUG_PORT.println(F("Device\t\tT [deg. C]"));
        FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
        FOSSASAT_DEBUG_PORT.print(F("Y panel \t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorPanelY));
        FOSSASAT_DEBUG_PORT.print(F("Top panel\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorTop));
        FOSSASAT_DEBUG_PORT.print(F("Bottom panel\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorBottom));
        FOSSASAT_DEBUG_PORT.print(F("Battery \t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorBattery));
        FOSSASAT_DEBUG_PORT.print(F("Second Battery\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Read_Temperature(tempSensorSecBattery));
        FOSSASAT_DEBUG_PORT.println();

        // IMU
        FOSSASAT_DEBUG_PORT.println(F("Device\t\tomega [deg./s]\ta [m/s^2]\tB [gauss]"));
        FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
        FOSSASAT_DEBUG_PORT.print(F("X axis\t\t"));
        FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gx));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.ax));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(imu.calcMag(imu.mx));
        FOSSASAT_DEBUG_PORT.print(F("Y axis\t\t"));
        FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gy));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.ay));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(imu.calcMag(imu.my));
        FOSSASAT_DEBUG_PORT.print(F("Z axis\t\t"));
        FOSSASAT_DEBUG_PORT.print(imu.calcGyro(imu.gz));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.print(imu.calcAccel(imu.az));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(imu.calcMag(imu.mz));
        FOSSASAT_DEBUG_PORT.println();

        // current sensors
        FOSSASAT_DEBUG_PORT.println(F("Device\t\tI [mA]\t\tV [mV]"));
        FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
        FOSSASAT_DEBUG_PORT.print(F("X panel A\t"));
        FOSSASAT_DEBUG_PORT.print(currSensorXA.readCurrent());
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(currSensorXA.readBusVoltage());
        FOSSASAT_DEBUG_PORT.print(F("X panel B\t"));
        FOSSASAT_DEBUG_PORT.print(currSensorXB.readCurrent());
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(currSensorXB.readBusVoltage());

        FOSSASAT_DEBUG_PORT.print(F("Z panel A\t"));
        FOSSASAT_DEBUG_PORT.print(currSensorZA.readCurrent());
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(currSensorZA.readBusVoltage());
        FOSSASAT_DEBUG_PORT.print(F("Z panel B\t"));
        FOSSASAT_DEBUG_PORT.print(currSensorZB.readCurrent());
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(currSensorZB.readBusVoltage());

        FOSSASAT_DEBUG_PORT.print(F("Y panel \t"));
        FOSSASAT_DEBUG_PORT.print(currSensorY.readCurrent());
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(currSensorY.readBusVoltage());

        FOSSASAT_DEBUG_PORT.print(F("MPPT output\t"));
        FOSSASAT_DEBUG_PORT.print(currSensorMPPT.readCurrent());
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(currSensorMPPT.readBusVoltage());
        FOSSASAT_DEBUG_PORT.println();

        // light sensors
        FOSSASAT_DEBUG_PORT.println(F("Device\t\tE [lx]"));
        FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
        FOSSASAT_DEBUG_PORT.print(F("Y panel \t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Read_Light(lightSensorPanelY));
        FOSSASAT_DEBUG_PORT.print(F("Top panel\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Read_Light(lightSensorTop));
        FOSSASAT_DEBUG_PORT.println();

        // ADCS H-bridge drivers
        FOSSASAT_DEBUG_PORT.println(F("Device\t\tFault #"));
        FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
        FOSSASAT_DEBUG_PORT.print(F("X axis \t"));
        FOSSASAT_DEBUG_PORT.println(bridgeX.getFault());
        FOSSASAT_DEBUG_PORT.print(F("Y axis \t"));
        FOSSASAT_DEBUG_PORT.println(bridgeY.getFault());
        FOSSASAT_DEBUG_PORT.print(F("Z axis \t"));
        FOSSASAT_DEBUG_PORT.println(bridgeZ.getFault());

        FOSSASAT_DEBUG_PORT.println(F("============================================================="));

        // pet watchdog
        PowerControl_Watchdog_Heartbeat(false);
      }
    }

    // increment deployment counter
    attemptNumber++;
    PersistentStorage_Set(FLASH_DEPLOYMENT_COUNTER, attemptNumber);

  } else if(attemptNumber <= 3) {
    // mid-flight reset, deploy
  
    // sleep before deployment
#ifdef ENABLE_DEPLOYMENT_SLEEP
    PowerControl_Wait(DEPLOYMENT_SLEEP_LENGTH, LOW_POWER_SLEEP);
#endif

    // check voltage
#ifdef ENABLE_DEPLOYMENT_CHARGING
    uint32_t chargingStart = millis();
    int16_t voltageLimit = PersistentStorage_Get<int16_t>(FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT);
    while (PowerControl_Get_Battery_Voltage() < voltageLimit) {
      // voltage below 3.7V, wait until charged
      if (millis() - chargingStart >= (uint32_t)DEPLOYMENT_CHARGE_LIMIT * (uint32_t)3600 * (uint32_t)1000) {
        // reached maximum charging interval, stop further charging
        break;
      }

      // sleep for variable amount of time
      uint32_t interval = PowerControl_Get_Sleep_Interval();
      FOSSASAT_DEBUG_PRINT(F("Sleep for "));
      FOSSASAT_DEBUG_PRINTLN(interval);
      FOSSASAT_DEBUG_DELAY(10);
      PowerControl_Wait(interval, LOW_POWER_SLEEP, false);
    }
#endif
    // voltage above 3.7V, deploy
    PowerControl_Deploy();

    // increment deployment counter
    attemptNumber++;
    PersistentStorage_Set(FLASH_DEPLOYMENT_COUNTER, attemptNumber);
  }
#endif
}

// cppcheck-suppress unusedFunction
void loop() {
  // check RTC time
  if(!rtc.isTimeSet()) {
    FOSSASAT_DEBUG_PRINTLN(F("RTC time not set, restoring last saved epoch"));
    rtc.setEpoch(PersistentStorage_Get<uint32_t>(FLASH_RTC_EPOCH));
  } else {
    PersistentStorage_Set<uint32_t>(FLASH_RTC_EPOCH, rtc.getEpoch());
  }
  FOSSASAT_DEBUG_PRINT(F("On-board time: "));
  FOSSASAT_DEBUG_PRINT_RTC_TIME();
  
  // check battery voltage
  FOSSASAT_DEBUG_PRINT(F("Battery check: "));
  float battVoltage = PowerControl_Get_Battery_Voltage();
  FOSSASAT_DEBUG_PRINTLN(battVoltage, 2);
  PowerControl_Manage_Battery();
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_SYSTEM_INFO_START, FLASH_EXT_PAGE_SIZE)

  // update all stats when not in low power mode
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) == LOW_POWER_NONE) {
    PersistentStorage_Update_Stats(0xFF);
  }
  #else
  PersistentStorage_Update_Stats(0xFF);
  #endif

  // CW beacon
  Communication_Set_Modem(MODEM_FSK);
  FOSSASAT_DEBUG_DELAY(10);
  
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED) == 0) {
    FOSSASAT_DEBUG_PRINTLN(F("Tx off by cmd"));
  } else {
    if(battVoltage >= PersistentStorage_Get<int16_t>(FLASH_BATTERY_CW_BEEP_VOLTAGE_LIMIT)) {
    // transmit full Morse beacon
  #endif
  
    Communication_Send_Morse_Beacon(battVoltage);
  
  #ifdef ENABLE_TRANSMISSION_CONTROL
    } else {
      // battery is low, transmit CW beeps
      for(uint8_t i = 0; i < NUM_CW_BEEPS; i++) {
        Communication_CW_Beep(500);
        PowerControl_Wait(1000, LOW_POWER_SLEEP);
      }
    }
  }
  #endif

  // send FSK system info
  Communication_Set_Modem(MODEM_FSK);
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) == LOW_POWER_NONE) {
    Communication_Send_Full_System_Info();

    // send stats too (if it's enabled)
    if(PersistentStorage_Get<uint8_t>(FLASH_AUTO_STATISTICS) == 1) {
      Communication_Send_Statistics(0xFF);
    }
    
  } else {
    // send only basic info in low power mode
    Communication_Send_Basic_System_Info();
  }
  #else
    Communication_Send_Full_System_Info();
    Communication_Send_Statistics(0xFF);
  #endif

  // send LoRa system info if not in low power mode
  Communication_Set_Modem(MODEM_LORA);
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) == LOW_POWER_NONE) {
    Communication_Send_Basic_System_Info();
  }
  #else
    Communication_Send_Basic_System_Info();
  #endif

  // LoRa receive
  uint8_t windowLenLoRa = PersistentStorage_Get<uint8_t>(FLASH_LORA_RECEIVE_LEN);
  FOSSASAT_DEBUG_PRINT(F("LoRa Rx "));
  if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
    // use only half of the interval in low power mode
    windowLenLoRa /= 2;
    FOSSASAT_DEBUG_PRINT(F("(halved due to LP mode) "));
  }
  FOSSASAT_DEBUG_PRINTLN(windowLenLoRa);
  FOSSASAT_DEBUG_DELAY(100);
  radio.startReceive();

  for(uint8_t i = 0; i < windowLenLoRa; i++) {
    PowerControl_Wait(1000, LOW_POWER_SLEEP);
    if(digitalRead(RADIO_DIO1)) {
      radio.standby();
      Communication_Process_Packet();
      radio.startReceive();
    }
  }

  // GFSK receive
  uint8_t windowLenFsk = PersistentStorage_Get<uint8_t>(FLASH_FSK_RECEIVE_LEN);
  Communication_Set_Modem(MODEM_FSK);
  FOSSASAT_DEBUG_PRINT(F("FSK Rx "));
  if(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
    // use only half of the interval in low power mode
    windowLenFsk /= 2;
    FOSSASAT_DEBUG_PRINT(F("(halved due to LP mode) "));
  }
  FOSSASAT_DEBUG_PRINTLN(windowLenFsk);
  FOSSASAT_DEBUG_DELAY(100);
  radio.startReceive();

  for(uint8_t i = 0; i < windowLenFsk; i++) {
    PowerControl_Wait(1000, LOW_POWER_SLEEP);
    if(digitalRead(RADIO_DIO1)) {
      radio.standby();
      Communication_Process_Packet();
      radio.startReceive();
    }
  }

  radio.standby();

  // update saved epoch
  PersistentStorage_Set<uint32_t>(FLASH_RTC_EPOCH, rtc.getEpoch());

  // set everything to sleep
  uint32_t interval = PowerControl_Get_Sleep_Interval();
  FOSSASAT_DEBUG_PRINT(F("Sleep for "));
  FOSSASAT_DEBUG_PRINTLN(interval);
  FOSSASAT_DEBUG_DELAY(10);
  PowerControl_Wait(interval, LOW_POWER_SLEEP, true);
}
