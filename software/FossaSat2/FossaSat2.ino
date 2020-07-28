#include "FossaSat2.h"

// compile-time checks
#if (!defined(RADIOLIB_VERSION)) || (RADIOLIB_VERSION < 0x04000000)
  #error "Unsupported RadioLib version (< 4.0.0)!"
#endif

#if (!defined(RADIOLIB_STATIC_ONLY))
  #error "RadioLib is using dynamic memory management, make sure static only is enabled in RadioLib/src/BuildOpt.h"
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

#ifdef RESET_SYSTEM_INFO
  PersistentStorage_Reset_System_Info();
  PersistentStorage_Reset_ADCS_Params();
#endif

  // load system info page
  PersistentStorage_Read(FLASH_SYSTEM_INFO, systemInfoBuffer, FLASH_EXT_PAGE_SIZE);

  // increment reset counter
  uint16_t restartCounter = PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_RESTART_COUNTER);
  FOSSASAT_DEBUG_PORT.print(F("Restart #"));
  FOSSASAT_DEBUG_PORT.println(restartCounter++);
  PersistentStorage_SystemInfo_Set(FLASH_RESTART_COUNTER, restartCounter);

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

#ifdef RESET_SYSTEM_INFO
  // reset stats
  PersistentStorage_Reset_Stats();
#endif

  // check deployment
#ifdef ENABLE_DEPLOYMENT_SEQUENCE
  uint8_t attemptNumber = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_DEPLOYMENT_COUNTER);
  FOSSASAT_DEBUG_PORT.print(F("Deployment attempt #"));
  FOSSASAT_DEBUG_PORT.println(attemptNumber);

  // check number of deployment attempts
  if (attemptNumber == 0) {
    // integration, reset system info
    PersistentStorage_Reset_System_Info();
    PersistentStorage_Reset_ADCS_Params();
    PersistentStorage_Reset_Stats();

    // print data for integration purposes (independently of FOSSASAT_DEBUG macro!)
    uint32_t start = millis();
    uint32_t lastSample = 0;
    while (millis() - start <= (uint32_t)DEPLOYMENT_DEBUG_LENGTH * (uint32_t)1000) {
      // update IMU
      Sensors_IMU_Update();

      // check if its time for next measurement
      if (millis() - lastSample >= (uint32_t)DEPLOYMENT_DEBUG_SAMPLE_PERIOD) {
        lastSample = millis();
        FOSSASAT_DEBUG_PORT.println();

        // temperature sensors
        FOSSASAT_DEBUG_PORT.println(F("Device\t\tT [deg. C]"));
        FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
        FOSSASAT_DEBUG_PORT.print(F("Y panel \t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorPanelY));
        FOSSASAT_DEBUG_PORT.print(F("Top panel\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorTop));
        FOSSASAT_DEBUG_PORT.print(F("Bottom panel\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorBottom));
        FOSSASAT_DEBUG_PORT.print(F("Battery \t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorBattery));
        FOSSASAT_DEBUG_PORT.print(F("Second Battery\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorSecBattery));
        FOSSASAT_DEBUG_PORT.print(F("MCU\t\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Temperature_Read(tempSensorMCU));
        FOSSASAT_DEBUG_PORT.println();

        // IMU
        float val[3];
        FOSSASAT_DEBUG_PORT.println(F("Axis\t\tX\tY\tZ"));
        FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
        FOSSASAT_DEBUG_PORT.print(F("omega [rad./s]\t"));
        Sensors_IMU_CalcGyro(imu.gx, imu.gy, imu.gz, FLASH_IMU_OFFSET_GYRO_X, val);
        FOSSASAT_DEBUG_PORT.print(val[0]);
        FOSSASAT_DEBUG_PORT.print('\t');
        FOSSASAT_DEBUG_PORT.print(val[1]);
        FOSSASAT_DEBUG_PORT.print('\t');
        FOSSASAT_DEBUG_PORT.println(val[2]);
        FOSSASAT_DEBUG_PORT.print(F("a [m/s^2]\t"));
        Sensors_IMU_CalcAccel(imu.ax, imu.ay, imu.az, FLASH_IMU_OFFSET_ACCEL_X, val);
        FOSSASAT_DEBUG_PORT.print(val[0]);
        FOSSASAT_DEBUG_PORT.print('\t');
        FOSSASAT_DEBUG_PORT.print(val[1]);
        FOSSASAT_DEBUG_PORT.print('\t');
        FOSSASAT_DEBUG_PORT.println(val[2]);
        FOSSASAT_DEBUG_PORT.print(F("B [tesla]\t"));
        Sensors_IMU_CalcMag(imu.mx, imu.my, imu.mz, FLASH_IMU_OFFSET_MAG_X, val);
        FOSSASAT_DEBUG_PORT.print(val[0]);
        FOSSASAT_DEBUG_PORT.print('\t');
        FOSSASAT_DEBUG_PORT.print(val[1]);
        FOSSASAT_DEBUG_PORT.print('\t');
        FOSSASAT_DEBUG_PORT.println(val[2]);
        FOSSASAT_DEBUG_PORT.println();

        // current sensors
        FOSSASAT_DEBUG_PORT.println(F("Device\t\tI [mA]\t\tV [mV]"));
        FOSSASAT_DEBUG_PORT.println(F("-------------------------------------------------------------"));
        FOSSASAT_DEBUG_PORT.print(F("X panel A\t"));
        FOSSASAT_DEBUG_PORT.print(Sensors_Current_Read(currSensorXA));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Current_ReadVoltage(currSensorXA));
        FOSSASAT_DEBUG_PORT.print(F("X panel B\t"));
        FOSSASAT_DEBUG_PORT.print(Sensors_Current_Read(currSensorXB));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Current_ReadVoltage(currSensorXB));

        FOSSASAT_DEBUG_PORT.print(F("Z panel A\t"));
        FOSSASAT_DEBUG_PORT.print(Sensors_Current_Read(currSensorZA));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Current_ReadVoltage(currSensorZA));
        FOSSASAT_DEBUG_PORT.print(F("Z panel B\t"));
        FOSSASAT_DEBUG_PORT.print(Sensors_Current_Read(currSensorZB));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Current_ReadVoltage(currSensorZB));

        FOSSASAT_DEBUG_PORT.print(F("Y panel \t"));
        FOSSASAT_DEBUG_PORT.print(Sensors_Current_Read(currSensorY));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Current_ReadVoltage(currSensorY));

        FOSSASAT_DEBUG_PORT.print(F("MPPT output\t"));
        FOSSASAT_DEBUG_PORT.print(Sensors_Current_Read(currSensorMPPT));
        FOSSASAT_DEBUG_PORT.print(F("\t\t"));
        FOSSASAT_DEBUG_PORT.println(Sensors_Current_ReadVoltage(currSensorMPPT));
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
    FOSSASAT_DEBUG_PORT.println(F("================================"));
    FOSSASAT_DEBUG_PORT.println(F("===  Integration loop done   ==="));
    FOSSASAT_DEBUG_PORT.println(F("=== \"We'll fly what we have\" ==="));
    FOSSASAT_DEBUG_PORT.println(F("================================"));
    attemptNumber++;
    PersistentStorage_SystemInfo_Set(FLASH_DEPLOYMENT_COUNTER, attemptNumber);
    PersistentStorage_Set_Buffer(FLASH_SYSTEM_INFO, systemInfoBuffer, FLASH_EXT_PAGE_SIZE);
    PowerControl_Wait(DEPLOYMENT_SLEEP_LENGTH, LOW_POWER_SLEEP);

  } else if(attemptNumber <= 3) {
    // mid-flight reset, deploy

    // sleep before deployment
#ifdef ENABLE_DEPLOYMENT_SLEEP
    PowerControl_Wait(DEPLOYMENT_SLEEP_LENGTH, LOW_POWER_SLEEP);
#endif

    // check voltage
#ifdef ENABLE_DEPLOYMENT_CHARGING
    uint32_t chargingStart = millis();
    int16_t voltageLimit = PersistentStorage_SystemInfo_Get<int16_t>(FLASH_DEPLOYMENT_BATTERY_VOLTAGE_LIMIT);
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
    PersistentStorage_SystemInfo_Set(FLASH_DEPLOYMENT_COUNTER, attemptNumber);
  }
#endif

  // update system info flash page
  PersistentStorage_Set_Buffer(FLASH_SYSTEM_INFO, systemInfoBuffer, FLASH_EXT_PAGE_SIZE);
}

// cppcheck-suppress unusedFunction
void loop() {
  // load system info page
  PersistentStorage_Read(FLASH_SYSTEM_INFO, systemInfoBuffer, FLASH_EXT_PAGE_SIZE);

  // check CRC
  if(!PersistentStorage_Check_CRC(systemInfoBuffer, FLASH_SYSTEM_INFO_CRC)) {
    FOSSASAT_DEBUG_PRINTLN(F("CRC check failed!"));
  }

  // check RTC time
  if(!rtc.isTimeSet()) {
    FOSSASAT_DEBUG_PRINTLN(F("RTC time not set, restoring last saved epoch"));
    rtc.setEpoch(PersistentStorage_SystemInfo_Get<uint32_t>(FLASH_RTC_EPOCH));
  } else {
    uint32_t rtcEpoch = rtc.getEpoch();
    memcpy(systemInfoBuffer + FLASH_RTC_EPOCH, &rtcEpoch, sizeof(rtcEpoch));
  }
  FOSSASAT_DEBUG_PRINT(F("On-board time: "));
  FOSSASAT_DEBUG_PRINT_RTC_TIME();

  // check battery voltage
  FOSSASAT_DEBUG_PRINT(F("Battery check: "));
  float battVoltage = PowerControl_Get_Battery_Voltage();
  FOSSASAT_DEBUG_PRINTLN(battVoltage, 2);
  PowerControl_Manage_Battery();
  FOSSASAT_DEBUG_PRINT_FLASH(FLASH_SYSTEM_INFO, FLASH_EXT_PAGE_SIZE)

  // update all stats when not in low power mode
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) == LOW_POWER_NONE) {
    PersistentStorage_Update_Stats(0xFF);
  }
  #else
  PersistentStorage_Update_Stats(0xFF);
  #endif

  // CW beacon
  Communication_Set_Modem(MODEM_FSK);
  FOSSASAT_DEBUG_DELAY(10);

  // get loop number
  uint8_t numLoops = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOOP_COUNTER);

  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED) == 0) {
    FOSSASAT_DEBUG_PRINTLN(F("Tx off by cmd"));
  } else {
    if((battVoltage >= PersistentStorage_SystemInfo_Get<int16_t>(FLASH_BATTERY_CW_BEEP_VOLTAGE_LIMIT)) && (numLoops % MORSE_BEACON_LOOP_FREQ == 0)) {
    // transmit full Morse beacon
  #endif

    Communication_Send_Morse_Beacon(battVoltage);

  #ifdef ENABLE_TRANSMISSION_CONTROL
    } else {
      // set delay between beeps according to battery voltage
      uint32_t delayLen = battVoltage - MORSE_BATTERY_MIN;
      if(battVoltage < MORSE_BATTERY_MIN + MORSE_BATTERY_STEP) {
        delayLen = MORSE_BATTERY_STEP;
      }

      // transmit the beeps
      for(uint8_t i = 0; i < NUM_CW_BEEPS; i++) {
        Communication_CW_Beep(500);
        PowerControl_Wait(delayLen, LOW_POWER_NONE);
      }
    }
  }
  #endif

  // send FSK system info
  Communication_Set_Modem(MODEM_FSK);
  Communication_Send_Full_System_Info();

  // send stats too (if it's enabled)
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_AUTO_STATISTICS) == 1) {
    Communication_Send_Statistics(0xFF);
  }

  // send LoRa system info if not in low power mode
  Communication_Set_Modem(MODEM_LORA);
  #ifdef ENABLE_TRANSMISSION_CONTROL
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) == LOW_POWER_NONE) {
    Communication_Send_Basic_System_Info();
  }
  #else
    Communication_Send_Basic_System_Info();
  #endif

  // LoRa receive
  uint8_t windowLenLoRa = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LORA_RECEIVE_LEN);
  FOSSASAT_DEBUG_PRINT(F("LoRa Rx "));
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
    // use only half of the interval in low power mode
    windowLenLoRa /= 2;
    FOSSASAT_DEBUG_PRINT(F("(halved due to LP mode) "));
  }
  FOSSASAT_DEBUG_PRINTLN(windowLenLoRa);
  FOSSASAT_DEBUG_DELAY(100);
  radio.startReceive();

  uint32_t loraStart = rtc.getEpoch();
  while(rtc.getEpoch() - loraStart < windowLenLoRa) {
    PowerControl_Wait(1000, LOW_POWER_SLEEP);
    Communication_Check_New_Packet();
  }

  // GFSK receive
  uint8_t windowLenFsk = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_FSK_RECEIVE_LEN);
  Communication_Set_Modem(MODEM_FSK);
  FOSSASAT_DEBUG_PRINT(F("FSK Rx "));
  if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) != LOW_POWER_NONE) {
    // use only half of the interval in low power mode
    windowLenFsk /= 2;
    FOSSASAT_DEBUG_PRINT(F("(halved due to LP mode) "));
  }
  FOSSASAT_DEBUG_PRINTLN(windowLenFsk);
  FOSSASAT_DEBUG_DELAY(100);
  radio.startReceive();

  uint32_t gfskStart = rtc.getEpoch();
  while(rtc.getEpoch() - gfskStart < windowLenFsk) {
    PowerControl_Wait(1000, LOW_POWER_SLEEP);
    Communication_Check_New_Packet();
  }

  radio.standby();

  // update saved epoch
  uint32_t rtcEpoch = rtc.getEpoch();
  memcpy(systemInfoBuffer + FLASH_RTC_EPOCH, &rtcEpoch, sizeof(rtcEpoch));

  // update loop counter
  numLoops++;
  memcpy(systemInfoBuffer + FLASH_LOOP_COUNTER, &numLoops, sizeof(numLoops));

  // update system info flash page
  PersistentStorage_Set_Buffer(FLASH_SYSTEM_INFO, systemInfoBuffer, FLASH_EXT_PAGE_SIZE);

  // set everything to sleep
  uint32_t interval = PowerControl_Get_Sleep_Interval();
  FOSSASAT_DEBUG_PRINT(F("Sleep for "));
  FOSSASAT_DEBUG_PRINTLN(interval);
  FOSSASAT_DEBUG_DELAY(10);
  PowerControl_Wait(interval, LOW_POWER_SLEEP, true);
}
