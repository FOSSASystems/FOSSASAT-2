#include "PowerControl.h"

uint32_t PowerControl_Get_Sleep_Interval() {
  // sleep interval in ms
  uint16_t interval = 0;

  #ifdef ENABLE_INTERVAL_CONTROL
    // get battery voltage
    int16_t batt = PowerControl_Get_Battery_Voltage() * 1000.0;

    // get number of intervals
    uint8_t numIntervals = systemInfoBuffer[FLASH_NUM_SLEEP_INTERVALS];

    // get the applicable interval
    uint8_t intervalSize = sizeof(int16_t) + sizeof(uint16_t);
    bool intervalFound = false;
    for(uint8_t i = 0; i < numIntervals; i++) {
      int16_t voltage = 0;
      memcpy(&voltage, systemInfoBuffer + FLASH_SLEEP_INTERVALS + i*intervalSize, sizeof(int16_t));
      if(batt > voltage) {
        memcpy(&interval, systemInfoBuffer + FLASH_SLEEP_INTERVALS + sizeof(int16_t) + i*intervalSize, sizeof(uint16_t));
        intervalFound = true;
        break;
      }
    }

    // check if interval was found
    if(!intervalFound) {
      // set to last inteval length in that case
      memcpy(&interval, systemInfoBuffer + FLASH_SLEEP_INTERVALS + sizeof(int16_t) + (numIntervals - 1)*intervalSize, sizeof(uint16_t));
    }
  #endif

  return((uint32_t)interval * (uint32_t)1000);
}

void PowerControl_Wait(uint32_t ms, uint8_t type, bool radioSleep) {
  if (ms == 0) {
    return;
  }

  // override sleep mode while ADCS is active
  if(adcsState.active) {
    type = LOW_POWER_NONE;
  }

  // calculate number of required loops (rounded up)
  float stepSize = 1000.0;
  if (type == LOW_POWER_NONE) {
    // 50 ms steps when no sleep mode is active
    stepSize = 50.0;
  }
  float numLoops = 0.5 + (float)ms / stepSize;

  // set radio to sleep
  if(radioSleep) {
    radio.sleep();
  }

  // perform all loops
  for (uint32_t i = 0; i < (uint32_t)numLoops; i++) {
    PowerControl_Watchdog_Heartbeat();
    switch(type) {
      case LOW_POWER_NONE:
        delay((uint32_t)stepSize);
        break;
      case LOW_POWER_IDLE:
        LowPower.idle((uint32_t)stepSize);
        break;
      case LOW_POWER_SLEEP:
        LowPower.sleep((uint32_t)stepSize);
        break;
      case LOW_POWER_DEEP_SLEEP:
        LowPower.deepSleep((uint32_t)stepSize);
        break;
      default:
        return;
    }
  }

  // wake up radio
  if(radioSleep) {
    radio.standby();
  }
}

void PowerControl_Watchdog_Heartbeat(bool manageBattery) {
  // toggle watchdog pin
  digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));

  // manage battery (low power, heater, charging)
  if(manageBattery) {
    PowerControl_Manage_Battery();
  }
}

void PowerControl_Watchdog_Restart() {
  // do not pet watchdog for more than 30 seconds to restart
  FOSSASAT_DEBUG_PRINTLN(F("Restart in 30 seconds ..."));
  FOSSASAT_DEBUG_DELAY(10);
  LowPower.deepSleep(30000);
}

void PowerControl_Deploy() {
  FOSSASAT_DEBUG_PRINTLN(F("Deploy"));
  FOSSASAT_DEBUG_DELAY(10);

  // enable MOSFETs one at a time
  digitalWrite(DEPLOYMENT_FET_1, HIGH);
  PowerControl_Wait(1000, LOW_POWER_SLEEP);
  digitalWrite(DEPLOYMENT_FET_1, LOW);

  digitalWrite(DEPLOYMENT_FET_2, HIGH);
  PowerControl_Wait(1000, LOW_POWER_SLEEP);
  digitalWrite(DEPLOYMENT_FET_2, LOW);
}

float PowerControl_Get_Battery_Voltage() {
  return(Sensors_Current_ReadVoltage(currSensorMPPT));
}

void PowerControl_Manage_Battery() {
  // check battery voltage
  if((PowerControl_Get_Battery_Voltage() <= PersistentStorage_SystemInfo_Get<uint16_t>(FLASH_LOW_POWER_MODE_VOLTAGE_LIMIT)) && (PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED) == 1)) {
    // activate low power mode
    systemInfoBuffer[FLASH_LOW_POWER_MODE] = LOW_POWER_SLEEP;

    // write the change immediately if power mode changed
    if(PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_LOW_POWER_MODE) == LOW_POWER_NONE) {
      PersistentStorage_Set_Buffer(FLASH_SYSTEM_INFO, systemInfoBuffer, FLASH_EXT_PAGE_SIZE);
    }

  } else {
    // deactivate low power mode
    systemInfoBuffer[FLASH_LOW_POWER_MODE] = LOW_POWER_NONE;
  }

  // check temperature limit to enable/disable charging
  float mpptTempLimit = PersistentStorage_SystemInfo_Get<float>(FLASH_MPPT_TEMP_LIMIT);
  uint8_t mpptKeepAlive = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_MPPT_KEEP_ALIVE_ENABLED);
  uint8_t mpptTempSwitch = PersistentStorage_SystemInfo_Get<uint8_t>(FLASH_MPPT_TEMP_SWITCH_ENABLED);
  if(mpptKeepAlive == 1) {
    // MPPT keep alive is enabled, force charging regardless of everything else
    digitalWrite(MPPT_OFF, LOW);
  } else if((mpptTempSwitch == 1) && ((Sensors_Temperature_Read(tempSensorBattery) <= mpptTempLimit) || (Sensors_Temperature_Read(tempSensorSecBattery) <= mpptTempLimit))) {
    // at least one battery has temperature below limit, disable charging
    digitalWrite(MPPT_OFF, HIGH);
  } else {
    // temperature above limit, enable charging
    digitalWrite(MPPT_OFF, LOW);
  }

  // check temperature and voltage limit to enable heater
  float heaterTempLimit = PersistentStorage_SystemInfo_Get<float>(FLASH_BATTERY_HEATER_TEMP_LIMIT);
  if((Sensors_Temperature_Read(tempSensorBattery) <= heaterTempLimit) &&
     (Sensors_Temperature_Read(tempSensorSecBattery) <= heaterTempLimit) &&
     (PowerControl_Get_Battery_Voltage() >= PersistentStorage_SystemInfo_Get<int16_t>(FLASH_HEATER_BATTERY_VOLTAGE_LIMIT))) {
    // both temperatures are below limit and battery is above limit, enable heater
    analogWrite(BATTERY_HEATER_FET, PersistentStorage_SystemInfo_Get<uint8_t>(BATTERY_HEATER_DUTY_CYCLE));
  } else {
    // temperature is too high or battery is too low, disable heater
    digitalWrite(BATTERY_HEATER_FET, LOW);
  }
}
