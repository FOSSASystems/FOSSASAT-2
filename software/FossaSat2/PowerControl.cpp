#include "PowerControl.h"

void PowerControl_Wait(uint32_t ms, uint8_t type, bool radioSleep) {
  if (ms == 0) {
    return;
  }

  // calculate number of required loops (rounded up)
  float stepSize = 500.0;
  if (type == LOW_POWER_NONE) {
    // 10 ms steps when no sleep mode is active
    stepSize = 10.0;
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
      case LOW_POWER_SHUTDOWN:
        LowPower.shutdown((uint32_t)stepSize);
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

void PowerControl_Watchdog_Heartbeat() {
  // toggle watchdog pin
  digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));

  // check voltage
  PowerControl_Check_Battery_Limit();
}

void PowerControl_Watchdog_Restart() {
  // do not pet watchdog for more than 15 seconds to restart
  FOSSASAT_DEBUG_PRINTLN(F("Restart in 15 seconds ..."));
  FOSSASAT_DEBUG_DELAY(10);
  LowPower.deepSleep(16000);
}

void PowerControl_Deploy() {
  FOSSASAT_DEBUG_PRINTLN(F("Deploy"));
  FOSSASAT_DEBUG_DELAY(10);

  // burn the nichrome wires
  digitalWrite(DEPLOYMENT_FET_1, HIGH);
  digitalWrite(DEPLOYMENT_FET_2, HIGH);

  // wait a bit
  PowerControl_Wait(1500, LOW_POWER_SLEEP);

  // set MOSFETs low
  digitalWrite(DEPLOYMENT_FET_1, LOW);
  digitalWrite(DEPLOYMENT_FET_2, LOW);
}

float PowerControl_Get_Battery_Voltage() {
  return(currSensorMPPT.readBusVoltage());
}

void PowerControl_Check_Battery_Limit() {
  // check battery voltage
  if((PowerControl_Get_Battery_Voltage() <= LOW_POWER_MODE_VOLTAGE_LIMIT) && (PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED) == 1)) {
    // activate low power mode
   PersistentStorage_Set<uint8_t>(FLASH_LOW_POWER_MODE, LOW_POWER_SLEEP);
  } else {
    // deactivate low power mode
    PersistentStorage_Set<uint8_t>(FLASH_LOW_POWER_MODE, LOW_POWER_NONE);
  }
}

void PowerControl_Print_Power_Config() {
  FOSSASAT_DEBUG_PORT.println(F("--- Power Configuration ---"));
  FOSSASAT_DEBUG_PORT.print(F("Transmissions enabled: "));
  FOSSASAT_DEBUG_PORT.println(PersistentStorage_Get<uint8_t>(FLASH_TRANSMISSIONS_ENABLED));
  FOSSASAT_DEBUG_PORT.print(F("Low power mode enabled: "));
  FOSSASAT_DEBUG_PORT.println(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE_ENABLED));
  FOSSASAT_DEBUG_PORT.print(F("Low power mode: "));
  FOSSASAT_DEBUG_PORT.println(PersistentStorage_Get<uint8_t>(FLASH_LOW_POWER_MODE));
  FOSSASAT_DEBUG_PORT.println(F("---------------------------"));
}
