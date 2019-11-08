#include "PowerControl.h"

void PowerControl_Wait(uint32_t ms, uint8_t type) {
  if(ms == 0) {
    return;
  }

  // calculate number of required loops (rounded up)
  float stepSize = 1000.0;
  if(type == LOW_POWER_NONE) {
    // 10 ms steps when no sleep mode is active
    stepSize = 10.0;
  }
  float numLoops = 0.5 + (float)ms / stepSize;

  // perform all loops
  for(uint32_t i = 0; i < (uint32_t)numLoops; i++) {
    PowerControl_Watchdog_Heartbeat();

    #warning "LowPower is disabled due to https://github.com/stm32duino/STM32LowPower/issues/14"
    delay((uint32_t)stepSize);
    /*switch(type) {
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
    }*/
  }
}

void PowerControl_Watchdog_Heartbeat() {
  // toggle watchdog pin
  digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));

  // save timestamp
  lastHeartbeat = millis();
}

void PowerControl_Watchdog_Restart() {
  // do not pet watchdog for more than 15 seconds to restart
  FOSSASAT_DEBUG_PRINTLN(F("Restart in 15 seconds ..."));
  LowPower.deepSleep(16000);
}

void PowerControl_Deploy() {
  FOSSASAT_DEBUG_PRINTLN(F("Deploy"));

  // burn the nichrome wires
  digitalWrite(DEPLOYMENT_FET_1, HIGH);
  digitalWrite(DEPLOYMENT_FET_1, HIGH);

  // wait a bit
  PowerControl_Wait(1200, LOW_POWER_NONE);

  // set MOSFETs low
  digitalWrite(DEPLOYMENT_FET_1, LOW);
  digitalWrite(DEPLOYMENT_FET_1, LOW);
}
