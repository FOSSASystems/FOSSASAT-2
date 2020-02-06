#include "PowerControl.h"

void PowerControl_Watchdog_Heartbeat(bool manageBattery) {
  // toggle watchdog pin
  digitalWrite(WATCHDOG_IN, !digitalRead(WATCHDOG_IN));

  // manage battery (low power, heater, charging)
  if(manageBattery) {
    //PowerControl_Manage_Battery();
  }
}
