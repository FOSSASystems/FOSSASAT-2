#ifndef _FOSSASAT_POWER_CONTROL_H
#define _FOSSASAT_POWER_CONTROL_H

#include "FossaSat2.h"

/*
   Low Power Modes
*/

#define LOW_POWER_NONE                                  0
#define LOW_POWER_IDLE                                  1
#define LOW_POWER_SLEEP                                 2
#define LOW_POWER_DEEP_SLEEP                            3

uint32_t PowerControl_Get_Sleep_Interval();
void PowerControl_Wait(uint32_t ms, uint8_t type = LOW_POWER_NONE, bool radioSleep = false);

void PowerControl_Watchdog_Heartbeat(bool manageBattery = true);
void PowerControl_Watchdog_Restart();

void PowerControl_Deploy();

float PowerControl_Get_Battery_Voltage();
void PowerControl_Manage_Battery();

#endif
