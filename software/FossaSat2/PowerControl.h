#ifndef _FOSSASAT_POWER_CONTROL_H
#define _FOSSASAT_POWER_CONTROL_H

#include "FossaSat2.h"

void PowerControl_Wait(uint32_t ms, uint8_t type);

void PowerControl_Watchdog_Heartbeat();
void PowerControl_Watchdog_Restart();

void PowerControl_Deploy();

#endif
