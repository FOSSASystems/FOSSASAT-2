#ifndef _FOSSASAT_PERSISTENT_STORAGE_H
#define _FOSSASAT_PERSISTENT_STORAGE_H

#include "FossaSat2.h"

// TODO external flash control

template <class T>
T PersistentStorage_Read_Internal(uint16_t addr);
template <class T>
T PersistentStorage_Read_External(uint32_t addr);

template <class T>
void PersistentStorage_Write_Internal(uint16_t addr, T val);
template <class T>
void PersistentStorage_Write_External(uint32_t addr, T val);

void PersistentStorage_Wipe_Internal();
void PersistentStorage_Wipe_External();

void PersistentStorage_Set_Callsign(char* newCallsign);
void PersistentStorage_Get_Callsign(char* buff, uint8_t len);

#endif
